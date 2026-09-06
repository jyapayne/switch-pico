#include "input/switch_native_output.h"
#include "input/native_output_scheduler.h"
#include "usb/switch/switch_native_haptics.h"
#include "usb/switch/switch_haptics_amplitudes.h"

#include <btstack.h>
#include <pico/critical_section.h>
#include <pico/stdlib.h>
#include <uni.h>
extern "C" {
#include <parser/uni_hid_parser_switch.h>
}
#include <string.h>

namespace {
constexpr uint8_t kSlots = 4;
constexpr uint8_t kCapacity = 16;
constexpr uint64_t kHostExpiryUs = 50000;
constexpr uint64_t kRefreshUs = 40000;
constexpr uint64_t kCommandWindowUs = 8000;
constexpr uint8_t kNeutral[8] = {0, 1, 0x40, 0x40, 0, 1, 0x40, 0x40};
constexpr uint32_t kConnected = 1, kApproved = 2, kActive = 4,
                   kMono = 8, kFeedback = 16;

struct Command {
    ControllerRumbleOutput rumble{};
    uint64_t received_us = 0;
    bool stateful = false;
    uint64_t first_received_us = 0;
};
struct SharedSlot {
    uint32_t generation = 0;
    bool accepting = false;
    bool stop_pending = false;
    bool host_active = false;
    bool lost = false;
    uint8_t head = 0, count = 0;
    Command queue[kCapacity]{};
    SwitchNativeOutputDiagnostics diagnostics{};
    uint32_t latency_histogram[64]{};
};
struct OutputSlot {
    uni_hid_device_t* device = nullptr;
    ControllerIdentity identity{};
    uint32_t generation = 0;
    bool approved = false, active = false, mono = false;
    bool neutral_needed = false, dirty = false, host_valid = false;
    bool feedback_active = false, pending_host = false, pending_trimmed = false;
    bool permission_requested = false, processing = false, urgent_stop = false;
    bool requested_urgent_stop = false;
    uint64_t deadline_us = 0, requested_deadline_us = 0;
    uint64_t feedback_until_us = 0, last_send_us = 0, retry_us = 0;
    uint64_t pending_since_us = 0;
    uint64_t pending_valid_until_us = UINT64_MAX;
    ControllerRumbleOutput feedback{};
    Command host{};
    SwitchNativeHapticsEncoder encoder{};
    SwitchNativeHapticsPackets packets{};
    uint8_t packet_index = 0;
};

critical_section_t g_lock;
SharedSlot g_shared[kSlots];
OutputSlot g_outputs[kSlots];
AdapterConfiguration g_configuration{};
uint32_t g_configuration_generation = 0;
bool g_configured = false, g_prepared = false, g_runloop_ready = false;
bool g_work_pending = false, g_timer_armed = false;
bool g_polling = false;
btstack_data_source_t g_data_source{};
btstack_timer_source_t g_timer{};

void poll();
void process_slot(uint8_t slot);
bool granted(uni_hid_device_t* device, uint16_t cid, uint32_t generation);
void increment(uint32_t& value, uint32_t amount = 1) {
    value = amount > UINT32_MAX - value ? UINT32_MAX : value + amount;
}
bool silent(const ControllerRumbleOutput& rumble) {
    if (rumble.low_frequency_magnitude || rumble.high_frequency_magnitude) return false;
    for (const auto& actuator : rumble.hd.actuators)
        for (uint8_t i = 0; i < actuator.sample_count && i < 3; ++i)
            if (actuator.samples[i].low_amplitude_q15 || actuator.samples[i].high_amplitude_q15)
                return false;
    return true;
}
bool same_hold(const Command& previous, const Command& next, bool endpoint) {
    if (previous.stateful != next.stateful) return false;
    if (next.stateful)
        return previous.rumble.low_frequency_magnitude == next.rumble.low_frequency_magnitude &&
               previous.rumble.high_frequency_magnitude == next.rumble.high_frequency_magnitude;
    for (uint8_t side = 0; side < 2; ++side) {
        const auto& a = previous.rumble.hd.actuators[side];
        const auto& b = next.rumble.hd.actuators[side];
        if (!a.sample_count || a.sample_count > 3 || b.sample_count != 1 ||
            (!endpoint && a.sample_count != 1)) return false;
        const auto& held = a.samples[a.sample_count - 1];
        const auto& wanted = b.samples[0];
        if (held.low_frequency_index != wanted.low_frequency_index ||
            held.high_frequency_index != wanted.high_frequency_index ||
            held.low_amplitude_q15 != wanted.low_amplitude_q15 ||
            held.high_amplitude_q15 != wanted.high_amplitude_q15) return false;
    }
    return true;
}

ControllerRumbleOutput magnitudes(uint8_t low, uint8_t high) {
    ControllerRumbleOutput result{};
    result.low_frequency_magnitude = low;
    result.high_frequency_magnitude = high;
    result.hd.actuators[0].sample_count = 1;
    result.hd.actuators[1].sample_count = 1;
    constexpr uint32_t maximum = SwitchHapticsTables::kAmplitudeQ15[228];
    result.hd.actuators[0].samples[0].low_amplitude_q15 = (maximum * low + 127) / 255;
    result.hd.actuators[1].samples[0].high_amplitude_q15 = (maximum * high + 127) / 255;
    return result;
}
void publish_flags(uint8_t slot) {
    const OutputSlot& output = g_outputs[slot];
    critical_section_enter_blocking(&g_lock);
    auto& state = g_shared[slot];
    state.accepting = output.active && output.approved;
    state.diagnostics.flags = (output.device ? kConnected : 0) |
        (output.approved ? kApproved : 0) | (output.active ? kActive : 0) |
        (output.mono ? kMono : 0) | (output.feedback_active ? kFeedback : 0);
    critical_section_exit(&g_lock);
}
void reset_pending(OutputSlot& output) {
    output.packets = {};
    output.packet_index = 0;
    output.pending_host = false;
    output.encoder.reset();
    output.neutral_needed = true;
    output.retry_us = 0;
}
void data_source(btstack_data_source_t*, btstack_data_source_callback_type_t) {
    if (__atomic_exchange_n(&g_work_pending, false, __ATOMIC_ACQ_REL)) poll();
}
void timer(btstack_timer_source_t*) {
    g_timer_armed = false;
    poll();
}
void ensure_runloop() {
    if (g_runloop_ready) return;
    btstack_run_loop_set_data_source_handler(&g_data_source, data_source);
    btstack_run_loop_enable_data_source_callbacks(&g_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&g_data_source);
    btstack_run_loop_set_timer_handler(&g_timer, timer);
    g_runloop_ready = true;
}
uint64_t periodic_deadline(const OutputSlot& output) {
    uint64_t deadline = UINT64_MAX;
    if (output.host_valid || output.feedback_active)
        deadline = output.last_send_us + kRefreshUs;
    if (output.feedback_active && output.feedback_until_us < deadline)
        deadline = output.feedback_until_us;
    if (!output.feedback_active && output.host_valid && !output.host.stateful &&
        output.host.received_us + kHostExpiryUs < deadline)
        deadline = output.host.received_us + kHostExpiryUs;
    return deadline;
}
bool ready(const OutputSlot& output, uint64_t now) {
    return output.active && now >= output.retry_us &&
        (output.neutral_needed || output.dirty || output.packets.count ||
         ((output.feedback_active || output.host_valid) &&
          now >= output.last_send_us + kRefreshUs));
}
void schedule() {
    if (g_timer_armed) {
        btstack_run_loop_remove_timer(&g_timer);
        g_timer_armed = false;
    }
    uint64_t due = UINT64_MAX;
    const uint64_t now = time_us_64();
    for (const auto& output : g_outputs) {
        if (!output.active) continue;
        native_output_scheduler_reserve(output.device, output.generation,
                                        periodic_deadline(output));
        if ((output.neutral_needed || output.dirty || output.packets.count) &&
            !output.permission_requested && output.retry_us < due) due = output.retry_us;
        if (output.feedback_active && output.feedback_until_us < due)
            due = output.feedback_until_us;
        if (output.host_valid && !output.host.stateful &&
            output.host.received_us + kHostExpiryUs < due)
            due = output.host.received_us + kHostExpiryUs;
        if (!output.neutral_needed && !output.dirty && !output.packets.count &&
            (output.feedback_active || output.host_valid) &&
            output.last_send_us + kRefreshUs < due)
            due = output.last_send_us + kRefreshUs;
    }
    if (due == UINT64_MAX) return;
    // Pico adds one tick itself. The handler checks absolute microseconds.
    const uint64_t delta_ms = due > now ? (due - now) / 1000 : 0;
    btstack_run_loop_set_timer(&g_timer, delta_ms > 0 ? delta_ms - 1 : 0);
    g_timer_armed = true;
    btstack_run_loop_add_timer(&g_timer);
}
bool permission(uint8_t slot) {
    OutputSlot& output = g_outputs[slot];
    if (native_output_scheduler_granted(output.device)) return true;
    if (!output.permission_requested ||
        output.requested_deadline_us != output.deadline_us ||
        output.requested_urgent_stop != output.urgent_stop) {
        const bool waiting = output.permission_requested;
        output.permission_requested = true;  // Callback may be synchronous.
        output.requested_deadline_us = output.deadline_us;
        output.requested_urgent_stop = output.urgent_stop;
        if (native_output_scheduler_request(output.device, output.generation,
                output.deadline_us, output.urgent_stop, granted) != 0) {
            output.permission_requested = false;
            output.retry_us = time_us_64() + 1000;
        }
        if (!waiting && !native_output_scheduler_granted(output.device)) {
            critical_section_enter_blocking(&g_lock);
            increment(g_shared[slot].diagnostics.congested_attempts);
            critical_section_exit(&g_lock);
        }
    }
    return native_output_scheduler_granted(output.device);
}

bool send(uint8_t slot, const uint8_t* bytes) {
    OutputSlot& output = g_outputs[slot];
    if (!permission(slot)) return false;
    auto* const device = output.device;
    const uint32_t generation = output.generation;
    const bool sent = uni_hid_parser_switch_native_send(device, bytes);
    const uint64_t now = time_us_64();
    if (output.device != device || output.generation != generation) return false;
    critical_section_enter_blocking(&g_lock);
    auto& diagnostics = g_shared[slot].diagnostics;
    if (sent) {
        increment(diagnostics.submitted_reports);
        memcpy(diagnostics.last_wire, bytes, 8);
    } else {
        increment(diagnostics.congested_attempts);
    }
    critical_section_exit(&g_lock);
    output.retry_us = sent ? 0 : now + 1000;
    if (sent) output.last_send_us = now;
    return sent;
}
void finish_command(uint8_t slot) {
    OutputSlot& output = g_outputs[slot];
    critical_section_enter_blocking(&g_lock);
    SharedSlot& shared = g_shared[slot];
    if (output.pending_host) {
        const uint64_t elapsed = time_us_64() - output.pending_since_us;
        const uint32_t latency = elapsed > UINT32_MAX ? UINT32_MAX : elapsed;
        increment(shared.diagnostics.completed_commands);
        increment(shared.latency_histogram[latency / 250 < 63 ? latency / 250 : 63]);
        if (latency > shared.diagnostics.max_latency_us)
            shared.diagnostics.max_latency_us = latency;
        if (output.packets.raw) increment(shared.diagnostics.raw_commands);
        if (output.packets.quantized || output.pending_trimmed)
            increment(shared.diagnostics.quantized_commands);
    }
    critical_section_exit(&g_lock);
    output.packets = {};
    output.packet_index = 0;
    output.pending_host = false;
}
ControllerRumbleOutput current_host(const Command& command, uint64_t now,
                                   bool& trimmed) {
    if (command.stateful) return magnitudes(command.rumble.low_frequency_magnitude,
                                            command.rumble.high_frequency_magnitude);
    ControllerRumbleOutput result = command.rumble;
    const uint64_t age = now > command.received_us ? now - command.received_us : 0;
    for (auto& actuator : result.hd.actuators) {
        const uint8_t count = actuator.sample_count > 3 ? 3 : actuator.sample_count;
        if (!count) continue;
        const uint8_t first = age >= kCommandWindowUs ? count - 1 : age * count / kCommandWindowUs;
        if (first) {
            for (uint8_t i = first; i < count; ++i) actuator.samples[i - first] = actuator.samples[i];
            actuator.sample_count = count - first;
            trimmed = true;
        }
    }
    if (trimmed) result.raw_unmodified = false;
    return result;
}
void restore_compatibility(OutputSlot& output, uint64_t now) {
    if (native_output_scheduler_granted(output.device))
        native_output_scheduler_complete(output.device, output.generation);
    native_output_scheduler_cancel(output.device);
    output.permission_requested = false;
    uni_hid_parser_switch_native_release(output.device);
    output.active = false;
    if (output.device->report_parser.play_dual_rumble == nullptr) return;
    ControllerRumbleOutput rumble{};
    uint16_t duration = 0;
    if (output.feedback_active && now < output.feedback_until_us) {
        rumble = output.feedback;
        duration = (output.feedback_until_us - now + 999) / 1000;
    } else if (output.host_valid && (output.host.stateful || now < output.host.received_us + kHostExpiryUs)) {
        rumble = output.host.rumble;
        duration = output.host.stateful ? UINT16_MAX :
            (output.host.received_us + kHostExpiryUs - now + 999) / 1000;
    }
    output.device->report_parser.play_dual_rumble(output.device, 0,
        silent(rumble) ? 0 : duration, rumble.high_frequency_magnitude, rumble.low_frequency_magnitude);
    output.host_valid = false;
    output.feedback_active = false;
    output.dirty = false;
}
void process_slot_step(uint8_t slot) {
    OutputSlot& output = g_outputs[slot];
    if (!output.active) return;
    const uint64_t now = time_us_64();
    Command newest{};
    bool have_new = false, stop = false, loss = false;
    critical_section_enter_blocking(&g_lock);
    SharedSlot& shared = g_shared[slot];
    stop = shared.stop_pending;
    loss = shared.lost;
    shared.stop_pending = shared.lost = false;
    if (shared.count) {
        newest = shared.queue[(shared.head + shared.count - 1) % kCapacity];
        have_new = true;
        if (shared.count > 1) {
            increment(shared.diagnostics.dropped_commands, shared.count - 1);
            loss = true;
        }
        shared.count = shared.head = 0;
    }
    critical_section_exit(&g_lock);
    if (stop && output.host_valid && !output.feedback_active) output.urgent_stop = true;
    if (have_new && !loss && !output.feedback_active) {
        const bool pending_hold = output.pending_host &&
            same_hold(output.host, newest, false);
        const bool applied_hold = !output.pending_host && !output.dirty &&
            !output.neutral_needed && !output.packets.count &&
            (output.host_valid || silent(newest.rumble)) &&
            same_hold(output.host, newest, true);
        if (pending_hold || applied_hold) {
            // A held state needs only the regular watchdog refresh, not another
            // radio packet for every identical USB report. Keep original pending
            // latency, but extend expiry from the newest receipt.
            output.host = newest;
            output.host_valid = !silent(newest.rumble);
            critical_section_enter_blocking(&g_lock);
            increment(g_shared[slot].diagnostics.coalesced_commands);
            critical_section_exit(&g_lock);
            have_new = stop = false;
        }
    }
    if (have_new) {
        if (!output.feedback_active) {
            if (output.pending_host) {
                critical_section_enter_blocking(&g_lock);
                increment(g_shared[slot].diagnostics.dropped_commands);
                critical_section_exit(&g_lock);
            }
            // Preparing advances the codec model even before a successful send.
            // Discarding ANY prepared schedule requires a physical baseline.
            if (loss || stop || output.packets.count) reset_pending(output);
            output.pending_host = true;
            output.pending_since_us = newest.first_received_us;
            if (output.approved)
                output.deadline_us = newest.first_received_us + kCommandWindowUs;
            output.dirty = true;
        }
        output.host = newest;
        output.host_valid = !silent(newest.rumble);
    } else if ((stop || loss) && !output.feedback_active) {
        reset_pending(output);
        output.dirty = true;
    }
    if (output.host_valid && !output.host.stateful && now >= output.host.received_us + kHostExpiryUs) {
        output.host_valid = false;
        if (!output.feedback_active) {
            reset_pending(output);
            output.dirty = true;
            output.deadline_us = output.host.received_us + kHostExpiryUs;
            output.urgent_stop = true;
        }
    }
    if (output.feedback_active && now >= output.feedback_until_us) {
        output.feedback_active = false;
        reset_pending(output);
        output.dirty = true;
        output.deadline_us = output.feedback_until_us;
        output.urgent_stop = !output.host_valid && !silent(output.feedback);
    }
    if (output.packets.count && now >= output.pending_valid_until_us) {
        const bool host_command = output.pending_host;
        reset_pending(output);
        output.pending_host = host_command;
        output.dirty = true;
    }
    if (now < output.retry_us) return;
    if (output.neutral_needed) {
        if (!send(slot, kNeutral)) return;
        output.encoder.reset();
        output.neutral_needed = false;
        output.urgent_stop = false;
        critical_section_enter_blocking(&g_lock);
        increment(g_shared[slot].diagnostics.resynchronizations);
        critical_section_exit(&g_lock);
        if (output.approved && output.dirty && !output.host_valid && !output.feedback_active) {
            output.dirty = false;
            finish_command(slot);  // This neutral already fulfills stop/expiry.
            publish_flags(slot);
            return;
        }
        // A baseline consumes its own grant, even when a codec packet follows.
        if (output.approved) return;
    }
    if (!output.approved) {
        restore_compatibility(output, now);
        publish_flags(slot);
        return;
    }
    if (output.feedback_active) {
        // Host updates still replace the retained timeline, but do not restart
        // or interfere with an active local confirmation.
        output.pending_host = false;
    }
    if (!output.packets.count && (output.dirty ||
        ((output.host_valid || output.feedback_active) && now >= output.last_send_us + kRefreshUs))) {
        // Request actual credit availability before mutating the encoder model.
        // Timer polling misses short free-buffer windows behind HCI credit writes.
        if (!output.dirty) output.deadline_us = output.last_send_us + kRefreshUs;
        if (!permission(slot)) return;
        ControllerRumbleOutput effective{};
        output.pending_trimmed = false;
        if (output.feedback_active) effective = output.feedback;
        else if (output.host_valid) effective = current_host(output.host, now, output.pending_trimmed);
        output.pending_valid_until_us = UINT64_MAX;
        if (!output.feedback_active && output.host_valid && !output.host.stateful) {
            const uint64_t age = now - output.host.received_us;
            for (const auto& actuator : output.host.rumble.hd.actuators) {
                const uint8_t count = actuator.sample_count > 3 ? 3 : actuator.sample_count;
                if (count < 2 || age >= kCommandWindowUs) continue;
                const uint8_t current = age * count / kCommandWindowUs;
                if (current + 1 >= count) continue;
                const uint64_t boundary = output.host.received_us +
                    ((current + 1) * kCommandWindowUs + count - 1) / count;
                if (boundary < output.pending_valid_until_us)
                    output.pending_valid_until_us = boundary;
            }
        }
        const uint64_t begin = time_us_64();
        output.packets = output.encoder.encode(effective, output.mono, true);
        const uint32_t cost = time_us_64() - begin;
        critical_section_enter_blocking(&g_lock);
        if (cost > g_shared[slot].diagnostics.max_encode_us)
            g_shared[slot].diagnostics.max_encode_us = cost;
        critical_section_exit(&g_lock);
        output.packet_index = 0;
        output.dirty = false;
    }
    // One legal wire packet per grant. The wrapper releases and reacquires
    // through the shared arbiter before submitting a prepared tail.
    if (output.packet_index < output.packets.count) {
        if (!send(slot, output.packets.bytes[output.packet_index])) return;
        ++output.packet_index;
    }
    if (output.packets.count && output.packet_index == output.packets.count) finish_command(slot);
    publish_flags(slot);
}
void process_slot(uint8_t slot) {
    OutputSlot& output = g_outputs[slot];
    if (output.processing) return;
    output.processing = true;
    auto* const device = output.device;
    const uint32_t generation = output.generation;
    // A neutral barrier plus the codec's two-packet schedule is the maximum
    // immediate work. Each attempt releases its token before another request.
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        process_slot_step(slot);
        if (output.device != device || output.generation != generation) return;
        const bool had_grant = native_output_scheduler_granted(output.device);
        if (had_grant) native_output_scheduler_complete(device, generation);
        if (!had_grant || !ready(output, time_us_64())) break;
    }
    output.processing = false;
}
bool granted(uni_hid_device_t* device, uint16_t cid, uint32_t generation) {
    if (!g_prepared || device == nullptr) return false;
    for (uint8_t slot = 0; slot < kSlots; ++slot) {
        auto& output = g_outputs[slot];
        if (output.device != device || output.generation != generation || !output.active ||
            device->conn.interrupt_cid != cid || !output.permission_requested ||
            !native_output_scheduler_granted(device)) continue;
        output.permission_requested = false;
        output.retry_us = 0;
        // Synchronous delivery resumes the caller's permission() immediately.
        // A different slot can run here even while the outer poll is active.
        if (!output.processing) {
            process_slot(slot);
            schedule();
        }
        return false;  // Nintendo's generic LED FIFO must still see the event.
    }
    native_output_scheduler_complete(device, generation);  // Retired/no-send callback.
    return false;
}
void poll() {
    if (g_polling) return;
    g_polling = true;
    for (uint8_t slot = 0; slot < kSlots; ++slot) process_slot(slot);
    schedule();
    g_polling = false;
}
uint32_t percentile(const SharedSlot& shared, uint32_t percent) {
    const uint64_t target = (uint64_t{shared.diagnostics.completed_commands} * percent + 99) / 100;
    if (!target) return 0;
    uint64_t count = 0;
    for (uint8_t i = 0; i < 64; ++i) {
        count += shared.latency_histogram[i];
        if (count >= target) return i == 63 ? shared.diagnostics.max_latency_us : (i + 1) * 250u;
    }
    return shared.diagnostics.max_latency_us;
}
}  // namespace

void switch_native_output_prepare() {
    if (g_prepared) return;
    critical_section_init(&g_lock);
    for (uint8_t i = 0; i < kSlots; ++i) g_shared[i].diagnostics.slot = i;
    g_prepared = true;
}
void switch_native_output_attach(uint8_t slot, uint32_t generation,
                                uni_hid_device_t* device,
                                const ControllerIdentity& identity) {
    if (!g_prepared || slot >= kSlots || device == nullptr || identity.vendor_id != 0x057e) return;
    uint8_t type = 0, hi = 0, lo = 0;
    if (!uni_hid_parser_switch_native_info(device, &type, &hi, &lo) ||
        !((type == 3 && identity.product_id == 0x2009) ||
          (type == 1 && identity.product_id == 0x2006) ||
          (type == 2 && identity.product_id == 0x2007))) return;
    ensure_runloop();
    OutputSlot& output = g_outputs[slot];
    if (output.device) native_output_scheduler_cancel(output.device);
    output = {};
    output.device = device;
    output.generation = generation;
    output.deadline_us = time_us_64() + kCommandWindowUs;
    output.identity = identity;
    output.mono = type != 3;
    critical_section_enter_blocking(&g_lock);
    g_shared[slot] = {};
    g_shared[slot].generation = generation;
    auto& diagnostics = g_shared[slot].diagnostics;
    diagnostics.slot = slot;
    diagnostics.type = type;
    diagnostics.firmware_hi = hi;
    diagnostics.firmware_lo = lo;
    diagnostics.generation = generation;
    critical_section_exit(&g_lock);
    output.approved = adapter_configuration_native_switch_approved(g_configuration, identity);
    if (output.approved && uni_hid_parser_switch_native_acquire(device)) {
        output.active = true;
        reset_pending(output);
    }
    publish_flags(slot);
    poll();
}
void switch_native_output_detach(uni_hid_device_t* device) {
    if (!g_prepared || device == nullptr) return;
    for (uint8_t i = 0; i < kSlots; ++i) {
        if (g_outputs[i].device != device) continue;
        // Parser teardown owns timer retirement; never transmit on a dead CID.
        native_output_scheduler_cancel(device);
        g_outputs[i] = {};
        critical_section_enter_blocking(&g_lock);
        g_shared[i].accepting = false;
        g_shared[i].count = g_shared[i].head = 0;
        g_shared[i].stop_pending = g_shared[i].lost = false;
        g_shared[i].host_active = false;
        g_shared[i].diagnostics.flags = 0;
        critical_section_exit(&g_lock);
    }
    schedule();
}
void switch_native_output_configure(const AdapterConfiguration& configuration,
                                   uint32_t generation) {
    if (!g_prepared || (g_configured && generation == g_configuration_generation)) return;
    g_configuration = configuration;
    g_configuration_generation = generation;
    g_configured = true;
    for (uint8_t i = 0; i < kSlots; ++i) {
        OutputSlot& output = g_outputs[i];
        if (!output.device) continue;
        const bool approved = adapter_configuration_native_switch_approved(configuration, output.identity);
        if (approved == output.approved) continue;
        output.approved = approved;
        native_output_scheduler_cancel(output.device);
        output.permission_requested = false;
        output.deadline_us = time_us_64() + kCommandWindowUs;
        output.urgent_stop = !approved && output.active;
        if (approved && !output.active && uni_hid_parser_switch_native_acquire(output.device)) {
            output.active = true;
            reset_pending(output);
        } else if (!approved && output.active) {
            reset_pending(output);
            output.deadline_us = time_us_64();
            output.urgent_stop = true;
        }
        publish_flags(i);
    }
    poll();
}
bool switch_native_output_submit(uint8_t slot, uint32_t generation,
                                 uint64_t received_us,
                                 const ControllerRumbleOutput& rumble,
                                 bool stateful) {
    if (!g_prepared || slot >= kSlots) return false;
    critical_section_enter_blocking(&g_lock);
    SharedSlot& shared = g_shared[slot];
    const bool accepted = shared.accepting && shared.generation == generation;
    if (accepted) {
        const Command update{rumble, received_us, stateful, received_us};
        Command* previous = shared.count
            ? &shared.queue[(shared.head + shared.count - 1) % kCapacity] : nullptr;
        if (previous && same_hold(*previous, update, false)) {
            previous->received_us = received_us;
            increment(shared.diagnostics.coalesced_commands);
        } else {
            if (shared.count == kCapacity) {
                shared.head = (shared.head + 1) % kCapacity;
                --shared.count;
                shared.lost = true;
                increment(shared.diagnostics.dropped_commands);
            }
            shared.queue[(shared.head + shared.count) % kCapacity] = update;
            ++shared.count;
        }
        const bool active = !silent(rumble);
        shared.stop_pending = shared.stop_pending || (shared.host_active && !active);
        shared.host_active = active;
        increment(shared.diagnostics.received_commands);
    }
    critical_section_exit(&g_lock);
    if (accepted) {
        __atomic_store_n(&g_work_pending, true, __ATOMIC_RELEASE);
        // Unlike execute_on_main_thread(), this IRQ-safe wake does not acquire
        // the radio async-context lock on the USB core.
        btstack_run_loop_poll_data_sources_from_irq();
    }
    return accepted;
}

bool switch_native_output_owns(const uni_hid_device_t* device) {
    if (!g_prepared || device == nullptr) return false;
    for (const auto& output : g_outputs)
        if (output.device == device && output.active) return true;
    return false;
}
bool switch_native_output_feedback(uni_hid_device_t* device, uint8_t low,
                                   uint8_t high, uint16_t duration_ms) {
    if (!g_prepared || device == nullptr) return false;
    for (auto& output : g_outputs) {
        if (output.device != device || !output.active) continue;
        const uint64_t now = time_us_64();
        const bool stopped = output.feedback_active && !silent(output.feedback) &&
            (duration_ms == 0 || (low == 0 && high == 0));
        output.feedback = magnitudes(low, high);
        output.feedback_until_us = now + uint64_t{duration_ms} * 1000;
        output.feedback_active = duration_ms != 0;
        reset_pending(output);
        output.dirty = true;
        output.deadline_us = now + kCommandWindowUs;
        if (duration_ms && output.feedback_until_us < output.deadline_us)
            output.deadline_us = output.feedback_until_us;
        output.urgent_stop = stopped;
        poll();
        return true;
    }
    return false;
}
void switch_native_output_snapshot(uint8_t slot, SwitchNativeOutputDiagnostics* output) {
    if (output == nullptr) return;
    *output = {};
    if (!g_prepared || slot >= kSlots) return;
    critical_section_enter_blocking(&g_lock);
    const SharedSlot& shared = g_shared[slot];
    *output = shared.diagnostics;
    output->queue_depth = shared.count;
    output->p50_upper_us = percentile(shared, 50);
    output->p95_upper_us = percentile(shared, 95);
    output->p99_upper_us = percentile(shared, 99);
    critical_section_exit(&g_lock);
}
