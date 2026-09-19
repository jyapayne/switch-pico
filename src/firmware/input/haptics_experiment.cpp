#include "input/haptics_experiment.h"
#include "input/haptics_transport_probe.h"
#include "input/native_output_scheduler.h"
#include "input/switch_hd_rumble_synth.h"

#include <btstack.h>
#include <pico/critical_section.h>
#include <pico/stdlib.h>
#include <uni.h>

#if SWITCH_PICO_HAPTICS_EXPERIMENT_RAM
#define HAPTICS_HOT(name) __time_critical_func(name)
#define HAPTICS_DATA __not_in_flash("haptics_experiment_waveform")
#else
#define HAPTICS_HOT(name) name
#define HAPTICS_DATA
#endif

namespace {

constexpr uint32_t kPacketFrames = SWITCH_PICO_HD_PACKET_FRAMES;
static_assert(kPacketFrames == 32, "HD output requires the qualified 32-frame packet format");
constexpr uint32_t kPacketDenominator = 3;
constexpr uint32_t kPackets = 18432 / kPacketFrames;
constexpr uint32_t kPrimingPackets = 3072 / kPacketFrames;
constexpr uint32_t kToneEndPacket = 15360 / kPacketFrames;
constexpr uint32_t kPhasePackets = 768 / kPacketFrames;
constexpr uint32_t kDrainTimeoutUs = 100000;
// Briefly retain ownership while compatibility output drains. Its parser timer
// is canceled synchronously; it must not survive a device disconnect/reuse.
constexpr uint32_t kRestoreSettleUs = 3000;
constexpr uint16_t kReportBytes = 143;  // A2 + 142-byte report 0x32.
constexpr uint16_t kCrcOffset = kReportBytes - 4;
constexpr uint8_t kNoSlot = 0xff;

enum Error : uint8_t {
    kNoError = 0,
    kUnsupported = 1,
    kMtu = 2,
    kConnection = 3,
    kTimeout = 4,
    kTransport = 5,
    kQueuedOutput = 6,
};

enum class Phase { kIdle, kPrepare, kPattern, kDrain, kRestore };

struct Attachment {
    uni_hid_device_t* device = nullptr;
    uint32_t generation = 0;
    uint16_t cid = 0;
};

struct Command {
    bool pending = false;
    uint8_t action = 0;
    uint8_t slot = kNoSlot;
    uint32_t run_id = 0;
    Attachment connection{};
    uint8_t mode = 0;
};

enum class HostKind : uint8_t { kSwitch, kNative, kRumble, kFeedback };

struct HostUpdate {
    uint64_t received_us = 0;
    union {
        SwitchHapticsFrame frame;
        NativeHapticsFrame native_frame;
    };
    HostKind kind = HostKind::kSwitch;
    uint32_t duration_us = 0;
    uint8_t low = 0;
    uint8_t high = 0;
    HostUpdate() : frame{} {}
};
constexpr uint8_t kHostQueueCapacity = 16;

// Only the attachment identities, mailbox and published snapshot cross cores.
// No BTstack call (including synchronous reentry) holds this lock.
critical_section_t g_lock;
HapticsExperimentDiagnostics g_snapshot;
bool g_snapshot_waiting = false;
uint32_t g_snapshot_request_us = 0;
Command g_command;
bool g_busy = false;
bool g_prepared = false;
bool g_accept_host = false;
HostUpdate g_host_queue[kHostQueueCapacity];
uint8_t g_host_head = 0;
uint8_t g_host_count = 0;
uint32_t g_host_updates = 0;
uint32_t g_host_drops = 0;
uint8_t g_native_cancel = 0;
bool g_have_host_timestamp = false;
uint64_t g_last_host_us = 0;

// Written on core 1 under the lock; core 0 only reads identities for requests.
Attachment g_attachments[4];

// All remaining state belongs exclusively to the BTstack core.
HapticsExperimentDiagnostics g_diagnostics;
Attachment g_connection;
Phase g_phase = Phase::kIdle;
HapticsExperimentState g_finish_state = HapticsExperimentState::kCompleted;
btstack_timer_source_t g_cadence_timer{};
btstack_timer_source_t g_lifecycle_timer{};
bool g_cadence_armed = false;
bool g_lifecycle_armed = false;
bool g_send_requested = false;
bool g_request_in_progress = false;
bool g_in_callback = false;
bool g_last_was_silence = true;
bool g_first_tone_sent = false;
uint32_t g_next_packet = 0;
uint64_t g_start_us = 0;
uint64_t g_end_us = 0;
uint64_t g_lifecycle_due_us = 0;
uint64_t g_request_us = 0;
uint64_t g_restore_deadline_us = 0;
SwitchHdRumbleSynth g_synth;
bool g_synth_started = false;

// round(32 * sin(2*pi*n/30)). A stride of one is 100 Hz at 3 kHz;
// a stride of two is 200 Hz. Preserve phase across all 12 packets of a tone.
static const int8_t HAPTICS_DATA kSine[30] = {
    0, 7, 13, 19, 24, 28, 30, 32, 32, 30, 28, 24, 19, 13, 7,
    0, -7, -13, -19, -24, -28, -30, -32, -32, -30, -28, -24, -19, -13, -7,
};

void cadence_timer(btstack_timer_source_t*);
void lifecycle_timer(btstack_timer_source_t*);
void request_send(bool promote = false);
void restore_compatibility(HapticsExperimentState state);
void start_stream();
bool on_output_grant(uni_hid_device_t* device, uint16_t cid, uint32_t generation);

bool gameplay() {
    return g_diagnostics.mode == 1;
}

uint32_t packet_numerator_us() {
    return kPacketFrames * 1000u;
}

void drain_host_updates() {
    // Bound work even if USB keeps publishing while the BT core drains.
    for (uint8_t index = 0; index <= kHostQueueCapacity; ++index) {
        HostUpdate update;
        critical_section_enter_blocking(&g_lock);
        const uint8_t cancel = g_native_cancel;
        g_native_cancel = 0;
        const bool available = g_host_count != 0 && index < kHostQueueCapacity;
        if (available) {
            update = g_host_queue[g_host_head];
            g_host_head = (g_host_head + 1) % kHostQueueCapacity;
            --g_host_count;
        }
        critical_section_exit(&g_lock);
        if (cancel) g_synth.cancel_native(cancel);
        if (!available) break;
        switch (update.kind) {
            case HostKind::kRumble:
                g_synth.push_rumble(update.low, update.high, update.received_us);
                break;
            case HostKind::kNative:
                g_synth.push_native(update.native_frame, update.received_us);
                break;
            case HostKind::kFeedback:
                g_synth.feedback(update.received_us, update.duration_us, update.low, update.high);
                break;
            case HostKind::kSwitch:
                g_synth.push(update.frame, update.received_us);
                break;
        }
    }
}

bool selected_locked(uint8_t slot, uint32_t generation) {
    return g_busy && g_snapshot.slot == slot &&
           g_snapshot.connection_generation == generation &&
           g_attachments[slot].device != nullptr &&
           g_attachments[slot].generation == generation;
}

bool submit_host_update(uint8_t slot, uint32_t generation,
                        const HostUpdate& update) {
    if (!g_prepared || slot >= 4) return false;
    critical_section_enter_blocking(&g_lock);
    const bool host = update.kind != HostKind::kFeedback;
    const bool ordered = !host || !g_have_host_timestamp ||
                         update.received_us - g_last_host_us <= INT64_MAX;
    const bool accepted = g_accept_host && g_snapshot.mode == 1 &&
                          selected_locked(slot, generation) && ordered;
    if (accepted) {
        if (g_host_count == kHostQueueCapacity) {
            g_host_head = (g_host_head + 1) % kHostQueueCapacity;
            --g_host_count;
            if (g_host_drops != UINT32_MAX) ++g_host_drops;
        }
        const uint8_t index = (g_host_head + g_host_count) % kHostQueueCapacity;
        g_host_queue[index] = update;
        ++g_host_count;
        if (host) {
            g_have_host_timestamp = true;
            g_last_host_us = update.received_us;
            if (g_host_updates != UINT32_MAX) ++g_host_updates;
        }
    }
    critical_section_exit(&g_lock);
    return accepted;
}

void update_max(uint32_t* value, uint32_t candidate) {
    if (candidate > *value) {
        *value = candidate;
    }
}

uint64_t packet_due(uint32_t packet) {
    // Round each absolute rational deadline up, never its relative interval.
    return g_start_us +
           (static_cast<uint64_t>(packet) * packet_numerator_us() +
            kPacketDenominator - 1) / kPacketDenominator;
}

bool connection_current() {
    if (g_phase == Phase::kIdle || g_diagnostics.slot >= 4) {
        return false;
    }
    const Attachment& attached = g_attachments[g_diagnostics.slot];
    return attached.device == g_connection.device &&
           attached.generation == g_connection.generation &&
           attached.cid == g_connection.cid;
}

void publish(bool finished = false) {
    if (g_phase != Phase::kIdle) {
        g_diagnostics.elapsed_us =
            static_cast<uint32_t>(time_us_64() - g_start_us);
    }
    critical_section_enter_blocking(&g_lock);
    // A newly accepted start must not be overwritten by the preceding run.
    if (g_snapshot.run_id == g_diagnostics.run_id) {
        g_diagnostics.host_updates = g_host_updates;
        const uint64_t dropped = uint64_t{g_host_drops} +
                                 (gameplay() && g_synth_started ? g_synth.dropped_updates() : 0);
        g_diagnostics.dropped_updates =
            dropped > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(dropped);
        g_snapshot = g_diagnostics;
        g_snapshot_waiting = g_send_requested;
        g_snapshot_request_us = static_cast<uint32_t>(g_request_us);
        if (finished) {
            g_busy = false;
            g_accept_host = false;
        }
    }
    critical_section_exit(&g_lock);
}

void cancel_timer(btstack_timer_source_t* timer, bool* armed) {
    if (*armed) {
        btstack_run_loop_remove_timer(timer);
        *armed = false;
    }
}

void schedule_timer(btstack_timer_source_t* timer, bool* armed,
                    uint64_t deadline_us) {
    cancel_timer(timer, armed);
    const uint64_t now_us = time_us_64();
    // Pico's relative timer is floor(now_us/1000) + timeout_ms + 1.
    // Aim for the deadline's millisecond (possibly early); the handlers check
    // microseconds again. Only that final fractional tick needs a zero-delay
    // rearm. There is no permanent millisecond polling timer.
    const uint64_t now_ms = now_us / 1000;
    const uint64_t due_ms = deadline_us / 1000;
    const uint32_t delay_ms = due_ms > now_ms + 1
                                  ? static_cast<uint32_t>(due_ms - now_ms - 1)
                                  : 0;
    btstack_run_loop_set_timer(timer, delay_ms);
    *armed = true;
    btstack_run_loop_add_timer(timer);
}

void schedule_lifecycle(uint64_t deadline_us) {
    g_lifecycle_due_us = deadline_us;
    schedule_timer(&g_lifecycle_timer, &g_lifecycle_armed, deadline_us);
}

void schedule_packet() {
    const uint64_t due_us = packet_due(g_next_packet);
    native_output_scheduler_reserve(g_connection.device,
                                     g_connection.generation, due_us);
    schedule_timer(&g_cadence_timer, &g_cadence_armed, due_us);
}

void reserve_after_packet() {
    const uint64_t due_us =
        g_phase == Phase::kPattern && (gameplay() || g_next_packet + 1 < kPackets)
            ? packet_due(g_next_packet + 1) : UINT64_MAX;
    native_output_scheduler_reserve(g_connection.device,
                                     g_connection.generation, due_us);
}

void finish(HapticsExperimentState state, uint8_t error) {
    haptics_transport_probe_end();
    native_output_scheduler_cancel(g_connection.device);
    cancel_timer(&g_cadence_timer, &g_cadence_armed);
    cancel_timer(&g_lifecycle_timer, &g_lifecycle_armed);
    g_send_requested = false;
    g_diagnostics.elapsed_us =
        static_cast<uint32_t>(time_us_64() - g_start_us);
    g_diagnostics.state = state;
    if (error != kNoError) {
        g_diagnostics.last_error = error;
    }
    g_phase = Phase::kIdle;
    g_connection = {};
    publish(true);
}

void account_wait(uint64_t now_us) {
    update_max(&g_diagnostics.max_request_wait_us,
               static_cast<uint32_t>(now_us - g_request_us));
}

void timeout_drain() {
    if (g_send_requested) {
        account_wait(time_us_64());
    }
    ++g_diagnostics.send_failures;
    g_diagnostics.last_error = kTimeout;
    // Cancel native admission; an outstanding stack notification may still
    // arrive later and belongs to the ordinary compatibility FIFO.
    g_send_requested = false;
    restore_compatibility(HapticsExperimentState::kError);
}

void begin_drain(HapticsExperimentState state, uint64_t deadline_us) {
    critical_section_enter_blocking(&g_lock);
    g_accept_host = false;
    critical_section_exit(&g_lock);
    cancel_timer(&g_cadence_timer, &g_cadence_armed);
    g_phase = Phase::kDrain;
    g_finish_state = state;
    schedule_lifecycle(deadline_us);
    if (time_us_64() >= deadline_us) {
        timeout_drain();
    } else {
        // Promote a pending tone in place: its eventual grant must send the
        // urgent stop, without requesting a second stack notification.
        request_send(true);
    }
}

void end_pattern() {
    g_diagnostics.skipped_packets += kPackets - g_next_packet;
    g_next_packet = kPackets;
    if (!g_send_requested && g_diagnostics.sent_packets != 0 &&
        g_last_was_silence) {
        restore_compatibility(HapticsExperimentState::kCompleted);
    } else {
        // Do not let a never-delivered CAN_SEND_NOW strand ownership forever.
        begin_drain(HapticsExperimentState::kCompleted,
                    g_end_us + kDrainTimeoutUs);
    }
}

void restore_compatibility(HapticsExperimentState state) {
    critical_section_enter_blocking(&g_lock);
    g_accept_host = false;
    critical_section_exit(&g_lock);
    cancel_timer(&g_cadence_timer, &g_cadence_armed);
    native_output_scheduler_cancel(g_connection.device);
    g_send_requested = false;
    g_phase = Phase::kRestore;
    g_finish_state = g_diagnostics.send_failures != 0
                         ? HapticsExperimentState::kError
                         : state;
    // duration=0 is a no-op when the parser already believes rumble is off.
    // Force the HAPTICS_SELECT / compatible-vibration report with zero motors.
    // Set the phase first: synchronous notifications here belong to that FIFO.
    g_connection.device->report_parser.play_dual_rumble(
        g_connection.device, 0, 1, 0, 0);
    // Immediately cancel the newly installed parser timer and emit its zero
    // stop. Upstream device deletion does not remove private parser timers.
    // No host effect can interleave between these two calls on the BT core.
    g_connection.device->report_parser.play_dual_rumble(
        g_connection.device, 0, 0, 0, 0);
    const uint64_t now_us = time_us_64();
    g_restore_deadline_us = now_us + kDrainTimeoutUs;
    schedule_lifecycle(now_us + kRestoreSettleUs);
    publish();
}

void request_send(bool promote) {
    if ((g_send_requested && !promote) || g_request_in_progress ||
        (g_phase != Phase::kPattern && g_phase != Phase::kDrain)) {
        return;
    }
    const uint64_t now_us = time_us_64();
    const bool stopping = g_phase == Phase::kDrain;
    if (!stopping) {
        if (now_us >= g_end_us) {
            end_pattern();
            return;
        }
        if (now_us < packet_due(g_next_packet)) {
            schedule_packet();
            return;
        }
        const uint32_t current = static_cast<uint32_t>(
            ((now_us - g_start_us) * kPacketDenominator) / packet_numerator_us());
        g_diagnostics.skipped_packets += current - g_next_packet;
        g_next_packet = current;
        reserve_after_packet();
    }
    if (!g_send_requested) {
        g_request_us = now_us;
    }
    g_send_requested = true;
    g_request_in_progress = true;
    ++g_diagnostics.can_send_requests;
    if (gameplay() && g_phase == Phase::kPattern) {
        schedule_lifecycle(g_request_us + kDrainTimeoutUs);
    }
    const uint8_t status = native_output_scheduler_request(
        g_connection.device, g_connection.generation,
        stopping ? now_us : packet_due(g_next_packet), stopping, on_output_grant);
    if (stopping && g_send_requested) {
        // Clear cadence only after promotion: reserve() can pump a pending
        // request synchronously, so the urgent bit must already be installed.
        reserve_after_packet();
    }
    g_request_in_progress = false;
    if (status == ERROR_CODE_SUCCESS && !g_send_requested) {
        // The synchronous callback already published its result and armed the
        // next deadline. Do not touch that timer or copy its snapshot again.
        return;
    }
    if (status != ERROR_CODE_SUCCESS && g_send_requested) {
        g_send_requested = false;
        ++g_diagnostics.send_failures;
        g_diagnostics.last_error = kTransport;
        // A failed request is not permission. Retry on a future deadline, not
        // through a recursive callback or a tight loop.
        if (g_phase == Phase::kPattern) {
            const uint64_t now_us = time_us_64();
            if (now_us >= g_end_us) {
                end_pattern();
            } else {
                const uint32_t current = static_cast<uint32_t>(
                    ((now_us - g_start_us) * kPacketDenominator) /
                    packet_numerator_us());
                g_diagnostics.skipped_packets += current + 1 - g_next_packet;
                g_next_packet = current + 1;
                if (gameplay() || g_next_packet < kPackets) {
                    schedule_packet();
                }
            }
        } else if (g_phase == Phase::kDrain) {
            schedule_timer(&g_cadence_timer, &g_cadence_armed,
                           time_us_64() + (packet_numerator_us() + 2) / 3);
        }
    }
    publish();
}

// Returns whether the block contains a scheduled tone (not actuator evidence).
bool HAPTICS_HOT(generate_packet)(uint8_t* report, uint32_t packet,
                                  bool silence) {
    for (uint16_t i = 0; i < kReportBytes; ++i) {
        report[i] = 0;
    }
    report[0] = 0xa2;
    report[1] = 0x32;
    uint16_t sample_offset;
    uint32_t frames;
    if (g_diagnostics.sent_packets == 0) {
        // Enable the controller's audio path before submitting PCM. A zero
        // SetState block does not write AudioControl. Keep volume, preamp,
        // microphone mute, triggers and lighting validity flags untouched.
        report[2] = 0x10;
        report[3] = 0x90;
        report[4] = 63;
        report[5] = 0x80; // AllowAudioControl; default route/MicSelect.
        sample_offset = 68;
        frames = 0; // State-only setup, not an audio sample interval.
    } else {
        // Preserve the physically qualified full-control, single-block format.
        frames = 32;
        report[3] = 0x91;
        report[4] = 7;
        report[5] = 0xfe;
        report[10] = 0xff;
        report[11] = static_cast<uint8_t>(g_diagnostics.sent_packets - 1);
        report[12] = 0x92;
        report[13] = 64;
        sample_offset = 14;
    }
    bool tone = false;
    if (gameplay() && !silence) {
        drain_host_updates();
        // One report of causal lookback retains fixed 16-frame native samples
        // and legacy 8 ms substeps, without a PCM backlog.
        if (packet != 0 && g_diagnostics.sent_packets != 0) {
            const uint64_t first_sample = uint64_t{packet - 1} * frames;
            g_synth.render(first_sample, frames, report + sample_offset);
            for (uint32_t frame = 0; frame < frames; ++frame) {
                if (report[sample_offset + frame * 2] != 0 ||
                    report[sample_offset + frame * 2 + 1] != 0) {
                    tone = true;
                    if (!g_first_tone_sent) {
                        g_diagnostics.first_tone_due_us = static_cast<uint32_t>(
                            g_start_us + ((first_sample + frame) * 1000 + 2) / 3);
                    }
                    break;
                }
            }
        }
    } else if (frames != 0 && !gameplay() && !silence && packet >= kPrimingPackets &&
               packet < kToneEndPacket) {
        const uint32_t relative = packet - kPrimingPackets;
        const uint32_t phase = (relative / kPhasePackets) % 4;
        if (phase == 0 || phase == 2) {
            tone = true;
            const uint32_t stride = phase == 0 ? 1 : 2;
            uint32_t wave = ((relative % kPhasePackets) * kPacketFrames * stride) % 30;
            const uint32_t channel = phase == 0 ? 0 : 1;
            for (uint32_t frame = 0; frame < frames; ++frame) {
                report[sample_offset + frame * 2 + channel] =
                    static_cast<uint8_t>(kSine[wave]);
                wave += stride;
                if (wave >= 30) {
                    wave -= 30;
                }
            }
        }
    }
    // Bluetooth CRC includes the A2 transaction byte and excludes only CRC.
    uint32_t crc = 0xffffffffu;
    for (uint16_t i = 0; i < kCrcOffset; ++i) {
        crc ^= report[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    crc = ~crc;
    for (uint8_t byte = 0; byte < 4; ++byte) {
        report[kCrcOffset + byte] = static_cast<uint8_t>(crc >> (byte * 8));
    }
    return tone;
}

void cadence_timer(btstack_timer_source_t*) {
    g_cadence_armed = false;
    if (!connection_current()) {
        if (g_phase != Phase::kIdle) {
            finish(HapticsExperimentState::kDisconnected, kConnection);
        }
        return;
    }
    const uint64_t now_us = time_us_64();
    if (g_phase == Phase::kPrepare) {
        if (uni_circular_buffer_is_empty(&g_connection.device->outgoing_buffer)) {
            start_stream();
        } else if (now_us >= g_lifecycle_due_us) {
            finish(HapticsExperimentState::kError, kQueuedOutput);
        } else {
            schedule_timer(&g_cadence_timer, &g_cadence_armed, now_us + 2000);
        }
        return;
    }
    if (g_phase == Phase::kPattern) {
        const uint64_t due_us = packet_due(g_next_packet);
        haptics_transport_probe_timer(
            now_us > due_us ? static_cast<uint32_t>(now_us - due_us) : 0);
    }
    if (g_phase == Phase::kPattern) {
        if (now_us >= g_end_us) {
            end_pattern();
        } else if (now_us < packet_due(g_next_packet)) {
            schedule_packet();
        } else {
            request_send();
        }
    } else if (g_phase == Phase::kDrain) {
        if (now_us >= g_lifecycle_due_us) {
            timeout_drain();
        } else {
            request_send();
        }
    }
}

void lifecycle_timer(btstack_timer_source_t*) {
    g_lifecycle_armed = false;
    if (!connection_current()) {
        if (g_phase != Phase::kIdle) {
            finish(HapticsExperimentState::kDisconnected, kConnection);
        }
        return;
    }
    if (time_us_64() < g_lifecycle_due_us) {
        schedule_lifecycle(g_lifecycle_due_us);
        return;
    }
    if (g_phase == Phase::kPrepare) {
        finish(HapticsExperimentState::kError, kQueuedOutput);
    } else if (g_phase == Phase::kPattern) {
        if (gameplay()) {
            timeout_drain();
        } else {
            end_pattern();
        }
    } else if (g_phase == Phase::kDrain) {
        timeout_drain();
    } else if (g_phase == Phase::kRestore) {
        if (uni_circular_buffer_is_empty(&g_connection.device->outgoing_buffer)) {
            finish(g_finish_state, kNoError);
        } else if (time_us_64() < g_restore_deadline_us) {
            schedule_lifecycle(g_restore_deadline_us);
        } else {
            ++g_diagnostics.send_failures;
            finish(HapticsExperimentState::kError, kTimeout);
        }
    }
}

void start_stream() {
    cancel_timer(&g_cadence_timer, &g_cadence_armed);
    g_start_us = time_us_64();
    g_end_us = gameplay() ? UINT64_MAX : g_start_us + 6144000;
    g_diagnostics.start_us = static_cast<uint32_t>(g_start_us);
    g_diagnostics.first_tone_due_us =
        gameplay() ? 0 : static_cast<uint32_t>(packet_due(kPrimingPackets));
    g_diagnostics.state = HapticsExperimentState::kRunning;
    g_phase = Phase::kPattern;
    g_next_packet = 0;
    g_send_requested = false;
    g_last_was_silence = true;
    g_first_tone_sent = false;
    if (gameplay()) {
        g_synth.reset(g_start_us);
        g_synth_started = true;
    }
    schedule_lifecycle(gameplay() ? g_start_us + kDrainTimeoutUs : g_end_us);
    request_send();
}

void start(const Command& command) {
    g_diagnostics = {};
    g_diagnostics.run_id = command.run_id;
    g_diagnostics.slot = command.slot;
    g_diagnostics.mode = command.action == 2 ? 1 : 0;
    g_synth_started = false;
    g_connection = command.connection;
    g_diagnostics.connection_generation = g_connection.generation;
    g_start_us = time_us_64();
    g_diagnostics.start_us = static_cast<uint32_t>(g_start_us);
    uni_hid_device_t* device = g_connection.device;
    const Attachment& attached = g_attachments[command.slot];
    const bool current = device != nullptr && attached.device == device &&
                         attached.generation == g_connection.generation &&
                         attached.cid == g_connection.cid;
    haptics_transport_probe_begin(
        command.run_id, g_connection.generation,
        current ? device->conn.handle : 0xffff);
    if (!current) {
        finish(HapticsExperimentState::kDisconnected, kConnection);
        return;
    }
    if (device->vendor_id != 0x054c ||
        (device->product_id != 0x0ce6 && device->product_id != 0x0df2) ||
        gap_get_connection_type(device->conn.handle) != GAP_CONNECTION_ACL ||
        device->report_parser.play_dual_rumble == nullptr) {
        finish(HapticsExperimentState::kUnsupported, kUnsupported);
        return;
    }
    if (g_connection.cid == 0 || !device->conn.connected ||
        device->conn.interrupt_cid != g_connection.cid) {
        finish(HapticsExperimentState::kDisconnected, kConnection);
        return;
    }
    if (l2cap_get_remote_mtu_for_local_cid(g_connection.cid) < kReportBytes) {
        finish(HapticsExperimentState::kUnsupported, kMtu);
        return;
    }
    // Never discard unrelated LED/control reports or allow them to switch the
    // controller back to compatibility midstream. A queued start is retryable
    // once the ordinary sender has drained it.
    if (!gameplay() && !uni_circular_buffer_is_empty(&device->outgoing_buffer)) {
        finish(HapticsExperimentState::kError, kQueuedOutput);
        return;
    }
    // Cancel any existing parser duration/delayed-start timer before taking
    // over. In the already-disabled case this deliberately emits no report.
    device->report_parser.play_dual_rumble(device, 0, 0, 0, 0);
    btstack_run_loop_set_timer_handler(&g_cadence_timer, cadence_timer);
    btstack_run_loop_set_timer_handler(&g_lifecycle_timer, lifecycle_timer);
    if (!uni_circular_buffer_is_empty(&device->outgoing_buffer)) {
        if (gameplay()) {
            // Let connection setup/LED reports drain before taking over.
            g_phase = Phase::kPrepare;
            g_diagnostics.state = HapticsExperimentState::kPending;
            schedule_lifecycle(time_us_64() + kDrainTimeoutUs);
            schedule_timer(&g_cadence_timer, &g_cadence_armed, time_us_64() + 2000);
            publish();
        } else {
            finish(HapticsExperimentState::kError, kQueuedOutput);
        }
        return;
    }
    start_stream();
}

}  // namespace

void haptics_experiment_prepare() {
    if (!g_prepared) {
        critical_section_init(&g_lock);
        haptics_transport_probe_prepare();
        g_prepared = true;
    }
}

bool haptics_experiment_request(uint8_t action, uint8_t slot) {
    if (action > 2 || slot >= 4) {
        return false;
    }
    critical_section_enter_blocking(&g_lock);
    bool accepted = true;
    if (action != 0) {
        if (g_busy) {
            accepted = false;
        } else {
            const uint32_t run_id = g_snapshot.run_id + 1;
            g_snapshot = {};
            g_snapshot.run_id = run_id;
            g_snapshot.slot = slot;
            g_snapshot.connection_generation = g_attachments[slot].generation;
            g_snapshot.state = HapticsExperimentState::kPending;
            g_snapshot.mode = action == 2 ? 1 : 0;
            g_accept_host = action == 2;
            g_host_head = 0;
            g_host_count = 0;
            g_native_cancel = 0;
            g_have_host_timestamp = false;
            g_host_updates = 0;
            g_host_drops = 0;
            g_snapshot_waiting = false;
            g_busy = true;
            g_command = {true, action, slot, run_id, g_attachments[slot], g_snapshot.mode};
        }
    } else if (g_busy) {
        if (slot != g_snapshot.slot) {
            accepted = false;
        } else {
            g_accept_host = false;
            // Replaces even an unconsumed start, without a FIFO of commands.
            g_command = {
                true, action, slot, g_snapshot.run_id,
                {nullptr, g_snapshot.connection_generation, 0}, g_snapshot.mode};
        }
    }
    critical_section_exit(&g_lock);
    return accepted;
}

bool haptics_experiment_submit(uint8_t slot, uint32_t generation,
                               uint64_t received_us,
                               const SwitchHapticsFrame& frame) {
    if (frame.actuators[0].sample_count == 0 && frame.actuators[1].sample_count == 0) {
        return false;
    }
    for (const auto& actuator : frame.actuators) {
        if (actuator.sample_count > 3) return false;
        for (unsigned index = 0; index < actuator.sample_count; ++index) {
            const auto& sample = actuator.samples[index];
            if (sample.low_frequency_index > 127 || sample.high_frequency_index > 127 ||
                sample.low_amplitude_q15 > 32768 || sample.high_amplitude_q15 > 32768) {
                return false;
            }
        }
    }
    HostUpdate update;
    update.received_us = received_us;
    update.frame = frame;
    return submit_host_update(slot, generation, update);
}

bool haptics_experiment_submit_native(uint8_t slot, uint32_t generation,
                                      uint64_t received_us,
                                      const NativeHapticsFrame& frame) {
    if (!SwitchHdRumbleSynth::valid_native(frame) ||
        (!frame.actuators[0].sample_count && !frame.actuators[1].sample_count)) {
        return false;
    }
    HostUpdate update;
    update.received_us = received_us;
    update.kind = HostKind::kNative;
    update.native_frame = frame;
    return submit_host_update(slot, generation, update);
}

bool haptics_experiment_native_selected(uint8_t slot, uint32_t generation) {
    if (!g_prepared || slot >= 4) return false;
    critical_section_enter_blocking(&g_lock);
    const bool selected = selected_locked(slot, generation);
    critical_section_exit(&g_lock);
    return selected;
}

void haptics_experiment_cancel_native(uint8_t slot, uint32_t generation,
                                      uint8_t side_mask) {
    side_mask &= 3;
    if (!g_prepared || slot >= 4 || !side_mask) return;
    critical_section_enter_blocking(&g_lock);
    if (selected_locked(slot, generation)) {
        g_native_cancel |= side_mask;
        uint8_t kept = 0;
        for (uint8_t index = 0; index < g_host_count; ++index) {
            HostUpdate& update = g_host_queue[(g_host_head + index) % kHostQueueCapacity];
            if (update.kind == HostKind::kNative) {
                for (unsigned side = 0; side < 2; ++side) {
                    if (side_mask & (1u << side)) update.native_frame.actuators[side].sample_count = 0;
                }
                if (!update.native_frame.actuators[0].sample_count &&
                    !update.native_frame.actuators[1].sample_count) continue;
            }
            if (kept != index) g_host_queue[(g_host_head + kept) % kHostQueueCapacity] = update;
            ++kept;
        }
        g_host_count = kept;
    }
    critical_section_exit(&g_lock);
}

bool haptics_experiment_submit_rumble(uint8_t slot, uint32_t generation,
                                      uint64_t received_us,
                                      uint8_t low, uint8_t high) {
    HostUpdate update;
    update.received_us = received_us;
    update.kind = HostKind::kRumble;
    update.low = low;
    update.high = high;
    return submit_host_update(slot, generation, update);
}

void haptics_experiment_snapshot(HapticsExperimentDiagnostics* output) {
    if (output == nullptr) {
        return;
    }
    critical_section_enter_blocking(&g_lock);
    *output = g_snapshot;
    const bool waiting = g_snapshot_waiting;
    const uint32_t requested_us = g_snapshot_request_us;
    critical_section_exit(&g_lock);
    output->packet_frames = kPacketFrames;
    if (output->state == HapticsExperimentState::kRunning) {
        const uint32_t now_us = static_cast<uint32_t>(time_us_64());
        output->elapsed_us = now_us - output->start_us;
        if (waiting) {
            update_max(&output->max_request_wait_us, now_us - requested_us);
        }
    }
}

void haptics_experiment_attach(uint8_t slot, uint32_t generation,
                               uni_hid_device_t* device) {
    if (slot >= 4 || device == nullptr) {
        return;
    }
    const uint16_t cid = device->conn.interrupt_cid;
    const bool migrate_live = g_phase != Phase::kIdle && gameplay() &&
                              g_diagnostics.slot == slot &&
                              g_connection.device == device && g_connection.cid == cid &&
                              g_connection.generation != generation;
    if (migrate_live) {
        // A source epoch is not a Bluetooth reconnect. Cancel only admission
        // tied to the retired generation; keep oscillator/other-side state.
        native_output_scheduler_cancel(device);
        g_send_requested = false;
        g_connection.generation = generation;
        g_diagnostics.connection_generation = generation;
    } else if (g_phase != Phase::kIdle &&
               (g_diagnostics.slot == slot || g_connection.device == device) &&
               (g_diagnostics.slot != slot || g_connection.device != device ||
                g_connection.generation != generation || g_connection.cid != cid)) {
        finish(HapticsExperimentState::kDisconnected, kConnection);
    }
    // A reused instance must not remain selectable through an old slot.
    critical_section_enter_blocking(&g_lock);
    const bool migrate_pending = g_busy && g_snapshot.mode == 1 &&
                                 g_snapshot.slot == slot &&
                                 g_attachments[slot].device == device &&
                                 g_attachments[slot].cid == cid;
    if (migrate_pending || migrate_live) {
        g_snapshot.connection_generation = generation;
        if (g_command.pending && g_command.slot == slot) {
            g_command.connection.generation = generation;
        }
    }
    for (Attachment& attached : g_attachments) {
        if (attached.device == device) {
            attached = {};
        }
    }
    g_attachments[slot] = {device, generation, cid};
    critical_section_exit(&g_lock);
    if (migrate_live && (g_phase == Phase::kPattern || g_phase == Phase::kDrain)) {
        request_send();
    }
}

void haptics_experiment_detach(uni_hid_device_t* device) {
    if (device == nullptr) {
        return;
    }
    native_output_scheduler_cancel(device);
    if (g_phase != Phase::kIdle && g_connection.device == device) {
        // Do not dereference the device or send restoration on a dead link.
        finish(HapticsExperimentState::kDisconnected, kConnection);
    }
    critical_section_enter_blocking(&g_lock);
    for (Attachment& attached : g_attachments) {
        if (attached.device == device) {
            attached = {};
        }
    }
    critical_section_exit(&g_lock);
}

void haptics_experiment_poll() {
    critical_section_enter_blocking(&g_lock);
    const Command command = g_command;
    g_command.pending = false;
    critical_section_exit(&g_lock);
    if (!command.pending) {
        return;
    }
    if (command.action != 0) {
        start(command);
    } else if (g_phase != Phase::kIdle &&
               g_diagnostics.run_id == command.run_id) {
        if (g_phase == Phase::kPrepare) {
            finish(HapticsExperimentState::kStopped, kNoError);
        } else if (g_phase == Phase::kRestore) {
            if (g_finish_state != HapticsExperimentState::kError) {
                g_finish_state = HapticsExperimentState::kStopped;
            }
        } else if (g_phase != Phase::kDrain ||
                   g_finish_state != HapticsExperimentState::kStopped) {
            begin_drain(HapticsExperimentState::kStopped,
                        time_us_64() + kDrainTimeoutUs);
        }
    } else {
        // Stop preempted a start still in the mailbox, or raced completion.
        if (g_diagnostics.run_id != command.run_id) {
            g_diagnostics = {};
            g_diagnostics.run_id = command.run_id;
            g_diagnostics.slot = command.slot;
            g_diagnostics.connection_generation = command.connection.generation;
            g_diagnostics.state = HapticsExperimentState::kStopped;
            g_diagnostics.mode = command.mode;
            haptics_transport_probe_begin(
                command.run_id, command.connection.generation, 0xffff);
            haptics_transport_probe_end();
        }
        publish(true);
    }
}

bool haptics_experiment_owns(const uni_hid_device_t* device) {
    return device != nullptr && device == g_connection.device &&
           connection_current();
}

bool haptics_experiment_gameplay_owns(const uni_hid_device_t* device) {
    return gameplay() && g_phase == Phase::kPattern &&
           haptics_experiment_owns(device);
}

bool haptics_experiment_feedback(uni_hid_device_t* device,
                                 uint8_t low, uint8_t high, uint16_t duration_ms) {
    if (device == nullptr) return false;
    critical_section_enter_blocking(&g_lock);
    const uint8_t slot = g_snapshot.slot;
    const uint32_t generation = g_snapshot.connection_generation;
    const bool selected = slot < 4 && selected_locked(slot, generation) &&
                          g_attachments[slot].device == device;
    critical_section_exit(&g_lock);
    if (!selected) return false;
    HostUpdate update;
    update.received_us = time_us_64();
    update.kind = HostKind::kFeedback;
    update.duration_us = uint32_t{duration_ms} * 1000;
    update.low = low;
    update.high = high;
    // Pending gameplay cues retain their original lifetime. Fixture, drain,
    // and restoration are exclusive: consume the cue without compatibility.
    submit_host_update(slot, generation, update);
    return true;
}

bool haptics_experiment_native_feedback(uni_hid_device_t* device, uint64_t received_us,
                                        uint8_t left, uint8_t right, uint16_t duration_ms) {
    if (!haptics_experiment_gameplay_owns(device)) return false;
    drain_host_updates();
    g_synth.feedback_native(received_us, uint32_t{duration_ms} * 1000, left, right);
    return true;
}

bool haptics_experiment_blocks_generic(const uni_hid_device_t* device) {
    return haptics_experiment_owns(device) &&
           (g_phase == Phase::kDrain ||
            (g_phase == Phase::kPattern && !gameplay()));
}

namespace {

bool HAPTICS_HOT(on_output_grant)(uni_hid_device_t* device, uint16_t cid, uint32_t generation) {
    if (!native_output_scheduler_granted(device)) {
        return haptics_experiment_blocks_generic(device);
    }
    if (g_in_callback) {
        return true;
    }
    if (device != g_connection.device || generation != g_connection.generation || !connection_current() ||
        (g_phase != Phase::kPattern && g_phase != Phase::kDrain)) {
        native_output_scheduler_complete(device, generation);
        return false;
    }
    if (gameplay() && g_phase == Phase::kPattern && !g_in_callback &&
        !uni_circular_buffer_is_empty(&device->outgoing_buffer)) {
        // Gameplay permits ordinary LED reports, never compatibility rumble.
        // Yield this credit to the generic FIFO, then request fresh permission.
        if (cid == g_connection.cid && g_send_requested) {
            account_wait(time_us_64());
            g_send_requested = false;
            schedule_timer(&g_cadence_timer, &g_cadence_armed, time_us_64() + 1000);
        }
        native_output_scheduler_complete(device, generation);
        return false;
    }
    // Only the scheduler's current interrupt-CID grant is PCM permission.
    if (cid != g_connection.cid || !g_send_requested) {
        native_output_scheduler_complete(device, generation);
        return haptics_experiment_blocks_generic(device);
    }
    g_send_requested = false;
    g_in_callback = true;
    if (g_request_in_progress) {
        ++g_diagnostics.synchronous_callbacks;
    }
    const uint64_t now_us = time_us_64();
    account_wait(now_us);
    haptics_transport_probe_permission(
        static_cast<uint32_t>(now_us - g_request_us));
    if (g_phase == Phase::kPattern && now_us >= g_end_us) {
        g_diagnostics.skipped_packets += kPackets - g_next_packet;
        g_next_packet = kPackets;
        g_phase = Phase::kDrain;
        g_finish_state = HapticsExperimentState::kCompleted;
        schedule_lifecycle(g_end_us + kDrainTimeoutUs);
    }
    if (g_phase == Phase::kDrain && now_us >= g_lifecycle_due_us) {
        native_output_scheduler_complete(device, generation);
        timeout_drain();
        g_in_callback = false;
        return true;
    }
    if (g_phase == Phase::kPattern && now_us < packet_due(g_next_packet)) {
        // Notifications are not reservations of credit for a future deadline.
        schedule_packet();
        g_in_callback = false;
        native_output_scheduler_complete(device, generation);
        publish();
        return true;
    }
    const bool stopping = g_phase == Phase::kDrain;
    uint64_t due_us = now_us;
    if (!stopping) {
        const uint32_t current = static_cast<uint32_t>(
            ((now_us - g_start_us) * kPacketDenominator) / packet_numerator_us());
        g_diagnostics.skipped_packets += current - g_next_packet;
        g_next_packet = current;
        due_us = packet_due(current);
    }
    reserve_after_packet();
    uint8_t report[kReportBytes];
    const uint64_t generate_start_us = time_us_64();
    const bool tone = generate_packet(report, g_next_packet, stopping);
    const uint64_t submit_us = time_us_64();
    update_max(&g_diagnostics.max_generate_us,
               static_cast<uint32_t>(submit_us - generate_start_us));
    ++g_diagnostics.generated_packets;
    update_max(&g_diagnostics.max_lateness_us,
               static_cast<uint32_t>(submit_us - due_us));
    if (!stopping && submit_us >= packet_due(g_next_packet + 1)) {
        // A flash/interrupt stall can occur during synthesis as well as before
        // CAN_SEND_NOW. Never submit a now-obsolete tone after its phase ended.
        uint32_t next = static_cast<uint32_t>(
            ((submit_us - g_start_us) * kPacketDenominator) /
            packet_numerator_us()) + 1;
        if (!gameplay() && next > kPackets) {
            next = kPackets;
        }
        g_diagnostics.skipped_packets += next - g_next_packet;
        g_next_packet = next;
        if (gameplay() || g_next_packet < kPackets) {
            schedule_packet();
        } else {
            native_output_scheduler_reserve(device, g_connection.generation,
                                             UINT64_MAX);
        }
        g_in_callback = false;
        native_output_scheduler_complete(device, generation);
        publish();
        return true;
    }
    if (stopping && submit_us >= g_lifecycle_due_us) {
        native_output_scheduler_complete(device, generation);
        timeout_drain();
        g_in_callback = false;
        return true;
    }
    const uint64_t send_started_us = time_us_64();
    const uint8_t status = l2cap_send(cid, report, sizeof(report));
    const uint64_t send_returned_us = time_us_64();
    native_output_scheduler_complete(device, generation);
    haptics_transport_probe_send(
        static_cast<uint32_t>(send_returned_us - send_started_us),
        static_cast<uint32_t>(send_returned_us),
        tone && !g_first_tone_sent && status == ERROR_CODE_SUCCESS);
    if (!connection_current()) {
        // A transport may synchronously report teardown. Detach already
        // published the terminal state; do not rearm a timer on its old CID.
        g_in_callback = false;
        return true;
    }
    if (status == ERROR_CODE_SUCCESS) {
        if (g_diagnostics.sent_packets != 0) {
            update_max(&g_diagnostics.max_send_gap_us,
                       static_cast<uint32_t>(submit_us) - g_diagnostics.last_sent_us);
        }
        if (gameplay() && !stopping && g_diagnostics.sent_packets != 0 && g_next_packet != 0) {
            // Packet n renders [(n-1)*32, n*32). A causal-lookback packet sent
            // after cue admission may still precede the cue on this timeline.
            g_diagnostics.last_pcm_end_us = static_cast<uint32_t>(packet_due(g_next_packet));
        }
        ++g_diagnostics.sent_packets;
        g_diagnostics.last_sent_us = static_cast<uint32_t>(submit_us);
        g_last_was_silence = !tone;
        g_diagnostics.last_packet_nonzero = tone;
        if (tone && !g_first_tone_sent) {
            g_first_tone_sent = true;
            g_diagnostics.first_tone_sent_us = static_cast<uint32_t>(submit_us);
        }
    } else {
        ++g_diagnostics.send_failures;
        g_diagnostics.last_error = kTransport;
    }
    if (stopping) {
        if (status == ERROR_CODE_SUCCESS) {
            restore_compatibility(g_finish_state);
        } else {
            schedule_timer(&g_cadence_timer, &g_cadence_armed,
                           time_us_64() + (packet_numerator_us() + 2) / 3);
        }
    } else {
        ++g_next_packet;
        if (gameplay() || g_next_packet < kPackets) {
            schedule_packet();
        }
        if (gameplay()) {
            schedule_lifecycle(packet_due(g_next_packet) + kDrainTimeoutUs);
        }
    }
    g_in_callback = false;
    publish();
    return true;
}

}  // namespace
