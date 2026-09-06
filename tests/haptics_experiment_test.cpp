#include "input/haptics_experiment.h"
#include "input/haptics_transport_probe.h"
#include "input/native_output_scheduler.h"
#include "usb/switch/switch_haptics.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include <btstack.h>
#include <pico/critical_section.h>
#include <uni.h>

namespace {

enum class Delivery { kImmediate, kDeferred, kNever };
enum class GenericKind { kCompatibility, kLed };

struct Pcm {
    uint64_t at_us;
    uint16_t cid;
    std::array<uint8_t, 143> bytes;
};
struct Generic {
    uni_hid_device_t* device;
    uint64_t at_us;
    GenericKind kind;
    uint8_t weak;
    uint8_t strong;
};

uint64_t now_us = 10000123;
std::array<uni_hid_device_t, 4> devices;
std::vector<btstack_timer_source_t*> timers;
std::vector<Pcm> pcm;
std::vector<Generic> generic_sent;
std::vector<Generic> generic_queue;
Delivery delivery = Delivery::kImmediate;
unsigned request_depth = 0;
unsigned max_request_depth = 0;
unsigned request_calls = 0;
unsigned send_calls = 0;
unsigned timer_calls = 0;
unsigned fail_requests = 0;
unsigned fail_sends = 0;
uint32_t send_cost_us = 0;
uint32_t request_cost_us = 0;
bool reenter_send = false;
bool detach_during_send = false;

void no_lock() {
    assert(native_haptics_lock_depth == 0);
}

HapticsExperimentDiagnostics snapshot() {
    HapticsExperimentDiagnostics out;
    haptics_experiment_snapshot(&out);
    return out;
}

uni_hid_device_t* device_for_cid(uint16_t cid) {
    for (auto& device : devices) {
        if (device.conn.interrupt_cid == cid) {
            return &device;
        }
    }
    assert(false && "stale or unknown L2CAP CID");
    return nullptr;
}

void emit_generic(uni_hid_device_t* device, GenericKind kind,
                  uint8_t weak = 0, uint8_t strong = 0) {
    no_lock();
    Generic report{device, now_us, kind, weak, strong};
    if (device->credit) {
        generic_sent.push_back(report);
    } else {
        generic_queue.push_back(report);
        ++device->outgoing_buffer.queued;
    }
}

void parser_off(btstack_timer_source_t* timer) {
    auto* device = static_cast<uni_hid_device_t*>(timer->context);
    assert(device->parser_rumble_active);
    device->parser_rumble_active = false;
    emit_generic(device, GenericKind::kCompatibility);
}

void parser_delayed_on(btstack_timer_source_t* timer) {
    auto* device = static_cast<uni_hid_device_t*>(timer->context);
    assert(device->parser_rumble_delayed);
    device->parser_rumble_delayed = false;
    emit_generic(device, GenericKind::kCompatibility, 17, 23);
}

// Behavioral fake of the relevant Bluepad32 DS5 parser contract: duration=0
// does not emit anything if already disabled; duration>0 forces compatibility
// and installs a timer-off. This catches a "restore" which is actually a no-op.
void play_rumble(uni_hid_device_t* device, uint16_t delay_ms,
                 uint16_t duration_ms, uint8_t weak, uint8_t strong) {
    no_lock();
    const bool was_active = device->parser_rumble_active ||
                            device->parser_rumble_delayed;
    if (was_active) {
        btstack_run_loop_remove_timer(&device->parser_timer);
    }
    device->parser_rumble_active = false;
    device->parser_rumble_delayed = false;
    device->parser_timer.context = device;
    if (delay_ms != 0) {
        device->parser_rumble_delayed = true;
        device->parser_timer.process = parser_delayed_on;
        btstack_run_loop_set_timer(&device->parser_timer, delay_ms);
        btstack_run_loop_add_timer(&device->parser_timer);
    } else if (duration_ms != 0) {
        emit_generic(device, GenericKind::kCompatibility, weak, strong);
        device->parser_rumble_active = true;
        device->parser_timer.process = parser_off;
        btstack_run_loop_set_timer(&device->parser_timer, duration_ms);
        btstack_run_loop_add_timer(&device->parser_timer);
    } else if (was_active) {
        emit_generic(device, GenericKind::kCompatibility);
    }
}

bool dispatch(uni_hid_device_t* device, uint16_t cid) {
    no_lock();
    if (cid == device->conn.interrupt_cid) {
        device->notification_pending = false;
        device->native_ready_cid = cid;
    }
    const bool exclusive = haptics_experiment_blocks_generic(device);
    const bool consumed =
        native_output_scheduler_on_can_send_now(device, cid) || exclusive;
    if (!consumed && device->credit) {
        const auto it = std::find_if(generic_queue.begin(), generic_queue.end(),
                                     [device](const Generic& report) {
                                         return report.device == device;
                                     });
        if (it != generic_queue.end()) {
            Generic report = *it;
            report.at_us = now_us;
            generic_sent.push_back(report);
            generic_queue.erase(it);
            --device->outgoing_buffer.queued;
            device->native_ready_cid = 0;
        }
    }
    return consumed;
}

void run_until(uint64_t target_us) {
    assert(target_us >= now_us);
    unsigned iterations = 0;
    while (!timers.empty()) {
        const auto it = std::min_element(
            timers.begin(), timers.end(),
            [](const auto* a, const auto* b) { return a->timeout_us < b->timeout_us; });
        btstack_timer_source_t* timer = *it;
        if (timer->timeout_us > target_us) {
            break;
        }
        now_us = std::max(now_us, timer->timeout_us);
        timers.erase(it);
        assert(++iterations < 3000 && "recursive or permanently polling timer");
        ++timer_calls;
        timer->process(timer);
    }
    now_us = std::max(now_us, target_us);
}

void reset(uint64_t at_us = 10000123) {
    for (auto& device : devices) {
        haptics_experiment_detach(&device);
    }
    if (snapshot().state == HapticsExperimentState::kPending) {
        assert(haptics_experiment_request(0, snapshot().slot));
        haptics_experiment_poll();
    }
    timers.clear();
    native_output_scheduler_prepare();
    devices = {};
    pcm.clear();
    generic_sent.clear();
    generic_queue.clear();
    now_us = at_us;
    delivery = Delivery::kImmediate;
    request_depth = max_request_depth = request_calls = send_calls = timer_calls = 0;
    fail_requests = fail_sends = 0;
    send_cost_us = request_cost_us = 0;
    reenter_send = false;
    detach_during_send = false;
    for (unsigned slot = 0; slot < devices.size(); ++slot) {
        auto& device = devices[slot];
        device.vendor_id = 0x054c;
        device.product_id = 0x0ce6;
        device.conn.handle = static_cast<uint16_t>(slot);
        // Bluepad32's Classic path does not initialize this cached field.
        device.conn.protocol = UNI_BT_CONN_PROTOCOL_NONE;
        device.conn.connected = true;
        device.conn.interrupt_cid = static_cast<uint16_t>(0x40 + slot * 2);
        device.conn.control_cid = device.conn.interrupt_cid + 1;
        device.report_parser.play_dual_rumble = play_rumble;
        haptics_experiment_attach(static_cast<uint8_t>(slot), 100 + slot, &device);
    }
    assert(pcm.empty() && generic_sent.empty());  // No pairing/boot tone.
}

uint64_t start(uint8_t slot = 0) {
    const uint32_t previous_id = snapshot().run_id;
    assert(haptics_experiment_request(1, slot));
    const auto pending = snapshot();
    assert(pending.run_id == previous_id + 1);
    assert(pending.slot == slot && pending.state == HapticsExperimentState::kPending);
    assert(!haptics_experiment_request(1, slot));
    assert(!haptics_experiment_request(1, static_cast<uint8_t>((slot + 1) % 4)));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kRunning);
    assert(haptics_experiment_owns(&devices[slot]));
    assert(!haptics_experiment_owns(&devices[(slot + 1) % 4]));
    return now_us;
}

uint64_t due(uint64_t started, uint32_t packet, uint32_t frames = 64) {
    return started + (static_cast<uint64_t>(packet) * frames * 1000 + 2) / 3;
}

void verify_block(const Pcm& packet, uint32_t index, bool forced_silence = false) {
    const auto& b = packet.bytes;
    assert(b[0] == 0xa2 && b[1] == 0x32 && b[2] == 0);
    unsigned sample_offset = 10;
    unsigned frames = 64;
    if (b[3] == 0x90) {
        assert(b[4] == 63);
        for (unsigned i = 5; i < 68; ++i) assert(b[i] == 0);
        assert(b[68] == 0x92 && b[69] == 64);
        sample_offset = 70;
        frames = 32;
    } else {
        assert(b[3] == 0x91 && b[4] == 3 && b[5] == 0x62);
        assert(b[6] == 16 && b[8] == 0xd2 && b[9] == 64);
    }
    for (unsigned i = sample_offset + frames * 2; i < 139; ++i) {
        assert(b[i] == 0);
    }
    const bool pattern_tone = index >= 48 && index < 240 &&
                              ((index - 48) / 12) % 2 == 0;
    const bool tone = pattern_tone && !forced_silence;
    const unsigned phase = tone ? ((index - 48) / 12) % 4 : 0;
    const unsigned side = phase == 0 ? 0 : 1;
    const double hz = phase == 0 ? 100.0 : 200.0;
    for (unsigned frame = 0; frame < frames; ++frame) {
        for (unsigned channel = 0; channel < 2; ++channel) {
            const int value = static_cast<int8_t>(b[sample_offset + frame * 2 + channel]);
            if (!tone || channel != side) {
                assert(value == 0);
            } else {
                const unsigned sample = ((index - 48) % 12) * 64 + frame;
                const int expected = static_cast<int>(std::lround(
                    32.0 * std::sin(2.0 * 3.14159265358979323846 * hz * sample / 3000.0)));
                assert(std::abs(value - expected) <= 1);
                assert(std::abs(value) <= 32);
            }
        }
    }
}

void nominal_run(const char* corpus_path) {
    reset();
    const uint64_t started = start();
    // This is the BTstack synchronous reentry reproduction: the very first
    // request sends one block before request_can_send_now() returns.
    assert(pcm.size() == 1 && snapshot().synchronous_callbacks == 1);
    assert(!devices[0].notification_pending);
    const uint64_t early_tick = due(started, 1) / 1000 * 1000;
    run_until(early_tick);
    assert(pcm.size() == 1);  // SDK early millisecond wake must not send early.
    run_until(started + 6148000);
    const auto done = snapshot();
    assert(done.state == HapticsExperimentState::kCompleted);
    assert(done.sent_packets == 288 && done.generated_packets == 288);
    assert(done.skipped_packets == 0 && done.send_failures == 0);
    assert(done.can_send_requests == 288 && done.synchronous_callbacks == 288);
    assert(max_request_depth == 1 && request_calls == 288);
    assert(timer_calls < 1250);  // No permanent 1 ms poll for this 6.144 s run.
    assert(pcm.size() == 288);
    assert(done.first_tone_due_us == static_cast<uint32_t>(started + 1024000));
    assert(done.first_tone_sent_us == static_cast<uint32_t>(pcm[48].at_us));
    assert(done.last_sent_us == static_cast<uint32_t>(pcm.back().at_us));
    assert(done.max_send_gap_us <= 22000 && done.max_lateness_us < 1000);
    assert(done.elapsed_us >= 6147000 && done.elapsed_us < 6148000);
    assert(done.max_generate_us == 0);
    assert(!haptics_experiment_owns(&devices[0]) && timers.empty());
    assert(generic_sent.size() == 2);  // Forced compatibility, then parser off.
    assert(generic_sent.front().at_us >= started + 6144000);
    assert(!devices[0].parser_rumble_active);
    for (const auto& report : generic_sent) {
        assert(report.kind == GenericKind::kCompatibility);
        assert(report.weak == 0 && report.strong == 0);
    }
    std::ofstream corpus(corpus_path, std::ios::binary);
    assert(corpus.is_open());
    for (uint32_t i = 0; i < pcm.size(); ++i) {
        assert(pcm[i].at_us >= due(started, i));
        assert(pcm[i].at_us - due(started, i) < 1000);
        if (i != 0) assert(pcm[i].bytes[7] == static_cast<uint8_t>(i * 2));
        verify_block(pcm[i], i);
        corpus.write(reinterpret_cast<const char*>(pcm[i].bytes.data()), 143);
    }
    // Reference-sized 0x10 native-mode state followed by a silent PCM block.
    // Independent known answer computed with Python zlib over the A2 prefix.
    assert(pcm[0].bytes[3] == 0x90);
    assert(pcm[0].bytes[139] == 0x00 && pcm[0].bytes[140] == 0x41 &&
           pcm[0].bytes[141] == 0xfd && pcm[0].bytes[142] == 0x53);
    corpus.close();
    run_until(now_us + 200000);
    assert(snapshot().elapsed_us == done.elapsed_us && pcm.size() == 288);
}

void stalled_deadlines() {
    reset();
    const uint64_t started = start();
    run_until(due(started, 102) + 1000);
    const unsigned before = static_cast<unsigned>(pcm.size());
    const uint32_t next = snapshot().sent_packets;
    now_us += 250000;  // The main loop did not run at all during this stall.
    const uint32_t current = static_cast<uint32_t>((now_us - started) * 3 / 64000);
    run_until(now_us);
    assert(pcm.size() == before + 1);  // No catch-up replay burst.
    assert(snapshot().skipped_packets == current - next);
    verify_block(pcm.back(), current);
    assert(snapshot().max_send_gap_us >= 250000);
    run_until(started + 6148000);
    const auto done = snapshot();
    assert(done.state == HapticsExperimentState::kCompleted);
    assert(done.sent_packets + done.skipped_packets == 288);
    assert(done.elapsed_us < 6148000);
}

void deferred_and_missing_callbacks() {
    reset();
    delivery = Delivery::kDeferred;
    const uint64_t started = start();
    assert(pcm.empty() && devices[0].notification_pending);
    run_until(started + 2300000);
    assert(request_calls == 1 && timer_calls == 0);
    assert(!dispatch(&devices[1], devices[1].conn.interrupt_cid));
    assert(dispatch(&devices[0], devices[0].conn.control_cid));
    assert(pcm.empty() && devices[0].notification_pending);
    assert(dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.size() == 1);
    assert(snapshot().max_request_wait_us == 2300000);
    assert(snapshot().synchronous_callbacks == 0);
    assert(snapshot().skipped_packets == 107);
    verify_block(pcm.back(), 107);
    delivery = Delivery::kImmediate;
    run_until(started + 6148000);
    assert(snapshot().state == HapticsExperimentState::kCompleted);
    assert(snapshot().sent_packets + snapshot().skipped_packets == 288);

    reset();
    delivery = Delivery::kNever;
    const uint64_t missing_start = start();
    run_until(missing_start + 6248000);
    const auto missing = snapshot();
    assert(missing.state == HapticsExperimentState::kError);
    assert(missing.last_error == 4 && missing.send_failures == 1);
    assert(missing.sent_packets == 0 && missing.skipped_packets == 288);
    assert(missing.max_request_wait_us >= 6244000);
    assert(request_calls == 1 && timer_calls < 10 && timers.empty());
    assert(!haptics_experiment_owns(&devices[0]));
    assert(!dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.empty());
}

void stop_preemption_and_restore() {
    reset();
    const uint64_t started = start();
    run_until(due(started, 98) + 1000);
    delivery = Delivery::kDeferred;
    run_until(due(started, 99) + 1000);
    assert(devices[0].notification_pending);
    const auto before = snapshot();
    assert(!haptics_experiment_request(0, 1));
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    run_until(now_us + 10000);
    const unsigned sent_before = static_cast<unsigned>(pcm.size());
    assert(dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.size() == sent_before + 1);
    verify_block(pcm.back(), 99, true);  // Pending tone permission now sends silence.
    assert(!haptics_experiment_request(1, 0));  // Compatibility is still settling.
    run_until(now_us + 4000);
    assert(snapshot().state == HapticsExperimentState::kStopped);
    assert(snapshot().run_id == before.run_id);
    assert(!devices[0].parser_rumble_active && timers.empty());
    assert(generic_sent.size() == 2 && generic_sent[0].at_us >= pcm.back().at_us);
    const auto stopped_elapsed = snapshot().elapsed_us;
    run_until(now_us + 7000000);
    assert(pcm.size() == sent_before + 1 && snapshot().elapsed_us == stopped_elapsed);

    reset();
    const uint32_t previous = snapshot().run_id;
    assert(haptics_experiment_request(1, 0));
    const uint32_t pending_generation = snapshot().connection_generation;
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kStopped);
    assert(snapshot().run_id == previous + 1 && pcm.empty() && timers.empty());
    assert(snapshot().connection_generation == pending_generation);

    // Disconnect in the ownership-settling window, immediately after restore.
    // Bluepad32 deletes the instance without removing private parser timers.
    reset();
    start();
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    assert(haptics_experiment_owns(&devices[0]));
    assert(!devices[0].parser_rumble_active);
    haptics_experiment_detach(&devices[0]);
    const auto compatibility_count = generic_sent.size();
    devices[0] = {};  // Simulate upstream zeroing/reusing the parser instance.
    run_until(now_us + 10000);
    assert(snapshot().state == HapticsExperimentState::kDisconnected);
    assert(timers.empty() && generic_sent.size() == compatibility_count);

    reset();
    delivery = Delivery::kNever;
    devices[0].credit = false;
    start();
    const uint64_t stopped_at = now_us;
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    // Repeated stops must not perpetually extend the lifecycle watchdog.
    run_until(stopped_at + 50000);
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    run_until(stopped_at + 202000);
    assert(snapshot().state == HapticsExperimentState::kError);
    assert(snapshot().last_error == 4 && snapshot().send_failures >= 1);
    assert(snapshot().sent_packets == 0 && !haptics_experiment_owns(&devices[0]));
    assert(timers.empty() && generic_sent.empty() && generic_queue.size() == 2);
    devices[0].credit = true;
    assert(!dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(!dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(generic_queue.empty() && generic_sent.size() == 2 && pcm.empty());
}

void compatibility_queue_and_parser_timers() {
    reset();
    devices[0].credit = false;
    emit_generic(&devices[0], GenericKind::kLed);
    assert(haptics_experiment_request(1, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kError && snapshot().last_error == 6);
    assert(pcm.empty() && generic_queue.size() == 1 && !haptics_experiment_owns(&devices[0]));
    devices[0].credit = true;
    assert(!dispatch(&devices[0], devices[0].conn.control_cid));
    assert(generic_sent.size() == 1 && generic_sent[0].kind == GenericKind::kLed);
    start();  // Retry only after the unrelated report was delivered, not discarded.
    assert(pcm.size() == 1);
    devices[0].credit = false;
    emit_generic(&devices[0], GenericKind::kLed);
    devices[0].credit = true;
    const auto sent_count = pcm.size();
    assert(dispatch(&devices[0], devices[0].conn.control_cid));
    assert(dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.size() == sent_count && generic_queue.size() == 1);

    reset();
    play_rumble(&devices[0], 3000, 100, 17, 23);
    const uint64_t started = start();
    run_until(started + 6148000);
    assert(snapshot().state == HapticsExperimentState::kCompleted);
    for (const auto& report : generic_sent) {
        assert(report.weak == 0 && report.strong == 0);
        assert(report.at_us <= started || report.at_us >= started + 6144000);
    }
    assert(!devices[0].parser_rumble_active && !devices[0].parser_rumble_delayed);

    // Canceling an already running parser can itself queue a stop report.
    // Reject before the first native packet if that compatibility report blocks.
    reset();
    play_rumble(&devices[0], 0, 3000, 17, 23);
    devices[0].credit = false;
    assert(haptics_experiment_request(1, 0));
    haptics_experiment_poll();
    assert(snapshot().last_error == 6 && pcm.empty());
    assert(!devices[0].parser_rumble_active && timers.empty());
}

void reconnect_and_pending_generation() {
    reset();
    delivery = Delivery::kDeferred;
    start();
    const uint16_t old_cid = devices[0].conn.interrupt_cid;
    haptics_experiment_detach(&devices[0]);
    const auto detached = snapshot();
    assert(detached.state == HapticsExperimentState::kDisconnected);
    assert(detached.last_error == 3 && timers.empty() && generic_sent.empty());
    assert(!dispatch(&devices[0], old_cid));
    assert(pcm.empty());
    devices[0].conn.interrupt_cid = 0x70;
    haptics_experiment_attach(0, 101, &devices[0]);
    start();
    assert(snapshot().connection_generation == 101);
    assert(dispatch(&devices[0], old_cid));  // Never permission for the new CID.
    assert(pcm.empty());
    assert(dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.size() == 1);
    haptics_experiment_attach(0, 102, &devices[0]);
    assert(snapshot().state == HapticsExperimentState::kDisconnected && timers.empty());

    reset();
    assert(haptics_experiment_request(1, 0));
    const uint32_t requested_generation = snapshot().connection_generation;
    haptics_experiment_detach(&devices[0]);
    haptics_experiment_attach(0, requested_generation + 1, &devices[0]);
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kDisconnected);
    assert(snapshot().connection_generation == requested_generation && pcm.empty());
    // The rejected request did not reserve the slot permanently.
    start();
    assert(snapshot().connection_generation == requested_generation + 1);
}

void support_and_transport_errors() {
    reset();
    assert(!haptics_experiment_request(3, 0));
    assert(!haptics_experiment_request(1, 4));
    devices[0].remote_mtu = 142;
    assert(haptics_experiment_request(1, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kUnsupported);
    assert(snapshot().last_error == 2 && pcm.empty() && !haptics_experiment_owns(&devices[0]));
    devices[0].remote_mtu = 143;
    devices[0].connection_type = GAP_CONNECTION_LE;
    assert(haptics_experiment_request(1, 0));
    haptics_experiment_poll();
    assert(snapshot().last_error == 1 && pcm.empty());
    devices[0].connection_type = GAP_CONNECTION_ACL;
    devices[0].product_id = 0x0df2;
    start();  // Exact MTU boundary and DualSense Edge.
    assert(pcm.size() == 1);

    reset();
    start(3);
    assert(pcm.size() == 1 && pcm.front().cid == devices[3].conn.interrupt_cid);
    assert(snapshot().connection_generation == 103);

    reset();
    fail_requests = 1;
    start();
    assert(pcm.empty());
    run_until(now_us + 24000);
    assert(pcm.size() == 1 && snapshot().send_failures == 1);
    assert(snapshot().skipped_packets == 1);
    fail_sends = 1;
    run_until(now_us + 24000);
    assert(snapshot().send_failures == 2);
    run_until(static_cast<uint64_t>(snapshot().start_us) + 6148000);
    assert(snapshot().state == HapticsExperimentState::kError);
    assert(snapshot().last_error == 5);
    assert(snapshot().generated_packets == snapshot().sent_packets + 1);
    assert(max_request_depth == 1 && timers.empty());
}

void timing_cost_reentrancy_and_wrap() {
    reset();
    send_cost_us = 500;
    request_cost_us = 200;
    reenter_send = true;
    start();
    const uint64_t started = snapshot().start_us;
    run_until(started + 6148000);
    const auto done = snapshot();
    assert(done.state == HapticsExperimentState::kCompleted);
    assert(done.sent_packets == 288 && send_calls == 288);
    assert(done.max_generate_us == 0);  // Neither request nor send is generation.
    assert(done.max_request_wait_us == 200 && max_request_depth == 1);
    for (unsigned i = 0; i < pcm.size(); ++i) {
        assert(pcm[i].at_us >= due(started, i));
        assert(pcm[i].at_us - due(started, i) < 1200);
    }

    reset((uint64_t{1} << 32) - 1000123);
    const uint64_t wrap_start = start();
    run_until(wrap_start + 6148000);
    const auto wrapped = snapshot();
    assert(wrapped.state == HapticsExperimentState::kCompleted);
    assert(wrapped.start_us == static_cast<uint32_t>(wrap_start));
    assert(wrapped.first_tone_due_us == static_cast<uint32_t>(wrap_start + 1024000));
    assert(wrapped.first_tone_sent_us == static_cast<uint32_t>(pcm[48].at_us));
    assert(wrapped.last_sent_us == static_cast<uint32_t>(pcm.back().at_us));
    assert(wrapped.elapsed_us >= 6147000 && wrapped.elapsed_us < 6148000);
    assert(wrapped.max_send_gap_us <= 22000 && wrapped.sent_packets == 288);
}
void synchronous_teardown_releases_admission() {
    reset();
    detach_during_send = true;
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kDisconnected);
    assert(!haptics_experiment_owns(&devices[0]));
    assert(!native_output_scheduler_granted(&devices[0]) && timers.empty());
    const size_t sent = pcm.size();
    run_until(now_us + 100000);
    assert(pcm.size() == sent);

    detach_during_send = false;
    haptics_experiment_attach(0, 101, &devices[0]);
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    run_until(now_us + 50000);
    assert(snapshot().state == HapticsExperimentState::kRunning);
    assert(snapshot().connection_generation == 101);
    assert(snapshot().sent_packets >= 3);
}


void gameplay_led_yield_releases_admission() {
    reset();
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    const uint64_t started = snapshot().start_us;
    delivery = Delivery::kDeferred;
    run_until(due(started, 1, snapshot().packet_frames) + 1000);
    assert(devices[0].notification_pending);
    devices[0].credit = false;
    emit_generic(&devices[0], GenericKind::kLed);
    devices[0].credit = true;
    assert(!dispatch(&devices[0], devices[0].conn.interrupt_cid));
    assert(pcm.size() == 1 && generic_queue.empty());
    assert(generic_sent.size() == 1 && generic_sent[0].kind == GenericKind::kLed);
    delivery = Delivery::kImmediate;
    run_until(now_us + 2000);
    assert(snapshot().state == HapticsExperimentState::kRunning);
    assert(pcm.size() == 2);
}

void gameplay_timeline_and_lifecycle() {
    reset();
    SwitchHapticsDecoder decoder;
    const auto feed = [&](bool left) {
        const uint32_t active = (1u << 30) | (96u << 23) | (64u << 16) | (64u << 2);
        const uint32_t words[] = {left ? active : 0x40400100u,
                                  left ? 0x40400100u : active};
        uint8_t bytes[8]{};
        for (unsigned side = 0; side < 2; ++side) {
            for (unsigned byte = 0; byte < 4; ++byte) {
                bytes[side * 4 + byte] = static_cast<uint8_t>(words[side] >> (8 * byte));
            }
        }
        const auto decoded = decoder.decode(bytes);
        assert(haptics_experiment_submit(0, 100, now_us, decoded.hd));
    };
    assert(haptics_experiment_request(2, 0));
    const uint64_t started = now_us;
    feed(true);  // A sole first command survives the Pending -> Running boundary.
    haptics_experiment_poll();
    assert(snapshot().mode == 1 && snapshot().state == HapticsExperimentState::kRunning);
    assert(haptics_experiment_gameplay_owns(&devices[0]));
    now_us = started + 8000;
    feed(false);
    const uint32_t frames = snapshot().packet_frames;
    run_until(due(started, 1, frames) + 1000);
    assert(pcm.size() == 2 && snapshot().host_updates == 2);
    assert(pcm[1].bytes[7] == frames / 32);
    assert(pcm[1].bytes[8] == (frames == 32 ? 0x92 : 0xd2));
    for (unsigned byte = 10 + frames * 2; byte < 139; ++byte)
        assert(pcm[1].bytes[byte] == 0);
    unsigned left_nonzero = 0, right_nonzero = 0;
    for (unsigned frame = 0; frame < frames; ++frame) {
        const auto left = pcm[1].bytes[10 + frame * 2];
        const auto right = pcm[1].bytes[11 + frame * 2];
        if (frame < 24) {
            assert(right == 0);
            left_nonzero += left != 0;
        } else {
            assert(left == 0);
            right_nonzero += right != 0;
        }
    }
    assert(left_nonzero > 10 && right_nonzero > (frames - 24) / 2);
    SwitchHapticsFrame stale{};
    stale.actuators[0].sample_count = 1;
    stale.actuators[0].samples[0].low_amplitude_q15 = 16000;
    assert(!haptics_experiment_submit(0, 101, now_us, stale));
    run_until(started + 6300000);
    assert(snapshot().state == HapticsExperimentState::kRunning);
    assert(snapshot().sent_packets > 288 && snapshot().skipped_packets == 0);
    for (unsigned byte = 10; byte < 138; ++byte) assert(pcm.back().bytes[byte] == 0);
    assert(snapshot().dropped_updates == 0 && generic_sent.empty());
    assert(haptics_experiment_feedback(&devices[0], 100, 60, 30));
    run_until(now_us + 22000);
    assert(generic_sent.empty());  // Local confirmation must not leave PCM mode.
    assert(haptics_experiment_request(0, 0));
    haptics_experiment_poll();
    run_until(now_us + 10000);
    assert(snapshot().state == HapticsExperimentState::kStopped && snapshot().mode == 1);
    assert(!haptics_experiment_owns(&devices[0]) && generic_sent.size() == 2);
}

void gameplay_missing_callback_is_bounded() {
    reset();
    delivery = Delivery::kNever;
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    run_until(now_us + 105000);
    assert(snapshot().state == HapticsExperimentState::kError && snapshot().last_error == 4);
    assert(!haptics_experiment_owns(&devices[0]) && timers.empty());
}

void gameplay_queued_start_and_command_overflow() {
    reset();
    devices[0].credit = false;
    emit_generic(&devices[0], GenericKind::kLed);
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kPending && pcm.empty());
    run_until(now_us + 10000);
    devices[0].credit = true;
    assert(!dispatch(&devices[0], devices[0].conn.control_cid));
    run_until(now_us + 3000);
    assert(snapshot().state == HapticsExperimentState::kRunning && pcm.size() == 1);
    assert(generic_queue.empty() && generic_sent.size() == 1);
    assert(generic_sent.front().kind == GenericKind::kLed);

    SwitchHapticsFrame frame{};
    frame.actuators[0].sample_count = 1;
    frame.actuators[0].samples[0].low_amplitude_q15 = 20000;
    const size_t sent_before = pcm.size();
    for (unsigned i = 0; i < 17; ++i) {
        now_us += 8000;
        assert(haptics_experiment_submit(0, 100, now_us, frame));
    }
    run_until(now_us);
    assert(snapshot().state == HapticsExperimentState::kRunning);
    assert(snapshot().host_updates == 17 && snapshot().dropped_updates == 1);
    assert(snapshot().skipped_packets != 0 && pcm.size() == sent_before + 1);
}

void stateful_rumble_prepare_feedback_and_zero() {
    reset();
    devices[0].credit = false;
    emit_generic(&devices[0], GenericKind::kLed);
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kPending);
    assert(haptics_experiment_submit_rumble(0, 100, now_us, 180, 0));
    assert(!haptics_experiment_submit_rumble(1, 101, now_us, 255, 255));
    assert(!haptics_experiment_submit_rumble(0, 99, now_us, 255, 255));
    run_until(now_us + 70000);  // Preparation must not age out held strengths.
    devices[0].credit = true;
    assert(!dispatch(&devices[0], devices[0].conn.control_cid));
    run_until(now_us + 3000);
    assert(snapshot().state == HapticsExperimentState::kRunning);
    const auto assert_channels = [](bool left, bool right) {
        const uint32_t frames = snapshot().packet_frames;
        unsigned active[2]{};
        for (uint32_t frame = 0; frame < frames; ++frame) {
            active[0] += pcm.back().bytes[10 + frame * 2] != 0;
            active[1] += pcm.back().bytes[11 + frame * 2] != 0;
        }
        assert(left ? active[0] > frames / 2 : active[0] == 0);
        assert(right ? active[1] > frames / 2 : active[1] == 0);
    };
    run_until(now_us + 300000);
    assert_channels(true, false);
    assert(haptics_experiment_feedback(&devices[0], 255, 255, 100));
    assert(haptics_experiment_submit_rumble(0, 100, now_us, 0, 170));
    run_until(now_us + 60000);
    assert_channels(true, true);
    run_until(now_us + 200000);
    assert_channels(false, true);  // Overlay reveals the newest held command.
    assert(generic_sent.size() == 1 &&
           generic_sent.front().kind == GenericKind::kLed);
    assert(haptics_experiment_submit_rumble(0, 100, now_us, 0, 0));
    run_until(now_us + 80000);
    assert_channels(false, false);
    assert(haptics_experiment_request(0, 0));
    assert(!haptics_experiment_submit_rumble(0, 100, now_us, 255, 255));
    haptics_experiment_poll();
    run_until(now_us + 10000);
    assert(snapshot().state == HapticsExperimentState::kStopped);

    assert(haptics_experiment_request(1, 0));
    assert(!haptics_experiment_submit_rumble(0, 100, now_us, 255, 255));
    haptics_experiment_poll();
    assert(snapshot().mode == 0 && snapshot().packet_frames == 64);
}

void stateful_rumble_generation_and_overflow() {
    reset();
    assert(haptics_experiment_request(2, 0));
    assert(haptics_experiment_submit_rumble(0, 100, now_us, 255, 0));
    haptics_experiment_detach(&devices[0]);
    haptics_experiment_attach(0, 101, &devices[0]);
    assert(!haptics_experiment_submit_rumble(0, 100, now_us, 255, 0));
    haptics_experiment_poll();
    assert(snapshot().state == HapticsExperimentState::kDisconnected);
    assert(haptics_experiment_request(2, 0));
    haptics_experiment_poll();
    run_until(now_us + 100000);
    for (unsigned byte = 10; byte < 138; ++byte)
        assert(pcm.back().bytes[byte] == 0);
    for (unsigned command = 0; command < 17; ++command) {
        assert(haptics_experiment_submit_rumble(
            0, 101, now_us, command == 16 ? 0 : 255, 0));
    }
    run_until(now_us + 80000);
    assert(snapshot().host_updates == 17 && snapshot().dropped_updates == 1);
    for (unsigned byte = 10; byte < 138; ++byte)
        assert(pcm.back().bytes[byte] == 0);
    assert(generic_sent.empty());
}

}  // namespace

// Transport attribution has its own native fixture; this fixture isolates PCM
// scheduling and packet content from the optional measurement backend.
void haptics_transport_probe_prepare() {}
void haptics_transport_probe_begin(uint32_t, uint32_t, uint16_t) {}
void haptics_transport_probe_end() {}
void haptics_transport_probe_timer(uint32_t) {}
void haptics_transport_probe_permission(uint32_t) {}
void haptics_transport_probe_send(uint32_t, uint32_t, bool) {}

uint64_t time_us_64() {
    return now_us;
}

void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer, void (*handler)(btstack_timer_source_t*)) {
    no_lock();
    timer->process = handler;
}

void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t timeout_ms) {
    no_lock();
    // Exactly pico_btstack/btstack_run_loop_async_context.c, including +1.
    timer->timeout_us = (now_us / 1000 + timeout_ms + 1) * 1000;
}

void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    no_lock();
    assert(timer->process != nullptr);
    assert(std::find(timers.begin(), timers.end(), timer) == timers.end());
    timers.push_back(timer);
}

int btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    no_lock();
    const auto it = std::find(timers.begin(), timers.end(), timer);
    if (it == timers.end()) {
        return 0;
    }
    timers.erase(it);
    return 1;
}

gap_connection_type_t gap_get_connection_type(uint16_t handle) {
    no_lock();
    return handle < devices.size() ? devices[handle].connection_type
                                   : GAP_CONNECTION_INVALID;
}

uint16_t l2cap_get_remote_mtu_for_local_cid(uint16_t cid) {
    no_lock();
    return device_for_cid(cid)->remote_mtu;
}

bool l2cap_can_send_packet_now(uint16_t cid) {
    no_lock();
    const auto* device = device_for_cid(cid);
    return device->credit && device->native_ready_cid == cid;
}

int hci_number_free_acl_slots_for_handle(uint16_t handle) {
    no_lock();
    return handle < devices.size() && devices[handle].credit ? 4 : 0;
}

uint8_t uni_circular_buffer_is_empty(const uni_circular_buffer_t* buffer) {
    return buffer->queued == 0;
}

uint8_t l2cap_request_can_send_now_event(uint16_t cid) {
    no_lock();
    ++request_calls;
    if (fail_requests != 0) {
        --fail_requests;
        return 0x44;
    }
    auto* device = device_for_cid(cid);
    assert(!device->notification_pending);
    device->notification_pending = true;
    ++request_depth;
    max_request_depth = std::max(max_request_depth, request_depth);
    now_us += request_cost_us;
    if (delivery == Delivery::kImmediate && device->credit) {
        dispatch(device, cid);  // Real BTstack can call here, BEFORE return.
    }
    --request_depth;
    return ERROR_CODE_SUCCESS;
}

uint8_t l2cap_send(uint16_t cid, const uint8_t* data, uint16_t size) {
    no_lock();
    auto* device = device_for_cid(cid);
    assert(device->native_ready_cid == cid && device->credit);
    assert(native_output_scheduler_granted(device));
    device->native_ready_cid = 0;
    assert(size == 143);
    ++send_calls;
    const uint64_t submitted_us = now_us;
    now_us += send_cost_us;
    if (reenter_send) {
        assert(native_output_scheduler_on_can_send_now(device, cid));
        const auto concurrent_snapshot = snapshot();
        assert(concurrent_snapshot.state == HapticsExperimentState::kPending ||
               concurrent_snapshot.state == HapticsExperimentState::kRunning);
    }
    if (fail_sends != 0) {
        --fail_sends;
        return 0x55;
    }
    Pcm packet{submitted_us, cid, {}};
    std::copy(data, data + size, packet.bytes.begin());
    pcm.push_back(packet);
    if (detach_during_send) {
        haptics_experiment_detach(device);
    }
    return ERROR_CODE_SUCCESS;
}

int main(int argc, char** argv) {
    assert(argc == 2);
    haptics_experiment_prepare();
    native_output_scheduler_prepare();
    assert(snapshot().state == HapticsExperimentState::kIdle);
    nominal_run(argv[1]);
    stalled_deadlines();
    deferred_and_missing_callbacks();
    stop_preemption_and_restore();
    compatibility_queue_and_parser_timers();
    reconnect_and_pending_generation();
    support_and_transport_errors();
    timing_cost_reentrancy_and_wrap();
    synchronous_teardown_releases_admission();
    gameplay_led_yield_releases_admission();
    gameplay_timeline_and_lifecycle();
    gameplay_queued_start_and_command_overflow();
    stateful_rumble_prepare_feedback_and_zero();
    stateful_rumble_generation_and_overflow();
    gameplay_missing_callback_is_bounded();
    std::cout << "haptics experiment behavioral regressions passed\n";
}
