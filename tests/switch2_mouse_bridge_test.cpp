#include "controller_input.h"
#include "model.h"
#include "input/bluepad32_input_backend.h"
#include "input/switch2_mouse_capture.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "parser/uni_hid_parser_switch2.h"
#include "pico/stdlib.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>

static uint64_t now;
static uint32_t stage;
static BootselPairingButtonEvent next_button_event = BootselPairingButtonEvent::kNone;
static unsigned pairing_requests, clear_requests, button_polls;
BootselPairingButtonEvent bootsel_pairing_button_task() {
    ++button_polls;
    const auto event = next_button_event;
    next_button_event = BootselPairingButtonEvent::kNone;
    return event;
}
void bluepad32_input_backend_open_pairing_window() { ++pairing_requests; }
uint32_t bluepad32_input_backend_clear_pairings() { ++clear_requests; return 1; }
static const uint8_t source_address[] = {0x98,0xe2,0x55,7,0xdf,0};
static const uint8_t other_address[] = {0x98,0xe2,0x55,7,0xdf,1};
static constexpr uint16_t other_product_id = SWITCH2_PROBE_JOYCON_LEFT ? 0x2066 : 0x2067;
static constexpr uint8_t other_report_id = SWITCH2_PROBE_JOYCON_LEFT ? 8 : 7;
void system_clock_initialize() {}
void bluepad32_input_backend_init() { stage = 1; }
void controller_profile_runtime_reset() {}
void bluepad32_input_backend_start() { stage = 2; }
void bluepad32_input_backend_diagnostics(Bluepad32BackendDiagnostics* out) {
    *out = {}; out->initialization_stage = stage;
}
absolute_time_t make_timeout_time_ms(uint32_t timeout) { return now + timeout; }
absolute_time_t get_absolute_time() { return now; }
uint32_t to_ms_since_boot(absolute_time_t value) { return static_cast<uint32_t>(value); }
bool time_reached(absolute_time_t deadline) { return now >= deadline; }
void sleep_ms(uint32_t milliseconds) { now += milliseconds; }

using NativeReport = std::array<uint8_t, 63>;

static NativeReport native_report(uint8_t counter, uint8_t motion_length,
                                  int16_t x = 1, int16_t y = -2) {
    NativeReport report{};
    // Deliberately opaque, nonzero bytes, including NFC and reserved fields.
    // The model-specific length declares 30/40 packed motion bytes; do not
    // decode them or normalize the unused tail of a 30-byte sample.
    for (size_t i = 0; i < report.size(); ++i)
        report[i] = static_cast<uint8_t>((i * 37 + counter) % 255 + 1);
    report[0] = counter;
    report[1] = 0x93;
    report[2] = 0x12; report[3] = 0xd1;
    report[4] = 0xe7;
    report[5] = 0x23; report[6] = 0x81; report[7] = 0x45;
    report[8] = 0x38;
    report[9] = static_cast<uint8_t>(x);
    report[10] = static_cast<uint16_t>(x) >> 8;
    report[11] = static_cast<uint8_t>(y);
    report[12] = static_cast<uint16_t>(y) >> 8;
    report[13] = 0x1b;
    report[PROBE_IMU_LENGTH_OFFSET] = motion_length;
    return report;
}

static void emit(const NativeReport& report, const uint8_t* address = source_address,
                 uint16_t product_id = PROBE_JOYCON_PID,
                 uint8_t report_id = PROBE_NATIVE_REPORT_ID,
                 uint16_t length = 63) {
    assert(length <= report.size());
    switch_pico_switch2_mouse_report(product_id, address, report_id, report.data(),
                                    length, static_cast<uint32_t>(now));
}

static void disconnect(const uint8_t* address = source_address,
                       uint16_t product_id = PROBE_JOYCON_PID) {
    switch_pico_switch2_mouse_report(product_id, address, 0, nullptr, 0,
                                    static_cast<uint32_t>(now));
}

static probe_controller_input poll(uint32_t timestamp = static_cast<uint32_t>(now),
                                   uint8_t instance = 0) {
    probe_controller_input input{};
    probe_controller_input_poll(instance, timestamp, &input);
    return input;
}

static uint32_t expect_report(const NativeReport& expected,
                              uint32_t timestamp = static_cast<uint32_t>(now),
                              uint8_t instance = 0) {
    NativeReport actual;
    actual.fill(0xa5);
    const uint32_t serial =
        probe_controller_input_peek_native_report(instance, timestamp, actual.data());
    assert(serial != 0 && actual == expected);
    return serial;
}

static void expect_empty(uint8_t instance = 0) {
    NativeReport actual;
    actual.fill(0xa5);
    const auto untouched = actual;
    assert(probe_controller_input_peek_native_report(instance, static_cast<uint32_t>(now), actual.data()) == 0);
    assert(actual == untouched);
}

static void expect_inactive(const probe_controller_input& input) {
    assert(!input.active && input.mouse_epoch == 0);
    assert(input.buttons[0] == 0 && input.buttons[1] == 0);
    assert(input.stick[0] == 0 && input.stick[1] == 0 && input.stick[2] == 0);
    assert(input.native_status == 0 && input.mouse_surface == 0);
    assert(input.mouse_total_x == 0 && input.mouse_total_y == 0);
}

static void test_startup_pairing_and_stream_gate() {
    next_button_event = BootselPairingButtonEvent::kOpenPairing;
    assert(!probe_controller_input_pairing_task() && button_polls == 0 && pairing_requests == 0);
    probe_controller_input_set_native_stream(0, true);
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, 1));
    expect_inactive(poll());
    probe_controller_input_clock_init();
    probe_controller_input_init();

    // Even an enable request after init must not open the pre-flash-ready gate.
    probe_controller_input_set_native_stream(0, true);
    const auto report = native_report(0x31, 30, -6, 9);
    emit(report);
    expect_empty();
    assert(probe_controller_input_start());
    expect_empty();
    assert(probe_controller_input_pairing_task() && pairing_requests == 1);
    assert(!probe_controller_input_pairing_task());
    next_button_event = BootselPairingButtonEvent::kClearPairings;
    assert(!probe_controller_input_pairing_task() && clear_requests == 0 && pairing_requests == 1);
    next_button_event = BootselPairingButtonEvent::kOpenPairing;
    assert(probe_controller_input_pairing_task() && pairing_requests == 2 && clear_requests == 0);

    const auto input = poll();
    assert(input.active && input.buttons[0] == 0x12 && input.buttons[1] == 0xd1);
    assert(input.stick[0] == 0x23 && input.stick[1] == 0x81 && input.stick[2] == 0x45);
    assert(input.native_status == 0x38 && input.mouse_surface == 0x1b);
    assert(input.mouse_total_x == -6 && input.mouse_total_y == 9);
    probe_controller_input_set_native_stream(0, true);
    expect_empty(); // Enabling never replays the latest input or raw ring.
    emit(report);
    const uint32_t pending = expect_report(report);
    probe_controller_input_set_native_stream(0, false);
    assert(!probe_controller_input_commit_native_report(0, pending));
    emit(report); // Selected input continues updating while native USB is gated.
    assert(poll().active);
    expect_empty();
    probe_controller_input_set_native_stream(0, true);
    expect_empty();
    emit(report);
    const uint32_t resumed = expect_report(report);
    assert(resumed > pending);
    assert(probe_controller_input_commit_native_report(0, resumed));
    expect_empty();
}

static void test_opaque_fidelity_order_and_retry() {
    now = 100;
    const auto first = native_report(0xfe, 30, -6, 9);
    const auto repeated = native_report(0xff, 40, -32768, 32767);
    const auto last = native_report(0x00, 30, 1, -2);
    emit(first);
    const uint32_t first_serial = expect_report(first);

    // A failed USB submission simply does not commit. New arrivals must not
    // overwrite that retry, combine deltas, or collapse identical packets.
    ++now; emit(repeated);
    ++now; emit(repeated);
    ++now; emit(last);
    const uint32_t last_serial = poll().serial;
    assert(last_serial > first_serial);
    assert(!probe_controller_input_commit_native_report(0, last_serial));
    assert(!probe_controller_input_commit_native_report(0, 0));
    probe_controller_input_set_native_stream(0, true);
    assert(expect_report(first) == first_serial);
    assert(expect_report(first) == first_serial);
    assert(probe_controller_input_commit_native_report(0, first_serial));
    assert(!probe_controller_input_commit_native_report(0, first_serial));

    const uint32_t second_serial = expect_report(repeated);
    assert(second_serial > first_serial);
    assert(probe_controller_input_commit_native_report(0, second_serial));
    const uint32_t third_serial = expect_report(repeated);
    assert(third_serial > second_serial);
    assert(!probe_controller_input_commit_native_report(0, second_serial));
    assert(expect_report(repeated) == third_serial);
    assert(probe_controller_input_commit_native_report(0, third_serial));
    assert(expect_report(last) == last_serial);
    assert(probe_controller_input_commit_native_report(0, last_serial));
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, last_serial));
    expect_empty(); // No cached duplicate report when the source has not advanced.
}

static void test_selected_source_isolation_and_reconnect() {
    now = 200;
    const auto first = native_report(0x41, 30, -17, 19);
    const auto second = native_report(0x42, 40, 31, -37);
    const auto unrelated = native_report(0x99, 40, 300, 300);
    emit(first);
    const auto selected = poll();
    const uint32_t first_serial = expect_report(first);
    assert(first_serial == selected.serial);
    for (unsigned i = 0; i < 30; ++i) emit(unrelated, other_address);
    emit(unrelated, source_address, other_product_id, other_report_id);
    emit(unrelated, source_address, PROBE_JOYCON_PID, other_report_id);
    emit(unrelated, source_address, PROBE_JOYCON_PID, 5);
    emit(unrelated, source_address, PROBE_JOYCON_PID, 0xc0, 12);
    emit(unrelated, source_address, PROBE_JOYCON_PID, PROBE_NATIVE_REPORT_ID, 62);
    emit(unrelated, source_address, PROBE_JOYCON_PID, 0, 1); // Not a teardown.
    uint8_t oversized[64];
    memcpy(oversized, unrelated.data(), unrelated.size());
    oversized[63] = 0x5a;
    switch_pico_switch2_mouse_report(PROBE_JOYCON_PID, source_address,
                                    PROBE_NATIVE_REPORT_ID, oversized,
                                    sizeof(oversized), static_cast<uint32_t>(now));
    disconnect(other_address);
    disconnect(source_address, other_product_id);
    const auto isolated = poll();
    assert(isolated.active && isolated.serial == selected.serial);
    assert(isolated.mouse_epoch == selected.mouse_epoch);
    assert(isolated.mouse_total_x == selected.mouse_total_x &&
           isolated.mouse_total_y == selected.mouse_total_y);
    assert(expect_report(first) == first_serial);
    ++now; emit(second);
    const uint32_t second_serial = poll().serial;
    assert(probe_controller_input_commit_native_report(0, first_serial));
    assert(expect_report(second) == second_serial);
    assert(probe_controller_input_commit_native_report(0, second_serial));
    expect_empty(); // Unrelated ring entries neither evict nor enter the FIFO.

    emit(first);
    const uint32_t disconnected_serial = expect_report(first);
    disconnect();
    ++now; emit(second); // Disconnect and reconnect both occur between polls.
    const auto reconnected = poll();
    assert(reconnected.active && reconnected.mouse_epoch != selected.mouse_epoch);
    assert(reconnected.mouse_total_x == 31 && reconnected.mouse_total_y == -37);
    assert(!probe_controller_input_commit_native_report(0, disconnected_serial));
    assert(expect_report(second) == reconnected.serial);
    assert(reconnected.serial > disconnected_serial);
    assert(probe_controller_input_commit_native_report(0, reconnected.serial));
    expect_empty();

    emit(first);
    const uint32_t pending = expect_report(first);
    disconnect();
    for (unsigned i = 0; i < 30; ++i) emit(unrelated, other_address);
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, pending));
    ++now; emit(second);
    const auto resumed = poll();
    assert(resumed.active && resumed.mouse_epoch != reconnected.mouse_epoch);
    const uint32_t resumed_serial = expect_report(second);
    assert(resumed_serial > pending);
    assert(probe_controller_input_commit_native_report(0, resumed_serial));

    emit(first);
    const uint32_t old_source = expect_report(first);
    emit(unrelated, other_address);
    switch2_mouse_capture_select_input(0, other_address, PROBE_JOYCON_PID);
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, old_source));
    probe_controller_input_set_native_stream(0, true);
    expect_empty(); // Selection cannot revive the other peer's raw history.
    emit(first);
    expect_empty();
    emit(unrelated, other_address);
    const uint32_t new_source = expect_report(unrelated);
    assert(new_source > old_source);
    switch2_mouse_capture_select_input(0, source_address, PROBE_JOYCON_PID);
    expect_inactive(poll());
    probe_controller_input_set_native_stream(0, true);
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, new_source));
    emit(second);
    const uint32_t restored = expect_report(second);
    assert(restored > new_source);
    assert(probe_controller_input_commit_native_report(0, restored));
}

static void test_side_switch_and_sample_ownership() {
    now = 500;
    const auto first = native_report(0x71, 30);
    auto opposite = native_report(0x72, 40);
    opposite[SWITCH2_PROBE_JOYCON_LEFT ? 15 : 14] = 40;
    emit(first);
    const uint32_t old_packet = expect_report(first);
    assert(poll().active);
    uint64_t old_cue = 0;
    assert(probe_controller_input_play_sample(0, 3, &old_cue) && old_cue != 0);
    uint64_t taken = 0;
    uint8_t sample = 0;
    // Unrelated callers cannot take a cue, even with a timestamp that would
    // otherwise expire it. Ownership is checked before mutating its lifetime.
    assert(!switch_pico_switch2_sample_take(other_product_id, source_address,
                                            now + 2000, &sample, &taken));
    assert(!switch_pico_switch2_sample_take(PROBE_JOYCON_PID, other_address,
                                            now + 2000, &sample, &taken));
    assert(probe_controller_input_sample_result(0, old_cue, now) == 0);
    assert(!switch_pico_switch2_sample_result(PROBE_JOYCON_PID, source_address,
                                              old_cue, 1, now)); // Not dispatched.
    assert(switch_pico_switch2_sample_take(PROBE_JOYCON_PID, source_address,
                                           now, &sample, &taken));
    assert(taken == old_cue && sample == 3);
    switch2_mouse_capture_select_input(0, source_address, 0x2069); // Invalid PID.
    switch2_mouse_capture_select_input(0, nullptr, PROBE_JOYCON_PID);
    assert(expect_report(first) == old_packet && poll().active);
    assert(probe_controller_input_sample_result(0, old_cue, now) == 0);

    // Same address, different side is still a new source. Native and cue
    // tokens from the prior selection cannot acknowledge or consume it.
    switch2_mouse_capture_select_input(0, source_address, other_product_id);
    expect_empty();
    expect_inactive(poll());
    assert(!probe_controller_input_commit_native_report(0, old_packet));
    assert(probe_controller_input_sample_result(0, old_cue, now) == -1);
    uint64_t new_cue = 0;
    assert(!probe_controller_input_play_sample(0, 4, &new_cue));
    emit(first);
    emit(opposite, source_address, other_product_id, PROBE_NATIVE_REPORT_ID);
    expect_inactive(poll());
    emit(opposite, source_address, other_product_id, other_report_id);
    assert(poll().active);
    expect_empty(); // Selection disabled native output even for valid input.
    probe_controller_input_set_native_stream(0, true);
    expect_empty();
    emit(opposite, source_address, other_product_id, other_report_id);
    const uint32_t new_packet = expect_report(opposite);
    assert(new_packet > old_packet);
    assert(probe_controller_input_play_sample(0, 4, &new_cue) && new_cue > old_cue);
    assert(!switch_pico_switch2_sample_result(PROBE_JOYCON_PID, source_address,
                                              old_cue, 1, now + 2000));
    assert(switch_pico_switch2_sample_take(other_product_id, source_address,
                                           now, &sample, &taken));
    assert(sample == 4 && taken == new_cue);
    assert(!switch_pico_switch2_sample_result(PROBE_JOYCON_PID, source_address,
                                              new_cue, 1, now + 2000));
    assert(!switch_pico_switch2_sample_result(other_product_id, other_address,
                                              new_cue, -1, now + 2000));
    assert(!switch_pico_switch2_sample_result(other_product_id, source_address,
                                              old_cue, 1, now + 2000));
    assert(probe_controller_input_sample_result(0, old_cue, now + 2000) == -1);
    disconnect(source_address);
    assert(poll().active && expect_report(opposite) == new_packet);
    assert(probe_controller_input_sample_result(0, new_cue, now) == 0);
    assert(switch_pico_switch2_sample_result(other_product_id, source_address,
                                             new_cue, 1, now));
    disconnect(source_address, other_product_id);
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, new_packet));
    assert(probe_controller_input_sample_result(0, new_cue, now) == -1);

    emit(opposite, source_address, other_product_id, other_report_id);
    assert(probe_controller_input_play_sample(0, 5, &new_cue) && new_cue > old_cue);
    old_cue = new_cue;
    switch2_mouse_capture_select_input(0, other_address, other_product_id);
    expect_inactive(poll());
    assert(probe_controller_input_sample_result(0, old_cue, now) == -1);
    assert(!probe_controller_input_play_sample(0, 6, &new_cue));
    emit(opposite, other_address, other_product_id, other_report_id);
    assert(probe_controller_input_play_sample(0, 6, &new_cue) && new_cue > old_cue);
    assert(switch_pico_switch2_sample_take(other_product_id, other_address,
                                           now, &sample, &taken));
    assert(taken == new_cue && sample == 6);
    assert(!switch_pico_switch2_sample_result(other_product_id, source_address,
                                              old_cue, 1, now));
    assert(switch_pico_switch2_sample_result(other_product_id, other_address,
                                             new_cue, 1, now));
    assert(probe_controller_input_sample_result(0, new_cue, now) == 1);
    assert(probe_controller_input_sample_result(0, new_cue, now) == -1);
    switch2_mouse_capture_select_input(0, source_address, PROBE_JOYCON_PID);
    probe_controller_input_set_native_stream(0, true);
    expect_empty();
}

static void test_bounded_overflow() {
    now = 1000;
    const auto first = native_report(0x50, 30);
    emit(first);
    const uint32_t old_serial = expect_report(first);
    // The 32-entry contract bounds backlog independently of the diagnostic ring.
    for (unsigned i = 1; i < 32; ++i) {
        ++now;
        emit(native_report(static_cast<uint8_t>(0x50 + i), 40));
    }
    assert(expect_report(first) == old_serial);
    const auto newest = native_report(0xbb, 30, -101, 103);
    ++now; emit(newest);
    assert(!probe_controller_input_commit_native_report(0, old_serial));
    const uint32_t newest_serial = expect_report(newest);
    assert(newest_serial > old_serial);
    const auto following = native_report(0xbc, 40, 107, -109);
    ++now; emit(following);
    assert(expect_report(newest) == newest_serial);
    assert(probe_controller_input_commit_native_report(0, newest_serial));
    const uint32_t following_serial = expect_report(following);
    assert(following_serial > newest_serial);
    assert(probe_controller_input_commit_native_report(0, following_serial));
    expect_empty(); // Overflow discarded all prior history, not merely its head.
}

static void test_continuous_state_coalescing_preserves_events_and_borrowed_head() {
    now = 1500;
    disconnect();
    probe_controller_input_set_native_stream(0, true);
    const auto first = native_report(0x80, 30, 0, 0);
    auto newest = first;
    emit(first);
    for (unsigned i = 1; i <= 24; ++i) {
        now += 8;
        newest[0] = static_cast<uint8_t>(0x80 + i);
        newest[5] = static_cast<uint8_t>(first[5] + i);
        newest[PROBE_IMU_LENGTH_OFFSET + 1] = static_cast<uint8_t>(i);
        emit(newest);
    }
    // No 192ms history of analog/IMU-only updates is replayed to a slow consumer.
    assert(probe_controller_input_commit_native_report(0, expect_report(newest)));
    expect_empty();

    emit(first);
    const uint32_t borrowed = expect_report(first);
    for (unsigned i = 0; i < 6; ++i) {
        now += 8;
        ++newest[0];
        ++newest[6];
        emit(newest);
    }
    assert(expect_report(first) == borrowed);
    assert(probe_controller_input_commit_native_report(0, borrowed));
    assert(probe_controller_input_commit_native_report(0, expect_report(newest)));
    expect_empty();

    auto pressed = first;
    pressed[2] ^= 1;
    auto last_pressed = pressed;
    ++last_pressed[0]; ++last_pressed[5];
    auto released = last_pressed;
    released[2] = first[2];
    auto surface = released;
    surface[13] ^= 0x10;
    auto opaque_tail = surface;
    opaque_tail.back() ^= 0x80;
    auto other_format = opaque_tail;
    other_format[PROBE_IMU_LENGTH_OFFSET] = 40;
    emit(first); emit(pressed); emit(last_pressed); emit(released);
    emit(surface); emit(opaque_tail); emit(other_format);
    for (const auto& expected : {first, last_pressed, released, surface, opaque_tail, other_format})
        assert(probe_controller_input_commit_native_report(0, expect_report(expected)));
    expect_empty();

    auto mouse = first;
    mouse[9] = 7;
    emit(first); emit(mouse); emit(first);
    for (const auto& expected : {first, mouse, first})
        assert(probe_controller_input_commit_native_report(0, expect_report(expected)));
    expect_empty();
}

static void test_expiry_and_wrapping_clock() {
    now = 2000;
    const auto first = native_report(0x61, 30);
    const auto fresh = native_report(0x62, 40);
    emit(first);
    const uint32_t expired = expect_report(first);
    now += 499;
    assert(poll().active && expect_report(first) == expired);
    ++now;
    emit(fresh, other_address); // Wrong-source traffic cannot refresh the timeout.
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, expired));

    ++now; emit(first);
    const uint32_t stale_head = expect_report(first);
    now += 499; emit(fresh);
    assert(expect_report(first) == stale_head);
    ++now;
    assert(poll().active); // Latest source is fresh, but its queued head is not.
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, stale_head));
    emit(fresh);
    const uint32_t resumed = expect_report(fresh);
    assert(resumed > stale_head);
    assert(probe_controller_input_commit_native_report(0, resumed));
    expect_empty();

    now = static_cast<uint64_t>(UINT32_MAX) - 100;
    emit(first);
    const uint32_t wrapped = expect_report(first);
    // The producer can timestamp input one millisecond after the caller samples
    // its clock; a signed age must accept this race rather than expire the input.
    const uint32_t before_capture = static_cast<uint32_t>(now) - 1;
    assert(poll(before_capture).active);
    assert(expect_report(first, before_capture) == wrapped);
    now += 499; // Cross the uint32 millisecond rollover with a fresh packet.
    assert(poll().active && expect_report(first) == wrapped);
    ++now;
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(0, wrapped));
    ++now; emit(fresh);
    assert(poll().active);
    const uint32_t after_wrap = expect_report(fresh);
    assert(after_wrap > wrapped);
    assert(probe_controller_input_commit_native_report(0, after_wrap));
    expect_empty();
}

#if SWITCH2_PROBE_COMPOSITE
static void test_simultaneous_sources() {
    const uint8_t left_address[] = {0x98,0xe2,0x55,7,0xe9,0xd3};
    now = 10000;
    disconnect();
    const auto right = native_report(0x31, 30, 7, -9);
    auto left = native_report(0x62, 40, -13, 17);
    left[probe_model_imu_length_offset(1)] = 40;
    probe_controller_input_set_native_stream(0, true);
    probe_controller_input_set_native_stream(1, true);
    expect_empty(0);
    expect_empty(1);
    emit(right);
    const uint32_t r0 = expect_report(right);
    emit(left, left_address, probe_model_pid(1), probe_model_report_id(1));
    const uint32_t l0 = expect_report(left, now, 1);
    emit(right);
    const uint32_t r1 = poll().serial;
    emit(left, left_address, probe_model_pid(1), probe_model_report_id(1));
    const auto li = poll(now, 1);
    const auto ri = poll();
    assert(r0 < l0 && l0 < r1 && r1 < li.serial);
    assert(ri.mouse_epoch == r0 && li.mouse_epoch == l0);
    assert(ri.mouse_total_x == 14 && ri.mouse_total_y == -18);
    assert(li.mouse_total_x == -26 && li.mouse_total_y == 34);
    // R is already owned: selecting it for L must not duplicate or steal it.
    switch2_mouse_capture_select_input(1, source_address, probe_model_pid(0));
    assert(expect_report(right) == r0);
    assert(expect_report(left, now, 1) == l0);
    assert(!probe_controller_input_commit_native_report(1, r0));
    assert(!probe_controller_input_commit_native_report(0, l0));
    assert(probe_controller_input_commit_native_report(1, l0));
    assert(expect_report(left, now, 1) == li.serial);
    assert(probe_controller_input_commit_native_report(1, li.serial));
    expect_empty(1);
    assert(expect_report(right) == r0); // L consumption never moves stalled R.

    // R overflow drops only its own backlog; L's retry remains byte-identical.
    emit(left, left_address, probe_model_pid(1), probe_model_report_id(1));
    const uint32_t left_retry = expect_report(left, now, 1);
    for (unsigned i = 0; i < 31; ++i) emit(right);
    assert(!probe_controller_input_commit_native_report(0, r0));
    assert(expect_report(left, now, 1) == left_retry);
    const uint32_t r2 = expect_report(right);
    assert(r2 > left_retry);

    uint64_t rcue, lcue, taken;
    uint8_t sample;
    assert(probe_controller_input_play_sample(0, 3, &rcue));
    assert(probe_controller_input_play_sample(1, 5, &lcue) && lcue > rcue);
    assert(switch_pico_switch2_sample_take(probe_model_pid(0), source_address, now, &sample, &taken));
    assert(sample == 3 && taken == rcue);
    assert(switch_pico_switch2_sample_take(probe_model_pid(1), left_address, now, &sample, &taken));
    assert(sample == 5 && taken == lcue);
    assert(!switch_pico_switch2_sample_result(probe_model_pid(1), left_address, rcue, 1, now + 2000));
    assert(!switch_pico_switch2_sample_result(probe_model_pid(0), source_address, lcue, -1, now + 2000));
    assert(probe_controller_input_sample_result(0, lcue, now + 2000) == -1);
    assert(probe_controller_input_sample_result(1, rcue, now + 2000) == -1);
    assert(probe_controller_input_sample_result(0, rcue, now) == 0);
    assert(probe_controller_input_sample_result(1, lcue, now) == 0);

    probe_controller_input_set_native_stream(0, false);
    probe_controller_input_cancel_sample(0);
    assert(probe_controller_input_sample_result(0, rcue, now) == -1);
    assert(probe_controller_input_sample_result(1, lcue, now) == 0);
    assert(expect_report(left, now, 1) == left_retry);
    assert(switch_pico_switch2_sample_result(probe_model_pid(1), left_address, lcue, 1, now));
    assert(probe_controller_input_sample_result(1, lcue, now) == 1);
    assert(probe_controller_input_sample_result(1, lcue, now) == -1);

    // R selection/disconnect cannot revoke L's pending packet or cue.
    assert(probe_controller_input_play_sample(1, 6, &lcue));
    switch2_mouse_capture_select_input(0, other_address, probe_model_pid(0));
    disconnect(source_address, probe_model_pid(0));
    assert(expect_report(left, now, 1) == left_retry);
    assert(probe_controller_input_sample_result(1, lcue, now) == 0);
    assert(poll(now, 1).mouse_epoch == l0);
    switch2_mouse_capture_select_input(0, source_address, probe_model_pid(0));
    probe_controller_input_set_native_stream(0, true);
    emit(right);
    const uint32_t right_retry = expect_report(right);
    assert(probe_controller_input_play_sample(0, 7, &rcue) && rcue > lcue);
    disconnect(left_address, probe_model_pid(1));
    expect_empty(1);
    expect_inactive(poll(now, 1));
    assert(probe_controller_input_sample_result(1, lcue, now) == -1);
    assert(expect_report(right) == right_retry);
    assert(probe_controller_input_sample_result(0, rcue, now) == 0);
    assert(switch_pico_switch2_sample_take(probe_model_pid(0), source_address, now, &sample, &taken));
    assert(sample == 7 && taken == rcue);
    assert(switch_pico_switch2_sample_result(probe_model_pid(0), source_address, rcue, 1, now));
    assert(probe_controller_input_sample_result(0, rcue, now) == 1);

    // Reconnection creates a distinct L epoch and does not replay its old queue.
    emit(left, left_address, probe_model_pid(1), probe_model_report_id(1));
    const auto resumed_left = poll(now, 1);
    assert(resumed_left.mouse_epoch != l0 && resumed_left.mouse_total_x == -13);
    assert(!probe_controller_input_commit_native_report(1, left_retry));
    assert(probe_controller_input_commit_native_report(1, expect_report(left, now, 1)));
    assert(probe_controller_input_commit_native_report(0, right_retry));
    expect_empty(0);
    expect_empty(1);

    // A refreshed L packet cannot refresh R's independent source deadline.
    now += 499;
    emit(left, left_address, probe_model_pid(1), probe_model_report_id(1));
    ++now;
    expect_inactive(poll());
    assert(poll(now, 1).active);
    assert(probe_controller_input_commit_native_report(1, expect_report(left, now, 1)));
    expect_empty(0);
    expect_empty(1);
}
#endif

int main() {
    test_startup_pairing_and_stream_gate();
    test_opaque_fidelity_order_and_retry();
    test_selected_source_isolation_and_reconnect();
    test_side_switch_and_sample_ownership();
    test_bounded_overflow();
    test_continuous_state_coalescing_preserves_events_and_borrowed_head();
    test_expiry_and_wrapping_clock();
#if SWITCH2_PROBE_COMPOSITE
    test_simultaneous_sources();
#endif
    puts("Native packet fidelity, FIFO retry/order, source barriers, overflow, expiry and pairing passed");
}
