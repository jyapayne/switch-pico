#include "controller_input.h"
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
    // Byte 15 declares 30/40 packed motion bytes at 16..55; do not decode them
    // or normalize the unused tail of a 30-byte sample.
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
    report[15] = motion_length;
    return report;
}

static void emit(const NativeReport& report, const uint8_t* address = source_address,
                 uint16_t product_id = 0x2066, uint8_t report_id = 8,
                 uint16_t length = 63) {
    assert(length <= report.size());
    switch_pico_switch2_mouse_report(product_id, address, report_id, report.data(),
                                    length, static_cast<uint32_t>(now));
}

static void disconnect(const uint8_t* address = source_address,
                       uint16_t product_id = 0x2066) {
    switch_pico_switch2_mouse_report(product_id, address, 0, nullptr, 0,
                                    static_cast<uint32_t>(now));
}

static probe_controller_input poll(uint32_t timestamp = static_cast<uint32_t>(now)) {
    probe_controller_input input{};
    probe_controller_input_poll(timestamp, &input);
    return input;
}

static uint32_t expect_report(const NativeReport& expected,
                              uint32_t timestamp = static_cast<uint32_t>(now)) {
    NativeReport actual;
    actual.fill(0xa5);
    const uint32_t serial =
        probe_controller_input_peek_native_report(timestamp, actual.data());
    assert(serial != 0 && actual == expected);
    return serial;
}

static void expect_empty() {
    NativeReport actual;
    actual.fill(0xa5);
    const auto untouched = actual;
    assert(probe_controller_input_peek_native_report(
               static_cast<uint32_t>(now), actual.data()) == 0);
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
    probe_controller_input_set_native_stream(true);
    expect_empty();
    assert(!probe_controller_input_commit_native_report(1));
    expect_inactive(poll());
    probe_controller_input_clock_init();
    probe_controller_input_init();

    // Even an enable request after init must not open the pre-flash-ready gate.
    probe_controller_input_set_native_stream(true);
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
    probe_controller_input_set_native_stream(true);
    expect_empty(); // Enabling never replays the latest input or raw ring.
    emit(report);
    const uint32_t pending = expect_report(report);
    probe_controller_input_set_native_stream(false);
    assert(!probe_controller_input_commit_native_report(pending));
    emit(report); // Selected input continues updating while native USB is gated.
    assert(poll().active);
    expect_empty();
    probe_controller_input_set_native_stream(true);
    expect_empty();
    emit(report);
    const uint32_t resumed = expect_report(report);
    assert(resumed > pending);
    assert(probe_controller_input_commit_native_report(resumed));
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
    assert(!probe_controller_input_commit_native_report(last_serial));
    assert(!probe_controller_input_commit_native_report(0));
    probe_controller_input_set_native_stream(true);
    assert(expect_report(first) == first_serial);
    assert(expect_report(first) == first_serial);
    assert(probe_controller_input_commit_native_report(first_serial));
    assert(!probe_controller_input_commit_native_report(first_serial));

    const uint32_t second_serial = expect_report(repeated);
    assert(second_serial > first_serial);
    assert(probe_controller_input_commit_native_report(second_serial));
    const uint32_t third_serial = expect_report(repeated);
    assert(third_serial > second_serial);
    assert(!probe_controller_input_commit_native_report(second_serial));
    assert(expect_report(repeated) == third_serial);
    assert(probe_controller_input_commit_native_report(third_serial));
    assert(expect_report(last) == last_serial);
    assert(probe_controller_input_commit_native_report(last_serial));
    expect_empty();
    assert(!probe_controller_input_commit_native_report(last_serial));
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
    emit(unrelated, source_address, 0x2067); // Left Joy-Con at the same address.
    emit(unrelated, source_address, 0x2066, 5);
    emit(unrelated, source_address, 0x2066, 0xc0, 12);
    emit(unrelated, source_address, 0x2066, 8, 62);
    uint8_t oversized[64];
    memcpy(oversized, unrelated.data(), unrelated.size());
    oversized[63] = 0x5a;
    switch_pico_switch2_mouse_report(0x2066, source_address, 8, oversized,
                                    sizeof(oversized), static_cast<uint32_t>(now));
    disconnect(other_address);
    disconnect(source_address, 0x2067);
    const auto isolated = poll();
    assert(isolated.active && isolated.serial == selected.serial);
    assert(isolated.mouse_epoch == selected.mouse_epoch);
    assert(isolated.mouse_total_x == selected.mouse_total_x &&
           isolated.mouse_total_y == selected.mouse_total_y);
    assert(expect_report(first) == first_serial);
    ++now; emit(second);
    const uint32_t second_serial = poll().serial;
    assert(probe_controller_input_commit_native_report(first_serial));
    assert(expect_report(second) == second_serial);
    assert(probe_controller_input_commit_native_report(second_serial));
    expect_empty(); // Unrelated ring entries neither evict nor enter the FIFO.

    emit(first);
    const uint32_t disconnected_serial = expect_report(first);
    disconnect();
    ++now; emit(second); // Disconnect and reconnect both occur between polls.
    const auto reconnected = poll();
    assert(reconnected.active && reconnected.mouse_epoch != selected.mouse_epoch);
    assert(reconnected.mouse_total_x == 31 && reconnected.mouse_total_y == -37);
    assert(!probe_controller_input_commit_native_report(disconnected_serial));
    assert(expect_report(second) == reconnected.serial);
    assert(reconnected.serial > disconnected_serial);
    assert(probe_controller_input_commit_native_report(reconnected.serial));
    expect_empty();

    emit(first);
    const uint32_t pending = expect_report(first);
    disconnect();
    for (unsigned i = 0; i < 30; ++i) emit(unrelated, other_address);
    expect_inactive(poll());
    expect_empty();
    assert(!probe_controller_input_commit_native_report(pending));
    ++now; emit(second);
    const auto resumed = poll();
    assert(resumed.active && resumed.mouse_epoch != reconnected.mouse_epoch);
    const uint32_t resumed_serial = expect_report(second);
    assert(resumed_serial > pending);
    assert(probe_controller_input_commit_native_report(resumed_serial));

    emit(first);
    const uint32_t old_source = expect_report(first);
    emit(unrelated, other_address);
    switch2_mouse_capture_select_input(other_address);
    expect_empty();
    assert(!probe_controller_input_commit_native_report(old_source));
    probe_controller_input_set_native_stream(true);
    expect_empty(); // Selection cannot revive the other peer's raw history.
    emit(first);
    expect_empty();
    emit(unrelated, other_address);
    const uint32_t new_source = expect_report(unrelated);
    assert(new_source > old_source);
    switch2_mouse_capture_select_input(source_address);
    probe_controller_input_set_native_stream(true);
    expect_empty();
    assert(!probe_controller_input_commit_native_report(new_source));
    emit(second);
    const uint32_t restored = expect_report(second);
    assert(restored > new_source);
    assert(probe_controller_input_commit_native_report(restored));
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
    assert(!probe_controller_input_commit_native_report(old_serial));
    const uint32_t newest_serial = expect_report(newest);
    assert(newest_serial > old_serial);
    const auto following = native_report(0xbc, 40, 107, -109);
    ++now; emit(following);
    assert(expect_report(newest) == newest_serial);
    assert(probe_controller_input_commit_native_report(newest_serial));
    const uint32_t following_serial = expect_report(following);
    assert(following_serial > newest_serial);
    assert(probe_controller_input_commit_native_report(following_serial));
    expect_empty(); // Overflow discarded all prior history, not merely its head.
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
    assert(!probe_controller_input_commit_native_report(expired));

    ++now; emit(first);
    const uint32_t stale_head = expect_report(first);
    now += 499; emit(fresh);
    assert(expect_report(first) == stale_head);
    ++now;
    assert(poll().active); // Latest source is fresh, but its queued head is not.
    expect_empty();
    assert(!probe_controller_input_commit_native_report(stale_head));
    emit(fresh);
    const uint32_t resumed = expect_report(fresh);
    assert(resumed > stale_head);
    assert(probe_controller_input_commit_native_report(resumed));
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
    assert(!probe_controller_input_commit_native_report(wrapped));
    ++now; emit(fresh);
    assert(poll().active);
    const uint32_t after_wrap = expect_report(fresh);
    assert(after_wrap > wrapped);
    assert(probe_controller_input_commit_native_report(after_wrap));
    expect_empty();
}

int main() {
    test_startup_pairing_and_stream_gate();
    test_opaque_fidelity_order_and_retry();
    test_selected_source_isolation_and_reconnect();
    test_bounded_overflow();
    test_expiry_and_wrapping_clock();
    puts("Native packet fidelity, FIFO retry/order, source barriers, overflow, expiry and pairing passed");
}
