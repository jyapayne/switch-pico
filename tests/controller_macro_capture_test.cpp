#include "input/controller_macro_capture.h"
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

CaptureEvent captured_event(const ControllerMacroCapture& capture,
                            uint16_t index) {
    CaptureEvent event{};
    require(capture.event(index, &event), "recorded event was inaccessible");
    return event;
}

void require_terminal_frozen(ControllerMacroCapture& capture,
                             CaptureState expected, uint32_t elapsed) {
    const uint16_t count = capture.event_count();
    capture.stop(90000000);
    capture.disconnect(capture.slot(), capture.generation(), 90000001);
    capture.observe(capture.slot(), capture.generation(), 90000002,
                    controller_neutral_state());
    capture.tick(90000003);
    require(capture.state() == expected &&
                capture.elapsed_us(90000004) == elapsed &&
                capture.event_count() == count,
            "a terminal capture changed state, end time or event count");
}

void test_initial_snapshot_and_start_validation() {
    ControllerMacroCapture capture;
    CaptureEvent output{};
    require(capture.state() == CaptureState::kIdle &&
                capture.elapsed_us(1234) == 0 &&
                !capture.event(0, &output),
            "idle capture exposed an event or running clock");
    capture.stop(10);
    capture.disconnect(0, 0, 11);
    capture.tick(12);
    require(capture.state() == CaptureState::kIdle,
            "an operation without a run changed idle state");

    CaptureOptions options;
    options.channels = 31;
    ControllerState initial = controller_neutral_state();
    initial.button_south = true;
    initial.button_left_stick = true;
    initial.dpad_up = true;
    initial.left_stick_x = -768;
    initial.right_stick_y = 768;
    initial.left_trigger = 1536;
    initial.right_trigger = UINT16_MAX;
    require(capture.start(3, 77, options, 100, initial),
            "valid capture did not start");
    const uint32_t run = capture.run_id();
    const CaptureEvent first = captured_event(capture, 0);
    require(run != 0 && capture.slot() == 3 && capture.generation() == 77 &&
                capture.event_count() == 1 && first.at_us == 0 &&
                first.buttons == ((1u << 0) | (1u << 10) | (1u << 12)) &&
                first.left_x == -1024 && first.right_y == 1024 &&
                first.left_trigger == 2048 && first.right_trigger == UINT16_MAX,
            "capture did not retain the quantized initial state at time zero");
    require(!capture.start(0, 1, options, 200, controller_neutral_state()) &&
                capture.run_id() == run && capture.elapsed_us(180) == 80 &&
                captured_event(capture, 0).buttons == first.buttons,
            "start replaced an active capture");
    require(!capture.event(1, &output) && !capture.event(0, nullptr),
            "event accessor accepted an absent event or null output");
    capture.stop(180);

    const CaptureOptions invalid[] = {
        {0, 512, 1024, 8, 10000},
        {32, 512, 1024, 8, 10000},
        {1, 0, 1024, 8, 10000},
        {1, 32768, 1024, 8, 10000},
        {1, 512, 0, 8, 10000},
        {1, 512, 1024, 0, 10000},
        {1, 512, 1024, 129, 10000},
        {1, 512, 1024, 8, 0},
        {1, 512, 1024, 8, 80001},
    };
    for (const CaptureOptions& bad : invalid) {
        require(!capture.start(0, 1, bad, 200, initial),
                "invalid capture options were accepted");
    }
    require(!capture.start(4, 1, options, 200, initial) &&
                capture.run_id() == run &&
                capture.state() == CaptureState::kStopped &&
                capture.elapsed_us(200) == 80 &&
                captured_event(capture, 0).buttons == first.buttons,
            "rejected starts damaged the stopped recording");

    options.axis_quantum = INT16_MAX;
    options.trigger_quantum = UINT16_MAX;
    options.max_events = 128;
    options.max_duration_ms = 80000;
    require(capture.start(0, 1, options, 300, controller_neutral_state()) &&
                capture.run_id() == run + 1 && capture.event_count() == 1 &&
                captured_event(capture, 0).buttons == 0 &&
                capture.elapsed_us(300) == 0,
            "a successful restart did not replace the old run at valid bounds");
}

void test_positional_logical_buttons() {
    constexpr bool ControllerState::* buttons[] = {
        &ControllerState::button_south,
        &ControllerState::button_east,
        &ControllerState::button_west,
        &ControllerState::button_north,
        &ControllerState::button_left_shoulder,
        &ControllerState::button_right_shoulder,
        &ControllerState::button_select,
        &ControllerState::button_start,
        &ControllerState::button_system,
        &ControllerState::button_capture,
        &ControllerState::button_left_stick,
        &ControllerState::button_right_stick,
        &ControllerState::dpad_up,
        &ControllerState::dpad_down,
        &ControllerState::dpad_left,
        &ControllerState::dpad_right,
    };
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_events = 17;
    require(capture.start(0, 1, options, 0, controller_neutral_state()),
            "button capture did not start");
    for (uint16_t i = 0; i < 16; ++i) {
        ControllerState state = controller_neutral_state();
        state.*buttons[i] = true;
        capture.observe(0, 1, i + 1, state);
        const CaptureEvent event = captured_event(capture, i + 1);
        require(event.buttons == (1u << i) && event.at_us == i + 1u,
                "a positional button mapped to the wrong logical bit");
    }
}

ControllerState channel_state(uint8_t channels) {
    ControllerState state = controller_neutral_state();
    state.button_south = (channels & 1u) != 0;
    if ((channels & 2u) != 0) {
        state.left_stick_x = 1279;
        state.left_stick_y = -1279;
    }
    if ((channels & 4u) != 0) {
        state.right_stick_x = -1791;
        state.right_stick_y = 1791;
    }
    if ((channels & 8u) != 0) {
        state.left_trigger = 2047;
    }
    if ((channels & 16u) != 0) {
        state.right_trigger = 3071;
    }
    state.motion_sample_count = 1;
    state.motion_samples[0].gyro_x = 1000;
    return state;
}

void test_selected_channels_ignore_unrelated_changes() {
    for (uint8_t channel = 1; channel <= 16; channel <<= 1) {
        ControllerMacroCapture capture;
        CaptureOptions options;
        options.channels = channel;
        require(capture.start(0, 1, options, 10, controller_neutral_state()),
                "single-channel capture did not start");
        capture.observe(0, 1, 20, channel_state(31u ^ channel));
        require(capture.event_count() == 1,
                "unselected channels or motion produced a capture edge");
        capture.observe(0, 1, 30, channel_state(31));
        require(capture.event_count() == 2,
                "selected channel did not produce exactly one edge");
        const CaptureEvent event = captured_event(capture, 1);
        require(event.at_us == 20 &&
                    event.buttons == (channel == 1 ? 1 : 0) &&
                    event.left_x == (channel == 2 ? 1024 : 0) &&
                    event.left_y == (channel == 2 ? -1024 : 0) &&
                    event.right_x == (channel == 4 ? -1536 : 0) &&
                    event.right_y == (channel == 4 ? 1536 : 0) &&
                    event.left_trigger == (channel == 8 ? 2048 : 0) &&
                    event.right_trigger == (channel == 16 ? 3072 : 0),
                "selected output leaked or omitted a channel");
    }
}

void test_analog_quantization_symmetry_and_endpoints() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.channels = 30;
    ControllerState state = controller_neutral_state();
    state.left_stick_x = 255;
    state.left_stick_y = -255;
    state.left_trigger = 511;
    state.right_trigger = 511;
    require(capture.start(0, 1, options, 100, state),
            "analog capture did not start");
    CaptureEvent event = captured_event(capture, 0);
    require(event.left_x == 0 && event.left_y == 0 &&
                event.left_trigger == 0 && event.right_trigger == 0,
            "near-zero inputs did not quantize to rest");

    state.left_stick_x = 256;
    state.left_stick_y = -256;
    state.right_stick_x = -768;
    state.right_stick_y = 768;
    state.left_trigger = 512;
    state.right_trigger = 1536;
    capture.observe(0, 1, 101, state);
    event = captured_event(capture, 1);
    require(event.left_x == 512 && event.left_y == -512 &&
                event.right_x == -1024 && event.right_y == 1024 &&
                event.left_trigger == 1024 && event.right_trigger == 2048,
            "half-quantum ties did not round symmetrically away from zero");
    state.left_stick_x = 767;
    state.left_stick_y = -767;
    state.right_stick_x = -1279;
    state.right_stick_y = 1279;
    state.left_trigger = 1535;
    state.right_trigger = 2559;
    capture.observe(0, 1, 102, state);
    require(capture.event_count() == 2,
            "jitter within a quantized bucket produced another edge");

    state.left_stick_x = INT16_MIN;
    state.left_stick_y = INT16_MAX;
    state.right_stick_x = -INT16_MAX;
    state.right_stick_y = INT16_MAX - 1;
    state.left_trigger = 0;
    state.right_trigger = UINT16_MAX;
    capture.observe(0, 1, 103, state);
    event = captured_event(capture, 2);
    require(event.left_x == INT16_MIN && event.left_y == INT16_MAX &&
                event.right_x == -INT16_MAX && event.right_y == INT16_MAX &&
                event.left_trigger == 0 && event.right_trigger == UINT16_MAX,
            "quantization wrapped, lost full scale or broke signed symmetry");
    capture.observe(0, 1, 104, controller_neutral_state());
    event = captured_event(capture, 3);
    require(event.left_x == 0 && event.left_y == 0 && event.right_x == 0 &&
                event.right_y == 0 && event.left_trigger == 0 &&
                event.right_trigger == 0,
            "returning to exact rest retained a quantized analog value");
    capture.stop(105);

    options.axis_quantum = 3;
    options.trigger_quantum = 3;
    state.left_stick_x = 1;
    state.left_stick_y = -1;
    state.right_stick_x = 2;
    state.right_stick_y = -2;
    state.left_trigger = 1;
    state.right_trigger = 2;
    require(capture.start(0, 2, options, 200, state),
            "odd-quantum capture did not start");
    event = captured_event(capture, 0);
    require(event.left_x == 0 && event.left_y == 0 && event.right_x == 3 &&
                event.right_y == -3 && event.left_trigger == 0 &&
                event.right_trigger == 3,
            "odd quanta shifted the signed or unsigned rounding threshold");
}

void test_same_timestamp_coalesces_before_budget_check() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_events = 2;
    ControllerState state = controller_neutral_state();
    require(capture.start(0, 1, options, 10, state),
            "coalescing capture did not start");
    state.button_south = true;
    capture.observe(0, 1, 10, state);
    state.button_south = false;
    capture.observe(0, 1, 10, state);
    require(capture.event_count() == 1 &&
                captured_event(capture, 0).buttons == 0,
            "same-time initial changes made zero-length intermediate events");
    state.button_south = true;
    capture.observe(0, 1, 20, state);
    state.button_south = false;
    state.button_west = true;
    capture.observe(0, 1, 20, state);
    capture.observe(0, 1, 21, state);
    require(capture.state() == CaptureState::kRecording &&
                capture.event_count() == 2 &&
                captured_event(capture, 1).buttons == (1u << 2),
            "a full event budget rejected coalescing or an unchanged sample");
    state.button_west = false;
    state.button_north = true;
    capture.observe(0, 1, 22, state);
    require(capture.state() == CaptureState::kFull &&
                capture.elapsed_us(30) == 12 && capture.event_count() == 2 &&
                captured_event(capture, 1).at_us == 10 &&
                captured_event(capture, 1).buttons == (1u << 2),
            "overflow did not retain the last complete recorded state");
    require_terminal_frozen(capture, CaptureState::kFull, 12);
}

void test_full_static_capacity() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_events = 128;
    ControllerState state = controller_neutral_state();
    require(capture.start(0, 1, options, 0, state),
            "maximum-capacity capture did not start");
    for (uint16_t i = 1; i < 128; ++i) {
        state.button_south = (i & 1u) != 0;
        capture.observe(0, 1, i, state);
    }
    require(capture.event_count() == 128 &&
                capture.state() == CaptureState::kRecording,
            "capture stopped before an edge exceeded its full capacity");
    state.button_south = false;
    capture.observe(0, 1, 128, state);
    require(capture.state() == CaptureState::kFull &&
                capture.event_count() == 128 && capture.elapsed_us(129) == 128 &&
                captured_event(capture, 127).at_us == 127 &&
                captured_event(capture, 127).buttons == 1,
            "capacity overflow overwrote retained events or lost its end time");
}

void test_clock_wrap_and_stopped_end() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    ControllerState state = controller_neutral_state();
    constexpr uint32_t started = UINT32_MAX - 100u;
    require(capture.start(0, 1, options, started, state),
            "wrap capture did not start");
    state.button_east = true;
    capture.observe(0, 1, 49, state);
    require(captured_event(capture, 1).at_us == 150 &&
                capture.elapsed_us(49) == 150,
            "microsecond clock wrap changed elapsed event timing");
    capture.stop(149);
    require_terminal_frozen(capture, CaptureState::kStopped, 250);
}

void test_long_holds_and_exact_timeout() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_duration_ms = 80000;
    ControllerState held = controller_neutral_state();
    held.button_south = true;
    constexpr uint32_t start = 123;
    require(capture.start(0, 1, options, start, held),
            "long-hold capture did not start");
    capture.tick(start + 10000000);
    require(capture.event_count() == 2 &&
                captured_event(capture, 1).at_us == 10000000,
            "a live ten-second hold did not split at its boundary");
    capture.observe(0, 1, start + 15000000, held);
    require(capture.event_count() == 2,
            "unchanged observation added a spurious hold edge");
    capture.tick(start + 35000000);
    require(capture.event_count() == 4,
            "late tick omitted required ten-second hold boundaries");
    capture.tick(start + 90000000);
    require(capture.state() == CaptureState::kTimedOut &&
                capture.event_count() == 8 &&
                capture.elapsed_us(start + 90000000) == 80000000,
            "long hold did not clip at the exact duration and fit eight steps");
    for (uint16_t i = 0; i < 8; ++i) {
        const CaptureEvent event = captured_event(capture, i);
        require(event.at_us == i * 10000000u && event.buttons == 1,
                "hold split changed the captured state or interval length");
    }
    require_terminal_frozen(capture, CaptureState::kTimedOut, 80000000);

    options.max_duration_ms = 25000;
    require(capture.start(0, 1, options, 0, held),
            "partial-hold timeout capture did not start");
    capture.tick(40000000);
    require(capture.state() == CaptureState::kTimedOut &&
                capture.event_count() == 3 &&
                captured_event(capture, 2).at_us == 20000000 &&
                capture.elapsed_us(40000000) == 25000000,
            "timeout beyond a hold boundary lost the partial final interval");

    options.max_duration_ms = 10000;
    options.max_events = 1;
    require(capture.start(0, 1, options, 0, held),
            "single-step timeout capture did not start");
    capture.tick(10000000);
    require(capture.state() == CaptureState::kTimedOut &&
                capture.event_count() == 1 &&
                capture.elapsed_us(20000000) == 10000000,
            "an exact-duration hold created a zero-length edge or false full");
}

void test_hold_splits_follow_edges_and_coalesce() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_events = 3;
    options.max_duration_ms = 80000;
    ControllerState state = controller_neutral_state();
    state.button_south = true;
    require(capture.start(0, 1, options, 0, state),
            "edge-relative hold capture did not start");
    state.button_south = false;
    state.button_east = true;
    capture.observe(0, 1, 5000000, state);
    capture.tick(15000000);
    state.button_east = false;
    state.button_west = true;
    capture.observe(0, 1, 15000000, state);
    require(capture.state() == CaptureState::kRecording &&
                capture.event_count() == 3 &&
                captured_event(capture, 1).at_us == 5000000 &&
                captured_event(capture, 2).at_us == 15000000 &&
                captured_event(capture, 2).buttons == (1u << 2),
            "hold boundary did not follow or coalesce with a real edge");
    capture.stop(25000000);
    require(capture.state() == CaptureState::kStopped &&
                capture.event_count() == 3 &&
                capture.elapsed_us(30000000) == 25000000,
            "stop at the final hold boundary added an unnecessary event");
}

void test_hold_budget_exhaustion_and_stop_catchup() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_events = 2;
    options.max_duration_ms = 80000;
    ControllerState state = controller_neutral_state();
    state.button_south = true;
    require(capture.start(0, 1, options, 0, state),
            "bounded hold capture did not start");
    capture.tick(UINT32_MAX);
    require(capture.state() == CaptureState::kFull &&
                capture.elapsed_us(UINT32_MAX) == 20000000 &&
                capture.event_count() == 2 &&
                captured_event(capture, 1).at_us == 10000000 &&
                captured_event(capture, 1).buttons == 1,
            "large tick did not stop full at the first missing hold boundary");
    require_terminal_frozen(capture, CaptureState::kFull, 20000000);

    options.max_events = 4;
    require(capture.start(0, 1, options, 0, state),
            "stop-catchup capture did not start");
    capture.stop(35000000);
    require(capture.state() == CaptureState::kStopped &&
                capture.event_count() == 4 &&
                captured_event(capture, 3).at_us == 30000000 &&
                capture.elapsed_us(50000000) == 35000000,
            "stop without intervening ticks omitted long-hold splits");
}

void test_generation_replacement_disconnect_and_slot_isolation() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    ControllerState initial = controller_neutral_state();
    initial.button_south = true;
    ControllerState replacement = controller_neutral_state();
    replacement.button_east = true;
    require(capture.start(2, 7, options, 100, initial),
            "generation capture did not start");
    capture.observe(3, 99, 150, replacement);
    capture.disconnect(3, 7, 170);
    capture.disconnect(2, 6, 180);
    require(capture.state() == CaptureState::kRecording &&
                capture.event_count() == 1,
            "another slot or stale disconnect interrupted capture");
    capture.observe(2, 8, 200, replacement);
    require(capture.state() == CaptureState::kDisconnected &&
                capture.generation() == 7 && capture.event_count() == 1 &&
                captured_event(capture, 0).buttons == 1 &&
                capture.elapsed_us(250) == 100,
            "generation replacement recorded the new device or lost the end");
    require_terminal_frozen(capture, CaptureState::kDisconnected, 100);

    require(capture.start(2, 8, options, 300, replacement),
            "replacement generation could not start a new capture");
    capture.disconnect(2, 7, 400);
    require(capture.state() == CaptureState::kRecording,
            "old-generation disconnect ended a replacement run");
    capture.disconnect(2, 8, 450);
    require_terminal_frozen(capture, CaptureState::kDisconnected, 150);
}

void test_deadline_precedes_late_edges_and_terminal_requests() {
    ControllerMacroCapture capture;
    CaptureOptions options;
    options.max_duration_ms = 1;
    ControllerState state = controller_neutral_state();
    require(capture.start(0, 1, options, 0, state),
            "deadline capture did not start");
    state.button_south = true;
    capture.observe(0, 1, 1000, state);
    require(capture.state() == CaptureState::kTimedOut &&
                capture.event_count() == 1 &&
                captured_event(capture, 0).buttons == 0 &&
                capture.elapsed_us(2000) == 1000,
            "an edge at the deadline escaped timeout clipping");
    require(capture.start(0, 1, options, 0, state),
            "late-stop capture did not start");
    capture.stop(2000);
    require_terminal_frozen(capture, CaptureState::kTimedOut, 1000);
    require(capture.start(0, 1, options, 0, state),
            "late-disconnect capture did not start");
    capture.disconnect(0, 1, 2000);
    require_terminal_frozen(capture, CaptureState::kTimedOut, 1000);
}

}  // namespace

int main() {
    test_initial_snapshot_and_start_validation();
    test_positional_logical_buttons();
    test_selected_channels_ignore_unrelated_changes();
    test_analog_quantization_symmetry_and_endpoints();
    test_same_timestamp_coalesces_before_budget_check();
    test_full_static_capacity();
    test_clock_wrap_and_stopped_end();
    test_long_holds_and_exact_timeout();
    test_hold_splits_follow_edges_and_coalesce();
    test_hold_budget_exhaustion_and_stop_catchup();
    test_generation_replacement_disconnect_and_slot_isolation();
    test_deadline_precedes_late_edges_and_terminal_requests();
    return 0;
}
