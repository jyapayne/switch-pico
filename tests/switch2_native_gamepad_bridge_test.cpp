#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "controller_input.h"
#include "input/bluepad32_input_backend.h"
#include "model.h"
#include "pico/stdlib.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "profile/controller_profile_runtime.h"

namespace {
uint64_t now_us = 1000000;
uint32_t stage;
Bluepad32NativeGamepadSnapshot source;
ControllerProfile profile;
bool alternating_shortcut;
bool shortcut_phase;
probe_controller_input controls[2];
uint8_t reports[2][63];
}

uint32_t time_us_32() { return static_cast<uint32_t>(now_us); }
absolute_time_t get_absolute_time() { return now_us; }
uint32_t to_ms_since_boot(absolute_time_t time) { return static_cast<uint32_t>(time / 1000); }
void system_clock_initialize() {}
extern "C" int probe_debug_printf(const char*, ...) { return 0; }
BootselPairingButtonEvent bootsel_pairing_button_task() { return BootselPairingButtonEvent::kNone; }
void bluepad32_input_backend_init() { stage = 1; }
void bluepad32_input_backend_start() { stage = 2; }
void bluepad32_input_backend_poll() {}
void bluepad32_input_backend_diagnostics(Bluepad32BackendDiagnostics* out) { *out = {}; out->initialization_stage = stage; }
void bluepad32_input_backend_open_pairing_window() {}
void bluepad32_input_backend_select_native_source(const uint8_t*) {}
void bluepad32_input_backend_native_snapshot(Bluepad32NativeGamepadSnapshot* out) { *out = source; }
bool bluepad32_input_backend_native_sample_request(uint8_t, uint8_t, uint64_t*) { return false; }
int bluepad32_input_backend_native_sample_result(uint8_t, uint64_t) { return -1; }
void bluepad32_input_backend_native_sample_cancel(uint8_t) {}
void bluepad32_input_backend_queue_profile_feedback(uint8_t, uint32_t, uint8_t, ControllerProfileConfirmationPolicy) {}
void controller_profile_runtime_reset() { profile = controller_profile_default(controller_identity_global(), 0); }
bool controller_profile_runtime_take_initial_profile_indication(uint8_t, ControllerProfileRuntimeProfileChangeEvent*) { return false; }
bool controller_profile_runtime_take_profile_change(uint8_t, ControllerProfileRuntimeProfileChangeEvent*) { return false; }
ControllerProfileTransformResult controller_profile_runtime_transform(
    uint8_t, const Bluepad32SlotSnapshot& input, uint32_t, AdapterUsbMode) {
    if (!input.active) return {};
    auto result = controller_profile_transform(input.state, profile);
    if (alternating_shortcut) {
        // Model a runtime synthetic transition spanning the two halves. Two
        // evaluations for one paired report would expose contradictory states.
        shortcut_phase = !shortcut_phase;
        result.state.button_system = result.state.button_capture = shortcut_phase;
    }
    return result;
}

namespace {
uint32_t now_ms() { return to_ms_since_boot(now_us); }
void put_pair(uint8_t* out, uint16_t x, uint16_t y) {
    out[0] = static_cast<uint8_t>(x);
    out[1] = static_cast<uint8_t>((x >> 8) | (y << 4));
    out[2] = static_cast<uint8_t>(y >> 4);
}
void calibrate(uint8_t instance, uint16_t x, uint16_t y, uint16_t px, uint16_t py, uint16_t nx, uint16_t ny) {
    uint8_t record[9];
    put_pair(record, x, y);
    put_pair(record + 3, px, py);
    put_pair(record + 6, nx, ny);
    probe_controller_input_set_full_stick_calibration(instance, record);
}
uint16_t stick_x(uint8_t instance) { return reports[instance][5] | ((reports[instance][6] & 15u) << 8); }
uint16_t stick_y(uint8_t instance) { return (reports[instance][6] >> 4) | (reports[instance][7] << 4); }
uint8_t imu_length(uint8_t instance) { return reports[instance][probe_model_imu_length_offset(instance)]; }
uint32_t bits(const uint8_t* bytes, unsigned offset, unsigned count) {
    uint32_t value = 0;
    for (unsigned i = 0; i < count; ++i) value |= uint32_t((bytes[(offset + i) / 8] >> ((offset + i) % 8)) & 1) << i;
    return value;
}
void quaternion(uint8_t instance, double out[4]) {
    const uint8_t* imu = reports[instance] + probe_model_imu_data_offset(instance);
    assert(imu_length(instance) == 30);
    const unsigned largest = bits(imu, 32, 3);
    assert(largest < 4);
    double ratios[3], norm = 1;
    for (unsigned i = 0; i < 3; ++i) {
        ratios[i] = bits(imu, 35 + 31 * i, 31) / 1073741824.0 - 1;
        norm += ratios[i] * ratios[i];
    }
    out[largest] = 1 / sqrt(norm);
    for (unsigned i = 0; i < 3; ++i) out[(largest + i + 1) & 3] = ratios[i] * out[largest];
}
void publish(bool motion = true) {
    now_us += 4000;
    source.received_us = time_us_32();
    ++source.state_generation;
    if (motion) {
        source.accel_received_us = source.gyro_received_us = time_us_32();
        ++source.accel_sequence;
        ++source.gyro_sequence;
    }
}
uint32_t peek(uint8_t instance) {
    probe_controller_input_poll(instance, now_ms(), &controls[instance]);
    return probe_controller_input_peek_native_report(instance, now_ms(), reports[instance]);
}
void consume(uint8_t instance) {
    const uint32_t token = peek(instance);
    assert(token && probe_controller_input_commit_native_report(instance, token));
}
void pair() { consume(0); consume(1); }
void no_mouse_or_rails() {
    for (unsigned i = 0; i < 2; ++i) {
        assert((reports[i][3] & 0xc0) == 0);
        assert(reports[i][9] == 0 && reports[i][10] == 0 && reports[i][11] == 0 && reports[i][12] == 0);
        assert(reports[i][13] == 0xff);
    }
}

void mapped_halves_and_calibration() {
    source.slot = 2;
    source.controller.active = true;
    source.controller.connection_generation = 7;
    source.controller.identity = controller_identity_global();
    publish();
    // Neither an absent source nor an uncalibrated child masquerades as active.
    assert(!peek(0) && !controls[0].active);
    calibrate(0, 2000, 2100, 1500, 1400, 1600, 1700);
    assert(peek(0));
    assert(!peek(1) && !controls[1].active);
    calibrate(1, 1800, 1900, 1700, 1800, 1400, 1500);
    pair();
    assert(stick_x(0) == 2000 && stick_y(0) == 2100);
    assert(stick_x(1) == 1800 && stick_y(1) == 1900);
    // Actual profile transforms can move controls across native children.
    profile.button_map[static_cast<unsigned>(ControllerProfileLogicalButton::kSouth)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kDpadRight);
    profile.button_map[static_cast<unsigned>(ControllerProfileLogicalButton::kDpadLeft)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kEast);
    profile.triggers[0].digital_threshold = 20000;
    profile.triggers[1].digital_threshold = 30000;
    ControllerState& state = source.controller.state;
    state.button_south = state.dpad_left = true;
    state.button_left_shoulder = state.button_right_shoulder = true;
    state.button_select = state.button_start = true;
    state.button_left_stick = state.button_right_stick = true;
    state.button_system = state.button_capture = true;
    state.left_trigger = 19999;
    state.right_trigger = 30000;
    state.right_stick_x = INT16_MAX;
    state.left_stick_y = INT16_MIN;
    publish(); pair();
    assert(reports[0][2] == 0xf2 && reports[1][2] == 0xd2);
    assert(reports[0][3] == 1 && reports[1][3] == 1);
    assert(stick_x(0) == 3500 && stick_y(0) == 2100);
    assert(stick_x(1) == 1800 && stick_y(1) == 3700);
    no_mouse_or_rails();
    state = {};
    state.button_west = state.button_north = true;
    state.dpad_up = state.dpad_down = true;
    state.left_trigger = 20000;
    state.right_stick_x = INT16_MIN;
    state.left_stick_y = INT16_MAX;
    publish(); pair();
    assert(reports[0][2] == 0x0c && reports[1][2] == 0x29);
    assert(stick_x(0) == 400 && stick_y(1) == 400);
    // A malformed calibration may not spill a 12-bit axis into its neighbor.
    const uint32_t left_pending = peek(1);
    calibrate(0, 2000, 2100, 3000, 1400, 1600, 1700);
    assert(!peek(0));
    assert(probe_controller_input_commit_native_report(1, left_pending));
    calibrate(0, 2000, 2100, 1500, 1400, 1600, 1700);
    state = {};
    profile = controller_profile_default(controller_identity_global(), 0);
    alternating_shortcut = true;
    for (unsigned i = 0; i < 4; ++i) {
        publish(); pair();
        assert(reports[0][3] == reports[1][3]);
    }
    alternating_shortcut = false;
}

void independent_backpressure_and_resets() {
    publish();
    const uint32_t blocked_left = peek(1);
    const uint32_t right = peek(0);
    uint8_t saved[63]; memcpy(saved, reports[0], sizeof(saved));
    assert(peek(0) == right && memcmp(saved, reports[0], sizeof(saved)) == 0);
    assert(!probe_controller_input_commit_native_report(1, right));
    assert(probe_controller_input_commit_native_report(0, right));
    assert(!probe_controller_input_commit_native_report(0, right));
    assert(probe_controller_input_commit_native_report(1, blocked_left));
    publish();
    const uint32_t obsolete = peek(1);
    for (unsigned i = 0; i < 40; ++i) {
        source.controller.state.dpad_down = (i & 1) != 0;
        source.controller.state.button_east = (i & 1) != 0;
        publish(); consume(0);
    }
    const uint32_t latest = peek(1);
    assert(latest != obsolete && reports[1][2] == 1);
    assert(!probe_controller_input_commit_native_report(1, obsolete));
    probe_controller_input_set_native_stream(0, false);
    assert(!peek(0));
    assert(probe_controller_input_commit_native_report(1, latest));
    probe_controller_input_set_native_stream(0, true);
    const uint32_t right_pending = peek(0);
    probe_controller_input_set_native_stream(1, false);
    assert(probe_controller_input_commit_native_report(0, right_pending));
    probe_controller_input_set_native_stream(1, true);
    source.controller.state = {};
}

void real_motion_admission_and_loss() {
    source.accel_valid = source.gyro_valid = true;
    source.accel_q13[1] = 8192; // SDL face-up gravity -> native +Z, no mouse mounting.
    // These values have already passed the DS5 factory-calibration path.
    // Even a controller rotating at connection must not wait for stationary bias estimation.
    source.gyro_q10[0] = 0;
    source.gyro_q10[1] = 90 * 1024;
    source.gyro_q10[2] = 0;
    publish(); pair();
    assert(imu_length(0) == 30 && imu_length(1) == 30);
    source.gyro_q10[1] = 0;
    publish(); pair();
    // Polling and fresh button packets cannot create additional IMU samples.
    for (unsigned i = 0; i < 420; ++i) {
        publish(false); pair();
        assert(controls[0].active && controls[1].active);
        assert(imu_length(0) == 0 && imu_length(1) == 0);
    }
    publish(); pair(); // Fresh factory-calibrated data recovers without another settling delay.
    assert(imu_length(0) == 30 && imu_length(1) == 30);
    const uint8_t* right_imu = reports[0] + probe_model_imu_data_offset(0);
    const uint8_t* left_imu = reports[1] + probe_model_imu_data_offset(1);
    assert(memcmp(right_imu, left_imu, 30) == 0);
    assert(bits(right_imu, 128, 32) == 0 && bits(right_imu, 160, 32) == 0);
    assert(bits(right_imu, 192, 32) == (1u << 28));
    double initial[4]; quaternion(0, initial);
    // A new controls packet with no new IMU cannot emit the old sample again.
    source.controller.state.button_east = true;
    publish(false); pair();
    assert(reports[0][2] == 2 && imu_length(0) == 0 && imu_length(1) == 0);
    // A blocked child's motion is not consumed by the other child's endpoint.
    publish(); consume(0);
    const uint32_t left_pending = peek(1);
    assert(imu_length(1) == 30);
    consume(0); assert(imu_length(0) == 0);
    probe_controller_input_set_native_stream(0, false);
    assert(probe_controller_input_commit_native_report(1, left_pending));
    probe_controller_input_set_native_stream(0, true);
    publish(); pair();
    assert(imu_length(0) == 30 && imu_length(1) == 30); // No shared recalibration on USB reset.
    // One second of genuine 90dps yaw advances the same rigid orientation once,
    // not twice because two virtual endpoints happen to consume it.
    source.gyro_q10[1] += 90 * 1024;
    for (unsigned i = 0; i < 250; ++i) { publish(); pair(); }
    double turned[4]; quaternion(0, turned);
    double dot = 0;
    for (unsigned i = 0; i < 4; ++i) dot += initial[i] * turned[i];
    assert(fabs(fabs(dot) - sqrt(.5)) < .015);
    quaternion(1, initial);
    for (unsigned i = 0; i < 4; ++i) assert(fabs(initial[i] - turned[i]) < 1e-8);
    source.gyro_q10[1] -= 90 * 1024;
    publish();
    const uint32_t obsolete = peek(0);
    source.accel_valid = source.gyro_valid = false;
    assert(!probe_controller_input_commit_native_report(0, obsolete));
    publish(false); pair();
    assert(controls[0].active && reports[0][2] == 2 && imu_length(0) == 0 && imu_length(1) == 0);
    source.accel_valid = source.gyro_valid = true;
    publish(); pair();
    assert(imu_length(0) == 30);
    for (unsigned i = 0; i < 38; ++i) { publish(false); pair(); }
    assert(controls[0].active && reports[0][2] == 2 && imu_length(0) == 0 && imu_length(1) == 0);
    publish();
    const uint32_t old_right = peek(0), old_left = peek(1);
    // Even a reconnect whose teardown was missed retires both USB identities.
    ++source.controller.connection_generation;
    source.accel_valid = source.gyro_valid = false; // New epochs require fresh reports.
    assert(!probe_controller_input_commit_native_report(0, old_right));
    assert(!probe_controller_input_commit_native_report(1, old_left));
    pair();
    assert(controls[0].active && imu_length(0) == 0 && imu_length(1) == 0);
    source.controller.active = false;
    memset(reports[0], 0x5a, 63);
    assert(!peek(0) && !controls[0].active);
    for (uint8_t byte : reports[0]) assert(byte == 0x5a);
    assert(!peek(1) && !controls[1].active);
    source.controller.active = true;
    ++source.controller.connection_generation;
    publish(); pair();
    now_us += 500000;
    assert(!peek(0) && !peek(1));
    assert(!controls[0].active && !controls[1].active);
}
void selected_motion_target_keeps_both_control_halves() {
    source = {};
    source.slot = 0;
    source.controller.active = true;
    source.controller.connection_generation = 99;
    source.controller.state.button_south = true;
    source.controller.state.dpad_up = true;
    source.accel_valid = source.gyro_valid = true;
    source.accel_q13[1] = 8192;
    profile = controller_profile_default(controller_identity_global(), 0);
    calibrate(0, 2048, 2048, 2047, 2047, 2048, 2048);
    calibrate(1, 2048, 2048, 2047, 2047, 2048, 2048);
    publish(); pair();
    for (uint8_t instance = 0; instance < 2; ++instance) {
        assert(controls[instance].active);
        const bool enabled = (SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) != 0;
        assert(imu_length(instance) == (enabled ? 30 : 0));
    }
    assert(reports[0][2] == 0x01 && reports[1][2] == 0x08);
    source.gyro_q10[1] = 90 * 1024;
    publish(); pair();
    assert(imu_length(0) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & 1) ? 30 : 0));
    assert(imu_length(1) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & 2) ? 30 : 0));
    no_mouse_or_rails();
}

void wii_bias_and_independent_sensor_freshness() {
    ++source.controller.connection_generation;
    source.track_stationary_bias = true;
    source.gyro_q10[1] = 2 * 1024;
    publish(); pair();
    assert(controls[0].active && controls[1].active);
    assert(reports[0][2] == 0x01 && reports[1][2] == 0x08);
    for (uint8_t instance = 0; instance < 2; ++instance)
        assert(imu_length(instance) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) ? 30 : 0));
    for (unsigned i = 0; i < 400; ++i) { publish(); pair(); }
    for (uint8_t instance = 0; instance < 2; ++instance) {
        assert(imu_length(instance) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) ? 30 : 0));
    }
    // Accelerometer-only reports must not refresh a stalled MotionPlus stream.
    for (unsigned i = 0; i < 38; ++i) {
        publish(false);
        source.accel_received_us = time_us_32();
        ++source.accel_sequence;
        pair();
    }
    assert(controls[0].active && controls[1].active);
    assert(imu_length(0) == 0 && imu_length(1) == 0);
    source.gyro_valid = false;
    publish(); pair();
    assert(reports[0][2] == 0x01 && reports[1][2] == 0x08);
    assert(imu_length(0) == 0 && imu_length(1) == 0);
    // Fresh Wii sensors recover immediately, without borrowing the old bias.
    source.gyro_valid = true;
    publish(); pair();
    for (uint8_t instance = 0; instance < 2; ++instance)
        assert(imu_length(instance) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) ? 30 : 0));
    // A factory-calibrated source switching policy must initialize immediately.
    source.track_stationary_bias = false;
    publish(); pair();
    for (uint8_t instance = 0; instance < 2; ++instance) {
        assert(imu_length(instance) == ((SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) ? 30 : 0));
    }
}

void nunchuk_buttons_map_to_native_left_shoulders() {
    ++source.controller.connection_generation;
    source.controller.state = {};
    source.accel_valid = source.gyro_valid = false;
    profile = controller_profile_default(controller_identity_global(), 0);
    // The real Wii parser maps Nunchuk C to west and Z to north. These are
    // ordinary profile inputs, not the unrelated Switch2 extra "C" control.
    profile.button_map[static_cast<unsigned>(ControllerProfileLogicalButton::kWest)] =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.button_map[static_cast<unsigned>(ControllerProfileLogicalButton::kNorth)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kLeftShoulder);
    source.controller.state.button_west = true; // C -> ZL.
    publish(false); pair();
    assert(reports[0][2] == 0 && reports[1][2] == 0x20);
    source.controller.state.button_west = false;
    source.controller.state.button_north = true; // Z -> L.
    publish(false); pair();
    assert(reports[0][2] == 0 && reports[1][2] == 0x10);
    source.controller.state.button_right_shoulder = true; // Remote 2 -> R.
    publish(false); pair();
    assert(reports[0][2] == 0x10 && reports[1][2] == 0x10); // Real L+R across the pair.
    source.controller.state.button_west = true;
    publish(false); pair();
    assert(reports[0][2] == 0x10 && reports[1][2] == 0x30);
    source.controller.state = {};
    publish(false); pair();
    assert(reports[0][2] == 0 && reports[1][2] == 0); // No sticky synthetic chord.
    no_mouse_or_rails();
}

} // namespace

int main() {
    assert(!probe_controller_input_peek_native_report(0, now_ms(), reports[0]));
    probe_controller_input_init();
    assert(probe_controller_input_start());
    probe_controller_input_set_native_stream(0, true);
    probe_controller_input_set_native_stream(1, true);
    assert(!peek(0) && !peek(1));
    mapped_halves_and_calibration();
    independent_backpressure_and_resets();
    if (SWITCH2_BRIDGE_IMU_TARGET_MASK == 3) real_motion_admission_and_loss();
    selected_motion_target_keeps_both_control_halves();
    wii_bias_and_independent_sensor_freshness();
    nunchuk_buttons_map_to_native_left_shoulders();
    return 0;
}
