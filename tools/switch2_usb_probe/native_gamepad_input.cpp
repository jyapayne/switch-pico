#include "native_gamepad_input.h"

#if SWITCH2_BRIDGE_FULL_INPUT
#include <limits.h>
#include <string.h>

#include "input/bluepad32_input_backend.h"
#include "model.h"
#include "native_imu.h"
#include "pico/time.h"
#include "profile/controller_profile_runtime.h"

#if !SWITCH2_PROBE_HUB || SWITCH2_BRIDGE_WII_INPUT
#error "A full gamepad source requires the native R/L USB hub"
#endif
static_assert(PROBE_CONTROLLER_COUNT == 2);
extern "C" int probe_debug_printf(const char* format, ...);
#ifndef SWITCH2_BRIDGE_IMU_TARGET_MASK
#define SWITCH2_BRIDGE_IMU_TARGET_MASK 3
#endif
static_assert(SWITCH2_BRIDGE_IMU_TARGET_MASK >= 1 && SWITCH2_BRIDGE_IMU_TARGET_MASK <= 3);

namespace {
constexpr uint32_t kInputDeadlineUs = 500000;
constexpr uint32_t kSensorDeadlineUs = 150000;
constexpr uint32_t kOutputDeadlineUs = 100000;
#if !SWITCH2_BRIDGE_SOURCE_AUTO
constexpr uint8_t kSourceAddress[] = {SWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES};
static_assert(sizeof(kSourceAddress) == 6);
#endif

struct Child {
    bool enabled = false;
    bool calibrated = false;
    uint16_t center[2]{};
    uint16_t positive[2]{};
    uint16_t negative[2]{};
    probe_controller_input input{};
    uint8_t counter = 0;
    uint32_t pending_token = 0;
    uint32_t pending_us = 0;
    uint32_t pending_ticks = 0;
    uint32_t pending_accel_sequence = 0;
    uint32_t pending_gyro_sequence = 0;
    bool pending_motion = false;
    uint8_t pending_report[63]{};
    bool have_committed_motion = false;
    uint32_t committed_accel_sequence = 0;
    uint32_t committed_gyro_sequence = 0;
    uint32_t committed_ticks = 0;
};
Child g_children[PROBE_CONTROLLER_COUNT];
Bluepad32NativeGamepadSnapshot g_source;
ControllerProfileTransformResult g_mapped;
ProbeNativeMotion g_motion;
bool g_active;
bool g_evaluated;
uint32_t g_evaluated_ms;
uint32_t g_report_token;
int g_sensor_status = -1;
bool g_clock_started;
uint32_t g_clock_us;
uint32_t g_clock_ticks;
uint32_t g_clock_fraction;

void advance_clock(uint32_t now_us) {
    if (!g_clock_started) {
        g_clock_started = true;
        g_clock_us = now_us;
        return;
    }
    const uint64_t scaled = static_cast<uint64_t>(now_us - g_clock_us) * 960u + g_clock_fraction;
    g_clock_us = now_us;
    g_clock_ticks += static_cast<uint32_t>(scaled / 1000000u);
    g_clock_fraction = static_cast<uint32_t>(scaled % 1000000u);
}

void discard_output(Child& child) {
    child.pending_token = 0;
    child.have_committed_motion = false;
}

bool sensors_fresh(const Bluepad32NativeGamepadSnapshot& source, uint32_t now_us) {
    return source.accel_valid && source.gyro_valid &&
        now_us - source.accel_received_us < kSensorDeadlineUs &&
        now_us - source.gyro_received_us < kSensorDeadlineUs;
}

void unpack_stick_pair(const uint8_t* bytes, uint16_t pair[2]) {
    pair[0] = bytes[0] | (static_cast<uint16_t>(bytes[1] & 15) << 8);
    pair[1] = (bytes[1] >> 4) | (static_cast<uint16_t>(bytes[2]) << 4);
}

uint16_t calibrated_axis(const Child& child, int16_t value, unsigned axis, bool invert) {
    // Same signed endpoint/rounding convention as the native Wii adapter.
    const int32_t input = value;
    const bool input_positive = input >= 0;
    const bool output_positive = input_positive != invert;
    const int32_t magnitude = input_positive ? input : -input;
    const int32_t denominator = input_positive ? INT16_MAX : 32768;
    const int32_t travel = output_positive ? child.positive[axis] : child.negative[axis];
    const int32_t displacement = (magnitude * travel + denominator / 2) / denominator;
    return static_cast<uint16_t>(child.center[axis] +
        (output_positive ? displacement : -displacement));
}

void pack_controls(uint8_t instance) {
    Child& child = g_children[instance];
    child.input = {};
    child.input.serial = g_source.state_generation;
    if (!g_active || !child.calibrated) return;
    child.input.active = true;
    child.input.native_status = 0x30; // Host feature status is gated per model in main.
    child.input.mouse_surface = 0xff; // No optical sensor, clicks, or invented movement.
    const ControllerState& state = g_mapped.state;
    const bool left = probe_model_is_left(instance);
    if (left) {
        child.input.buttons[0] = static_cast<uint8_t>(
            (state.dpad_down ? 0x01 : 0) | (state.dpad_right ? 0x02 : 0) |
            (state.dpad_left ? 0x04 : 0) | (state.dpad_up ? 0x08 : 0) |
            (state.button_left_shoulder ? 0x10 : 0) |
            (state.left_trigger != 0 && state.left_trigger >= g_mapped.left_trigger_digital_threshold ? 0x20 : 0) |
            (state.button_select ? 0x40 : 0) | (state.button_left_stick ? 0x80 : 0));
        child.input.buttons[1] = static_cast<uint8_t>(
            (state.button_capture ? 0x01 : 0) |
            ((state.extra_buttons & (1u << 3)) ? 0x80 : 0) |
            ((state.extra_buttons & (1u << 4)) ? 0x40 : 0));
    } else {
        child.input.buttons[0] = static_cast<uint8_t>(
            (state.button_south ? 0x01 : 0) | (state.button_east ? 0x02 : 0) |
            (state.button_west ? 0x04 : 0) | (state.button_north ? 0x08 : 0) |
            (state.button_right_shoulder ? 0x10 : 0) |
            (state.right_trigger != 0 && state.right_trigger >= g_mapped.right_trigger_digital_threshold ? 0x20 : 0) |
            (state.button_start ? 0x40 : 0) | (state.button_right_stick ? 0x80 : 0));
        child.input.buttons[1] = static_cast<uint8_t>(
            (state.button_system ? 0x01 : 0) | ((state.extra_buttons & 1) ? 0x10 : 0) |
            ((state.extra_buttons & (1u << 5)) ? 0x80 : 0) |
            ((state.extra_buttons & (1u << 6)) ? 0x40 : 0));
    }
    const uint16_t x = calibrated_axis(child, left ? state.left_stick_x : state.right_stick_x, 0, false);
    const uint16_t y = calibrated_axis(child, left ? state.left_stick_y : state.right_stick_y, 1, true);
    child.input.stick[0] = static_cast<uint8_t>(x);
    child.input.stick[1] = static_cast<uint8_t>((x >> 8) | (y << 4));
    child.input.stick[2] = static_cast<uint8_t>(y >> 4);
}

void lose_source(uint32_t now_ms) {
    if (g_active) {
        Bluepad32SlotSnapshot inactive{};
        (void)controller_profile_runtime_transform(g_source.slot, inactive, now_ms, AdapterUsbMode::kSwitch);
        g_motion.reset();
        for (uint8_t i = 0; i < PROBE_CONTROLLER_COUNT; ++i) {
            discard_output(g_children[i]);
            bluepad32_input_backend_native_sample_cancel(i);
        }
        g_sensor_status = -1;
    }
    g_active = false;
    for (Child& child : g_children) child.input = {};
}

void refresh(uint32_t now_ms) {
    Bluepad32NativeGamepadSnapshot source;
    bluepad32_input_backend_native_snapshot(&source);
    // Snapshot first: source receipt timestamps must not be ahead of this clock.
    const uint32_t now_us = time_us_32();
    advance_clock(now_us);
    if (!source.controller.active || source.slot >= BLUEPAD32_INPUT_BACKEND_SLOT_COUNT ||
        now_us - source.received_us >= kInputDeadlineUs) {
        lose_source(now_ms);
        g_source = source;
        g_evaluated = false;
        return;
    }
    const bool changed_connection = !g_active || source.slot != g_source.slot ||
        source.controller.connection_generation != g_source.controller.connection_generation;
    // Both polls and both peeks in a paired output round share one profile and
    // motion evaluation. A real publication in the same millisecond still wins.
    if (!changed_connection && g_evaluated && g_evaluated_ms == now_ms &&
        source.state_generation == g_source.state_generation && source.received_us == g_source.received_us &&
        source.accel_sequence == g_source.accel_sequence && source.gyro_sequence == g_source.gyro_sequence &&
        source.accel_received_us == g_source.accel_received_us && source.gyro_received_us == g_source.gyro_received_us &&
        source.accel_valid == g_source.accel_valid && source.gyro_valid == g_source.gyro_valid &&
        source.track_stationary_bias == g_source.track_stationary_bias) return;
    if (changed_connection) {
        lose_source(now_ms);
        g_motion.reset();
        for (Child& child : g_children) discard_output(child);
        probe_debug_printf("[PROBE] Native gamepad source active in slot %u\n", source.slot);
    }
    g_source = source;
    g_active = true;
    g_evaluated = true;
    g_evaluated_ms = now_ms;
    g_mapped = controller_profile_runtime_transform(source.slot, source.controller, now_ms, AdapterUsbMode::kSwitch);
    ControllerProfileRuntimeProfileChangeEvent feedback{};
    if (controller_profile_runtime_take_initial_profile_indication(source.slot, &feedback) ||
        controller_profile_runtime_take_profile_change(source.slot, &feedback)) {
        bluepad32_input_backend_queue_profile_feedback(source.slot, feedback.connection_generation,
            feedback.active_profile_number, feedback.policy);
    }
    ProbeNativeMotionSample sample{};
    sample.accel_valid = source.accel_valid;
    sample.gyro_valid = source.gyro_valid;
    sample.accel_sequence = source.accel_sequence;
    sample.gyro_sequence = source.gyro_sequence;
    sample.accel_us = source.accel_received_us;
    sample.gyro_us = source.gyro_received_us;
    // SDL -> upright native body [X,-Z,Y], the same physical transform used by
    // the Wii adapter before its mouse-mount rotation. Both halves represent
    // one rigid, full controller: no solo-Joy-Con or mouse mounting rotation.
    sample.accel_g[0] = static_cast<float>(source.accel_q13[0]) / 8192.0f;
    sample.accel_g[1] = -static_cast<float>(source.accel_q13[2]) / 8192.0f;
    sample.accel_g[2] = static_cast<float>(source.accel_q13[1]) / 8192.0f;
    sample.gyro_dps[0] = static_cast<float>(source.gyro_q10[0]) / 1024.0f;
    sample.gyro_dps[1] = -static_cast<float>(source.gyro_q10[2]) / 1024.0f;
    sample.gyro_dps[2] = static_cast<float>(source.gyro_q10[1]) / 1024.0f;
    g_motion.update(now_us, source.controller.connection_generation, sample,
        source.track_stationary_bias ? ProbeNativeMotionBias::kTrackStationary :
                                          ProbeNativeMotionBias::kAlreadyCalibrated);
    const int status = !sensors_fresh(g_source, now_us) ? 0 : g_motion.ready() ? 2 : 1;
    if (status != g_sensor_status) {
        g_sensor_status = status;
        probe_debug_printf("[PROBE] Native gamepad IMU %s\n", status == 2 ? "ready" :
            status == 1 ? "waiting for a usable acceleration sample" : "waiting for supported fresh sensors");
    }
    for (uint8_t i = 0; i < PROBE_CONTROLLER_COUNT; ++i) {
        // Latest-only: a blocked endpoint never queues obsolete controls/IMU.
        g_children[i].pending_token = 0;
        pack_controls(i);
    }
}
} // namespace

void probe_native_gamepad_input_init() {
#if SWITCH2_BRIDGE_SOURCE_AUTO
    bluepad32_input_backend_select_native_source(nullptr);
#else
    bluepad32_input_backend_select_native_source(kSourceAddress);
#endif
}

void probe_native_gamepad_input_set_stick_calibration(uint8_t instance, const uint8_t calibration[9]) {
    if (instance >= PROBE_CONTROLLER_COUNT) return;
    Child& child = g_children[instance];
    child.calibrated = false;
    discard_output(child);
    if (calibration) {
        unpack_stick_pair(calibration, child.center);
        unpack_stick_pair(calibration + 3, child.positive);
        unpack_stick_pair(calibration + 6, child.negative);
        child.calibrated = true;
        for (unsigned axis = 0; axis < 2; ++axis) {
            if (!child.positive[axis] || !child.negative[axis] ||
                child.center[axis] + child.positive[axis] > 4095 ||
                child.negative[axis] > child.center[axis]) child.calibrated = false;
        }
    }
    pack_controls(instance);
}

void probe_native_gamepad_input_set_native_stream(uint8_t instance, bool enabled) {
    if (instance >= PROBE_CONTROLLER_COUNT) return;
    Child& child = g_children[instance];
    if (child.enabled != enabled || !enabled) discard_output(child);
    child.enabled = enabled;
    if (!enabled) bluepad32_input_backend_native_sample_cancel(instance);
}

void probe_native_gamepad_input_poll(uint8_t instance, uint32_t now_ms, probe_controller_input* out) {
    if (!out) return;
    if (instance >= PROBE_CONTROLLER_COUNT) { *out = {}; return; }
    refresh(now_ms);
    *out = g_children[instance].input;
}

uint32_t probe_native_gamepad_input_peek_native_report(uint8_t instance, uint32_t now_ms, uint8_t report[63]) {
    if (instance >= PROBE_CONTROLLER_COUNT || !report) return 0;
    refresh(now_ms);
    Child& child = g_children[instance];
    if (!child.enabled || !child.input.active) return 0;
    const uint32_t now_us = time_us_32();
    const bool motion_ready = (SWITCH2_BRIDGE_IMU_TARGET_MASK & (1u << instance)) != 0 &&
        g_motion.ready() && sensors_fresh(g_source, now_us) &&
        (!child.have_committed_motion || child.committed_accel_sequence != g_source.accel_sequence ||
         child.committed_gyro_sequence != g_source.gyro_sequence);
    if (child.pending_token && (now_us - child.pending_us >= kOutputDeadlineUs ||
        child.pending_motion != motion_ready)) child.pending_token = 0;
    if (!child.pending_token) {
        if (g_report_token == UINT32_MAX) return 0; // Boot-unique, including across children/resets.
        memset(child.pending_report, 0, sizeof(child.pending_report));
        child.pending_report[0] = child.counter;
        // Source battery level and the virtual controller's USB power are separate.
        const unsigned battery_level = (static_cast<unsigned>(g_source.battery) * 9u + 127u) / 255u;
        child.pending_report[1] = static_cast<uint8_t>((battery_level << 2) | 0x01u);
        memcpy(child.pending_report + 2, child.input.buttons, sizeof(child.input.buttons));
        child.pending_report[4] = 7;
        memcpy(child.pending_report + 5, child.input.stick, sizeof(child.input.stick));
        child.pending_report[8] = child.input.native_status;
        child.pending_report[13] = 0xff;
        child.pending_ticks = g_clock_ticks;
        const uint32_t elapsed = child.have_committed_motion ? child.pending_ticks - child.committed_ticks : 1;
        const uint16_t wire_elapsed = static_cast<uint16_t>(elapsed <= 0xfff ? elapsed : 1);
        child.pending_motion = motion_ready && probe_native_imu_pack(
            g_motion.quaternion(), g_motion.acceleration(), static_cast<uint16_t>(child.pending_ticks & 0xfff),
            wire_elapsed, 0, child.pending_report + probe_model_imu_data_offset(instance));
        if (child.pending_motion) child.pending_report[probe_model_imu_length_offset(instance)] = 30;
        child.pending_accel_sequence = g_source.accel_sequence;
        child.pending_gyro_sequence = g_source.gyro_sequence;
        child.pending_us = now_us;
        child.pending_token = ++g_report_token;
    }
    memcpy(report, child.pending_report, sizeof(child.pending_report));
    return child.pending_token;
}

bool probe_native_gamepad_input_commit_native_report(uint8_t instance, uint32_t token) {
    if (instance >= PROBE_CONTROLLER_COUNT || !token) return false;
    Child& child = g_children[instance];
    // Check the live source even when the caller did not poll after a disconnect.
    Bluepad32NativeGamepadSnapshot source;
    bluepad32_input_backend_native_snapshot(&source);
    const uint32_t now_us = time_us_32();
    if (!child.enabled || !g_active || child.pending_token != token ||
        !source.controller.active || source.slot != g_source.slot ||
        source.controller.connection_generation != g_source.controller.connection_generation ||
        source.state_generation != g_source.state_generation ||
        source.accel_sequence != g_source.accel_sequence || source.gyro_sequence != g_source.gyro_sequence ||
        source.accel_received_us != g_source.accel_received_us || source.gyro_received_us != g_source.gyro_received_us ||
        source.accel_valid != g_source.accel_valid || source.gyro_valid != g_source.gyro_valid ||
        source.track_stationary_bias != g_source.track_stationary_bias ||
        now_us - source.received_us >= kInputDeadlineUs || now_us - child.pending_us >= kOutputDeadlineUs ||
        (child.pending_motion && !sensors_fresh(source, now_us))) return false;
    child.pending_token = 0;
    if (child.pending_motion) {
        child.have_committed_motion = true;
        child.committed_accel_sequence = child.pending_accel_sequence;
        child.committed_gyro_sequence = child.pending_gyro_sequence;
        child.committed_ticks = child.pending_ticks;
    }
    ++child.counter;
    return true;
}
#endif
