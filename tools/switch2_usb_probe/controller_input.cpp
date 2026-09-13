#include "controller_input.h"
#include "model.h"

#include <string.h>

#include "input/bluepad32_input_backend.h"
#include "input/switch2_mouse_capture.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "platform/pico/system_clock.h"
#include "profile/controller_profile_runtime.h"
#include "pico/stdlib.h"
#if SWITCH2_PROBE_HUB
#include <inttypes.h>
extern "C" int probe_debug_printf(const char* format, ...);
#endif
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
#include "dualsense_input.h"
#endif
#if SWITCH2_BRIDGE_WII_INPUT
#include <math.h>
#include "input/wii_ir_pointer.h"
#include "native_imu.h"
#include "pico/time.h"
extern "C" int probe_debug_printf(const char* format, ...);
#endif

#if !SWITCH_PICO_SWITCH2_USB_BRIDGE || !SWITCH_PICO_BLUEPAD32
#error "The controller bridge requires Bluepad32"
#elif SWITCH2_BRIDGE_DUALSENSE_INPUT
#if !SWITCH_PICO_ENABLE_CLASSIC
#error "The DualSense bridge requires Classic Bluetooth"
#endif
#elif !SWITCH_PICO_ENABLE_BLE || !SWITCH_PICO_SWITCH2_MOUSE_CAPTURE || \
    !SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE
#error "The Joy-Con/Wii bridge requires Bluepad32 BLE and native Switch 2 capture"
#endif

namespace {
#if !SWITCH2_BRIDGE_DUALSENSE_INPUT
constexpr uint8_t kSourceAddress[] = {SWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES};
static_assert(sizeof(kSourceAddress) == 6, "Select one physical Bluetooth address");
#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
constexpr uint8_t kSecondSourceAddress[] = {SWITCH2_BRIDGE_SECOND_SOURCE_ADDRESS_BYTES};
static_assert(sizeof(kSecondSourceAddress) == 6, "Select the second physical Bluetooth address");
#endif
constexpr uint32_t kInputDeadlineMs = 500;
#endif
#if !SWITCH2_PROBE_HUB
constexpr uint32_t kFlashCoordinationTimeoutMs = 1000;
#endif
// Stage 2 publishes flash safety: both cores registered in dedicated-radio
// modes, or Core 0 registered with an SRAM-only/IRQ-disabled Core 1 in hub mode.
constexpr uint32_t kFlashCoordinationStage = 2;
bool g_initialized;
bool g_start_attempted;
bool g_flash_ready;
#if SWITCH2_BRIDGE_WII_INPUT
probe_controller_input g_input;
#elif !SWITCH2_BRIDGE_DUALSENSE_INPUT
probe_controller_input g_inputs[PROBE_CONTROLLER_COUNT];
uint32_t g_received_times[PROBE_CONTROLLER_COUNT];
#endif
#if SWITCH2_BRIDGE_WII_INPUT
#ifndef SWITCH2_WII_IR_SCREEN_CONFIG
#define SWITCH2_WII_IR_SCREEN_CONFIG 660, 370, 0, -115, 1920, 1080
#endif
constexpr float kIrScreenConfig[] = {SWITCH2_WII_IR_SCREEN_CONFIG};
static_assert(sizeof(kIrScreenConfig) / sizeof(kIrScreenConfig[0]) == 6);
bool g_screen_configured;
constexpr uint32_t kSensorDeadlineUs = 150000;
constexpr uint32_t kOutputDeadlineUs = 100000;
Bluepad32WiiBridgeSnapshot g_wii;
ProbeNativeMotion g_motion;
bool g_wii_active;
bool g_native_stream;
uint8_t g_native_features;
uint32_t g_wii_generation;
uint32_t g_orientation_requested_generation;
int g_sensor_status = -1;
uint16_t g_stick_center[2]{2048, 2048};
uint16_t g_stick_positive[2]{2047, 2047};
uint16_t g_stick_negative[2]{2048, 2048};
uint8_t g_power_info = 0x24; // Nominal battery until source status; no USB-power flag.
uint8_t g_report_counter;
uint32_t g_report_serial;
uint32_t g_pending_serial;
uint32_t g_pending_us;
uint32_t g_pending_generation;
uint32_t g_pending_ticks;
uint8_t g_pending_report[63];
WiiIrMouseReport g_pending_pointer{};
bool g_pending_motion_ready;
bool g_clock_started;
uint32_t g_clock_us;
uint32_t g_clock_ticks;
uint32_t g_clock_fraction;
bool g_have_committed_ticks;
uint32_t g_committed_ticks;
bool g_have_submission;
uint32_t g_submitted_us;
uint32_t g_output_open_us;
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
uint32_t g_last_ir_trace_us;
#endif

void unpack_stick_pair(const uint8_t* data, uint16_t values[2]) {
    values[0] = data[0] | (static_cast<uint16_t>(data[1] & 15) << 8);
    values[1] = (data[1] >> 4) | (static_cast<uint16_t>(data[2]) << 4);
}

uint16_t calibrated_stick_axis(int16_t value, unsigned axis, bool invert) {
    const int32_t input = value;
    const bool input_positive = input >= 0;
    const bool output_positive = input_positive != invert;
    const int32_t magnitude = input_positive ? input : -input;
    const int32_t denominator = input_positive ? INT16_MAX : 32768;
    const int32_t travel = output_positive ? g_stick_positive[axis] : g_stick_negative[axis];
    const int32_t displacement = (magnitude * travel + denominator / 2) / denominator;
    return static_cast<uint16_t>(g_stick_center[axis] +
                                 (output_positive ? displacement : -displacement));
}

void pack_wii_controls(const ControllerProfileTransformResult& mapped) {
    const ControllerState& state = mapped.state;
    g_input.buttons[0] = static_cast<uint8_t>(
        (state.button_south ? 0x01 : 0) | (state.button_east ? 0x02 : 0) |
        (state.button_west ? 0x04 : 0) | (state.button_north ? 0x08 : 0) |
        (state.button_right_shoulder ? 0x10 : 0) |
        (state.right_trigger != 0 &&
         state.right_trigger >= mapped.right_trigger_digital_threshold ? 0x20 : 0) |
        (state.button_start ? 0x40 : 0) | (state.button_right_stick ? 0x80 : 0));
    g_input.buttons[1] = static_cast<uint8_t>(
        (state.button_system ? 0x01 : 0) | ((state.extra_buttons & 1) ? 0x10 : 0) |
        ((state.extra_buttons & (1u << 5)) ? 0x80 : 0) |
        ((state.extra_buttons & (1u << 6)) ? 0x40 : 0));
    // One virtual right stick: honor a mapped right stick first, otherwise the
    // Nunchuk stick. Left-only buttons are not repurposed as mouse clicks.
    int16_t x = state.right_stick_x;
    int16_t y = state.right_stick_y;
    if (x == 0 && y == 0 && g_wii.layout == Bluepad32ControllerLayout::kWiiNunchuk) {
        x = state.left_stick_x;
        y = state.left_stick_y;
    }
    const uint16_t sx = calibrated_stick_axis(x, 0, false);
    const uint16_t sy = calibrated_stick_axis(y, 1, true);
    g_input.stick[0] = static_cast<uint8_t>(sx);
    g_input.stick[1] = static_cast<uint8_t>((sx >> 8) | (sy << 4));
    g_input.stick[2] = static_cast<uint8_t>(sy >> 4);
}

void advance_wii_clock(uint32_t now_us) {
    if (!g_clock_started) {
        g_clock_started = true;
        g_clock_us = now_us;
        return;
    }
    const uint64_t scaled = static_cast<uint64_t>(now_us - g_clock_us) * 960u +
                            g_clock_fraction;
    g_clock_us = now_us;
    g_clock_ticks += static_cast<uint32_t>(scaled / 1000000u);
    g_clock_fraction = static_cast<uint32_t>(scaled % 1000000u);
}

bool wii_sensors_fresh(uint32_t now_us) {
    return g_wii.accel_valid && g_wii.gyro_valid &&
           static_cast<int32_t>(now_us - g_wii.accel_received_us) <
               static_cast<int32_t>(kSensorDeadlineUs) &&
           static_cast<int32_t>(now_us - g_wii.gyro_received_us) <
               static_cast<int32_t>(kSensorDeadlineUs);
}

#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
void trace_wii_ir(uint32_t now_us, bool output_enabled) {
    // Ten snapshots/second keep camera diagnostics well below UART capacity.
    if (now_us - g_last_ir_trace_us < 100000) return;
    g_last_ir_trace_us = now_us;
    uint8_t data[WII_IR_MOUSE_DIAGNOSTIC_SIZE];
    if (wii_ir_pointer_diagnostics(data, sizeof(data)) != sizeof(data)) return;
    static constexpr char hex[] = "0123456789abcdef";
    char encoded[sizeof(data) * 2 + 1];
    for (size_t i = 0; i < sizeof(data); ++i) {
        encoded[2 * i] = hex[data[i] >> 4];
        encoded[2 * i + 1] = hex[data[i] & 15];
    }
    encoded[sizeof(encoded) - 1] = 0;
    // State bits: calibrated IMU, native stream, effective IR output gate.
    const unsigned state = static_cast<unsigned>(g_motion.ready()) |
                           (static_cast<unsigned>(g_native_stream) << 1) |
                           (static_cast<unsigned>(output_enabled) << 2);
    probe_debug_printf("[PROBE %lu] WII_IR_DIAGNOSTIC state=%u len=%u: %s\n",
                       static_cast<unsigned long>(to_ms_since_boot(get_absolute_time())), state,
                       static_cast<unsigned>(sizeof(data)), encoded);
}
#endif

void update_wii_ir_gate(uint32_t now_us) {
    const uint32_t last_progress = g_have_submission ? g_submitted_us : g_output_open_us;
    const bool output_fresh = static_cast<int32_t>(now_us - last_progress) <
                              static_cast<int32_t>(kOutputDeadlineUs);
    const bool enabled = g_native_stream && g_wii_active &&
        g_motion.ready() && wii_sensors_fresh(now_us) && output_fresh &&
        (g_native_features & 0x10);
    wii_ir_mouse_set_output_enabled(enabled);
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
    trace_wii_ir(now_us, enabled);
#endif
}

void discard_wii_output() {
    g_pending_serial = 0;
    g_have_committed_ticks = false;
    g_have_submission = false;
    g_output_open_us = time_us_32();
    wii_ir_mouse_set_output_enabled(false);
}

void lose_wii_source() {
    if (g_wii_active) {
        g_motion.reset();
        discard_wii_output();
        g_sensor_status = -1;
    }
    g_wii_active = false;
    g_input = {};
}

void poll_wii_source(uint32_t now_ms) {
    bluepad32_input_backend_wii_snapshot(&g_wii);
    // Read the clock after the coherent snapshot so Core1 receipt timestamps
    // cannot appear to be in the future to the motion integrator.
    const uint32_t now_us = time_us_32();
    advance_wii_clock(now_us);
    if (!g_wii.controller.active || g_wii.slot >= BLUEPAD32_INPUT_BACKEND_SLOT_COUNT ||
        static_cast<int32_t>(now_us - g_wii.received_us) >=
            static_cast<int32_t>(kInputDeadlineMs * 1000u)) {
        lose_wii_source();
        g_orientation_requested_generation = 0;
        return;
    }
    if (g_wii.layout == Bluepad32ControllerLayout::kWiiHorizontal) {
        if (g_orientation_requested_generation != g_wii.controller.connection_generation &&
            bluepad32_input_backend_set_wii_orientation(g_wii.controller.identity,
                g_wii.controller.connection_generation, true)) {
            g_orientation_requested_generation = g_wii.controller.connection_generation;
        }
        lose_wii_source();
        return; // Never emit a transient sideways mapping while Core1 switches.
    }
    if (!g_wii_active || g_wii_generation != g_wii.controller.connection_generation) {
        discard_wii_output();
        g_motion.reset();
        g_wii_generation = g_wii.controller.connection_generation;
        g_sensor_status = -1;
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
        g_last_ir_trace_us = now_us;
#endif
        g_power_info = 0x24;
        probe_debug_printf("[PROBE] Wii source active in slot %u; keep still for native motion calibration\n",
                           g_wii.slot);
    }
    g_wii_active = true;
    g_input.active = true;
    g_input.serial = g_wii.state_generation;
    g_input.native_status = static_cast<uint8_t>(0x30 | ((g_native_features & 0x20) ? 8 : 0));
    const ControllerProfileTransformResult mapped = controller_profile_runtime_transform(
        g_wii.slot, g_wii.controller, now_ms, AdapterUsbMode::kSwitch);
    pack_wii_controls(mapped);
    ControllerProfileRuntimeProfileChangeEvent feedback{};
    if (controller_profile_runtime_take_initial_profile_indication(g_wii.slot, &feedback) ||
        controller_profile_runtime_take_profile_change(g_wii.slot, &feedback)) {
        bluepad32_input_backend_queue_profile_feedback(g_wii.slot,
            feedback.connection_generation, feedback.active_profile_number, feedback.policy);
    }
    if (g_wii.battery != 0) {
        const unsigned level = (static_cast<unsigned>(g_wii.battery) * 9u + 127u) / 255u;
        g_power_info = static_cast<uint8_t>(level << 2);
    }
    ProbeNativeMotionSample motion{};
    motion.accel_valid = g_wii.accel_valid;
    motion.gyro_valid = g_wii.gyro_valid;
    motion.accel_sequence = g_wii.accel_sequence;
    motion.gyro_sequence = g_wii.gyro_sequence;
    motion.accel_us = g_wii.accel_received_us;
    motion.gyro_us = g_wii.gyro_received_us;
    // SDL -> physical right frame [X,-Z,Y], then a +90-degree mouse mounting
    // rotation about forward Y. A face-up Wii becomes rail-down native +X gravity.
    motion.accel_g[0] = static_cast<float>(g_wii.accel_q13[1]) / 8192.0f;
    motion.accel_g[1] = -static_cast<float>(g_wii.accel_q13[2]) / 8192.0f;
    motion.accel_g[2] = -static_cast<float>(g_wii.accel_q13[0]) / 8192.0f;
    motion.gyro_dps[0] = static_cast<float>(g_wii.gyro_q10[1]) / 1024.0f;
    motion.gyro_dps[1] = -static_cast<float>(g_wii.gyro_q10[2]) / 1024.0f;
    motion.gyro_dps[2] = -static_cast<float>(g_wii.gyro_q10[0]) / 1024.0f;
    g_motion.update(now_us, g_wii_generation, motion, ProbeNativeMotionBias::kEstimateStationary);
    WiiIrMouseReport optical{};
    (void)wii_ir_mouse_peek(&optical, 0);
    // Core 1 may publish during the peek. Read the clock after the snapshot.
    g_motion.observe_optical_heading(time_us_32(), optical.generation,
        optical.optical_sequence, optical.optical_received_us, optical.optical_yaw_radians,
        optical.tracking && optical.optical_valid && optical.owner == g_wii.slot &&
        optical.connection_generation == g_wii_generation);
    const int sensor_status = !wii_sensors_fresh(now_us) ? 0 : g_motion.ready() ? 2 : 1;
    if (sensor_status != g_sensor_status) {
        g_sensor_status = sensor_status;
        if (sensor_status == 2) {
            const float* bias = g_motion.bias();
            probe_debug_printf("[PROBE] Wii native IMU ready; bias_mdeg_s=%ld,%ld,%ld\n",
                               lroundf(bias[0] * 1000), lroundf(bias[1] * 1000),
                               lroundf(bias[2] * 1000));
        } else {
            probe_debug_printf("[PROBE] Wii native IMU %s\n", sensor_status ?
                "calibrating: keep still" : "waiting for fresh calibrated accelerometer/MotionPlus");
        }
    }
    update_wii_ir_gate(now_us);
}

uint32_t prepare_wii_report(uint8_t report[63]) {
    if (!report || !g_native_stream || !g_wii_active) return 0;
    const uint32_t now_us = time_us_32();
    if (static_cast<int32_t>(now_us - g_wii.received_us) >=
        static_cast<int32_t>(kInputDeadlineMs * 1000u)) return 0;
    advance_wii_clock(now_us);
    update_wii_ir_gate(now_us);
    WiiIrMouseReport pointer{};
    (void)wii_ir_mouse_peek(&pointer, INT16_MAX);
    const bool motion_ready = g_motion.ready() && wii_sensors_fresh(now_us);
    if (g_pending_serial &&
        (g_pending_generation != g_wii_generation ||
         g_pending_pointer.generation != pointer.generation ||
         g_pending_motion_ready != motion_ready ||
         static_cast<int32_t>(now_us - g_pending_us) >= static_cast<int32_t>(kOutputDeadlineUs))) {
        g_pending_serial = 0;
    }
    if (g_pending_serial) {
        memcpy(report, g_pending_report, sizeof(g_pending_report));
        return g_pending_serial;
    }
    if (g_report_serial == UINT32_MAX) return 0; // Never reuse a submission token.
    memset(g_pending_report, 0, sizeof(g_pending_report));
    g_pending_report[0] = g_report_counter;
    g_pending_report[1] = g_power_info;
    memcpy(g_pending_report + 2, g_input.buttons, sizeof(g_input.buttons));
    g_pending_report[4] = 7;
    memcpy(g_pending_report + 5, g_input.stick, sizeof(g_input.stick));
    g_pending_report[8] = g_input.native_status;
    g_pending_report[13] = 0xff; // Observed no-surface value.
    g_pending_ticks = g_clock_ticks;
    const uint32_t elapsed = g_have_committed_ticks ? g_pending_ticks - g_committed_ticks : 1;
    const uint16_t wire_elapsed = static_cast<uint16_t>(elapsed <= 0xfff ? elapsed : 1);
    const bool have_motion = g_motion.ready() && wii_sensors_fresh(now_us) &&
        probe_native_imu_pack(g_motion.quaternion(), g_motion.acceleration(),
            static_cast<uint16_t>(g_pending_ticks & 0xfff), wire_elapsed, 0,
            g_pending_report + 16);
    if (have_motion) g_pending_report[15] = 30;
    if (have_motion && pointer.tracking && pointer.owner == g_wii.slot &&
        pointer.connection_generation == g_wii_generation) {
        const uint16_t dx = static_cast<uint16_t>(pointer.dx);
        // Native Joy-Con Y is opposite to the shared desktop-pointer convention.
        // Keep the original pointer delta for commit/consumption below.
        const uint16_t dy = static_cast<uint16_t>(-pointer.dy);
        g_pending_report[9] = static_cast<uint8_t>(dx);
        g_pending_report[10] = static_cast<uint8_t>(dx >> 8);
        g_pending_report[11] = static_cast<uint8_t>(dy);
        g_pending_report[12] = static_cast<uint8_t>(dy >> 8);
        g_pending_report[13] = 20; // Observed contact-range value for virtual IR tracking.
    }
    // IR buttons are deliberately NOT mapped to desktop/native click buttons.
    g_pending_pointer = pointer;
    g_pending_motion_ready = have_motion;
    g_pending_generation = g_wii_generation;
    g_pending_us = now_us;
    g_pending_serial = ++g_report_serial;
    memcpy(report, g_pending_report, sizeof(g_pending_report));
    return g_pending_serial;
}
#endif
}  // namespace

extern "C" void probe_controller_input_clock_init(void) {
    system_clock_initialize();
}

extern "C" void probe_controller_input_init(void) {
    if (g_initialized) return;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    bluepad32_input_backend_init();
    probe_dualsense_input_init();
#elif SWITCH2_BRIDGE_WII_INPUT
    bluepad32_input_backend_init();
    bluepad32_input_backend_select_wii_source(kSourceAddress);
    g_screen_configured = wii_ir_pointer_configure_screen(
        kIrScreenConfig[0], kIrScreenConfig[1], kIrScreenConfig[2],
        kIrScreenConfig[3], kIrScreenConfig[4], kIrScreenConfig[5]);
    if (!g_screen_configured) probe_debug_printf("[PROBE] Invalid native IR viewport configuration\n");
    wii_ir_mouse_set_output_enabled(false);
#else
    static_assert(SWITCH2_MOUSE_CAPTURE_SOURCE_COUNT == PROBE_CONTROLLER_COUNT,
                  "Each native controller requires an independent capture channel");
    switch2_mouse_capture_init();
    switch2_mouse_capture_select_input(0, kSourceAddress, probe_model_pid(0));
#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
    switch2_mouse_capture_select_input(1, kSecondSourceAddress, probe_model_pid(1));
#endif
    bluepad32_input_backend_init();
#endif
    controller_profile_runtime_reset();
    g_initialized = true;
}

extern "C" bool probe_controller_input_start(void) {
    if (!g_initialized) probe_controller_input_init();
    if (g_start_attempted) return g_flash_ready;
#if SWITCH2_BRIDGE_WII_INPUT
    if (!g_screen_configured) return false;
#endif
    g_start_attempted = true;
    bluepad32_input_backend_start();
#if SWITCH2_PROBE_HUB
    // Initialization is synchronous on Core 0; there is no radio Core 1 to
    // wait for. The SDK async context advances radio startup in task().
    Bluepad32BackendDiagnostics diagnostics;
    bluepad32_input_backend_diagnostics(&diagnostics);
    g_flash_ready = diagnostics.initialization_stage >= kFlashCoordinationStage;
    return g_flash_ready;
#else
    const absolute_time_t deadline = make_timeout_time_ms(kFlashCoordinationTimeoutMs);
    do {
        Bluepad32BackendDiagnostics diagnostics;
        bluepad32_input_backend_diagnostics(&diagnostics);
        if (diagnostics.initialization_stage >= kFlashCoordinationStage) {
            g_flash_ready = true;
            return true;
        }
        sleep_ms(1);
    } while (!time_reached(deadline));
    // Do not reset Core 1 or retry a partially launched backend. It may still
    // be running; a false return keeps USB and its flash writes fail-closed.
    return false;
#endif
}

extern "C" void probe_controller_input_task(void) {
#if SWITCH2_PROBE_HUB
    if (!g_flash_ready) return;
    bluepad32_input_backend_poll();
    static uint32_t last_diagnostics;
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t)(now - last_diagnostics) >= 1000u) {
        last_diagnostics = now;
        Bluepad32BackendDiagnostics diagnostics;
        bluepad32_input_backend_diagnostics(&diagnostics);
        probe_debug_printf("[HUB_RADIO] stage=%" PRIu32 " timers=%" PRIu32 "/%" PRIu32
                           " reports=%" PRIu32 "\n", diagnostics.initialization_stage,
                           diagnostics.rumble_timer_ticks, diagnostics.configuration_timer_ticks,
                           diagnostics.controller_reports);
    }
#endif
}

extern "C" bool probe_controller_input_pairing_task(void) {
    if (!g_flash_ready) return false;
    // Use the shared sampler/hold policy, but deliberately do not route its
    // kClearPairings event to recovery or any storage-clearing operation.
    if (bootsel_pairing_button_task() != BootselPairingButtonEvent::kOpenPairing)
        return false;
    bluepad32_input_backend_open_pairing_window();
    return true;
}

#if SWITCH2_BRIDGE_WII_INPUT
extern "C" void probe_controller_input_set_stick_calibration(const uint8_t calibration[9]) {
    if (!calibration) return;
    unpack_stick_pair(calibration, g_stick_center);
    unpack_stick_pair(calibration + 3, g_stick_positive);
    unpack_stick_pair(calibration + 6, g_stick_negative);
    g_pending_serial = 0;
}

extern "C" void probe_controller_input_set_native_features(uint8_t features) {
    if (g_native_features == features) return;
    g_native_features = features;
    g_pending_serial = 0;
    update_wii_ir_gate(time_us_32());
}
#endif

#if SWITCH2_BRIDGE_DUALSENSE_INPUT
extern "C" void probe_controller_input_set_full_stick_calibration(
    uint8_t instance, const uint8_t calibration[9]) {
    probe_dualsense_input_set_stick_calibration(instance, calibration);
}
#endif

extern "C" void probe_controller_input_set_native_stream(uint8_t instance, bool enabled) {
    if (instance >= PROBE_CONTROLLER_COUNT) return;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    probe_dualsense_input_set_native_stream(instance, enabled && g_flash_ready);
#elif SWITCH2_BRIDGE_WII_INPUT
    enabled = enabled && g_flash_ready;
    if (g_native_stream != enabled || !enabled) discard_wii_output();
    g_native_stream = enabled;
    update_wii_ir_gate(time_us_32());
#else
    switch2_mouse_capture_set_native_stream(instance, g_flash_ready && enabled);
#endif
}

extern "C" uint32_t probe_controller_input_peek_native_report(
    uint8_t instance, uint32_t now_ms, uint8_t report[63]) {
    if (instance >= PROBE_CONTROLLER_COUNT || !g_flash_ready) return 0;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    return probe_dualsense_input_peek_native_report(instance, now_ms, report);
#elif SWITCH2_BRIDGE_WII_INPUT
    (void)now_ms;
    return prepare_wii_report(report);
#else
    return switch2_mouse_capture_peek_native_report(instance, now_ms, report);
#endif
}

extern "C" bool probe_controller_input_commit_native_report(uint8_t instance, uint32_t serial) {
    if (instance >= PROBE_CONTROLLER_COUNT || !g_flash_ready) return false;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    return probe_dualsense_input_commit_native_report(instance, serial);
#elif SWITCH2_BRIDGE_WII_INPUT
    if (!g_native_stream || !serial || serial != g_pending_serial ||
        g_pending_generation != g_wii_generation || !g_wii_active) return false;
    wii_ir_mouse_commit(g_pending_pointer);
    g_pending_serial = 0;
    g_committed_ticks = g_pending_ticks;
    g_have_committed_ticks = true;
    g_have_submission = true;
    g_submitted_us = time_us_32();
    ++g_report_counter;
    return true;
#else
    return switch2_mouse_capture_commit_native_report(instance, serial);
#endif
}

extern "C" bool probe_controller_input_play_sample(uint8_t instance, uint8_t sample_id, uint64_t* token) {
    if (instance >= PROBE_CONTROLLER_COUNT || !g_flash_ready) {
        if (token != nullptr) *token = 0;
        return false;
    }
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    return bluepad32_input_backend_dualsense_sample_request(instance, sample_id, token);
#elif SWITCH2_BRIDGE_WII_INPUT
    return bluepad32_input_backend_wii_sample_request(sample_id, token);
#else
    return switch2_mouse_capture_request_sample(
        instance, sample_id, to_ms_since_boot(get_absolute_time()), token);
#endif
}

extern "C" int probe_controller_input_sample_result(uint8_t instance, uint64_t token, uint32_t now_ms) {
    if (instance >= PROBE_CONTROLLER_COUNT || !g_flash_ready) return -1;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    (void)now_ms;
    return bluepad32_input_backend_dualsense_sample_result(instance, token);
#elif SWITCH2_BRIDGE_WII_INPUT
    (void)now_ms;
    return bluepad32_input_backend_wii_sample_result(token);
#else
    return switch2_mouse_capture_sample_result(instance, token, now_ms);
#endif
}

extern "C" void probe_controller_input_cancel_sample(uint8_t instance) {
    if (instance >= PROBE_CONTROLLER_COUNT) return;
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    bluepad32_input_backend_dualsense_sample_cancel(instance);
#elif SWITCH2_BRIDGE_WII_INPUT
    bluepad32_input_backend_wii_sample_cancel();
#else
    switch2_mouse_capture_cancel_sample(instance);
#endif
}

extern "C" void probe_controller_input_poll(uint8_t instance, uint32_t now_ms,
                                            probe_controller_input* out) {
    if (out == nullptr) return;
    if (instance >= PROBE_CONTROLLER_COUNT || !g_flash_ready) {
        *out = {};
        return;
    }
#if SWITCH2_BRIDGE_DUALSENSE_INPUT
    probe_dualsense_input_poll(instance, now_ms, out);
    return;
#elif SWITCH2_BRIDGE_WII_INPUT
    poll_wii_source(now_ms);
#else
    probe_controller_input& g_input = g_inputs[instance];
    uint32_t& g_received_ms = g_received_times[instance];
    Switch2MouseCaptureInput sample;
    if (switch2_mouse_capture_latest_input(instance, g_input.serial, &sample)) {
        g_input.serial = sample.serial;
        g_input.active = sample.active;
        g_received_ms = sample.received_ms;
        memcpy(g_input.buttons, sample.buttons, sizeof(g_input.buttons));
        memcpy(g_input.stick, sample.stick, sizeof(g_input.stick));
        g_input.native_status = sample.native_status;
        g_input.mouse_epoch = sample.mouse_epoch;
        g_input.mouse_total_x = sample.mouse_total_x;
        g_input.mouse_total_y = sample.mouse_total_y;
        g_input.mouse_surface = sample.mouse_surface;
    }
    // A producer timestamp can be slightly ahead of this pre-poll clock.
    if (g_input.active && static_cast<int32_t>(now_ms - g_received_ms) >=
            static_cast<int32_t>(kInputDeadlineMs)) {
        g_input.active = false;
        memset(g_input.buttons, 0, sizeof(g_input.buttons));
        memset(g_input.stick, 0, sizeof(g_input.stick));
        g_input.native_status = 0;
        g_input.mouse_epoch = 0;
        g_input.mouse_total_x = 0;
        g_input.mouse_total_y = 0;
        g_input.mouse_surface = 0;
    }
#endif
#if !SWITCH2_BRIDGE_DUALSENSE_INPUT
    *out = g_input;
#endif
}
