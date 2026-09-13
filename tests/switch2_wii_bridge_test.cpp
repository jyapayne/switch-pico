#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "controller_input.h"
#include "input/bluepad32_input_backend.h"
#include "input/wii_ir_pointer.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "platform/pico/system_clock.h"
#include "profile/controller_profile_runtime.h"
#include "pico/stdlib.h"

static uint64_t now_us;
static uint32_t stage;
static Bluepad32WiiBridgeSnapshot source;
static bool swap_faces = true;
static bool publish_during_snapshot;
static uint32_t ir_sequence;
static uint32_t sensor_sequence;
static uint32_t last_sensor_us;
static uint16_t raw_ir_buttons;
static uint16_t ir_x[4] = {420, 620, 0, 0};
static uint16_t ir_y[4] = {384, 384, 0, 0};
static uint8_t ir_mask = 3;
static probe_controller_input controls;
static uint8_t packet[63];
static const int32_t bias_q10[3] = {1024, -2048, 512};

uint32_t time_us_32() { return static_cast<uint32_t>(now_us); }
absolute_time_t get_absolute_time() { return now_us; }
uint32_t to_ms_since_boot(absolute_time_t time) { return static_cast<uint32_t>(time / 1000); }
absolute_time_t make_timeout_time_ms(uint32_t ms) { return now_us + 1000ull * ms; }
bool time_reached(absolute_time_t deadline) { return now_us >= deadline; }
void sleep_ms(uint32_t ms) { now_us += 1000ull * ms; }
void system_clock_initialize() {}
extern "C" int probe_debug_printf(const char*, ...) { return 0; }
BootselPairingButtonEvent bootsel_pairing_button_task() { return BootselPairingButtonEvent::kNone; }

void bluepad32_input_backend_init() { stage = 1; wii_ir_pointer_init(); }
void bluepad32_input_backend_start() { stage = 2; }
void bluepad32_input_backend_diagnostics(Bluepad32BackendDiagnostics* out) { *out = {}; out->initialization_stage = stage; }
void bluepad32_input_backend_open_pairing_window() {}
void bluepad32_input_backend_select_wii_source(const uint8_t[6]) { wii_ir_pointer_reset(); }
void bluepad32_input_backend_wii_snapshot(Bluepad32WiiBridgeSnapshot* out) {
    if (publish_during_snapshot) {
        // Model Core1 publishing after Core0 entered the snapshot call.
        now_us += 300;
        source.accel_sequence = source.gyro_sequence = ++sensor_sequence;
        source.received_us = source.accel_received_us = source.gyro_received_us = time_us_32();
    }
    *out = source;
}
bool bluepad32_input_backend_set_wii_orientation(const ControllerIdentity&, uint32_t, bool) { return false; }
void bluepad32_input_backend_report_sent(uint8_t) {}
bool bluepad32_input_backend_wii_sample_request(uint8_t, uint64_t*) { return false; }
int bluepad32_input_backend_wii_sample_result(uint64_t) { return -1; }
void bluepad32_input_backend_wii_sample_cancel() {}
void bluepad32_input_backend_queue_profile_feedback(uint8_t, uint32_t, uint8_t, ControllerProfileConfirmationPolicy) {}
void controller_profile_runtime_reset() {}
bool controller_profile_runtime_take_initial_profile_indication(uint8_t, ControllerProfileRuntimeProfileChangeEvent*) { return false; }
bool controller_profile_runtime_take_profile_change(uint8_t, ControllerProfileRuntimeProfileChangeEvent*) { return false; }
ControllerProfileTransformResult controller_profile_runtime_transform(
    uint8_t, const Bluepad32SlotSnapshot& input, uint32_t, AdapterUsbMode) {
    ControllerProfileTransformResult result{};
    result.state = input.state;
    if (swap_faces) {
        result.state.button_south = input.state.button_east;
        result.state.button_east = input.state.button_south;
    }
    return result;
}

static void put_pair(uint8_t* out, uint16_t x, uint16_t y) {
    out[0] = static_cast<uint8_t>(x);
    out[1] = static_cast<uint8_t>((x >> 8) | (y << 4));
    out[2] = static_cast<uint8_t>(y >> 4);
}
static uint16_t stick_x(const uint8_t* p) { return p[5] | ((p[6] & 15u) << 8); }
static uint16_t stick_y(const uint8_t* p) { return (p[6] >> 4) | (p[7] << 4); }
static int32_t signed32(const uint8_t* p) {
    const uint32_t raw = p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    return raw <= INT32_MAX ? static_cast<int32_t>(raw) : -1 - static_cast<int32_t>(UINT32_MAX - raw);
}
static uint32_t get_bits(const uint8_t* p, unsigned start, unsigned count) {
    uint32_t result = 0;
    for (unsigned i = 0; i < count; ++i) result |= uint32_t((p[(start+i)/8] >> ((start+i)%8)) & 1) << i;
    return result;
}
static void decode_quaternion(const uint8_t* p, double out[4]) {
    const uint8_t* imu = p + 16;
    const unsigned index = get_bits(imu, 32, 3);
    assert(index < 4);
    double ratios[3], norm = 1;
    for (unsigned i = 0; i < 3; ++i) {
        ratios[i] = get_bits(imu, 35 + 31*i, 31) / 1073741824.0 - 1;
        norm += ratios[i] * ratios[i];
    }
    out[index] = 1 / sqrt(norm);
    for (unsigned i = 0; i < 3; ++i) out[(index+i+1)&3] = ratios[i] * out[index];
}
static void publish(bool gyro_fresh = true) {
    source.received_us = time_us_32();
    ++source.state_generation;
    if (time_us_32() - last_sensor_us >= 10000) {
        last_sensor_us = time_us_32();
        ++sensor_sequence;
        source.accel_sequence = sensor_sequence;
        source.accel_received_us = time_us_32();
        if (gyro_fresh) {
            source.gyro_sequence = sensor_sequence;
            source.gyro_received_us = time_us_32();
        }
    }
    wii_ir_pointer_observe(source.slot, source.controller.connection_generation,
                          ++ir_sequence, raw_ir_buttons, ir_x, ir_y, ir_mask, false, 0.0f, true);
}
static uint32_t poll(bool commit = true, bool gyro_fresh = true) {
    now_us += 4000;
    publish(gyro_fresh);
    probe_controller_input_poll(0, to_ms_since_boot(now_us), &controls);
    const uint32_t token = probe_controller_input_peek_native_report(0, to_ms_since_boot(now_us), packet);
    if (commit && token) assert(probe_controller_input_commit_native_report(0, token));
    return token;
}

int main() {
    probe_controller_input_init();
    assert(probe_controller_input_start());
    uint8_t calibration[9];
    put_pair(calibration, 2000, 2100);
    put_pair(calibration+3, 1500, 1400);
    put_pair(calibration+6, 1600, 1700);
    probe_controller_input_set_stick_calibration(calibration);
    probe_controller_input_set_native_features(0x37);
    probe_controller_input_set_native_stream(0, true);
    source.slot = 0;
    source.controller.active = true;
    source.controller.connection_generation = 7;
    source.layout = Bluepad32ControllerLayout::kWiiNunchuk;
    source.controller.state.button_south = true; // Profile maps this to native A.
    source.accel_valid = source.gyro_valid = true;
    source.accel_q13[1] = 8192; // SDL up -> virtual native rail-down +X.
    memcpy(source.gyro_q10, bias_q10, sizeof(bias_q10));
    assert(poll());
    assert(controls.active && packet[15] == 30); // Wii IMU starts before background bias learning.
    assert(packet[1] == 0x01); // USB power, no fabricated charge level or charging state.
    for (unsigned i = 1; i < 450; ++i) assert(poll());
    assert(controls.active && packet[15] == 30 && packet[19] == 0x0c);
    assert(signed32(packet+32) == (1 << 28));
    assert(signed32(packet+36) == 0 && signed32(packet+40) == 0);
    assert(packet[2] == 2 && packet[3] == 0 && packet[13] == 20);
    assert(stick_x(packet) == 2000 && stick_y(packet) == 2100);
    double initial[4]; decode_quaternion(packet, initial);

    raw_ir_buttons = 0x000c; // Physical A+B in pointer telemetry are not click overlays.
    for (unsigned i = 0; i < 8; ++i) assert(poll());
    assert(packet[2] == 2 && (packet[2] & 0x30) == 0);
    source.controller.state.left_stick_x = INT16_MAX;
    source.controller.state.left_stick_y = INT16_MIN;
    assert(poll());
    assert(stick_x(packet) == 3500 && stick_y(packet) == 3500);
    source.controller.state.left_stick_x = source.controller.state.left_stick_y = 0;

    // Native IR displacement must survive failed submission, then be consumed once.
    raw_ir_buttons = 0x0002; // A mapped game button must not clutch native IR movement.
    bool horizontal_motion = false;
    int32_t horizontal_total = 0, horizontal_cross_axis = 0;
    for (unsigned i = 0; i < 24; ++i) {
        ir_x[0] += 2; ir_x[1] += 2;
        assert(poll());
        horizontal_motion = horizontal_motion || packet[9] != 0 || packet[10] != 0;
        const uint16_t x = packet[9] | (uint16_t(packet[10]) << 8);
        const uint16_t y = packet[11] | (uint16_t(packet[12]) << 8);
        horizontal_total += x <= INT16_MAX ? int32_t(x) : int32_t(x) - 65536;
        horizontal_cross_axis += y <= INT16_MAX ? int32_t(y) : int32_t(y) - 65536;
    }
    assert(horizontal_motion);
    assert(horizontal_total < 0);
    assert(horizontal_cross_axis * 4 > horizontal_total && horizontal_cross_axis * 4 < -horizontal_total);
    ir_x[0] += 8; ir_x[1] += 8;
    const uint32_t ticket = poll(false);
    uint8_t retry[63];
    assert(ticket && probe_controller_input_peek_native_report(0, to_ms_since_boot(now_us), retry) == ticket);
    assert(memcmp(packet, retry, sizeof(packet)) == 0);
    assert(probe_controller_input_commit_native_report(0, ticket));
    assert(!probe_controller_input_commit_native_report(0, ticket));

    // Tilting the Wii up moves camera spots down. Native Joy-Con Y must
    // reverse the desktop-pointer convention without changing consumption.
    for (unsigned i = 0; i < 80; ++i) assert(poll());
    int32_t vertical_totals[2]{};
    for (unsigned direction = 0; direction < 2; ++direction) {
        for (unsigned i = 0; i < 104; ++i) {
            if (i < 24) {
                ir_y[0] += direction == 0 ? 2 : -2;
                ir_y[1] += direction == 0 ? 2 : -2;
            }
            assert(poll());
            const uint16_t y = packet[11] | (uint16_t(packet[12]) << 8);
            vertical_totals[direction] += y <= INT16_MAX ? int32_t(y) : int32_t(y) - 65536;
            WiiIrMouseReport pending{};
            (void)wii_ir_mouse_peek(&pending, INT16_MAX);
            assert(pending.dx == 0 && pending.dy == 0);
        }
    }
    assert(vertical_totals[0] > 0 && vertical_totals[1] < 0);
    // Upstream radial smoothing has a positional dead zone, so returning the
    // camera to its origin need not return the filtered pointer exactly there.
    // Per-report checks above instead defend once-only native consumption.

    // A visible, stationary bar must anchor native heading against residual drift.
    for (unsigned i = 0; i < 3000; ++i) assert(poll());
    decode_quaternion(packet, initial);
    source.gyro_q10[1] = bias_q10[1] + 614; // ~0.6dps warming drift about vertical.
    for (unsigned i = 0; i < 5000; ++i) assert(poll());
    double optically_held[4]; decode_quaternion(packet, optically_held);
    double heading_dot = 0;
    for (unsigned i = 0; i < 4; ++i) heading_dot += initial[i] * optically_held[i];
    assert(fabs(heading_dot) > cos(2.0 * acos(-1.0) / 360.0));
    memcpy(source.gyro_q10, bias_q10, sizeof(bias_q10));

    // Gyro-only motion remains available while the sensor bar is out of view.
    ir_mask = 0;
    // Give background correction a fresh quiet window after the simulated cooling.
    for (unsigned i = 0; i < 750; ++i) assert(poll());
    decode_quaternion(packet, initial);
    for (unsigned i = 0; i < 250; ++i) assert(poll());
    double after_bias[4]; decode_quaternion(packet, after_bias);
    double dot = 0; for (unsigned i=0;i<4;++i) dot += initial[i]*after_bias[i];
    assert(fabs(dot) > .99999);
    // SDL yaw -> native mouse-frame +X; one second at90dps rotates90 degrees.
    source.gyro_q10[1] = bias_q10[1] + 90 * 1024;
    for (unsigned i = 0; i < 250; ++i) assert(poll());
    double turned[4]; decode_quaternion(packet, turned);
    dot = 0; for (unsigned i=0;i<4;++i) dot += initial[i]*turned[i];
    assert(fabs(fabs(dot) - sqrt(.5)) < .015);
    memcpy(source.gyro_q10, bias_q10, sizeof(bias_q10));

    ir_mask = 0;
    assert(poll());
    assert(packet[9] == 0 && packet[10] == 0 && packet[11] == 0 && packet[12] == 0 && packet[13] == 0xff);
    ir_x[0] = 500; ir_x[1] = 700; ir_mask = 3;
    assert(poll());
    assert(packet[9] == 0 && packet[10] == 0 && packet[11] == 0 && packet[12] == 0);

    // Fresh controller/accelerometer traffic cannot rejuvenate stale MotionPlus.
    for (unsigned i = 0; i < 42; ++i) assert(poll(true, false));
    assert(controls.active && packet[15] == 0 && packet[13] == 0xff);
    const uint32_t obsolete = poll(false, false);
    ++source.controller.connection_generation;
    assert(poll(false));
    assert(!probe_controller_input_commit_native_report(0, obsolete));
    probe_controller_input_set_native_stream(0, false);
    assert(probe_controller_input_peek_native_report(0, to_ms_since_boot(now_us), retry) == 0);
    probe_controller_input_set_native_stream(0, true);
    publish_during_snapshot = true;
    for (unsigned i = 0; i < 450; ++i) assert(poll());
    assert(packet[15] == 30);
    source.controller.active = false;
    now_us += 4000;
    probe_controller_input_poll(0, to_ms_since_boot(now_us), &controls);
    assert(!controls.active);
    assert(probe_controller_input_peek_native_report(0, to_ms_since_boot(now_us), retry) == 0);
    return 0;
}
