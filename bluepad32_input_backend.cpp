#include "bluepad32_input_backend.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <btstack_run_loop.h>
#include <pico/critical_section.h>
#include <pico/cyw43_arch.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <uni.h>

namespace {

constexpr uint16_t kStickMidpoint = 32768;
constexpr int32_t kAxisMinimum = -512;
constexpr int32_t kAxisMaximum = 511;
constexpr int32_t kTriggerMaximum = 1023;
constexpr int32_t kTriggerThreshold = (kTriggerMaximum * 35) / 100;
constexpr uint16_t kRumbleDurationMs = 50;
constexpr uint32_t kRumblePollIntervalMs = 5;
constexpr uint kRumbleQueueDepth = 8;

struct RumblePacket {
    uint8_t bytes[8];
};

critical_section_t g_state_lock;
queue_t g_rumble_queue;
SwitchInputState g_shared_state;
bool g_shared_controller_active = false;
uint32_t g_shared_generation = 0;

// These generations are only read or written by Core 0.
uint32_t g_consumed_generation = 0;
uint32_t g_last_snapshot_generation = 0;
bool g_initialized = false;
bool g_started = false;

// This pointer and the timer are only read or written by Core 1 / BTstack.
uni_hid_device_t* g_active_device = nullptr;
btstack_timer_source_t g_rumble_timer{};

SwitchInputState make_neutral_state() {
    SwitchInputState state{};
    state.lx = kStickMidpoint;
    state.ly = kStickMidpoint;
    state.rx = kStickMidpoint;
    state.ry = kStickMidpoint;
    return state;
}

void publish_state(const SwitchInputState& state, bool controller_active) {
    critical_section_enter_blocking(&g_state_lock);
    g_shared_state = state;
    g_shared_controller_active = controller_active;
    ++g_shared_generation;
    critical_section_exit(&g_state_lock);
}

int32_t clamp_axis(int32_t value) {
    if (value < kAxisMinimum) {
        return kAxisMinimum;
    }
    if (value > kAxisMaximum) {
        return kAxisMaximum;
    }
    return value;
}

uint16_t scale_stick(int32_t value) {
    value = clamp_axis(value);
    if (value <= 0) {
        return static_cast<uint16_t>(
            (static_cast<int64_t>(value - kAxisMinimum) * kStickMidpoint) / -kAxisMinimum);
    }
    return static_cast<uint16_t>(
        kStickMidpoint + (static_cast<int64_t>(value) * (UINT16_MAX - kStickMidpoint)) / kAxisMaximum);
}

int16_t clamp_int16(int64_t value) {
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    return static_cast<int16_t>(value);
}

int64_t divide_round_nearest(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }
    return -((-numerator + denominator / 2) / denominator);
}

int16_t convert_accel(int64_t q13_value) {
    return clamp_int16(q13_value / 2);
}

int16_t convert_gyro(int64_t q10_value) {
    constexpr int64_t kNumeratorScale = 13371;
    constexpr int64_t kDenominator = 1024 * 936;
    return clamp_int16(divide_round_nearest(q10_value * kNumeratorScale, kDenominator));
}

bool has_motion(const uni_gamepad_t& gamepad) {
    for (size_t i = 0; i < 3; ++i) {
        if (gamepad.accel[i] != 0 || gamepad.gyro[i] != 0) {
            return true;
        }
    }
    return false;
}

SwitchInputState map_gamepad(const uni_gamepad_t& gamepad) {
    SwitchInputState state = make_neutral_state();

    state.dpad_up = (gamepad.dpad & DPAD_UP) != 0;
    state.dpad_down = (gamepad.dpad & DPAD_DOWN) != 0;
    state.dpad_left = (gamepad.dpad & DPAD_LEFT) != 0;
    state.dpad_right = (gamepad.dpad & DPAD_RIGHT) != 0;

    // Bluepad32's A/B/X/Y are positional: south/east/west/north.
    state.button_b = (gamepad.buttons & BUTTON_A) != 0;
    state.button_a = (gamepad.buttons & BUTTON_B) != 0;
    state.button_y = (gamepad.buttons & BUTTON_X) != 0;
    state.button_x = (gamepad.buttons & BUTTON_Y) != 0;
    state.button_l = (gamepad.buttons & BUTTON_SHOULDER_L) != 0;
    state.button_r = (gamepad.buttons & BUTTON_SHOULDER_R) != 0;
    state.button_zl = (gamepad.buttons & BUTTON_TRIGGER_L) != 0 || gamepad.brake >= kTriggerThreshold;
    state.button_zr = (gamepad.buttons & BUTTON_TRIGGER_R) != 0 || gamepad.throttle >= kTriggerThreshold;
    state.button_l3 = (gamepad.buttons & BUTTON_THUMB_L) != 0;
    state.button_r3 = (gamepad.buttons & BUTTON_THUMB_R) != 0;

    state.button_minus = (gamepad.misc_buttons & MISC_BUTTON_SELECT) != 0;
    state.button_plus = (gamepad.misc_buttons & MISC_BUTTON_START) != 0;
    state.button_home = (gamepad.misc_buttons & MISC_BUTTON_SYSTEM) != 0;
    state.button_capture = (gamepad.misc_buttons & MISC_BUTTON_CAPTURE) != 0;

    state.lx = scale_stick(gamepad.axis_x);
    state.ly = scale_stick(gamepad.axis_y);
    state.rx = scale_stick(gamepad.axis_rx);
    state.ry = scale_stick(gamepad.axis_ry);

    if (has_motion(gamepad)) {
        // Dependency patches normalize both arrays to SDL3 PlayStation axes.
        SwitchImuSample sample{};
        sample.accel_x = convert_accel(-static_cast<int64_t>(gamepad.accel[2]));
        sample.accel_y = convert_accel(-static_cast<int64_t>(gamepad.accel[0]));
        sample.accel_z = convert_accel(gamepad.accel[1]);
        sample.gyro_x = convert_gyro(-static_cast<int64_t>(gamepad.gyro[2]));
        sample.gyro_y = convert_gyro(-static_cast<int64_t>(gamepad.gyro[0]));
        sample.gyro_z = convert_gyro(gamepad.gyro[1]);
        state.imu_sample_count = 3;
        for (SwitchImuSample& destination : state.imu_samples) {
            destination = sample;
        }
    }

    return state;
}

void decode_rumble(const uint8_t bytes[8], uint8_t* left_magnitude, uint8_t* right_magnitude) {
    static constexpr uint8_t kNeutralPacket[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
    if (memcmp(bytes, kNeutralPacket, sizeof(kNeutralPacket)) == 0) {
        *left_magnitude = 0;
        *right_magnitude = 0;
        return;
    }

    uint16_t right_raw = static_cast<uint16_t>(((bytes[1] & 0x03) << 8) | bytes[0]);
    uint16_t left_raw = static_cast<uint16_t>(((bytes[5] & 0x03) << 8) | bytes[4]);
    if (left_raw < 8 && right_raw < 8) {
        left_raw = 0;
        right_raw = 0;
    }

    *left_magnitude = static_cast<uint8_t>((left_raw * UINT8_MAX + 511) / 1023);
    *right_magnitude = static_cast<uint8_t>((right_raw * UINT8_MAX + 511) / 1023);
}

void process_rumble_timer(btstack_timer_source_t* timer) {
    RumblePacket packet{};
    RumblePacket latest{};
    bool have_packet = false;
    while (queue_try_remove(&g_rumble_queue, &packet)) {
        latest = packet;
        have_packet = true;
    }

    if (have_packet && g_active_device != nullptr &&
        g_active_device->report_parser.play_dual_rumble != nullptr) {
        uint8_t left_magnitude = 0;
        uint8_t right_magnitude = 0;
        decode_rumble(latest.bytes, &left_magnitude, &right_magnitude);
        // Bluepad orders the weak (high-frequency) motor before the strong
        // (low-frequency) motor; the project decoder names those right/left.
        g_active_device->report_parser.play_dual_rumble(
            g_active_device, 0, kRumbleDurationMs, right_magnitude, left_magnitude);
    }

    btstack_run_loop_set_timer(timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(timer);
}

void platform_init(int argc, const char** argv) {
    (void)argc;
    (void)argv;
}

void platform_on_init_complete() {
    btstack_run_loop_set_timer_handler(&g_rumble_timer, process_rumble_timer);
    btstack_run_loop_set_timer(&g_rumble_timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(&g_rumble_timer);

    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
}

uni_error_t platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    (void)addr;
    (void)name;
    (void)cod;
    (void)rssi;
    return g_active_device == nullptr ? UNI_ERROR_SUCCESS : UNI_ERROR_IGNORE_DEVICE;
}

void platform_on_device_connected(uni_hid_device_t* device) {
    (void)device;
}

void resume_connections() {
    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
}

void platform_on_device_disconnected(uni_hid_device_t* device) {
    if (device == g_active_device) {
        g_active_device = nullptr;
        publish_state(make_neutral_state(), false);
        resume_connections();
    } else if (g_active_device == nullptr) {
        resume_connections();
    }
}

uni_error_t platform_on_device_ready(uni_hid_device_t* device) {
    if (!uni_hid_device_is_gamepad(device)) {
        return UNI_ERROR_INVALID_CONTROLLER;
    }
    if (g_active_device != nullptr && g_active_device != device) {
        return UNI_ERROR_NO_SLOTS;
    }

    g_active_device = device;
    publish_state(make_neutral_state(), true);
    uni_bt_stop_scanning_unsafe();
    uni_bt_allow_incoming_connections(false);
    return UNI_ERROR_SUCCESS;
}

void platform_on_controller_data(uni_hid_device_t* device, uni_controller_t* controller) {
    if (device != g_active_device || controller == nullptr || controller->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }
    publish_state(map_gamepad(controller->gamepad), true);
}

const uni_property_t* platform_get_property(uni_property_idx_t index) {
    (void)index;
    return nullptr;
}

void platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    (void)event;
    (void)data;
}

uni_platform* get_platform() {
    static uni_platform platform = {
        "Switch Pico",
        platform_init,
        platform_on_init_complete,
        platform_on_device_discovered,
        platform_on_device_connected,
        platform_on_device_disconnected,
        platform_on_device_ready,
        nullptr,
        platform_on_controller_data,
        platform_get_property,
        platform_on_oob_event,
        nullptr,
        nullptr,
    };
    return &platform;
}

[[noreturn]] void core1_main() {
    if (cyw43_arch_init() != 0) {
        publish_state(make_neutral_state(), false);
        while (true) {
            tight_loop_contents();
        }
    }

    uni_platform_set_custom(get_platform());
    if (uni_init(0, nullptr) != 0) {
        publish_state(make_neutral_state(), false);
        while (true) {
            tight_loop_contents();
        }
    }

    btstack_run_loop_execute();
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace

void bluepad32_input_backend_init() {
    if (g_initialized) {
        return;
    }

    critical_section_init(&g_state_lock);
    queue_init(&g_rumble_queue, sizeof(RumblePacket), kRumbleQueueDepth);
    g_shared_state = make_neutral_state();
    g_shared_controller_active = false;
    g_shared_generation = 0;
    g_consumed_generation = 0;
    g_last_snapshot_generation = 0;
    g_initialized = true;
}

void bluepad32_input_backend_start() {
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }
    if (g_started) {
        return;
    }

    g_started = true;
    multicore_launch_core1(core1_main);
}

bool bluepad32_input_backend_snapshot(SwitchInputState* out) {
    if (out == nullptr) {
        return false;
    }
    if (!g_initialized) {
        *out = make_neutral_state();
        return false;
    }

    critical_section_enter_blocking(&g_state_lock);
    *out = g_shared_state;
    const bool controller_active = g_shared_controller_active;
    const uint32_t generation = g_shared_generation;
    critical_section_exit(&g_state_lock);

    if (generation == g_consumed_generation) {
        out->imu_sample_count = 0;
    }
    g_last_snapshot_generation = generation;
    return controller_active;
}

void bluepad32_input_backend_report_sent() {
    if (!g_initialized) {
        return;
    }
    g_consumed_generation = g_last_snapshot_generation;
}

void bluepad32_input_backend_queue_rumble(const uint8_t rumble[8]) {
    if (!g_initialized || rumble == nullptr) {
        return;
    }

    RumblePacket packet{};
    memcpy(packet.bytes, rumble, sizeof(packet.bytes));
    if (!queue_try_add(&g_rumble_queue, &packet)) {
        RumblePacket discarded{};
        (void)queue_try_remove(&g_rumble_queue, &discarded);
        (void)queue_try_add(&g_rumble_queue, &packet);
    }
}
