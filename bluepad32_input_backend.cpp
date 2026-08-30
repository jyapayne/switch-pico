#include "bluepad32_input_backend.h"

#include <limits.h>
#include <stddef.h>

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
constexpr uint8_t kRumbleActivityLedTicks = 20;

enum class ConnectionStatus {
    Initializing,
    Scanning,
    Connecting,
    Ready,
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
ConnectionStatus g_connection_status = ConnectionStatus::Initializing;
uint16_t g_status_led_tick = 0;
bool g_status_led_on = false;
uint8_t g_rumble_activity_led_ticks = 0;

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

constexpr int32_t clamp_axis(int32_t value) {
    if (value < kAxisMinimum) {
        return kAxisMinimum;
    }
    if (value > kAxisMaximum) {
        return kAxisMaximum;
    }
    return value;
}

constexpr uint16_t scale_stick(int32_t value) {
    value = clamp_axis(value);
    if (value <= 0) {
        return static_cast<uint16_t>(
            (static_cast<int64_t>(value - kAxisMinimum) * kStickMidpoint) / -kAxisMinimum);
    }
    return static_cast<uint16_t>(
        kStickMidpoint + (static_cast<int64_t>(value) * (UINT16_MAX - kStickMidpoint)) / kAxisMaximum);
}

constexpr int16_t clamp_int16(int64_t value) {
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    return static_cast<int16_t>(value);
}

constexpr int64_t divide_round_nearest(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }
    return -((-numerator + denominator / 2) / denominator);
}

constexpr int16_t convert_accel(int64_t q13_value) {
    return clamp_int16(q13_value / 2);
}

constexpr int16_t convert_gyro(int64_t q10_value) {
    constexpr int64_t kNumeratorScale = 13371;
    constexpr int64_t kDenominator = 1024 * 936;
    return clamp_int16(divide_round_nearest(q10_value * kNumeratorScale, kDenominator));
}

static_assert(scale_stick(-512) == 0);
static_assert(scale_stick(0) == 32768);
static_assert(scale_stick(511) == UINT16_MAX);
static_assert(convert_accel(8192) == 4096);
static_assert(convert_accel(-8192) == -4096);
static_assert(convert_gyro(1024) == 14);
static_assert(convert_gyro(-1024) == -14);

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

void update_status_led() {
    ++g_status_led_tick;
    bool led_on = false;
    switch (g_connection_status) {
        case ConnectionStatus::Initializing:
            led_on = true;
            break;
        case ConnectionStatus::Ready:
            if (g_rumble_activity_led_ticks > 0) {
                --g_rumble_activity_led_ticks;
                led_on = false;
            } else {
                led_on = true;
            }
            break;
        case ConnectionStatus::Scanning:
            led_on = (g_status_led_tick % 200) < 100;
            break;
        case ConnectionStatus::Connecting:
            led_on = (g_status_led_tick % 40) < 20;
            break;
    }
    if (led_on != g_status_led_on) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        g_status_led_on = led_on;
    }
}

void process_rumble_timer(btstack_timer_source_t* timer) {
    SwitchRumbleOutput packet{};
    SwitchRumbleOutput latest{};
    bool have_packet = false;
    while (queue_try_remove(&g_rumble_queue, &packet)) {
        latest = packet;
        have_packet = true;
    }
    if (have_packet &&
        (latest.low_frequency_magnitude != 0 ||
         latest.high_frequency_magnitude != 0)) {
        g_rumble_activity_led_ticks = kRumbleActivityLedTicks;
    }

    if (have_packet && g_active_device != nullptr &&
        g_active_device->report_parser.play_dual_rumble != nullptr) {
        g_active_device->report_parser.play_dual_rumble(
            g_active_device, 0, kRumbleDurationMs,
            latest.high_frequency_magnitude, latest.low_frequency_magnitude);
    }
    update_status_led();

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
    g_connection_status = ConnectionStatus::Scanning;
    g_status_led_tick = 0;

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
    g_connection_status = ConnectionStatus::Connecting;
    g_status_led_tick = 0;
}

void resume_connections() {
    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
}

void platform_on_device_disconnected(uni_hid_device_t* device) {
    if (device == g_active_device) {
        g_active_device = nullptr;
        publish_state(make_neutral_state(), false);
        g_connection_status = ConnectionStatus::Scanning;
        g_status_led_tick = 0;
        resume_connections();
    } else if (g_active_device == nullptr) {
        resume_connections();
        g_connection_status = ConnectionStatus::Scanning;
        g_status_led_tick = 0;
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
    g_connection_status = ConnectionStatus::Ready;
    g_status_led_tick = 0;
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
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    g_status_led_on = true;

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
    queue_init(&g_rumble_queue, sizeof(SwitchRumbleOutput), kRumbleQueueDepth);
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

void bluepad32_input_backend_queue_rumble(const SwitchRumbleOutput& rumble) {
    if (!g_initialized) {
        return;
    }

    if (!queue_try_add(&g_rumble_queue, &rumble)) {
        SwitchRumbleOutput discarded{};
        (void)queue_try_remove(&g_rumble_queue, &discarded);
        (void)queue_try_add(&g_rumble_queue, &rumble);
    }
}
