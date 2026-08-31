#include "bluepad32_input_backend.h"

#include <limits.h>
#include <stddef.h>

#include <btstack_run_loop.h>
#include <pico/critical_section.h>
#include <pico/cyw43_arch.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <uni.h>

namespace {

constexpr uint16_t kStickMidpoint = 32768;
constexpr int32_t kAxisMinimum = -512;
constexpr int32_t kAxisMaximum = 511;
constexpr int32_t kTriggerMaximum = 1023;
constexpr int32_t kTriggerThreshold = (kTriggerMaximum * 35) / 100;
constexpr uint16_t kRumbleDurationMs = 50;
constexpr uint32_t kRumblePollIntervalMs = 5;
constexpr uint8_t kSlotCount = BLUEPAD32_INPUT_BACKEND_SLOT_COUNT;

static_assert(kSlotCount == 2);
static_assert(SWITCH_PICO_HID_INSTANCE_COUNT == kSlotCount);

enum class ConnectionStatus {
    Initializing,
    Scanning,
    Connecting,
    Ready,
};

struct RumbleEnvelope {
    uint8_t slot;
    uint32_t connection_generation;
    SwitchRumbleOutput rumble;
};

struct BackendSlot {
    SwitchInputState state;
    // Non-null with active=false is a connected device still becoming ready.
    uni_hid_device_t* device;
    uint32_t state_generation;
    uint32_t connection_generation;
    bool active;
    bool rumble_pending;
    RumbleEnvelope pending_rumble;
};

critical_section_t g_state_lock;
BackendSlot g_slots[kSlotCount];

// These acknowledgement generations are only read or written by Core 0.
uint32_t g_consumed_generation[kSlotCount]{};
uint32_t g_last_snapshot_generation[kSlotCount]{};
bool g_initialized = false;
bool g_started = false;

// The timer and connection status are only read or written by Core 1 / BTstack.
btstack_timer_source_t g_rumble_timer{};
ConnectionStatus g_connection_status = ConnectionStatus::Initializing;
uint16_t g_status_led_tick = 0;
bool g_status_led_on = false;

SwitchInputState make_neutral_state() {
    SwitchInputState state{};
    state.lx = kStickMidpoint;
    state.ly = kStickMidpoint;
    state.rx = kStickMidpoint;
    state.ry = kStickMidpoint;
    return state;
}

bool valid_slot(uint8_t slot) {
    return slot < kSlotCount;
}

int slot_for_device(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return -1;
    }
    const int slot = uni_hid_device_get_idx_for_instance(device);
    return slot >= 0 && slot < kSlotCount ? slot : -1;
}

ConnectionStatus compute_connection_status() {
    critical_section_enter_blocking(&g_state_lock);
    bool all_ready = true;
    bool any_connecting = false;
    for (const BackendSlot& slot : g_slots) {
        const bool has_device = slot.device != nullptr;
        all_ready = all_ready && slot.active && has_device;
        any_connecting = any_connecting || (!slot.active && has_device);
    }
    critical_section_exit(&g_state_lock);

    if (all_ready) {
        return ConnectionStatus::Ready;
    }
    return any_connecting ? ConnectionStatus::Connecting
                          : ConnectionStatus::Scanning;
}

void publish_device_state(uint8_t slot, uni_hid_device_t* device,
                          const SwitchInputState& state) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& target = g_slots[slot];
    if (target.active && target.device == device) {
        target.state = state;
        ++target.state_generation;
    }
    critical_section_exit(&g_state_lock);
}

void publish_all_neutral() {
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        slot.state = make_neutral_state();
        slot.device = nullptr;
        slot.active = false;
        slot.rumble_pending = false;
        ++slot.state_generation;
        ++slot.connection_generation;
    }
    critical_section_exit(&g_state_lock);
    g_connection_status = ConnectionStatus::Initializing;
    g_status_led_tick = 0;
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
        case ConnectionStatus::Ready:
            led_on = true;
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
    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        RumbleEnvelope envelope{};
        uni_hid_device_t* device = nullptr;
        bool dispatch = false;

        critical_section_enter_blocking(&g_state_lock);
        BackendSlot& slot = g_slots[slot_index];
        if (slot.rumble_pending) {
            envelope = slot.pending_rumble;
            slot.rumble_pending = false;
            dispatch = envelope.slot == slot_index && slot.active &&
                       slot.device != nullptr &&
                       envelope.connection_generation == slot.connection_generation;
            if (dispatch) {
                device = slot.device;
            }
        }
        critical_section_exit(&g_state_lock);

        if (dispatch && device->report_parser.play_dual_rumble != nullptr) {
            device->report_parser.play_dual_rumble(
                device, 0, kRumbleDurationMs,
                envelope.rumble.high_frequency_magnitude,
                envelope.rumble.low_frequency_magnitude);
        }
    }

    update_status_led();
    btstack_run_loop_set_timer(timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(timer);
}

void resume_connections() {
    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
}
void recompute_connection_status() {
    g_connection_status = compute_connection_status();
    g_status_led_tick = 0;
    if (g_connection_status == ConnectionStatus::Ready) {
        uni_bt_stop_scanning_unsafe();
        uni_bt_allow_incoming_connections(false);
    } else {
        resume_connections();
    }
}

void platform_init(int argc, const char** argv) {
    (void)argc;
    (void)argv;
}

void platform_on_init_complete() {
    btstack_run_loop_set_timer_handler(&g_rumble_timer, process_rumble_timer);
    btstack_run_loop_set_timer(&g_rumble_timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(&g_rumble_timer);
    recompute_connection_status();
}

uni_error_t platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    (void)addr;
    (void)name;
    (void)cod;
    (void)rssi;
    return compute_connection_status() == ConnectionStatus::Ready
               ? UNI_ERROR_IGNORE_DEVICE
               : UNI_ERROR_SUCCESS;
}

void platform_on_device_connected(uni_hid_device_t* device) {
    const int slot_index = slot_for_device(device);
    if (slot_index < 0) {
        return;
    }

    bool tracked_connection = false;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (!slot.active && slot.device == nullptr) {
        slot.device = device;
        slot.rumble_pending = false;
        tracked_connection = true;
    } else {
        tracked_connection = slot.device == device;
    }
    critical_section_exit(&g_state_lock);

    if (tracked_connection) {
        recompute_connection_status();
    }
}

void platform_on_device_disconnected(uni_hid_device_t* device) {
    const int slot_index = slot_for_device(device);
    if (slot_index < 0) {
        return;
    }

    bool disconnected_tracked_device = false;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.device == device) {
        if (slot.active) {
            slot.state = make_neutral_state();
            ++slot.state_generation;
        }
        slot.device = nullptr;
        slot.active = false;
        slot.rumble_pending = false;
        ++slot.connection_generation;
        disconnected_tracked_device = true;
    }
    critical_section_exit(&g_state_lock);

    if (disconnected_tracked_device) {
        recompute_connection_status();
    }
}

uni_error_t platform_on_device_ready(uni_hid_device_t* device) {
    if (device == nullptr || !uni_hid_device_is_gamepad(device)) {
        return UNI_ERROR_INVALID_CONTROLLER;
    }

    const int slot_index = slot_for_device(device);
    if (slot_index < 0) {
        return UNI_ERROR_NO_SLOTS;
    }

    bool occupied_mismatch = false;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    occupied_mismatch = slot.device != nullptr && slot.device != device;
    if (!occupied_mismatch) {
        slot.device = device;
        if (!slot.active) {
            slot.state = make_neutral_state();
            slot.active = true;
            slot.rumble_pending = false;
            ++slot.state_generation;
        }
    }
    critical_section_exit(&g_state_lock);

    if (occupied_mismatch) {
        return UNI_ERROR_NO_SLOTS;
    }

    recompute_connection_status();
    return UNI_ERROR_SUCCESS;
}

void platform_on_controller_data(uni_hid_device_t* device, uni_controller_t* controller) {
    const int slot_index = slot_for_device(device);
    if (slot_index < 0 || controller == nullptr ||
        controller->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }
    publish_device_state(static_cast<uint8_t>(slot_index), device,
                         map_gamepad(controller->gamepad));
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
        publish_all_neutral();
        while (true) {
            tight_loop_contents();
        }
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    g_status_led_on = true;

    uni_platform_set_custom(get_platform());
    if (uni_init(0, nullptr) != 0) {
        publish_all_neutral();
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
    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        BackendSlot& slot = g_slots[slot_index];
        slot = {};
        slot.state = make_neutral_state();
        slot.pending_rumble.slot = slot_index;
        g_consumed_generation[slot_index] = 0;
        g_last_snapshot_generation[slot_index] = 0;
    }
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

bool bluepad32_input_backend_snapshot(uint8_t slot_index, SwitchInputState* out) {
    if (out == nullptr || !valid_slot(slot_index)) {
        return false;
    }
    if (!g_initialized) {
        *out = make_neutral_state();
        return false;
    }

    critical_section_enter_blocking(&g_state_lock);
    *out = g_slots[slot_index].state;
    const bool controller_active = g_slots[slot_index].active;
    const uint32_t generation = g_slots[slot_index].state_generation;
    critical_section_exit(&g_state_lock);

    if (generation == g_consumed_generation[slot_index]) {
        out->imu_sample_count = 0;
    }
    g_last_snapshot_generation[slot_index] = generation;
    return controller_active;
}

void bluepad32_input_backend_report_sent(uint8_t slot_index) {
    if (!g_initialized || !valid_slot(slot_index)) {
        return;
    }
    g_consumed_generation[slot_index] = g_last_snapshot_generation[slot_index];
}

void bluepad32_input_backend_queue_rumble(uint8_t slot_index,
                                          const SwitchRumbleOutput& rumble) {
    if (!g_initialized || !valid_slot(slot_index)) {
        return;
    }

    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device != nullptr) {
        slot.pending_rumble = {slot_index, slot.connection_generation, rumble};
        slot.rumble_pending = true;
    }
    critical_section_exit(&g_state_lock);
}
