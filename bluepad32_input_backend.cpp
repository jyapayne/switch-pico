#include "bluepad32_input_backend.h"
#include "controller_hotkey_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <btstack_run_loop.h>
#include <pico/critical_section.h>
#include <pico/cyw43_arch.h>
#include <pico/flash.h>
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
constexpr uint32_t kPairingWindowDurationMs = 60000;
constexpr uint8_t kAllBlePairingMethods =
    SM_STK_GENERATION_METHOD_JUST_WORKS |
    SM_STK_GENERATION_METHOD_OOB |
    SM_STK_GENERATION_METHOD_PASSKEY |
    SM_STK_GENERATION_METHOD_NUMERIC_COMPARISON;
constexpr uint32_t kAbxyHotkeyButtonMask =
    SWITCH_ABXY_HOTKEY_BUTTON_MASK;
constexpr uint32_t kAbxyHotkeyMiscMask = SWITCH_ABXY_HOTKEY_MISC_MASK;
constexpr bool kDefaultSwapAbxy = SWITCH_ABXY_DEFAULT_SWAPPED != 0;
constexpr uint32_t kAbxyFeedbackDurationMs =
    SWITCH_ABXY_FEEDBACK_DURATION_MS;
constexpr uint8_t kAbxyFeedbackWeakMagnitude =
    SWITCH_ABXY_FEEDBACK_WEAK_MAGNITUDE;
constexpr uint8_t kAbxyFeedbackStrongMagnitude =
    SWITCH_ABXY_FEEDBACK_STRONG_MAGNITUDE;
constexpr uint32_t kMotionHotkeyDpadMask =
    SWITCH_MOTION_HOTKEY_DPAD_MASK;
constexpr uint32_t kMotionHotkeyButtonMask =
    SWITCH_MOTION_HOTKEY_BUTTON_MASK;
constexpr uint32_t kMotionHotkeyMiscMask =
    SWITCH_MOTION_HOTKEY_MISC_MASK;
constexpr bool kDefaultMotionEnabled =
    SWITCH_MOTION_DEFAULT_ENABLED != 0;
constexpr uint16_t kMotionDisabledFeedbackDurationMs =
    SWITCH_MOTION_DISABLED_FEEDBACK_DURATION_MS;
constexpr uint8_t kMotionDisabledFeedbackWeakMagnitude =
    SWITCH_MOTION_DISABLED_FEEDBACK_WEAK_MAGNITUDE;
constexpr uint8_t kMotionDisabledFeedbackStrongMagnitude =
    SWITCH_MOTION_DISABLED_FEEDBACK_STRONG_MAGNITUDE;
constexpr uint16_t kMotionEnabledFeedbackDurationMs =
    SWITCH_MOTION_ENABLED_FEEDBACK_DURATION_MS;
constexpr uint8_t kMotionEnabledFeedbackWeakMagnitude =
    SWITCH_MOTION_ENABLED_FEEDBACK_WEAK_MAGNITUDE;
constexpr uint8_t kMotionEnabledFeedbackStrongMagnitude =
    SWITCH_MOTION_ENABLED_FEEDBACK_STRONG_MAGNITUDE;

static_assert(kAbxyHotkeyButtonMask != 0);
static_assert(kAbxyHotkeyMiscMask != 0);
static_assert(kAbxyFeedbackDurationMs > 0);
static_assert(kMotionHotkeyDpadMask != 0);
static_assert(kMotionHotkeyButtonMask != 0);
static_assert(kMotionHotkeyMiscMask != 0);
static_assert(kMotionDisabledFeedbackDurationMs > 0);
static_assert(kMotionEnabledFeedbackDurationMs > 0);

static_assert(kSlotCount == 4);
static_assert(SWITCH_PICO_HID_INSTANCE_COUNT == kSlotCount);

enum class ConnectionStatus {
    Initializing,
    Scanning,
    Connecting,
    Ready,
};

enum class ConnectionPolicyState {
    Uninitialized,
    Open,
    Paused,
    FailedClosed,
};

struct RumbleEnvelope {
    uint8_t slot;
    uint32_t connection_generation;
    SwitchRumbleOutput rumble;
};
struct FeedbackEnvelope {
    uint32_t connection_generation;
    uint16_t duration_ms;
    uint8_t weak_magnitude;
    uint8_t strong_magnitude;
};


struct BackendSlot {
    SwitchInputState state;
    // Non-null with active=false is a connected device still becoming ready.
    uni_hid_device_t* device;
    uint32_t state_generation;
    uint32_t connection_generation;
    bool active;
    bool rumble_pending;
    bool swap_abxy;
    bool abxy_hotkey_latched;
    bool motion_enabled;
    bool motion_hotkey_latched;
    bool feedback_pending;
    uint32_t feedback_until_ms;
    RumbleEnvelope pending_rumble;
    FeedbackEnvelope pending_feedback;
};

critical_section_t g_state_lock;
BackendSlot g_slots[kSlotCount];

// These acknowledgement generations and the pairing request producer are only
// used by Core 0. The request is transferred under the cross-core state lock.
uint32_t g_consumed_generation[kSlotCount]{};
uint32_t g_last_snapshot_generation[kSlotCount]{};
bool g_pairing_window_requested = false;
bool g_initialized = false;
bool g_started = false;

// These fields are only read or written by Core 1 / BTstack.
btstack_timer_source_t g_rumble_timer{};
ConnectionStatus g_connection_status = ConnectionStatus::Initializing;
btstack_packet_callback_registration_t g_pairing_event_callback{};
ConnectionPolicyState g_connection_policy_state =
    ConnectionPolicyState::Uninitialized;
uint32_t g_pairing_window_deadline_ms = 0;
uint16_t g_status_led_tick = 0;
bool g_pairing_window_open = false;
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

bool has_free_slot() {
    critical_section_enter_blocking(&g_state_lock);
    bool free_slot = false;
    for (const BackendSlot& slot : g_slots) {
        free_slot = free_slot || slot.device == nullptr;
    }
    critical_section_exit(&g_state_lock);
    return free_slot;
}

bool has_active_controller() {
    critical_section_enter_blocking(&g_state_lock);
    bool active_controller = false;
    for (const BackendSlot& slot : g_slots) {
        active_controller = active_controller || slot.active;
    }
    critical_section_exit(&g_state_lock);
    return active_controller;
}

int slot_for_device(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return -1;
    }
    const int slot = uni_hid_device_get_idx_for_instance(device);
    return slot >= 0 && slot < kSlotCount ? slot : -1;
}
void apply_slot_lighting(uint8_t slot_index, uni_hid_device_t* device) {
    const SwitchRgbColor color =
        switch_pro_get_slot_light_color(slot_index);
    if (device->report_parser.set_lightbar_color != nullptr) {
        device->report_parser.set_lightbar_color(
            device, color.red, color.green, color.blue);
    } else if (device->report_parser.set_player_leds != nullptr) {
        device->report_parser.set_player_leds(
            device, static_cast<uint8_t>(1u << slot_index));
    }
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
    g_connection_policy_state = ConnectionPolicyState::FailedClosed;
    g_pairing_window_open = false;
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

constexpr int64_t divide_round_nearest(int64_t numerator,
                                       int64_t denominator) {
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

SwitchInputState map_gamepad(const uni_gamepad_t& gamepad,
                              bool swap_abxy,
                              bool motion_enabled) {
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
    if (swap_abxy) {
        bool temporary = state.button_a;
        state.button_a = state.button_b;
        state.button_b = temporary;
        temporary = state.button_x;
        state.button_x = state.button_y;
        state.button_y = temporary;
    }
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

    if (motion_enabled && has_motion(gamepad)) {
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
struct HotkeyDecision {
    bool swap_abxy;
    bool motion_enabled;
    uint32_t suppress_dpad;
    uint32_t suppress_buttons;
    uint32_t suppress_misc_buttons;
};

void queue_local_feedback(BackendSlot& slot, uint16_t duration_ms,
                          uint8_t weak_magnitude,
                          uint8_t strong_magnitude) {
    slot.feedback_pending = true;
    slot.pending_feedback = {
        slot.connection_generation, duration_ms, weak_magnitude,
        strong_magnitude};
}
void reset_slot_hotkeys(BackendSlot& slot) {
    slot.swap_abxy = kDefaultSwapAbxy;
    slot.abxy_hotkey_latched = false;
    slot.motion_enabled = kDefaultMotionEnabled;
    slot.motion_hotkey_latched = false;
    slot.feedback_pending = false;
    slot.feedback_until_ms = 0;
    slot.pending_feedback = {};
}


HotkeyDecision update_controller_hotkeys(
    uint8_t slot_index, uni_hid_device_t* device,
    const uni_gamepad_t& gamepad) {
    const bool abxy_pressed =
        (gamepad.buttons & kAbxyHotkeyButtonMask) ==
            kAbxyHotkeyButtonMask &&
        (gamepad.misc_buttons & kAbxyHotkeyMiscMask) ==
            kAbxyHotkeyMiscMask;
    const bool motion_pressed =
        (gamepad.dpad & kMotionHotkeyDpadMask) ==
            kMotionHotkeyDpadMask &&
        (gamepad.buttons & kMotionHotkeyButtonMask) ==
            kMotionHotkeyButtonMask &&
        (gamepad.misc_buttons & kMotionHotkeyMiscMask) ==
            kMotionHotkeyMiscMask;
    HotkeyDecision decision{
        kDefaultSwapAbxy, kDefaultMotionEnabled, 0, 0, 0};

    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device == device) {
        if (abxy_pressed && !slot.abxy_hotkey_latched) {
            slot.swap_abxy = !slot.swap_abxy;
            queue_local_feedback(
                slot, static_cast<uint16_t>(kAbxyFeedbackDurationMs),
                kAbxyFeedbackWeakMagnitude,
                kAbxyFeedbackStrongMagnitude);
        } else if (motion_pressed && !slot.motion_hotkey_latched) {
            slot.motion_enabled = !slot.motion_enabled;
            if (slot.motion_enabled) {
                queue_local_feedback(
                    slot, kMotionEnabledFeedbackDurationMs,
                    kMotionEnabledFeedbackWeakMagnitude,
                    kMotionEnabledFeedbackStrongMagnitude);
            } else {
                queue_local_feedback(
                    slot, kMotionDisabledFeedbackDurationMs,
                    kMotionDisabledFeedbackWeakMagnitude,
                    kMotionDisabledFeedbackStrongMagnitude);
            }
        }
        slot.abxy_hotkey_latched = abxy_pressed;
        slot.motion_hotkey_latched = motion_pressed;
        decision.swap_abxy = slot.swap_abxy;
        decision.motion_enabled = slot.motion_enabled;
        if (abxy_pressed) {
            decision.suppress_buttons |= kAbxyHotkeyButtonMask;
            decision.suppress_misc_buttons |= kAbxyHotkeyMiscMask;
        }
        if (motion_pressed) {
            decision.suppress_dpad |= kMotionHotkeyDpadMask;
            decision.suppress_buttons |= kMotionHotkeyButtonMask;
            decision.suppress_misc_buttons |= kMotionHotkeyMiscMask;
        }
    }
    critical_section_exit(&g_state_lock);
    return decision;
}


bool pairing_window_active_at(uint32_t now_ms) {
    return g_pairing_window_open &&
           static_cast<int32_t>(now_ms - g_pairing_window_deadline_ms) < 0;
}

void handle_pairing_hci_event(uint8_t packet_type, uint16_t channel,
                              uint8_t* packet, uint16_t size) {
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET || packet == nullptr || size < 8) {
        return;
    }

    bd_addr_t address{};
    const bool pairing_open =
        pairing_window_active_at(btstack_run_loop_get_time_ms());
    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            hci_event_user_confirmation_request_get_bd_addr(packet, address);
            if (pairing_open) {
                gap_ssp_confirmation_response(address);
            } else {
                gap_ssp_confirmation_negative(address);
            }
            break;
        case HCI_EVENT_USER_PASSKEY_REQUEST:
            hci_event_user_passkey_request_get_bd_addr(packet, address);
            if (pairing_open) {
                gap_ssp_passkey_response(address, 0);
            } else {
                gap_ssp_passkey_negative(address);
            }
            break;
        default:
            break;
    }
}

bool update_pairing_window(uint32_t now_ms) {
    critical_section_enter_blocking(&g_state_lock);
    const bool requested = g_pairing_window_requested;
    g_pairing_window_requested = false;
    critical_section_exit(&g_state_lock);

    if (requested) {
        g_pairing_window_open = true;
        g_pairing_window_deadline_ms = now_ms + kPairingWindowDurationMs;
        gap_set_bondable_mode(true);
        sm_set_accepted_stk_generation_methods(kAllBlePairingMethods);
        g_status_led_tick = 0;
        return true;
    }
    if (g_pairing_window_open && !pairing_window_active_at(now_ms)) {
        g_pairing_window_open = false;
        sm_set_accepted_stk_generation_methods(0);
        g_status_led_tick = 0;
        gap_set_bondable_mode(false);
        return true;
    }
    return false;
}

void apply_connection_policy() {
    const bool free_slot = has_free_slot();
    if ((!free_slot &&
         g_connection_policy_state == ConnectionPolicyState::Paused) ||
        (free_slot &&
         g_connection_policy_state == ConnectionPolicyState::Open)) {
        return;
    }

    uni_bt_allow_incoming_connections(false);
    uni_bt_stop_scanning_unsafe();

    if (!free_slot) {
        g_connection_policy_state = ConnectionPolicyState::Paused;
        return;
    }

    // Bluepad32's normal scan/autoconnect path handles both remembered
    // controllers powering on and controllers in explicit pairing mode.
    uni_bt_allow_incoming_connections(true);
    uni_bt_start_scanning_and_autoconnect_unsafe();
    g_connection_policy_state = ConnectionPolicyState::Open;
}

void update_status_led() {
    ++g_status_led_tick;
    bool led_on = false;

    if (pairing_window_active_at(btstack_run_loop_get_time_ms())) {
        const uint16_t phase = g_status_led_tick % 200;
        led_on = phase < 20 || (phase >= 40 && phase < 60);
    } else if (g_connection_status == ConnectionStatus::Connecting) {
        led_on = (g_status_led_tick % 40) < 20;
    } else if (g_connection_status == ConnectionStatus::Initializing ||
               has_active_controller()) {
        led_on = true;
    } else {
        led_on = (g_status_led_tick % 200) < 100;
    }

    if (led_on != g_status_led_on) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        g_status_led_on = led_on;
    }
}

void process_rumble_timer(btstack_timer_source_t* timer) {
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    update_pairing_window(now_ms);

    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        RumbleEnvelope envelope{};
        FeedbackEnvelope feedback{};
        uni_hid_device_t* device = nullptr;
        bool feedback_dispatch = false;
        bool host_dispatch = false;

        critical_section_enter_blocking(&g_state_lock);
        BackendSlot& slot = g_slots[slot_index];
        if (slot.feedback_pending) {
            feedback = slot.pending_feedback;
            feedback_dispatch =
                slot.active && slot.device != nullptr &&
                feedback.connection_generation ==
                    slot.connection_generation &&
                slot.device->report_parser.play_dual_rumble != nullptr;
            slot.feedback_pending = false;
            if (feedback_dispatch) {
                device = slot.device;
                slot.feedback_until_ms =
                    now_ms + feedback.duration_ms;
            }
        }

        const bool feedback_active =
            static_cast<int32_t>(now_ms - slot.feedback_until_ms) < 0;
        if (!feedback_dispatch && !feedback_active &&
            slot.rumble_pending) {
            envelope = slot.pending_rumble;
            slot.rumble_pending = false;
            host_dispatch =
                envelope.slot == slot_index && slot.active &&
                slot.device != nullptr &&
                envelope.connection_generation ==
                    slot.connection_generation;
            if (host_dispatch) {
                device = slot.device;
            }
        }
        critical_section_exit(&g_state_lock);

        if (feedback_dispatch) {
            device->report_parser.play_dual_rumble(
                device, 0, feedback.duration_ms,
                feedback.weak_magnitude, feedback.strong_magnitude);
        } else if (host_dispatch &&
                   device->report_parser.play_dual_rumble != nullptr) {
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

void recompute_connection_status() {
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    update_pairing_window(now_ms);
    g_connection_status = compute_connection_status();
    g_status_led_tick = 0;
    apply_connection_policy();
}

void platform_init(int argc, const char** argv) {
    (void)argc;
    (void)argv;
}

void platform_on_init_complete() {
    gap_set_bondable_mode(false);
    sm_set_accepted_stk_generation_methods(0);
    gap_ssp_set_auto_accept(false);
    g_pairing_event_callback.callback = handle_pairing_hci_event;
    hci_add_event_handler(&g_pairing_event_callback);
    // Keep Bluepad32 autoconnect active whenever at least one slot is free.
    btstack_run_loop_set_timer_handler(&g_rumble_timer, process_rumble_timer);
    btstack_run_loop_set_timer(&g_rumble_timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(&g_rumble_timer);
    recompute_connection_status();
}

uni_error_t platform_on_device_discovered(bd_addr_t addr, const char* name,
                                          uint16_t cod, uint8_t rssi) {
    (void)addr;
    (void)name;
    (void)cod;
    (void)rssi;
    return has_free_slot() &&
                   g_connection_policy_state == ConnectionPolicyState::Open
               ? UNI_ERROR_SUCCESS
               : UNI_ERROR_IGNORE_DEVICE;
}

void platform_on_device_connected(uni_hid_device_t* device) {
    if (device == nullptr) {
        return;
    }
    if (g_connection_policy_state != ConnectionPolicyState::Open) {
        uni_hid_device_disconnect(device);
        return;
    }

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
        reset_slot_hotkeys(slot);
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
        reset_slot_hotkeys(slot);
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
    bool became_active = false;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    occupied_mismatch = slot.device != nullptr && slot.device != device;
    if (!occupied_mismatch) {
        slot.device = device;
        if (!slot.active) {
            slot.state = make_neutral_state();
            slot.active = true;
            slot.rumble_pending = false;
            reset_slot_hotkeys(slot);
            ++slot.state_generation;
            became_active = true;
        }
    }
    critical_section_exit(&g_state_lock);

    if (occupied_mismatch) {
        return UNI_ERROR_NO_SLOTS;
    }
    if (became_active) {
        apply_slot_lighting(static_cast<uint8_t>(slot_index), device);
    }


    recompute_connection_status();
    return UNI_ERROR_SUCCESS;
}

void platform_on_controller_data(uni_hid_device_t* device,
                                 uni_controller_t* controller) {
    const int slot_index = slot_for_device(device);
    if (slot_index < 0 || controller == nullptr ||
        controller->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    uni_gamepad_t gamepad = controller->gamepad;
    const HotkeyDecision hotkeys = update_controller_hotkeys(
        static_cast<uint8_t>(slot_index), device, gamepad);
    gamepad.dpad &= ~hotkeys.suppress_dpad;
    gamepad.buttons &= ~hotkeys.suppress_buttons;
    gamepad.misc_buttons &= ~hotkeys.suppress_misc_buttons;
    publish_device_state(
        static_cast<uint8_t>(slot_index), device,
        map_gamepad(gamepad, hotkeys.swap_abxy,
                    hotkeys.motion_enabled));
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

[[noreturn]] void halt_wireless_backend() {
    publish_all_neutral();
    while (true) {
        tight_loop_contents();
    }
}

[[noreturn]] void core1_main() {
    if (!flash_safe_execute_core_init()) {
        halt_wireless_backend();
    }
    if (cyw43_arch_init() != 0) {
        halt_wireless_backend();
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    g_status_led_on = true;

    uni_platform_set_custom(get_platform());
    if (uni_init(0, nullptr) != 0) {
        halt_wireless_backend();
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
        reset_slot_hotkeys(slot);
        g_consumed_generation[slot_index] = 0;
        g_last_snapshot_generation[slot_index] = 0;
    }
    g_pairing_window_requested = false;
    g_connection_status = ConnectionStatus::Initializing;
    g_connection_policy_state = ConnectionPolicyState::Uninitialized;
    g_pairing_window_deadline_ms = 0;
    g_pairing_window_open = false;
    g_initialized = true;
}

void bluepad32_input_backend_start() {
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }
    if (g_started) {
        return;
    }
    // Core 0 services USB from flash while Core 1 owns BTstack. Register both
    // cores before either side can initiate a flash-backed BTstack TLV write.
    if (!flash_safe_execute_core_init()) {
        g_connection_policy_state = ConnectionPolicyState::FailedClosed;
        return;
    }


    g_started = true;
    multicore_launch_core1(core1_main);
}

void bluepad32_input_backend_open_pairing_window() {
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }

    critical_section_enter_blocking(&g_state_lock);
    g_pairing_window_requested = true;
    critical_section_exit(&g_state_lock);
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
