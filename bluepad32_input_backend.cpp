#include "bluepad32_input_backend.h"
#include "controller_hotkey_config.h"
#include "configuration_service.h"
#include "profile_service.h"

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
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
#include "adapter_usb_mode.h"
#endif

namespace {

constexpr int32_t kAxisMinimum = -512;
constexpr int32_t kAxisMaximum = 511;
constexpr int32_t kTriggerMaximum = 1023;
constexpr int32_t kTriggerFullScaleMinimum = 1020;
constexpr uint16_t kSwitchHostRumbleDurationMs = 50;
// XInput vibration is stateful and remains active until XInputSetState sends
// a new magnitude.
constexpr uint16_t kXInputHostRumbleDurationMs = UINT16_MAX;
constexpr uint32_t kRumblePollIntervalMs = 5;
constexpr uint32_t kConfigurationPollIntervalMs = 50;
constexpr uint8_t kSlotCount = BLUEPAD32_INPUT_BACKEND_SLOT_COUNT;
constexpr uint32_t kDefaultPairingWindowDurationMs =
    ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT * 1000u;
constexpr uint32_t kPairingResetFeedbackDurationMs = 2000;
// Bluetooth Classic units are 0.625 ms: 0x1900 = 4 seconds.
constexpr uint16_t kClassicLinkSupervisionTimeout = 0x1900;
constexpr uint8_t kAllBlePairingMethods =
    SM_STK_GENERATION_METHOD_JUST_WORKS |
    SM_STK_GENERATION_METHOD_OOB |
    SM_STK_GENERATION_METHOD_PASSKEY |
    SM_STK_GENERATION_METHOD_NUMERIC_COMPARISON;
constexpr uint16_t kProfileFeedbackPhaseDurationMs = 75;
constexpr uint8_t kProfileFeedbackWeakMagnitude = UINT8_MAX;
constexpr uint8_t kProfileFeedbackStrongMagnitude = UINT8_MAX;
// One initial indication can be followed by one committed switch before the
// Core 1 timer drains the queue. Profile commits are rate-limited well beyond
// the longest feedback sequence.
constexpr uint8_t kProfileFeedbackQueueCapacity = 2;
constexpr SwitchRgbColor kProfileLightbarPalette[CONTROLLER_PROFILE_COUNT] = {
    {0x00, 0x55, 0xff},
    {0x00, 0xcc, 0x66},
    {0xff, 0xaa, 0x00},
    {0xcc, 0x33, 0xff},
};
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

static_assert(kProfileFeedbackPhaseDurationMs == 75);
static_assert(CONTROLLER_PROFILE_COUNT == 4);
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
    Passive,
    Paused,
    FailedClosed,
};

struct RumbleEnvelope {
    uint8_t slot;
    uint32_t connection_generation;
    ControllerRumbleOutput rumble;
    uint16_t duration_ms;
};
struct FeedbackEnvelope {
    uint32_t connection_generation;
    uint16_t duration_ms;
    uint8_t weak_magnitude;
    uint8_t strong_magnitude;
};
struct ProfileFeedbackEnvelope {
    uint32_t connection_generation;
    uint8_t active_profile_number;
    ControllerProfileConfirmationPolicy policy;
};

struct ProfileFeedbackSequence {
    uint32_t connection_generation;
    uint32_t phase_deadline_ms;
    uint8_t pulse_count;
    uint8_t pulses_started;
    bool active;
    bool on;
    bool rumble_enabled;
    bool led_enabled;
};


// Security Manager identity events arrive before Bluepad32 publishes a ready
// device. Retain only the four live handle/address associations so a BLE RPA
// is never promoted to a stable identity on its own.
struct BleIdentityMapping {
    bool used;
    hci_con_handle_t connection_handle;
    bd_addr_t connection_address;
    uint8_t identity_address_type;
    bd_addr_t identity_address;
};


struct BackendSlot {
    ControllerState state;
    uint16_t pre_hotkey_button_mask;
    ControllerIdentity identity;
    // Non-null with active=false is a connected device still becoming ready.
    uni_hid_device_t* device;
    uint32_t state_generation;
    uint32_t connection_generation;
    bool active;
    bool rumble_pending;
    bool motion_enabled;
    bool motion_hotkey_latched;
    bool feedback_pending;
    uint32_t feedback_until_ms;
    uint8_t pending_profile_feedback_count;
    RumbleEnvelope pending_rumble;
    bool retained_host_rumble_valid;
    RumbleEnvelope retained_host_rumble;
    FeedbackEnvelope pending_feedback;
    ProfileFeedbackEnvelope
        pending_profile_feedback[kProfileFeedbackQueueCapacity];
    ProfileFeedbackSequence profile_feedback;
};

critical_section_t g_state_lock;
BackendSlot g_slots[kSlotCount];
BleIdentityMapping g_ble_identity_mappings[kSlotCount]{};

// These acknowledgement generations and request producers are only used by
// Core 0. Requests are transferred under the cross-core state lock.
uint32_t g_consumed_generation[kSlotCount]{};
uint32_t g_last_snapshot_generation[kSlotCount]{};
bool g_pairing_window_requested = false;
uint32_t g_clear_pairings_requested_token = 0;
uint32_t g_clear_pairings_in_progress_token = 0;
uint32_t g_next_clear_pairings_request_token = 1;
bool g_pairing_snapshot_requested = false;
bool g_initialized = false;
bool g_started = false;

// These fields are only read or written by Core 1 / BTstack.
btstack_timer_source_t g_rumble_timer{};
btstack_timer_source_t g_configuration_timer{};
ConnectionStatus g_connection_status = ConnectionStatus::Initializing;
btstack_packet_callback_registration_t g_pairing_event_callback{};
btstack_packet_callback_registration_t g_identity_event_callback{};
ConnectionPolicyState g_connection_policy_state =
    ConnectionPolicyState::Uninitialized;
uint32_t g_pairing_window_deadline_ms = 0;
uint32_t g_pairing_window_duration_ms =
    kDefaultPairingWindowDurationMs;
uint32_t g_pairing_reset_feedback_deadline_ms = 0;
uint16_t g_status_led_tick = 0;
bool g_pairing_window_open = false;
bool g_status_led_on = false;
Bluepad32PairingSnapshot g_pairing_snapshot{};

uint16_t host_rumble_duration_ms() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (adapter_host_probe_mode() == AdapterUsbMode::kXInput) {
        return kXInputHostRumbleDurationMs;
    }
#endif
    return kSwitchHostRumbleDurationMs;
}

ControllerState make_neutral_state() {
    return controller_neutral_state();
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

bool addresses_equal(const bd_addr_t first, const bd_addr_t second) {
    return memcmp(first, second, sizeof(bd_addr_t)) == 0;
}

BleIdentityMapping* find_ble_identity_mapping(
    hci_con_handle_t connection_handle,
    const bd_addr_t connection_address) {
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        if (mapping.used &&
            mapping.connection_handle == connection_handle &&
            addresses_equal(mapping.connection_address,
                            connection_address)) {
            return &mapping;
        }
    }
    return nullptr;
}

BleIdentityMapping* find_ble_identity_mapping_for_handle(
    hci_con_handle_t connection_handle) {
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        if (mapping.used &&
            mapping.connection_handle == connection_handle) {
            return &mapping;
        }
    }
    return nullptr;
}

BleIdentityMapping* reserve_ble_identity_mapping(
    hci_con_handle_t connection_handle) {
    BleIdentityMapping* available = nullptr;
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        if (mapping.used &&
            mapping.connection_handle == connection_handle) {
            return &mapping;
        }
        if (!mapping.used && available == nullptr) {
            available = &mapping;
        }
    }
    return available;
}

ControllerIdentity make_ble_identity(
    const BleIdentityMapping& mapping, const uni_hid_device_t* device) {
    ControllerIdentity identity{};
    identity.stable = true;
    identity.transport = ControllerTransport::kBle;
    identity.address_type = mapping.identity_address_type;
    memcpy(identity.address, mapping.identity_address,
           sizeof(identity.address));
    identity.vendor_id = device->vendor_id;
    identity.product_id = device->product_id;
    return identity;
}

ControllerIdentity identity_for_device(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return controller_identity_global();
    }
    switch (gap_get_connection_type(device->conn.handle)) {
        case GAP_CONNECTION_ACL: {
            ControllerIdentity identity{};
            identity.stable = true;
            identity.transport = ControllerTransport::kClassic;
            identity.address_type = BD_ADDR_TYPE_UNKNOWN;
            memcpy(identity.address, device->conn.btaddr,
                   sizeof(identity.address));
            identity.vendor_id = device->vendor_id;
            identity.product_id = device->product_id;
            return identity;
        }
        case GAP_CONNECTION_LE: {
            const BleIdentityMapping* mapping = find_ble_identity_mapping(
                device->conn.handle, device->conn.btaddr);
            if (mapping != nullptr) {
                return make_ble_identity(*mapping, device);
            }
            break;
        }
        case GAP_CONNECTION_INVALID:
        case GAP_CONNECTION_SCO:
            break;
    }
    return controller_identity_global();
}

void publish_ble_identity(const BleIdentityMapping& mapping) {
    ControllerIdentity observed_identity{};
    bool observe_identity = false;
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        if (slot.device != nullptr &&
            gap_get_connection_type(slot.device->conn.handle) ==
                GAP_CONNECTION_LE &&
            slot.device->conn.handle == mapping.connection_handle &&
            addresses_equal(slot.device->conn.btaddr,
                            mapping.connection_address)) {
            slot.identity = make_ble_identity(mapping, slot.device);
            if (slot.active) {
                observed_identity = slot.identity;
                observe_identity = true;
            }
        }
    }
    critical_section_exit(&g_state_lock);
    if (observe_identity) {
        profile_service_observe_identity_on_storage_core(
            observed_identity);
    }
}

void record_ble_identity(hci_con_handle_t connection_handle,
                         const bd_addr_t connection_address,
                         uint8_t identity_address_type,
                         const bd_addr_t identity_address) {
    BleIdentityMapping* mapping =
        reserve_ble_identity_mapping(connection_handle);
    if (mapping == nullptr) {
        return;
    }
    *mapping = {};
    mapping->used = true;
    mapping->connection_handle = connection_handle;
    memcpy(mapping->connection_address, connection_address,
           sizeof(mapping->connection_address));
    mapping->identity_address_type = identity_address_type;
    memcpy(mapping->identity_address, identity_address,
           sizeof(mapping->identity_address));
    publish_ble_identity(*mapping);
}

void clear_ble_identity_for_handle(hci_con_handle_t connection_handle) {
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        if (mapping.used &&
            mapping.connection_handle == connection_handle) {
            mapping = {};
        }
    }

    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        if (slot.device != nullptr &&
            gap_get_connection_type(slot.device->conn.handle) ==
                GAP_CONNECTION_LE &&
            slot.device->conn.handle == connection_handle) {
            slot.identity = controller_identity_global();
        }
    }
    critical_section_exit(&g_state_lock);
}

void clear_ble_identity_for_device(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return;
    }
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        if (mapping.used &&
            mapping.connection_handle == device->conn.handle &&
            addresses_equal(mapping.connection_address,
                            device->conn.btaddr)) {
            mapping = {};
        }
    }
}

void connection_address_for_handle(hci_con_handle_t connection_handle,
                                   const bd_addr_t fallback,
                                   bd_addr_t output) {
    const uni_hid_device_t* device =
        uni_hid_device_get_instance_for_connection_handle(
            connection_handle);
    if (device != nullptr) {
        memcpy(output, device->conn.btaddr, sizeof(bd_addr_t));
        return;
    }
    const BleIdentityMapping* mapping =
        find_ble_identity_mapping_for_handle(connection_handle);
    if (mapping != nullptr) {
        memcpy(output, mapping->connection_address, sizeof(bd_addr_t));
        return;
    }
    memcpy(output, fallback, sizeof(bd_addr_t));
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
bool valid_confirmation_policy(
    ControllerProfileConfirmationPolicy policy) {
    return static_cast<uint8_t>(policy) <=
           static_cast<uint8_t>(
               ControllerProfileConfirmationPolicy::kRumbleAndLed);
}

void apply_profile_lighting(
    uint8_t active_profile_number, uni_hid_device_t* device) {
    if (device == nullptr || active_profile_number == 0 ||
        active_profile_number > CONTROLLER_PROFILE_COUNT) {
        return;
    }
    if (device->report_parser.set_lightbar_color != nullptr) {
        const SwitchRgbColor color =
            kProfileLightbarPalette[active_profile_number - 1u];
        device->report_parser.set_lightbar_color(
            device, color.red, color.green, color.blue);
    } else if (device->report_parser.set_player_leds != nullptr) {
        device->report_parser.set_player_leds(
            device, static_cast<uint8_t>(
                        (1u << active_profile_number) - 1u));
    }
}

bool lighting_target_is_current(
    uint8_t slot_index, uint32_t connection_generation,
    const uni_hid_device_t* device) {
    critical_section_enter_blocking(&g_state_lock);
    const bool current =
        slot_index < kSlotCount && g_slots[slot_index].active &&
        g_slots[slot_index].device == device &&
        g_slots[slot_index].connection_generation ==
            connection_generation;
    critical_section_exit(&g_state_lock);
    return current;
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
                          uint16_t pre_hotkey_button_mask,
                          const ControllerState& state) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& target = g_slots[slot];
    if (target.active && target.device == device) {
        target.state = state;
        target.pre_hotkey_button_mask = pre_hotkey_button_mask;
        ++target.state_generation;
    }
    critical_section_exit(&g_state_lock);
}

void publish_all_neutral() {
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        slot.state = make_neutral_state();
        slot.pre_hotkey_button_mask = 0;
        slot.identity = controller_identity_global();
        slot.device = nullptr;
        slot.active = false;
        slot.rumble_pending = false;
        slot.retained_host_rumble_valid = false;
        slot.retained_host_rumble = {};
        slot.feedback_pending = false;
        slot.feedback_until_ms = 0;
        slot.pending_profile_feedback_count = 0;
        for (ProfileFeedbackEnvelope& feedback :
             slot.pending_profile_feedback) {
            feedback = {};
        }
        slot.profile_feedback = {};
        ++slot.state_generation;
        ++slot.connection_generation;
    }
    critical_section_exit(&g_state_lock);
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        mapping = {};
    }
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

constexpr int16_t scale_axis(int32_t value) {
    value = clamp_axis(value);
    if (value <= 0) {
        return static_cast<int16_t>(
            (static_cast<int64_t>(value) * -INT16_MIN) /
            -kAxisMinimum);
    }
    return static_cast<int16_t>(
        (static_cast<int64_t>(value) * INT16_MAX) /
        kAxisMaximum);
}

constexpr uint16_t scale_trigger(int32_t value) {
    if (value <= 0) {
        return 0;
    }
    if (value >= kTriggerFullScaleMinimum) {
        return UINT16_MAX;
    }
    return static_cast<uint16_t>(
        (static_cast<int64_t>(value) * UINT16_MAX) /
        kTriggerMaximum);
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

static_assert(scale_axis(-512) == INT16_MIN);
static_assert(scale_axis(0) == 0);
static_assert(scale_axis(511) == INT16_MAX);
static_assert(scale_trigger(0) == 0);
static_assert(scale_trigger(1016) == 65086);
static_assert(scale_trigger(1020) == UINT16_MAX);
static_assert(scale_trigger(1023) == UINT16_MAX);
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
constexpr uint16_t logical_button_bit(
    ControllerProfileLogicalButton button) {
    return static_cast<uint16_t>(
        1u << static_cast<uint8_t>(button));
}

constexpr uint16_t logical_button_mask(
    uint32_t dpad, uint32_t buttons, uint32_t misc_buttons) {
    return static_cast<uint16_t>(
        ((buttons & BUTTON_A) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kSouth)
             : 0u) |
        ((buttons & BUTTON_B) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kEast)
             : 0u) |
        ((buttons & BUTTON_X) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kWest)
             : 0u) |
        ((buttons & BUTTON_Y) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kNorth)
             : 0u) |
        ((buttons & BUTTON_SHOULDER_L) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kLeftShoulder)
             : 0u) |
        ((buttons & BUTTON_SHOULDER_R) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kRightShoulder)
             : 0u) |
        ((misc_buttons & MISC_BUTTON_SELECT) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kSelect)
             : 0u) |
        ((misc_buttons & MISC_BUTTON_START) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kStart)
             : 0u) |
        ((misc_buttons & MISC_BUTTON_SYSTEM) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kSystem)
             : 0u) |
        ((misc_buttons & MISC_BUTTON_CAPTURE) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kCapture)
             : 0u) |
        ((buttons & BUTTON_THUMB_L) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kLeftStick)
             : 0u) |
        ((buttons & BUTTON_THUMB_R) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kRightStick)
             : 0u) |
        ((dpad & DPAD_UP) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kDpadUp)
             : 0u) |
        ((dpad & DPAD_DOWN) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kDpadDown)
             : 0u) |
        ((dpad & DPAD_LEFT) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kDpadLeft)
             : 0u) |
        ((dpad & DPAD_RIGHT) != 0
             ? logical_button_bit(
                   ControllerProfileLogicalButton::kDpadRight)
             : 0u));
}

constexpr uint16_t kMotionHotkeyLogicalButtonMask =
    logical_button_mask(
        kMotionHotkeyDpadMask, kMotionHotkeyButtonMask,
        kMotionHotkeyMiscMask);

uint16_t logical_button_mask(const uni_gamepad_t& gamepad) {
    return logical_button_mask(
        gamepad.dpad, gamepad.buttons, gamepad.misc_buttons);
}

ControllerState map_gamepad(const uni_gamepad_t& gamepad,
                            bool motion_enabled,
                            uint16_t button_mask) {
    ControllerState state = make_neutral_state();

    state.dpad_up =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kDpadUp)) != 0;
    state.dpad_down =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kDpadDown)) != 0;
    state.dpad_left =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kDpadLeft)) != 0;
    state.dpad_right =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kDpadRight)) != 0;

    // Bluepad32's A/B/X/Y are positional: south/east/west/north. Persistent
    // profile mappings are the only button remapping layer.
    state.button_south =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kSouth)) != 0;
    state.button_east =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kEast)) != 0;
    state.button_west =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kWest)) != 0;
    state.button_north =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kNorth)) != 0;
    state.button_left_shoulder =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kLeftShoulder)) != 0;
    state.button_right_shoulder =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kRightShoulder)) != 0;
    state.left_trigger = scale_trigger(gamepad.brake);
    if (gamepad.brake == 0 &&
        (gamepad.buttons & BUTTON_TRIGGER_L) != 0) {
        state.left_trigger = UINT16_MAX;
    }
    state.right_trigger = scale_trigger(gamepad.throttle);
    if (gamepad.throttle == 0 &&
        (gamepad.buttons & BUTTON_TRIGGER_R) != 0) {
        state.right_trigger = UINT16_MAX;
    }
    state.button_left_stick =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kLeftStick)) != 0;
    state.button_right_stick =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kRightStick)) != 0;

    state.button_select =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kSelect)) != 0;
    state.button_start =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kStart)) != 0;
    state.button_system =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kSystem)) != 0;
    state.button_capture =
        (button_mask & logical_button_bit(
                           ControllerProfileLogicalButton::kCapture)) != 0;

    state.left_stick_x = scale_axis(gamepad.axis_x);
    state.left_stick_y = scale_axis(gamepad.axis_y);
    state.right_stick_x = scale_axis(gamepad.axis_rx);
    state.right_stick_y = scale_axis(gamepad.axis_ry);

    if (motion_enabled && has_motion(gamepad)) {
        // Dependency patches normalize both arrays to SDL3 PlayStation axes.
        ControllerMotionSample sample{};
        sample.accel_x = convert_accel(-static_cast<int64_t>(gamepad.accel[2]));
        sample.accel_y = convert_accel(-static_cast<int64_t>(gamepad.accel[0]));
        sample.accel_z = convert_accel(gamepad.accel[1]);
        sample.gyro_x = convert_gyro(-static_cast<int64_t>(gamepad.gyro[2]));
        sample.gyro_y = convert_gyro(-static_cast<int64_t>(gamepad.gyro[0]));
        sample.gyro_z = convert_gyro(gamepad.gyro[1]);
        state.motion_sample_count = 3;
        for (ControllerMotionSample& destination : state.motion_samples) {
            destination = sample;
        }
    }

    return state;
}
struct HotkeyDecision {
    bool motion_enabled;
    uint32_t suppress_dpad;
    uint32_t suppress_buttons;
    uint32_t suppress_misc_buttons;
    uint16_t suppress_logical_buttons;
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
    slot.motion_enabled = kDefaultMotionEnabled;
    slot.motion_hotkey_latched = false;
    slot.pre_hotkey_button_mask = 0;
    slot.feedback_pending = false;
    slot.feedback_until_ms = 0;
    slot.pending_feedback = {};
    slot.pending_profile_feedback_count = 0;
    for (ProfileFeedbackEnvelope& feedback :
         slot.pending_profile_feedback) {
        feedback = {};
    }
    slot.profile_feedback = {};
    slot.retained_host_rumble_valid = false;
    slot.retained_host_rumble = {};
}


HotkeyDecision update_controller_hotkeys(
    uint8_t slot_index, uni_hid_device_t* device,
    const uni_gamepad_t& gamepad) {
    const bool motion_pressed =
        (gamepad.dpad & kMotionHotkeyDpadMask) ==
            kMotionHotkeyDpadMask &&
        (gamepad.buttons & kMotionHotkeyButtonMask) ==
            kMotionHotkeyButtonMask &&
        (gamepad.misc_buttons & kMotionHotkeyMiscMask) ==
            kMotionHotkeyMiscMask;
    HotkeyDecision decision{
        kDefaultMotionEnabled, 0, 0, 0, 0};

    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device == device) {
        if (motion_pressed && !slot.motion_hotkey_latched) {
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
        slot.motion_hotkey_latched = motion_pressed;
        decision.motion_enabled = slot.motion_enabled;
        if (motion_pressed) {
            decision.suppress_dpad |= kMotionHotkeyDpadMask;
            decision.suppress_buttons |= kMotionHotkeyButtonMask;
            decision.suppress_misc_buttons |= kMotionHotkeyMiscMask;
            decision.suppress_logical_buttons |=
                kMotionHotkeyLogicalButtonMask;
        }
    }
    critical_section_exit(&g_state_lock);
    return decision;
}


bool pairing_window_active_at(uint32_t now_ms) {
    return g_pairing_window_open &&
           static_cast<int32_t>(now_ms - g_pairing_window_deadline_ms) < 0;
}

void handle_btstack_event(uint8_t packet_type, uint16_t channel,
                          uint8_t* packet, uint16_t size) {
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET || packet == nullptr || size < 2) {
        return;
    }

    bd_addr_t address{};
    bd_addr_t identity_address{};
    bd_addr_t connection_address{};
    hci_con_handle_t connection_handle = 0;
    const bool pairing_open =
        pairing_window_active_at(btstack_run_loop_get_time_ms());
    switch (hci_event_packet_get_type(packet)) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            if (size >= 11) {
                clear_ble_identity_for_handle(
                    sm_event_identity_resolving_started_get_handle(packet));
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            if (size >= 11) {
                clear_ble_identity_for_handle(
                    sm_event_identity_resolving_failed_get_handle(packet));
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            if (size >= 20) {
                connection_handle =
                    sm_event_identity_resolving_succeeded_get_handle(packet);
                sm_event_identity_resolving_succeeded_get_address(
                    packet, connection_address);
                sm_event_identity_resolving_succeeded_get_identity_address(
                    packet, identity_address);
                record_ble_identity(
                    connection_handle, connection_address,
                    sm_event_identity_resolving_succeeded_get_identity_addr_type(
                        packet),
                    identity_address);
            }
            break;
        case SM_EVENT_IDENTITY_CREATED:
            if (size >= 20) {
                connection_handle =
                    sm_event_identity_created_get_handle(packet);
                sm_event_identity_created_get_address(packet, address);
                sm_event_identity_created_get_identity_address(
                    packet, identity_address);
                connection_address_for_handle(
                    connection_handle, address, connection_address);
                record_ble_identity(
                    connection_handle, connection_address,
                    sm_event_identity_created_get_identity_addr_type(packet),
                    identity_address);
            }
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            if (size >= 11) {
                connection_handle =
                    sm_event_reencryption_started_get_handle(packet);
                sm_event_reencryption_started_get_address(
                    packet, identity_address);
                connection_address_for_handle(
                    connection_handle, identity_address,
                    connection_address);
                record_ble_identity(
                    connection_handle, connection_address,
                    sm_event_reencryption_started_get_addr_type(packet),
                    identity_address);
            }
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            if (size >= 12) {
                connection_handle =
                    sm_event_reencryption_complete_get_handle(packet);
                if (sm_event_reencryption_complete_get_status(packet) ==
                    ERROR_CODE_SUCCESS) {
                    sm_event_reencryption_complete_get_address(
                        packet, identity_address);
                    connection_address_for_handle(
                        connection_handle, identity_address,
                        connection_address);
                    record_ble_identity(
                        connection_handle, connection_address,
                        sm_event_reencryption_complete_get_addr_type(packet),
                        identity_address);
                } else {
                    clear_ble_identity_for_handle(connection_handle);
                }
            }
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            if (size < 8) {
                break;
            }
            hci_event_user_confirmation_request_get_bd_addr(packet, address);
            if (pairing_open) {
                gap_ssp_confirmation_response(address);
            } else {
                gap_ssp_confirmation_negative(address);
            }
            break;
        case HCI_EVENT_USER_PASSKEY_REQUEST:
            if (size < 8) {
                break;
            }
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
        ConfigurationServiceSnapshot configuration{};
        configuration_service_snapshot(&configuration);
        g_pairing_window_duration_ms =
            static_cast<uint32_t>(
                configuration.configuration.pairing_window_seconds) *
            1000u;
        g_pairing_window_open = true;
        g_pairing_window_deadline_ms =
            now_ms + g_pairing_window_duration_ms;
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
void append_pairing_record(
    Bluepad32PairingSnapshot& snapshot,
    Bluepad32PairingTransport transport, uint8_t address_type,
    const bd_addr_t address) {
    if (snapshot.record_count >= BLUEPAD32_PAIRING_RECORD_CAPACITY) {
        snapshot.overflow = true;
        return;
    }
    Bluepad32PairingRecord& record =
        snapshot.records[snapshot.record_count++];
    record.transport = transport;
    record.address_type = address_type;
    memcpy(record.address, address, sizeof(record.address));
}

void refresh_pairing_snapshot() {
    Bluepad32PairingSnapshot snapshot{};
    snapshot.status = Bluepad32PairingSnapshotStatus::kReady;

    btstack_link_key_iterator_t iterator{};
    if (gap_link_key_iterator_init(&iterator)) {
        bd_addr_t address{};
        link_key_t link_key{};
        link_key_type_t link_key_type{};
        while (gap_link_key_iterator_get_next(
            &iterator, address, link_key, &link_key_type)) {
            append_pairing_record(
                snapshot, Bluepad32PairingTransport::kClassic,
                BD_ADDR_TYPE_UNKNOWN, address);
        }
        gap_link_key_iterator_done(&iterator);
    }

    for (int index = 0; index < le_device_db_max_count(); ++index) {
        int address_type = BD_ADDR_TYPE_UNKNOWN;
        bd_addr_t address{};
        le_device_db_info(index, &address_type, address, nullptr);
        if (address_type == BD_ADDR_TYPE_UNKNOWN) {
            continue;
        }
        append_pairing_record(
            snapshot, Bluepad32PairingTransport::kBle,
            static_cast<uint8_t>(address_type), address);
    }

    critical_section_enter_blocking(&g_state_lock);
    snapshot.generation = g_pairing_snapshot.generation + 1;
    snapshot.completed_clear_pairings_token =
        g_pairing_snapshot.completed_clear_pairings_token;
    g_pairing_snapshot = snapshot;
    g_pairing_snapshot_requested = false;
    critical_section_exit(&g_state_lock);
}

void process_pairing_snapshot_request() {
    critical_section_enter_blocking(&g_state_lock);
    const bool requested = g_pairing_snapshot_requested;
    critical_section_exit(&g_state_lock);
    if (requested) {
        refresh_pairing_snapshot();
    }
}

void apply_connection_policy();

void process_clear_pairings(uint32_t now_ms) {
    uni_hid_device_t* devices[kSlotCount]{};
    critical_section_enter_blocking(&g_state_lock);
    const uint32_t request_token =
        g_clear_pairings_requested_token;
    if (request_token != 0) {
        g_clear_pairings_requested_token = 0;
        g_clear_pairings_in_progress_token = request_token;
    }
    if (request_token != 0) {
        g_pairing_window_requested = false;
        for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
            BackendSlot& slot = g_slots[slot_index];
            devices[slot_index] = slot.device;
            slot.state = make_neutral_state();
            slot.identity = controller_identity_global();
            slot.device = nullptr;
            slot.active = false;
            slot.rumble_pending = false;
            slot.feedback_pending = false;
            slot.feedback_until_ms = 0;
            reset_slot_hotkeys(slot);
            ++slot.state_generation;
            ++slot.connection_generation;
        }
    }
    critical_section_exit(&g_state_lock);
    if (request_token == 0) {
        return;
    }
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        mapping = {};
    }

    g_pairing_window_open = false;
    gap_set_bondable_mode(false);
    sm_set_accepted_stk_generation_methods(0);
    uni_bt_del_keys_unsafe();
    for (uni_hid_device_t* device : devices) {
        if (device != nullptr) {
            uni_hid_device_disconnect(device);
        }
    }
    refresh_pairing_snapshot();

    g_connection_status = ConnectionStatus::Scanning;
    g_status_led_tick = 0;
    g_pairing_reset_feedback_deadline_ms =
        now_ms + kPairingResetFeedbackDurationMs;
    apply_connection_policy();
    critical_section_enter_blocking(&g_state_lock);
    g_pairing_snapshot.completed_clear_pairings_token =
        request_token;
    g_clear_pairings_in_progress_token = 0;
    critical_section_exit(&g_state_lock);
}


void apply_connection_policy() {
    const bool free_slot = has_free_slot();
    const bool active_controller = has_active_controller();
    const bool pairing_open =
        pairing_window_active_at(btstack_run_loop_get_time_ms());
    const bool active_scan =
        free_slot && (!active_controller || pairing_open);
    const ConnectionPolicyState desired_state =
        !free_slot
            ? ConnectionPolicyState::Paused
            : (active_scan ? ConnectionPolicyState::Open
                           : ConnectionPolicyState::Passive);
    if (g_connection_policy_state == desired_state) {
        return;
    }

    // Classic inquiry and BLE scanning consume radio time and measurably delay
    // active controller HID traffic. Stop them before every policy transition.
    uni_bt_stop_scanning_unsafe();

    if (!free_slot) {
        uni_bt_allow_incoming_connections(false);
        g_connection_policy_state = ConnectionPolicyState::Paused;
        return;
    }

    // Passive mode still accepts controller-initiated reconnects without
    // running inquiry. Active discovery is reserved for zero-controller idle
    // state and the explicit BOOTSEL pairing window.
    uni_bt_allow_incoming_connections(true);
    if (active_scan) {
        uni_bt_start_scanning_and_autoconnect_unsafe();
        g_connection_policy_state = ConnectionPolicyState::Open;
    } else {
        g_connection_policy_state = ConnectionPolicyState::Passive;
    }
}

bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

bool advance_profile_feedback(ProfileFeedbackSequence* sequence,
                              uint32_t now_ms) {
    bool rumble_dispatch = false;
    for (uint8_t transition = 0;
         transition < CONTROLLER_PROFILE_COUNT * 2u &&
         sequence->active &&
         deadline_reached(now_ms, sequence->phase_deadline_ms);
         ++transition) {
        sequence->phase_deadline_ms +=
            kProfileFeedbackPhaseDurationMs;
        if (sequence->on) {
            sequence->on = false;
            rumble_dispatch = false;
        } else if (sequence->pulses_started >=
                   sequence->pulse_count) {
            sequence->active = false;
        } else {
            sequence->on = true;
            ++sequence->pulses_started;
            rumble_dispatch = sequence->rumble_enabled;
        }
    }
    return rumble_dispatch;
}

void update_status_led() {
    ++g_status_led_tick;
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    bool profile_led_override = false;
    bool profile_led_on = false;
    critical_section_enter_blocking(&g_state_lock);
    for (const BackendSlot& slot : g_slots) {
        if (slot.profile_feedback.active &&
            slot.profile_feedback.led_enabled) {
            profile_led_override = true;
            profile_led_on =
                profile_led_on || slot.profile_feedback.on;
        }
    }
    critical_section_exit(&g_state_lock);

    bool led_on = false;
    if (profile_led_override) {
        led_on = profile_led_on;
    } else if (static_cast<int32_t>(
                   now_ms - g_pairing_reset_feedback_deadline_ms) < 0) {
        led_on = (g_status_led_tick % 20) < 10;
    } else if (pairing_window_active_at(now_ms)) {
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

void process_configuration_timer(btstack_timer_source_t* timer) {
    btstack_run_loop_set_timer(timer, kConfigurationPollIntervalMs);
    btstack_run_loop_add_timer(timer);
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    configuration_service_task_on_storage_core(now_ms);
    profile_service_task_on_storage_core(now_ms);
}

void process_rumble_timer(btstack_timer_source_t* timer) {
    const uint32_t now_ms = btstack_run_loop_get_time_ms();

    process_clear_pairings(now_ms);
    process_pairing_snapshot_request();
    if (update_pairing_window(now_ms)) {
        apply_connection_policy();
    }
    const bool xinput_host_mode =
        host_rumble_duration_ms() == kXInputHostRumbleDurationMs;

    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        RumbleEnvelope envelope{};
        FeedbackEnvelope feedback{};
        ProfileFeedbackEnvelope profile_feedback{};
        uni_hid_device_t* device = nullptr;
        uni_hid_device_t* profile_lighting_device = nullptr;
        uint32_t profile_lighting_generation = 0;
        bool profile_lighting_dispatch = false;
        bool profile_lighting_restore = false;
        bool profile_rumble_dispatch = false;
        bool feedback_dispatch = false;
        bool host_dispatch = false;

        critical_section_enter_blocking(&g_state_lock);
        BackendSlot& slot = g_slots[slot_index];
        if (slot.retained_host_rumble_valid &&
            (!xinput_host_mode ||
             slot.retained_host_rumble.duration_ms !=
                 kXInputHostRumbleDurationMs ||
             slot.retained_host_rumble.slot != slot_index ||
             slot.retained_host_rumble.connection_generation !=
                 slot.connection_generation ||
             !slot.active || slot.device == nullptr)) {
            slot.retained_host_rumble_valid = false;
            slot.retained_host_rumble = {};
        }
        if (slot.profile_feedback.active &&
            slot.profile_feedback.connection_generation !=
                slot.connection_generation) {
            slot.profile_feedback = {};
        }
        const bool profile_feedback_was_active =
            slot.profile_feedback.active;
        const bool completed_feedback_had_rumble =
            slot.profile_feedback.rumble_enabled;
        const bool completed_feedback_had_led =
            slot.profile_feedback.led_enabled;
        const uint32_t completed_feedback_generation =
            slot.profile_feedback.connection_generation;
        profile_rumble_dispatch =
            advance_profile_feedback(&slot.profile_feedback, now_ms);
        if (profile_feedback_was_active &&
            !slot.profile_feedback.active &&
            completed_feedback_had_rumble &&
            slot.retained_host_rumble_valid) {
            slot.pending_rumble = slot.retained_host_rumble;
            slot.rumble_pending = true;
        }
        if (profile_feedback_was_active &&
            !slot.profile_feedback.active &&
            completed_feedback_had_led && slot.active &&
            slot.device != nullptr &&
            completed_feedback_generation ==
                slot.connection_generation) {
            profile_lighting_device = slot.device;
            profile_lighting_generation =
                completed_feedback_generation;
            profile_lighting_restore = true;
        }
        if (profile_rumble_dispatch) {
            device = slot.device;
        }

        const bool feedback_active =
            static_cast<int32_t>(now_ms - slot.feedback_until_ms) < 0;
        if (!slot.profile_feedback.active && !feedback_active &&
            slot.pending_profile_feedback_count != 0) {
            profile_feedback = slot.pending_profile_feedback[0];
            if (slot.pending_profile_feedback_count == 2) {
                slot.pending_profile_feedback[0] =
                    slot.pending_profile_feedback[1];
            }
            --slot.pending_profile_feedback_count;
            slot.pending_profile_feedback[
                slot.pending_profile_feedback_count] = {};
            const uint8_t policy =
                static_cast<uint8_t>(profile_feedback.policy);
            if (slot.active && slot.device != nullptr &&
                profile_feedback.connection_generation ==
                    slot.connection_generation &&
                profile_feedback.active_profile_number != 0 &&
                profile_feedback.active_profile_number <=
                    CONTROLLER_PROFILE_COUNT &&
                valid_confirmation_policy(profile_feedback.policy) &&
                profile_feedback.policy !=
                    ControllerProfileConfirmationPolicy::kNone) {
                slot.profile_feedback = {
                    slot.connection_generation,
                    now_ms + kProfileFeedbackPhaseDurationMs,
                    profile_feedback.active_profile_number,
                    1,
                    true,
                    true,
                    (policy & static_cast<uint8_t>(
                                  ControllerProfileConfirmationPolicy::
                                      kRumble)) != 0,
                    (policy & static_cast<uint8_t>(
                                  ControllerProfileConfirmationPolicy::
                                      kLed)) != 0,
                };
                device = slot.device;
                profile_lighting_device = slot.device;
                profile_lighting_generation =
                    slot.profile_feedback.connection_generation;
                profile_lighting_dispatch =
                    slot.profile_feedback.led_enabled;
                profile_rumble_dispatch =
                    slot.profile_feedback.rumble_enabled;
            }
        }

        if (!slot.profile_feedback.active &&
            slot.feedback_pending) {
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

        const bool local_feedback_active =
            slot.profile_feedback.active ||
            static_cast<int32_t>(
                now_ms - slot.feedback_until_ms) < 0;
        if (!profile_rumble_dispatch && !feedback_dispatch &&
            !local_feedback_active && slot.rumble_pending) {
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

        if (profile_lighting_restore &&
            lighting_target_is_current(
                slot_index, profile_lighting_generation,
                profile_lighting_device)) {
            apply_slot_lighting(slot_index,
                                profile_lighting_device);
        }
        if (profile_lighting_dispatch &&
            lighting_target_is_current(
                slot_index, profile_lighting_generation,
                profile_lighting_device)) {
            apply_profile_lighting(
                profile_feedback.active_profile_number,
                profile_lighting_device);
        }
        if (profile_rumble_dispatch && device != nullptr &&
            device->report_parser.play_dual_rumble != nullptr) {
            device->report_parser.play_dual_rumble(
                device, 0, kProfileFeedbackPhaseDurationMs,
                kProfileFeedbackWeakMagnitude,
                kProfileFeedbackStrongMagnitude);
        } else if (feedback_dispatch) {
            device->report_parser.play_dual_rumble(
                device, 0, feedback.duration_ms,
                feedback.weak_magnitude, feedback.strong_magnitude);
        } else if (host_dispatch &&
                   device->report_parser.play_dual_rumble != nullptr) {
            const bool stop =
                envelope.rumble.low_frequency_magnitude == 0 &&
                envelope.rumble.high_frequency_magnitude == 0;
            device->report_parser.play_dual_rumble(
                device, 0, stop ? 0 : envelope.duration_ms,
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
    gap_set_link_supervision_timeout(kClassicLinkSupervisionTimeout);
    gap_set_bondable_mode(false);
    sm_set_accepted_stk_generation_methods(0);
    gap_ssp_set_auto_accept(false);
    g_pairing_event_callback.callback = handle_btstack_event;
    g_identity_event_callback.callback = handle_btstack_event;
    sm_add_event_handler(&g_identity_event_callback);
    hci_add_event_handler(&g_pairing_event_callback);
    refresh_pairing_snapshot();
    // Keep Bluepad32 autoconnect active whenever at least one slot is free.
    btstack_run_loop_set_timer_handler(&g_rumble_timer, process_rumble_timer);
    btstack_run_loop_set_timer(&g_rumble_timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(&g_rumble_timer);
    btstack_run_loop_set_timer_handler(
        &g_configuration_timer, process_configuration_timer);
    btstack_run_loop_set_timer(
        &g_configuration_timer, kConfigurationPollIntervalMs);
    btstack_run_loop_add_timer(&g_configuration_timer);
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
    if (g_connection_policy_state != ConnectionPolicyState::Open &&
        g_connection_policy_state != ConnectionPolicyState::Passive) {
        uni_hid_device_disconnect(device);
        return;
    }

    const int slot_index = slot_for_device(device);
    if (slot_index < 0) {
        return;
    }
    const ControllerIdentity connection_identity =
        identity_for_device(device);

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
    if (tracked_connection) {
        slot.identity = connection_identity;
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
        slot.state = make_neutral_state();
        slot.identity = controller_identity_global();
        slot.device = nullptr;
        slot.active = false;
        slot.rumble_pending = false;
        reset_slot_hotkeys(slot);
        ++slot.state_generation;
        ++slot.connection_generation;
        disconnected_tracked_device = true;
    }
    critical_section_exit(&g_state_lock);
    clear_ble_identity_for_device(device);

    if (disconnected_tracked_device) {
        // Re-evaluate from scratch: resume discovery only after the final
        // active controller disconnects; otherwise keep passive incoming
        // reconnect support without inquiry-induced latency.
        g_connection_policy_state = ConnectionPolicyState::Uninitialized;
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
    const ControllerIdentity connection_identity =
        identity_for_device(device);

    bool occupied_mismatch = false;
    bool became_active = false;
    uint32_t lighting_generation = 0;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    occupied_mismatch = slot.device != nullptr && slot.device != device;
    if (!occupied_mismatch) {
        slot.identity = connection_identity;
        slot.device = device;
        if (!slot.active) {
            slot.state = make_neutral_state();
            slot.active = true;
            slot.rumble_pending = false;
            reset_slot_hotkeys(slot);
            ++slot.state_generation;
            became_active = true;
            lighting_generation = slot.connection_generation;
        }
    }
    critical_section_exit(&g_state_lock);

    if (occupied_mismatch) {
        return UNI_ERROR_NO_SLOTS;
    }
    if (became_active) {
        if (lighting_target_is_current(
                static_cast<uint8_t>(slot_index),
                lighting_generation, device)) {
            apply_slot_lighting(
                static_cast<uint8_t>(slot_index), device);
        }
        if (connection_identity.stable) {
            profile_service_observe_identity_on_storage_core(
                connection_identity);
        }
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
    const uint16_t pre_hotkey_button_mask =
        logical_button_mask(gamepad);
    const HotkeyDecision hotkeys = update_controller_hotkeys(
        static_cast<uint8_t>(slot_index), device, gamepad);
    gamepad.dpad &= ~hotkeys.suppress_dpad;
    gamepad.buttons &= ~hotkeys.suppress_buttons;
    gamepad.misc_buttons &= ~hotkeys.suppress_misc_buttons;
    const uint16_t output_button_mask = static_cast<uint16_t>(
        pre_hotkey_button_mask & ~hotkeys.suppress_logical_buttons);
    publish_device_state(
        static_cast<uint8_t>(slot_index), device,
        pre_hotkey_button_mask,
        map_gamepad(
            gamepad, hotkeys.motion_enabled, output_button_mask));
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
    configuration_service_initialize_on_storage_core();
    profile_service_initialize_on_storage_core();
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
    configuration_service_prepare();
    profile_service_prepare();
    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        BackendSlot& slot = g_slots[slot_index];
        slot = {};
        slot.state = make_neutral_state();
        slot.identity = controller_identity_global();
        slot.pending_rumble.slot = slot_index;
        reset_slot_hotkeys(slot);
        g_consumed_generation[slot_index] = 0;
        g_last_snapshot_generation[slot_index] = 0;
        g_ble_identity_mappings[slot_index] = {};
    }
    g_pairing_window_requested = false;
    g_pairing_snapshot_requested = false;
    g_pairing_snapshot = {};
    g_pairing_snapshot.status =
        Bluepad32PairingSnapshotStatus::kPending;
    g_clear_pairings_requested_token = 0;
    g_clear_pairings_in_progress_token = 0;
    g_next_clear_pairings_request_token = 1;
    g_connection_status = ConnectionStatus::Initializing;
    g_connection_policy_state = ConnectionPolicyState::Uninitialized;
    g_pairing_window_deadline_ms = 0;
    g_pairing_window_duration_ms =
        kDefaultPairingWindowDurationMs;
    g_pairing_reset_feedback_deadline_ms = 0;
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
uint32_t bluepad32_input_backend_clear_pairings() {
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }

    critical_section_enter_blocking(&g_state_lock);
    uint32_t request_token = g_clear_pairings_requested_token;
    if (request_token == 0) {
        request_token = g_clear_pairings_in_progress_token;
    }
    if (request_token == 0) {
        request_token = g_next_clear_pairings_request_token;
        g_next_clear_pairings_request_token =
            request_token == UINT32_MAX ? 1 : request_token + 1;
        g_clear_pairings_requested_token = request_token;
        g_pairing_snapshot.status =
            Bluepad32PairingSnapshotStatus::kPending;
    }
    critical_section_exit(&g_state_lock);
    return request_token;
}

void bluepad32_input_backend_request_pairing_snapshot() {
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }

    critical_section_enter_blocking(&g_state_lock);
    g_pairing_snapshot_requested = true;
    g_pairing_snapshot.status =
        Bluepad32PairingSnapshotStatus::kPending;
    critical_section_exit(&g_state_lock);
}

void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out) {
    if (out == nullptr) {
        return;
    }
    if (!g_initialized) {
        bluepad32_input_backend_init();
    }

    critical_section_enter_blocking(&g_state_lock);
    *out = g_pairing_snapshot;
    critical_section_exit(&g_state_lock);
}


void bluepad32_input_backend_snapshot(uint8_t slot_index,
                                      Bluepad32SlotSnapshot* out) {
    if (out == nullptr) {
        return;
    }
    *out = {};
    if (!valid_slot(slot_index) || !g_initialized) {
        return;
    }

    critical_section_enter_blocking(&g_state_lock);
    const BackendSlot& slot = g_slots[slot_index];
    out->active = slot.active;
    out->connection_generation = slot.connection_generation;
    out->identity = slot.identity;
    out->pre_hotkey_button_mask =
        slot.pre_hotkey_button_mask;
    out->state = slot.state;
    const uint32_t state_generation = slot.state_generation;
    critical_section_exit(&g_state_lock);

    if (state_generation == g_consumed_generation[slot_index]) {
        out->state.motion_sample_count = 0;
    }
    g_last_snapshot_generation[slot_index] = state_generation;
}

void bluepad32_input_backend_report_sent(uint8_t slot_index) {
    if (!g_initialized || !valid_slot(slot_index)) {
        return;
    }
    g_consumed_generation[slot_index] = g_last_snapshot_generation[slot_index];
}

void bluepad32_input_backend_queue_rumble(
    uint8_t slot_index, const ControllerRumbleOutput& rumble) {
    if (!g_initialized || !valid_slot(slot_index)) {
        return;
    }

    const uint16_t duration_ms = host_rumble_duration_ms();
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device != nullptr) {
        const RumbleEnvelope envelope{
            slot_index, slot.connection_generation, rumble,
            duration_ms};
        slot.pending_rumble = envelope;
        slot.rumble_pending = true;
        if (duration_ms == kXInputHostRumbleDurationMs) {
            slot.retained_host_rumble = envelope;
            slot.retained_host_rumble_valid = true;
        } else {
            slot.retained_host_rumble = {};
            slot.retained_host_rumble_valid = false;
        }
    }
    critical_section_exit(&g_state_lock);
}

void bluepad32_input_backend_queue_profile_feedback(
    uint8_t slot_index, uint32_t connection_generation,
    uint8_t active_profile_number,
    ControllerProfileConfirmationPolicy policy) {
    if (!g_initialized || !valid_slot(slot_index) ||
        active_profile_number == 0 ||
        active_profile_number > CONTROLLER_PROFILE_COUNT ||
        !valid_confirmation_policy(policy) ||
        policy == ControllerProfileConfirmationPolicy::kNone) {
        return;
    }

    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device != nullptr &&
        slot.connection_generation == connection_generation) {
        const ProfileFeedbackEnvelope feedback{
            connection_generation, active_profile_number, policy};
        if (slot.pending_profile_feedback_count <
            kProfileFeedbackQueueCapacity) {
            slot.pending_profile_feedback[
                slot.pending_profile_feedback_count++] = feedback;
        } else {
            slot.pending_profile_feedback[
                kProfileFeedbackQueueCapacity - 1u] = feedback;
        }
    }
    critical_section_exit(&g_state_lock);
}
