#include "input/bluepad32_input_backend.h"
#include "bluetooth_transport_config.h"
#include "input/controller_hotkey_config.h"
#include "input/switch2_wake.h"
#ifdef SWITCH_PICO_WII_IR
#include "input/wii_ir_pointer.h"
#include "parser/uni_hid_parser_wii_ir.h"
#endif
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
#include "input/switch_native_output.h"
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
#include "input/haptics_experiment.h"
#endif
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
#include "input/native_output_scheduler.h"
#endif
#include "configuration/configuration_service.h"
#include "profile/profile_service.h"
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
extern "C" {
#include "parser/uni_hid_parser_wii.h"
}
#include "parser/uni_hid_parser_switch2.h"
#include "parser/uni_switch2_haptics.h"
#include "parser/uni_switch2_pairing.h"
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
#include "adapter/adapter_usb_mode.h"
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
// LE units are 1.25 ms. All Switch 2 links request the 7.5 ms minimum.
constexpr uint16_t kSwitch2FastInterval = 6;
constexpr uint32_t kSwitch2IntervalSettleMs = 1000;
constexpr uint8_t kAllBlePairingMethods =
    SM_STK_GENERATION_METHOD_JUST_WORKS |
    SM_STK_GENERATION_METHOD_OOB |
    SM_STK_GENERATION_METHOD_PASSKEY |
    SM_STK_GENERATION_METHOD_NUMERIC_COMPARISON;
constexpr uint16_t kProfileFeedbackPhaseDurationMs = 75;
constexpr uint8_t kProfileFeedbackWeakMagnitude = UINT8_MAX;
constexpr uint8_t kProfileFeedbackStrongMagnitude = UINT8_MAX;
constexpr uint32_t kJoyConGestureHoldMs = 2000;
constexpr uint32_t kJoyConGestureFreshMs = 250;
constexpr uint16_t kJoyConGestureFeedbackMs = 75;
#ifdef SWITCH_PICO_WII_IR_GYRO
constexpr uint32_t kWiiAimChordHoldUs = 2000000;
constexpr uint32_t kWiiAimChordFreshUs = 150000;
constexpr uint16_t kWiiAimChordButtons = 0x0002 | 0x0001;
#endif
// One initial indication can be followed by one committed switch before the
// Core 1 timer drains the queue. Profile commits are rate-limited well beyond
// the longest feedback sequence.
constexpr uint8_t kProfileFeedbackQueueCapacity = 2;
constexpr SwitchRgbColor kProfileLightbarPalette[CONTROLLER_PROFILE_COUNT] = {
    {0x00, 0x55, 0xff},
    {0x00, 0xcc, 0x66},
    {0xff, 0xaa, 0x00},
    {0xcc, 0x33, 0xff},
    {0xff, 0x44, 0x44},
    {0x00, 0xdd, 0xdd},
    {0xff, 0x66, 0xbb},
    {0xcc, 0xff, 0x33},
};
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
static_assert(CONTROLLER_PROFILE_COUNT == 8);
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
    uint32_t received_ms = 0;
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
    uint64_t received_us = 0;
#endif
};

constexpr uint8_t kSwitch2IngressCapacity = 16;
struct Switch2HostCommand {
    RumbleEnvelope envelope;
    uint32_t generation;
    uint8_t accepted_halves;
};
struct Switch2Ingress {
    Switch2HostCommand commands[kSwitch2IngressCapacity];
    uint32_t generation;
    uint8_t host_mode;
    uint8_t head;
    uint8_t count;
    bool reset_pending;
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
struct WiiOrientationRequest {
    ControllerIdentity identity;
    uint32_t connection_generation;
    bool vertical;
};
#ifdef SWITCH_PICO_WII_IR_GYRO
struct WiiAimSource {
    uint32_t sequence;
    uint32_t last_report_us;
    uint32_t started_us;
    bool have_sequence;
    bool infrared;
    bool holding;
    bool masked;
    bool latched;
    bool reposition_masked;
};
#endif



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
    WiiAccelerometerSample accelerometer{};
    uint16_t pre_hotkey_button_mask;
    ControllerIdentity identity;
    // Non-null with active=false is a connected device still becoming ready.
    uni_hid_device_t* device;
    // A pair occupies one logical profile owner and output, but still consumes
    // two Bluepad32 physical device indices. device is always the left half.
    uni_hid_device_t* companion;
    uni_gamepad_t gamepad;
    uni_gamepad_t companion_gamepad;
    uint8_t extra_buttons;
    uint8_t companion_extra_buttons;
    uint32_t state_generation;
    uint32_t connection_generation;
    bool active;
    bool wii_orientation_pending;
    WiiOrientationRequest pending_wii_orientation;
#ifdef SWITCH_PICO_WII_IR_GYRO
    WiiAimSource wii_aim;
#endif
    bool rumble_pending;
    bool motion_enabled;
    bool feedback_pending;
    uint32_t feedback_until_ms;
    uint8_t pending_profile_feedback_count;
    RumbleEnvelope pending_rumble;
    bool retained_host_rumble_valid;
    RumbleEnvelope retained_host_rumble;
    Switch2Ingress switch2_ingress;
    FeedbackEnvelope pending_feedback;
    ProfileFeedbackEnvelope
        pending_profile_feedback[kProfileFeedbackQueueCapacity];
    ProfileFeedbackSequence profile_feedback;
};

critical_section_t g_state_lock;
uni_hid_device_t* g_retired_devices[kSlotCount]{};
BackendSlot g_slots[kSlotCount];
ControllerMacroCapture g_macro_capture;
// Catalog migration/compaction needs more than the 4 KiB scratch bank.
// Supply a dedicated static stack in main SRAM rather than overflowing it.
alignas(8) uint32_t g_core1_stack[4096];
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
bool g_background_scan_active = false;
struct Switch2IntervalRequest {
    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint16_t interval = 0;
    uint32_t requested_ms = 0;
};
Switch2IntervalRequest g_switch2_interval_requests[kSlotCount]{};
JoyConMode g_joycon_mode = JoyConMode::kPaired;
bool g_joycon_reconcile_requested = false;
// Live-link hints only: splitting two pairs must not exchange their members
// when the next Paired preference is applied.
struct JoyConPairHint {
    uni_hid_device_t* mate = nullptr;
    uint8_t owner_slot = 0;
};
JoyConPairHint g_joycon_pair_hints[kSlotCount]{};

enum class JoyConGroupingOverride : uint8_t {
    Default,
    Individual,
    Paired,
};
struct JoyConConnectionOverride {
    JoyConGroupingOverride mode = JoyConGroupingOverride::Default;
    uni_hid_device_t* mate = nullptr;
};
JoyConConnectionOverride g_joycon_overrides[kSlotCount]{};

// Core 1 physical-link state survives logical slot moves. Raw reports stay in
// BackendSlot; only the derived logical view consumes the reserved buttons.
struct JoyConGesture {
    uni_hid_device_t* device = nullptr;
    uint32_t last_report_ms = 0;
    uint32_t started_ms = 0;
    uint8_t participants = 0;
    bool held = false;
    bool released = true;
    bool masked = false;
    bool blocked = false;
    bool joining = false;
};
JoyConGesture g_joycon_gestures[kSlotCount]{};
uint32_t g_pairing_window_deadline_ms = 0;
uint32_t g_pairing_window_duration_ms =
    kDefaultPairingWindowDurationMs;
uint32_t g_pairing_reset_feedback_deadline_ms = 0;
uint16_t g_status_led_tick = 0;
bool g_pairing_window_open = false;
bool g_status_led_on = false;
Bluepad32PairingSnapshot g_pairing_snapshot{};
uint32_t g_initialization_stage = 0;
uint32_t g_rumble_timer_ticks = 0;
uint32_t g_configuration_timer_ticks = 0;
uint32_t g_controller_reports = 0;
uint32_t g_host_rumble_requests = 0;
uint32_t g_local_feedback_requests = 0;
uint32_t g_rumble_dispatches = 0;
uint32_t g_switch2_ingress_drops = 0;
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
uint32_t g_seeded_native_run_id = 0;
#endif

uint16_t host_rumble_duration_ms() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (adapter_host_probe_mode() == AdapterUsbMode::kXInput) {
        return kXInputHostRumbleDurationMs;
    }
#endif
    return kSwitchHostRumbleDurationMs;
}

uint8_t switch2_host_mode() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    return static_cast<uint8_t>(adapter_host_probe_mode());
#else
    return 0;
#endif
}

ControllerState make_neutral_state() {
    return controller_neutral_state();
}

bool valid_slot(uint8_t slot) {
    return slot < kSlotCount;
}

bool has_free_slot() {
    critical_section_enter_blocking(&g_state_lock);
    unsigned physical_count = 0;
    for (const BackendSlot& slot : g_slots) {
        physical_count += slot.device != nullptr;
        physical_count += slot.companion != nullptr;
    }
    critical_section_exit(&g_state_lock);
    return physical_count < kSlotCount;
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

int physical_index_for_device(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return -1;
    }
    const int index = uni_hid_device_get_idx_for_instance(device);
    return index >= 0 && index < kSlotCount ? index : -1;
}

int slot_for_device(const uni_hid_device_t* device) {
    if (device != nullptr) {
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            if (g_slots[index].device == device ||
                g_slots[index].companion == device) {
                return index;
            }
        }
    }
    return -1;
}

// Called under the state lock. Bluepad32's index is a transport resource, not
// an output index once two Joy-Cons merge. Never evict an unrelated output.
int reserve_device_slot(uni_hid_device_t* device) {
    const int physical_index = physical_index_for_device(device);
    if (physical_index < 0) {
        return -1;
    }
    const int tracked = slot_for_device(device);
    if (g_retired_devices[physical_index] == device &&
        uni_hid_parser_switch2_is_ble_device(device)) {
        return -1;
    }
    if (tracked >= 0) {
        return tracked;
    }
    for (const BackendSlot& slot : g_slots) {
        if ((slot.device != nullptr &&
             physical_index_for_device(slot.device) == physical_index) ||
            (slot.companion != nullptr &&
             physical_index_for_device(slot.companion) == physical_index)) {
            return -1;
        }
    }
    if (g_slots[physical_index].device == nullptr) {
        return physical_index;
    }
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        if (g_slots[index].device == nullptr) {
            return index;
        }
    }
    return -1;
}

int joycon_side(const uni_hid_device_t* device) {
    if (!uni_hid_parser_switch2_is_ble_device(device)) {
        return 0;
    }
    if (device->product_id == UNI_SW2_JOYCON_L_PID) {
        return -1;
    }
    return device->product_id == UNI_SW2_JOYCON_R_PID ? 1 : 0;
}

bool joycon_default_pairing_allowed(const uni_hid_device_t* device) {
    const int index = physical_index_for_device(device);
    return g_joycon_mode == JoyConMode::kPaired && index >= 0 &&
        g_joycon_overrides[index].mode == JoyConGroupingOverride::Default;
}

// Include the entire old attempt when another held solo makes selection
// ambiguous. None of its members may retry until all have released.
void block_joycon_gesture(uint8_t participants) {
    for (uint8_t pass = 0; pass < kSlotCount; ++pass) {
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            if (participants & (1u << index)) {
                participants |= g_joycon_gestures[index].participants;
            }
        }
    }
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        if (!(participants & (1u << index))) continue;
        g_joycon_gestures[index].participants = participants;
        g_joycon_gestures[index].blocked = true;
    }
}

bool joycon_gesture_live(uint8_t index) {
    const JoyConGesture& gesture = g_joycon_gestures[index];
    const int slot = slot_for_device(gesture.device);
    return gesture.device != nullptr && slot >= 0 && g_slots[slot].active &&
        physical_index_for_device(gesture.device) == index &&
        joycon_side(gesture.device) != 0;
}

void arm_joycon_gesture(uint8_t left, uint8_t right, uint32_t now_ms,
                       bool joining) {
    JoyConGesture& l = g_joycon_gestures[left];
    JoyConGesture& r = g_joycon_gestures[right];
    if (!l.held || !r.held || l.blocked || r.blocked ||
        l.participants != (1u << left) ||
        r.participants != (1u << right)) return;
    l.participants = r.participants = (1u << left) | (1u << right);
    l.started_ms = r.started_ms = now_ms;
    l.joining = r.joining = joining;
}

// Caller holds the state lock. Reports perform admission; the timer also
// expires attempts, but elapsed cached input alone can never complete a hold.
void refresh_joycon_gestures(uint32_t now_ms) {
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        JoyConGesture& gesture = g_joycon_gestures[index];
        if (gesture.participants == 0 || gesture.blocked) continue;
        if (!joycon_gesture_live(index) || !gesture.held ||
            now_ms - gesture.last_report_ms > kJoyConGestureFreshMs) {
            block_joycon_gesture(gesture.participants);
        }
    }
    for (JoyConGesture& gesture : g_joycon_gestures) {
        if (gesture.participants == 0) continue;
        bool released = true;
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            if ((gesture.participants & (1u << index)) &&
                !g_joycon_gestures[index].released) released = false;
        }
        if (!released) continue;
        const uint8_t participants = gesture.participants;
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            if (!(participants & (1u << index))) continue;
            g_joycon_gestures[index].participants = 0;
            g_joycon_gestures[index].blocked = false;
        }
    }

    int left = -1;
    int right = -1;
    uint8_t solos = 0;
    bool ambiguous = false;
    for (const BackendSlot& slot : g_slots) {
        if (!slot.active || joycon_side(slot.device) == 0) continue;
        const int index = physical_index_for_device(slot.device);
        if (index < 0) continue;
        if (slot.companion != nullptr) {
            const int mate = physical_index_for_device(slot.companion);
            if (mate >= 0) arm_joycon_gesture(index, mate, now_ms, false);
            continue;
        }
        const JoyConGesture& gesture = g_joycon_gestures[index];
        if (gesture.device != slot.device || !gesture.held || gesture.blocked) continue;
        solos |= 1u << index;
        int& side = joycon_side(slot.device) < 0 ? left : right;
        ambiguous = ambiguous || side >= 0;
        side = index;
    }
    if (ambiguous) {
        block_joycon_gesture(solos);
    } else if (left >= 0 && right >= 0) {
        arm_joycon_gesture(left, right, now_ms, true);
    }
}

void observe_joycon_gesture(uni_hid_device_t* device,
                            const uni_gamepad_t& raw, uint32_t now_ms) {
    const int side = joycon_side(device);
    const int index = physical_index_for_device(device);
    if (side == 0 || index < 0) return;
    // Expire before updating the timestamp: a returning stale report must not
    // hide a gap, even when no timer ran during it.
    refresh_joycon_gestures(now_ms);
    JoyConGesture& gesture = g_joycon_gestures[index];
    gesture.device = device;
    const bool trigger = (raw.buttons &
        (side < 0 ? BUTTON_TRIGGER_L : BUTTON_TRIGGER_R)) != 0;
    const bool menu = (raw.misc_buttons &
        (side < 0 ? MISC_BUTTON_SELECT : MISC_BUTTON_START)) != 0;
    gesture.last_report_ms = now_ms;
    gesture.held = trigger && menu;
    gesture.released = !trigger && !menu;
    if (gesture.held) {
        gesture.masked = true;
        if (gesture.participants == 0) gesture.participants = 1u << index;
    } else if (gesture.released) {
        gesture.masked = false;
    }
    refresh_joycon_gestures(now_ms);
}

bool joycon_gesture_masked(const uni_hid_device_t* device) {
    const int index = physical_index_for_device(device);
    return index >= 0 && g_joycon_gestures[index].device == device &&
        g_joycon_gestures[index].masked;
}

void mask_joycon_gesture(uni_gamepad_t& gamepad,
                         const uni_hid_device_t* device) {
    if (!joycon_gesture_masked(device)) return;
    if (joycon_side(device) < 0) {
        gamepad.buttons &= ~BUTTON_TRIGGER_L;
        gamepad.misc_buttons &= ~MISC_BUTTON_SELECT;
        gamepad.brake = 0;
    } else {
        gamepad.buttons &= ~BUTTON_TRIGGER_R;
        gamepad.misc_buttons &= ~MISC_BUTTON_START;
        gamepad.throttle = 0;
    }
}

bool waiting_for_joycon_mate(int side = 0) {
    critical_section_enter_blocking(&g_state_lock);
    unsigned physical_count = 0;
    bool pending = false;
    unsigned left_count = 0;
    unsigned right_count = 0;
    for (const BackendSlot& slot : g_slots) {
        physical_count += slot.device != nullptr;
        physical_count += slot.companion != nullptr;
        pending = pending || (slot.device != nullptr && !slot.active);
        // An explicit solo choice is complete, not a request for another
        // default-paired mate. Individual defaults retain balanced reconnects.
        if (slot.active && slot.companion == nullptr &&
            (g_joycon_mode == JoyConMode::kIndividual ||
             joycon_default_pairing_allowed(slot.device))) {
            const int candidate_side = joycon_side(slot.device);
            left_count += candidate_side < 0;
            right_count += candidate_side > 0;
        }
    }
    critical_section_exit(&g_state_lock);
    // Individual players still reconnect their remembered opposite half, but
    // a balanced set is complete even though no logical pair was created.
    const bool missing_left = g_joycon_mode == JoyConMode::kIndividual
                                  ? right_count > left_count
                                  : right_count != 0;
    const bool missing_right = g_joycon_mode == JoyConMode::kIndividual
                                   ? left_count > right_count
                                   : left_count != 0;
    return physical_count < kSlotCount && !pending &&
           ((side <= 0 && missing_left) || (side >= 0 && missing_right));
}

void stop_background_scan() {
    if (!SWITCH_PICO_ENABLE_BLE) {
        return;
    }
    if (g_background_scan_active) {
        // Direct LE scans do not update Bluepad32's aggregate scanning flag.
        uni_bt_le_scan_stop();
        g_background_scan_active = false;
    }
}
// Core 1 only. Reconcile every ready physical Switch 2 link to the fast interval,
// independently of player grouping, controller count, or Classic connections.
void apply_radio_connection_policy() {
    if (!SWITCH_PICO_ENABLE_BLE) {
        return;
    }
    uni_hid_device_t* ready[kSlotCount]{};
    for (const BackendSlot& slot : g_slots) {
        uni_hid_device_t* targets[] = {slot.device, slot.companion};
        for (uni_hid_device_t* target : targets) {
            const int index = physical_index_for_device(target);
            if (index < 0) continue;
            const auto type = gap_get_connection_type(target->conn.handle);
            if (type == GAP_CONNECTION_LE &&
                uni_hid_parser_switch2_is_ble_device(target)) {
                // The parser requests its initial interval during setup.
                // Do not race that request by changing a pending device here.
                if (slot.active) ready[index] = target;
            }
        }
    }
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        auto& request = g_switch2_interval_requests[index];
        if (ready[index] == nullptr) {
            request = {};
            continue;
        }
        const auto handle = ready[index]->conn.handle;
        if (request.handle != handle) request = {};
        const uint16_t actual = gap_le_connection_interval(handle);
        if (request.interval != 0 && actual != request.interval &&
            now_ms - request.requested_ms < kSwitch2IntervalSettleMs) {
            // Let an accepted asynchronous update settle before retrying it.
            // API success alone does not prove that negotiation completed.
            continue;
        }
        request.interval = 0;
        if (actual == kSwitch2FastInterval) continue;
        gap_update_connection_parameters(
            handle, kSwitch2FastInterval, kSwitch2FastInterval, 0, 600);
        // Reconcile negotiated state on the configuration timer. Rejected or
        // incomplete requests are retried at most once per second per link.
        request = {handle, kSwitch2FastInterval, now_ms};
    }
}


// Caller holds the cross-core state lock. Only Core 1 resets parser state.
void clear_switch2_ingress(BackendSlot& slot) {
    Switch2Ingress& ingress = slot.switch2_ingress;
    __atomic_add_fetch(&g_switch2_ingress_drops, ingress.count, __ATOMIC_RELAXED);
    ingress.head = 0;
    ingress.count = 0;
    ++ingress.generation;
    ingress.reset_pending = true;
}

void reset_switch2_outputs(BackendSlot& slot) {
    uni_hid_device_t* targets[] = {slot.device, slot.companion};
    for (uni_hid_device_t* target : targets) {
        if (uni_hid_parser_switch2_is_ble_device(target)) {
            uni_hid_parser_switch2_reset_haptics(target);
        }
    }
    slot.switch2_ingress.reset_pending = false;
}

bool switch2_has_hd(const ControllerRumbleOutput& rumble) {
    return rumble.hd.actuators[0].sample_count != 0 ||
           rumble.hd.actuators[1].sample_count != 0;
}

bool switch2_host_stop(const ControllerRumbleOutput& rumble) {
    if (!switch2_has_hd(rumble)) {
        return (rumble.low_frequency_magnitude | rumble.high_frequency_magnitude) == 0;
    }
    for (const SwitchHapticsActuatorFrame& side : rumble.hd.actuators) {
        if (side.sample_count == 0 || side.sample_count > 3) return false;
        for (uint8_t index = 0; index < side.sample_count; ++index) {
            if (side.samples[index].low_amplitude_q15 != 0 ||
                side.samples[index].high_amplitude_q15 != 0) return false;
        }
    }
    return true;
}

void encode_switch2_side(uni_switch2_haptics_side_t& output,
                         const SwitchHapticsActuatorFrame& input) {
    output.count = input.sample_count;
    for (uint8_t index = 0; index < input.sample_count; ++index) {
        const SwitchHapticsSample& sample = input.samples[index];
        uni_switch2_haptics_encode_sample(
            output.samples[index], sample.low_frequency_index,
            sample.high_frequency_index, sample.low_amplitude_q15,
            sample.high_amplitude_q15);
    }
}

uni_switch2_haptics_frame_t switch2_physical_frame(
    const ControllerRumbleOutput& rumble, const uni_hid_device_t* target,
    bool paired) {
    uni_switch2_haptics_frame_t frame{};
    const SwitchHapticsActuatorFrame& left = rumble.hd.actuators[0];
    const SwitchHapticsActuatorFrame& right = rumble.hd.actuators[1];
    const int side = joycon_side(target);
    if (side == 0) {
        encode_switch2_side(frame.sides[0], left);
        encode_switch2_side(frame.sides[1], right);
    } else if (paired) {
        encode_switch2_side(frame.sides[0], side < 0 ? left : right);
    } else {
        // Mono chooses each band's louder source independently. A short side
        // holds its final substep; an absent side contributes no update.
        frame.sides[0].count =
            left.sample_count > right.sample_count ? left.sample_count : right.sample_count;
        for (uint8_t index = 0; index < frame.sides[0].count; ++index) {
            const SwitchHapticsSample* l = left.sample_count == 0 ? nullptr :
                &left.samples[index < left.sample_count ? index : left.sample_count - 1];
            const SwitchHapticsSample* r = right.sample_count == 0 ? nullptr :
                &right.samples[index < right.sample_count ? index : right.sample_count - 1];
            const SwitchHapticsSample* low = !r || (l && l->low_amplitude_q15 >= r->low_amplitude_q15) ? l : r;
            const SwitchHapticsSample* high = !r || (l && l->high_amplitude_q15 >= r->high_amplitude_q15) ? l : r;
            uni_switch2_haptics_encode_sample(
                frame.sides[0].samples[index], low->low_frequency_index,
                high->high_frequency_index, low->low_amplitude_q15,
                high->high_amplitude_q15);
        }
    }
    return frame;
}

void drain_switch2_ingress(uint8_t slot_index, uint32_t now_ms) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (!slot.active || !uni_hid_parser_switch2_is_ble_device(slot.device)) {
        critical_section_exit(&g_state_lock);
        return;
    }
    Switch2Ingress& ingress = slot.switch2_ingress;
    const uint16_t duration_ms = host_rumble_duration_ms();
    const uint8_t host_mode = switch2_host_mode();
    if (ingress.host_mode != host_mode) {
        clear_switch2_ingress(slot);
        ingress.host_mode = host_mode;
    }
    if (ingress.reset_pending) reset_switch2_outputs(slot);
    for (uint8_t budget = 0; budget < kSwitch2IngressCapacity && ingress.count != 0; ++budget) {
        Switch2HostCommand& command = ingress.commands[ingress.head];
        const RumbleEnvelope& envelope = command.envelope;
        const bool hd = switch2_has_hd(envelope.rumble);
        const bool stop = switch2_host_stop(envelope.rumble);
        const bool stale = command.generation != ingress.generation ||
            envelope.connection_generation != slot.connection_generation ||
            envelope.duration_ms != duration_ms ||
            (!stop && (hd || duration_ms != kXInputHostRumbleDurationMs) &&
             static_cast<uint32_t>(now_ms - envelope.received_ms) >= UNI_SWITCH2_HAPTICS_WATCHDOG_MS);
        const bool invalid = envelope.rumble.hd.actuators[0].sample_count > 3 ||
                             envelope.rumble.hd.actuators[1].sample_count > 3;
        if (stale || invalid) {
            __atomic_add_fetch(&g_switch2_ingress_drops, 1, __ATOMIC_RELAXED);
        } else {
            uni_hid_device_t* targets[] = {slot.device, slot.companion};
            const uint8_t target_mask = slot.companion == nullptr ? 1 : 3;
            for (uint8_t half = 0; half < 2; ++half) {
                const uint8_t bit = 1u << half;
                if (!(target_mask & bit) || (command.accepted_halves & bit)) continue;
                bool accepted;
                if (hd) {
                    const uni_switch2_haptics_frame_t frame =
                        switch2_physical_frame(envelope.rumble, targets[half], slot.companion != nullptr);
                    if (frame.sides[0].count == 0 && frame.sides[1].count == 0) {
                        command.accepted_halves |= bit;
                        continue;
                    }
                    accepted = uni_hid_parser_switch2_queue_haptics(
                        targets[half], &frame, envelope.received_ms);
                } else {
                    accepted = uni_hid_parser_switch2_queue_rumble(
                        targets[half], envelope.rumble.high_frequency_magnitude,
                        envelope.rumble.low_frequency_magnitude,
                        stop ? 0 : envelope.duration_ms, envelope.received_ms);
                }
                if (accepted) {
                    command.accepted_halves |= bit;
                    __atomic_add_fetch(&g_rumble_dispatches, 1, __ATOMIC_RELAXED);
                }
            }
            if (command.accepted_halves != target_mask) break;
        }
        ingress.head = (ingress.head + 1u) % kSwitch2IngressCapacity;
        --ingress.count;
    }
    critical_section_exit(&g_state_lock);
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
            uint8_t address_type = BD_ADDR_TYPE_UNKNOWN;
            if (uni_hid_parser_switch2_identity_address_type(
                    device, &address_type)) {
                BleIdentityMapping proprietary{};
                proprietary.identity_address_type = address_type;
                memcpy(proprietary.identity_address, device->conn.btaddr,
                       sizeof(proprietary.identity_address));
                return make_ble_identity(proprietary, device);
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
    bool joycon_identity_changed = false;
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        if (slot.device != nullptr && slot.companion == nullptr &&
            gap_get_connection_type(slot.device->conn.handle) ==
                GAP_CONNECTION_LE &&
            slot.device->conn.handle == mapping.connection_handle &&
            addresses_equal(slot.device->conn.btaddr,
                            mapping.connection_address)) {
            const ControllerIdentity identity = make_ble_identity(mapping, slot.device);
            joycon_identity_changed = slot.active && joycon_side(slot.device) != 0 &&
                !controller_identity_equal(slot.identity, identity);
            slot.identity = identity;
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
        if (joycon_identity_changed && g_joycon_mode == JoyConMode::kPaired) {
            g_joycon_reconcile_requested = true;
        }
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
        if (slot.device != nullptr && slot.companion == nullptr &&
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
        device != nullptr && slot_index < kSlotCount &&
        g_slots[slot_index].active &&
        (g_slots[slot_index].device == device ||
         g_slots[slot_index].companion == device) &&
        g_slots[slot_index].connection_generation ==
            connection_generation;
    critical_section_exit(&g_state_lock);
    return current;
}



ConnectionStatus compute_connection_status() {
    critical_section_enter_blocking(&g_state_lock);
    bool all_ready = true;
    bool any_connecting = false;
    unsigned physical_count = 0;
    for (const BackendSlot& slot : g_slots) {
        const bool has_device = slot.device != nullptr;
        physical_count += has_device;
        physical_count += slot.companion != nullptr;
        all_ready = all_ready && (!has_device || slot.active);
        any_connecting = any_connecting || (!slot.active && has_device);
    }
    critical_section_exit(&g_state_lock);

    if (all_ready && physical_count == kSlotCount) {
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
#ifdef SWITCH_PICO_WII_IR_GYRO
        if (device->controller_type == CONTROLLER_TYPE_WiiController) {
            const ControllerMotionSample sample =
                state.motion_sample_count != 0
                    ? state.motion_samples[0] : ControllerMotionSample{};
            wii_ir_gyro_update_motion(
                slot, target.connection_generation, target.motion_enabled, sample);
            if (!target.motion_enabled) {
                target.state.motion_sample_count = 0;
            }
        }
#endif
        ++target.state_generation;
        g_macro_capture.observe(slot, target.connection_generation,
                                time_us_32(), target.state);
    }
    critical_section_exit(&g_state_lock);
}

void publish_all_neutral() {
    critical_section_enter_blocking(&g_state_lock);
#ifdef SWITCH_PICO_WII_IR
    wii_ir_pointer_reset();
#endif
    for (BackendSlot& slot : g_slots) {
        clear_switch2_ingress(slot);
        reset_switch2_outputs(slot);
        slot.state = make_neutral_state();
        slot.accelerometer = {};
        slot.pre_hotkey_button_mask = 0;
        slot.identity = controller_identity_global();
        slot.device = nullptr;
        slot.companion = nullptr;
        slot.gamepad = {};
        slot.companion_gamepad = {};
        slot.extra_buttons = 0;
        slot.companion_extra_buttons = 0;
        slot.active = false;
        slot.wii_orientation_pending = false;
        slot.pending_wii_orientation = {};
#ifdef SWITCH_PICO_WII_IR_GYRO
        slot.wii_aim = {};
#endif
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
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        g_joycon_gestures[index] = {};
        g_joycon_overrides[index] = {};
        g_joycon_pair_hints[index] = {};
    }
    critical_section_exit(&g_state_lock);
    for (BleIdentityMapping& mapping : g_ble_identity_mappings) {
        mapping = {};
    }
    g_connection_status = ConnectionStatus::Initializing;
    stop_background_scan();
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
bool wake_chord_rising_edge(uint8_t slot, uni_hid_device_t* device,
                            uint16_t button_mask) {
    const uint16_t chord =
        logical_button_bit(ControllerProfileLogicalButton::kLeftShoulder) |
        logical_button_bit(ControllerProfileLogicalButton::kRightShoulder) |
        logical_button_bit(ControllerProfileLogicalButton::kSystem);
    critical_section_enter_blocking(&g_state_lock);
    const BackendSlot& previous = g_slots[slot];
    const bool rising =
        previous.active && previous.device == device &&
        (button_mask & chord) == chord &&
        (previous.pre_hotkey_button_mask & chord) != chord;
    critical_section_exit(&g_state_lock);
    return rising;
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
int32_t negate_motion_axis(int32_t value) {
    return value == INT32_MIN ? INT32_MAX : -value;
}

void rotate_solo_joycon(uni_gamepad_t& gamepad, int side,
                         uint8_t extras) {
    const uint32_t buttons = gamepad.buttons;
    if (side < 0) {
        const int32_t x = gamepad.axis_x;
        gamepad.axis_x = clamp_axis(gamepad.axis_y);
        gamepad.axis_y = clamp_axis(-clamp_axis(x));
        gamepad.buttons &= ~(BUTTON_A | BUTTON_B | BUTTON_X | BUTTON_Y);
        gamepad.buttons |=
            ((gamepad.dpad & DPAD_LEFT) ? uint32_t{BUTTON_A} : 0u) |
            ((gamepad.dpad & DPAD_DOWN) ? uint32_t{BUTTON_B} : 0u) |
            ((gamepad.dpad & DPAD_UP) ? uint32_t{BUTTON_X} : 0u) |
            ((gamepad.dpad & DPAD_RIGHT) ? uint32_t{BUTTON_Y} : 0u) |
            ((extras & UNI_SW2_BUTTON_LEFT_SL) ? uint32_t{BUTTON_SHOULDER_L} : 0u) |
            ((extras & UNI_SW2_BUTTON_LEFT_SR) ? uint32_t{BUTTON_SHOULDER_R} : 0u);
    } else {
        gamepad.axis_x = clamp_axis(-clamp_axis(gamepad.axis_ry));
        gamepad.axis_y = clamp_axis(gamepad.axis_rx);
        gamepad.buttons &=
            ~(BUTTON_A | BUTTON_B | BUTTON_X | BUTTON_Y | BUTTON_THUMB_R);
        gamepad.buttons |=
            ((buttons & BUTTON_B) ? uint32_t{BUTTON_A} : 0u) |
            ((buttons & BUTTON_Y) ? uint32_t{BUTTON_B} : 0u) |
            ((buttons & BUTTON_A) ? uint32_t{BUTTON_X} : 0u) |
            ((buttons & BUTTON_X) ? uint32_t{BUTTON_Y} : 0u) |
            ((buttons & BUTTON_THUMB_R) ? uint32_t{BUTTON_THUMB_L} : 0u) |
            ((extras & UNI_SW2_BUTTON_RIGHT_SL) ? uint32_t{BUTTON_SHOULDER_L} : 0u) |
            ((extras & UNI_SW2_BUTTON_RIGHT_SR) ? uint32_t{BUTTON_SHOULDER_R} : 0u);
    }
    gamepad.dpad = 0;
    gamepad.axis_rx = 0;
    gamepad.axis_ry = 0;
    int32_t* motion_axes[] = {gamepad.accel, gamepad.gyro};
    for (int32_t* axes : motion_axes) {
        const int32_t x = axes[0];
        axes[0] = side < 0 ? negate_motion_axis(axes[1]) : axes[1];
        axes[1] = side < 0 ? x : negate_motion_axis(x);
    }
}
#ifdef SWITCH_PICO_WII_IR_GYRO
uint32_t wii_aim_chord_button_mask(const uni_hid_device_t* device) {
    if (device == nullptr ||
        device->controller_type != CONTROLLER_TYPE_WiiController) {
        return 0;
    }
    switch (device->controller_subtype) {
        case CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL:
        case CONTROLLER_SUBTYPE_WIIMOTE_ACCEL:
            return BUTTON_A | BUTTON_B;
        case CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL:
            return BUTTON_X | BUTTON_Y;
        case CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK:
        case CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK_ACCEL:
            return BUTTON_SHOULDER_L | BUTTON_SHOULDER_R;
        default:
            return 0;
    }
}
#endif

uni_gamepad_t logical_gamepad(const BackendSlot& slot) {
    uni_gamepad_t gamepad = slot.gamepad;
    mask_joycon_gesture(gamepad, slot.device);
#ifdef SWITCH_PICO_WII_IR_GYRO
    if (slot.wii_aim.masked) {
        gamepad.buttons &= ~wii_aim_chord_button_mask(slot.device);
    }
    if (slot.wii_aim.reposition_masked) {
        gamepad.buttons &= ~(BUTTON_X | BUTTON_SHOULDER_L);
    }
#endif
    if (slot.companion != nullptr) {
        const uni_gamepad_t& right = slot.companion_gamepad;
        gamepad.dpad |= right.dpad;
        const bool masked = joycon_gesture_masked(slot.companion);
        gamepad.buttons |=
            right.buttons & ~(masked ? uint32_t{BUTTON_TRIGGER_R} : 0u);
        gamepad.misc_buttons |=
            right.misc_buttons & ~(masked ? uint32_t{MISC_BUTTON_START} : 0u);
        gamepad.axis_rx = right.axis_rx;
        gamepad.axis_ry = right.axis_ry;
        gamepad.throttle = masked ? 0 : right.throttle;
        // The right half is the sole aim source. A left report must not
        // republish an already consumed right-hand motion sample.
        memcpy(gamepad.accel, right.accel, sizeof(gamepad.accel));
        memcpy(gamepad.gyro, right.gyro, sizeof(gamepad.gyro));
    } else {
        const int side = joycon_side(slot.device);
        if (side != 0) {
            rotate_solo_joycon(gamepad, side, slot.extra_buttons);
        }
    }
    return gamepad;
}

void refresh_topology_input(BackendSlot& slot) {
    const uni_gamepad_t gamepad = logical_gamepad(slot);
    slot.pre_hotkey_button_mask = logical_button_mask(gamepad);
    // Topology changes release the lost half immediately; motion stays neutral
    // until a fresh report from the newly selected source arrives.
    slot.state = map_gamepad(gamepad, false, slot.pre_hotkey_button_mask);
    slot.state.extra_buttons =
        slot.extra_buttons | slot.companion_extra_buttons;
}
struct HotkeyDecision {
    bool motion_enabled;
};

void queue_local_feedback(BackendSlot& slot, uint16_t duration_ms,
                          uint8_t weak_magnitude,
                          uint8_t strong_magnitude) {
    slot.feedback_pending = true;
    slot.pending_feedback = {
        slot.connection_generation, duration_ms, weak_magnitude,
        strong_magnitude};
    __atomic_add_fetch(&g_local_feedback_requests, 1, __ATOMIC_RELAXED);
}

void queue_profile_feedback(BackendSlot& slot,
                            const ProfileFeedbackEnvelope& feedback) {
    if (slot.pending_profile_feedback_count < kProfileFeedbackQueueCapacity) {
        slot.pending_profile_feedback[
            slot.pending_profile_feedback_count++] = feedback;
    } else {
        slot.pending_profile_feedback[kProfileFeedbackQueueCapacity - 1u] =
            feedback;
    }
}

#ifdef SWITCH_PICO_WII_IR_GYRO
void observe_wii_aim_chord(BackendSlot& slot, uni_hid_device_t* device,
                           const uni_gamepad_t& gamepad,
                           const uni_wii_ir_snapshot_t* infrared,
                           uint32_t now_us) {
    const uint32_t mapped_buttons = wii_aim_chord_button_mask(device);
    if (mapped_buttons == 0) return;
    WiiAimSource& aim = slot.wii_aim;
    const uint8_t slot_index = static_cast<uint8_t>(&slot - g_slots);
    if (mapped_buttons == (BUTTON_SHOULDER_L | BUTTON_SHOULDER_R)) {
        const uint32_t controls = BUTTON_X | BUTTON_SHOULDER_L;  // Nunchuk C + 1.
        const uint32_t pressed = gamepad.buttons & controls;
        if (aim.infrared && pressed == controls) {
            aim.reposition_masked = true;
        } else if (pressed == 0) {
            aim.reposition_masked = false;
        }
    } else {
        aim.reposition_masked = false;
    }

    // Expire before accepting a returning packet. Cached snapshots cannot
    // extend or complete a hold, and a gap requires a full release to retry.
    // A transport gap cancels the gesture, not the user's selected source.
    // The pointer's freshness guard stops IR output without falling back.
    if (aim.have_sequence &&
        now_us - aim.last_report_us >= kWiiAimChordFreshUs) {
        aim.holding = false;
        aim.latched = aim.latched || aim.masked;
    }
    const uint32_t pressed_buttons = gamepad.buttons & mapped_buttons;
    if (pressed_buttons == mapped_buttons) {
        aim.masked = true;
    } else {
        aim.holding = false;
        if (pressed_buttons == 0) {
            aim.masked = false;
            aim.latched = false;
        } else {
            aim.latched = aim.latched || aim.masked;
        }
    }
    if (infrared == nullptr ||
        (aim.have_sequence && infrared->sequence == aim.sequence) ||
        (!aim.have_sequence && infrared->sequence == 0)) {
        return;
    }
    aim.have_sequence = true;
    aim.sequence = infrared->sequence;
    aim.last_report_us = now_us;
    const uint16_t buttons = infrared->buttons & kWiiAimChordButtons;
    if (buttons != kWiiAimChordButtons || pressed_buttons != mapped_buttons) {
        aim.holding = false;
        aim.latched = aim.latched || aim.masked;
        return;
    }
    if (aim.latched) return;
    if (!aim.holding) {
        aim.started_us = now_us;
        aim.holding = true;
        return;
    }
    if (now_us - aim.started_us < kWiiAimChordHoldUs) return;
    aim.latched = true;
    if (!wii_ir_gyro_select(
            slot_index, slot.connection_generation, !aim.infrared)) {
        return;
    }
    aim.infrared = !aim.infrared;
    queue_profile_feedback(
        slot, {slot.connection_generation,
               static_cast<uint8_t>(aim.infrared ? 2 : 1),
               ControllerProfileConfirmationPolicy::kRumble});
    __atomic_add_fetch(&g_local_feedback_requests, 1, __ATOMIC_RELAXED);
}
#endif

void reset_slot_hotkeys(BackendSlot& slot) {
    g_macro_capture.disconnect(static_cast<uint8_t>(&slot - g_slots),
                               slot.connection_generation, time_us_32());
    slot.wii_orientation_pending = false;
    slot.pending_wii_orientation = {};
#ifdef SWITCH_PICO_WII_IR_GYRO
    slot.wii_aim = {};
#endif
    slot.motion_enabled = kDefaultMotionEnabled;
    slot.pre_hotkey_button_mask = 0;
    slot.accelerometer = {};
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

void invalidate_slot(BackendSlot& slot) {
#ifdef SWITCH_PICO_WII_IR
    wii_ir_pointer_disconnect(static_cast<uint8_t>(&slot - g_slots));
#endif
    clear_switch2_ingress(slot);
    reset_switch2_outputs(slot);
    reset_slot_hotkeys(slot);
    slot.rumble_pending = false;
    slot.pending_rumble = {};
    slot.state = make_neutral_state();
    ++slot.state_generation;
    ++slot.connection_generation;
}

void release_slot(BackendSlot& slot) {
    invalidate_slot(slot);
    slot.identity = controller_identity_global();
    slot.device = nullptr;
    slot.companion = nullptr;
    slot.gamepad = {};
    slot.companion_gamepad = {};
    slot.extra_buttons = 0;
    slot.companion_extra_buttons = 0;
    slot.active = false;
}

bool is_solo_wii_remote(const BackendSlot& slot) {
    if (!slot.active || slot.device == nullptr || slot.companion != nullptr ||
        slot.device->controller_type != CONTROLLER_TYPE_WiiController) {
        return false;
    }
    switch (slot.device->controller_subtype) {
        case CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL:
        case CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL:
        case CONTROLLER_SUBTYPE_WIIMOTE_ACCEL:
            return true;
        default:
            return false;
    }
}


HotkeyDecision update_controller_hotkeys(
    uint8_t slot_index, uni_hid_device_t* device) {
    HotkeyDecision decision{kDefaultMotionEnabled};
    critical_section_enter_blocking(&g_state_lock);
    const BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device == device) {
        decision.motion_enabled = slot.motion_enabled;
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
            if (SWITCH_PICO_ENABLE_BLE && size >= 11) {
                clear_ble_identity_for_handle(
                    sm_event_identity_resolving_started_get_handle(packet));
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            if (SWITCH_PICO_ENABLE_BLE && size >= 11) {
                clear_ble_identity_for_handle(
                    sm_event_identity_resolving_failed_get_handle(packet));
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            if (SWITCH_PICO_ENABLE_BLE && size >= 20) {
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
            if (SWITCH_PICO_ENABLE_BLE && size >= 20) {
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
            if (SWITCH_PICO_ENABLE_BLE && size >= 11) {
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
            if (SWITCH_PICO_ENABLE_BLE && size >= 12) {
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
            if (!SWITCH_PICO_ENABLE_CLASSIC || size < 8) {
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
            if (!SWITCH_PICO_ENABLE_CLASSIC || size < 8) {
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
    if (g_connection_policy_state == ConnectionPolicyState::FailedClosed) {
        return false;
    }

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
        if (SWITCH_PICO_ENABLE_BLE) {
            sm_set_accepted_stk_generation_methods(kAllBlePairingMethods);
        }
        g_status_led_tick = 0;
        return true;
    }
    if (g_pairing_window_open && !pairing_window_active_at(now_ms)) {
        g_pairing_window_open = false;
        if (SWITCH_PICO_ENABLE_BLE) {
            sm_set_accepted_stk_generation_methods(0);
        }
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
    for (uint8_t index = 0; index < UNI_SWITCH2_PAIRING_CAPACITY; ++index) {
        uint8_t address_type = BD_ADDR_TYPE_UNKNOWN;
        bd_addr_t address{};
        if (uni_switch2_pairing_get(index, &address_type, address)) {
            append_pairing_record(
                snapshot, Bluepad32PairingTransport::kBle, address_type, address);
        }
    }

    critical_section_enter_blocking(&g_state_lock);
    if (g_pairing_snapshot.status == Bluepad32PairingSnapshotStatus::kFailed) {
        snapshot.status = Bluepad32PairingSnapshotStatus::kFailed;
    }
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
void recompute_connection_status();

void process_clear_pairings(uint32_t now_ms) {
    uni_hid_device_t* devices[kSlotCount]{};
    uint8_t device_count = 0;
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
            if (slot.device != nullptr) {
                g_retired_devices[physical_index_for_device(slot.device)] =
                    slot.device;
                devices[device_count++] = slot.device;
            }
            if (slot.companion != nullptr) {
                g_retired_devices[physical_index_for_device(slot.companion)] =
                    slot.companion;
                devices[device_count++] = slot.companion;
            }
            release_slot(slot);
            g_joycon_gestures[slot_index] = {};
            g_joycon_overrides[slot_index] = {};
            g_joycon_pair_hints[slot_index] = {};
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
    if (SWITCH_PICO_ENABLE_BLE) {
        sm_set_accepted_stk_generation_methods(0);
    }
    const bool proprietary_cleared = uni_switch2_pairing_clear();
    uni_bt_del_keys_unsafe();
    for (uni_hid_device_t* device : devices) {
        if (device != nullptr) {
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
            switch_native_output_detach(device);
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
            haptics_experiment_detach(device);
#endif
            uni_hid_device_disconnect(device);
        }
    }
    refresh_pairing_snapshot();
    if (!proprietary_cleared) {
        stop_background_scan();
        uni_bt_stop_scanning_unsafe();
        uni_bt_allow_incoming_connections(false);
        g_connection_policy_state = ConnectionPolicyState::FailedClosed;
        critical_section_enter_blocking(&g_state_lock);
        g_pairing_snapshot.status = Bluepad32PairingSnapshotStatus::kFailed;
        g_clear_pairings_in_progress_token = 0;
        g_clear_pairings_requested_token = 0;
        g_pairing_window_requested = false;
        critical_section_exit(&g_state_lock);
        return;
    }

    g_connection_status = ConnectionStatus::Scanning;
    g_status_led_tick = 0;
    g_pairing_reset_feedback_deadline_ms =
        now_ms + kPairingResetFeedbackDurationMs;
    if (g_connection_policy_state == ConnectionPolicyState::FailedClosed) {
        g_connection_policy_state = ConnectionPolicyState::Uninitialized;
    }
    apply_connection_policy();
    critical_section_enter_blocking(&g_state_lock);
    g_pairing_snapshot.completed_clear_pairings_token =
        request_token;
    g_clear_pairings_in_progress_token = 0;
    critical_section_exit(&g_state_lock);
}


void apply_connection_policy() {
    if (g_connection_policy_state == ConnectionPolicyState::FailedClosed) {
        return;
    }
    apply_radio_connection_policy();
    const bool free_slot = has_free_slot();
    const bool active_controller = has_active_controller();
    const bool pairing_open =
        pairing_window_active_at(btstack_run_loop_get_time_ms());
    const bool active_scan =
        free_slot && (!active_controller || pairing_open);
    const bool background_scan = SWITCH_PICO_ENABLE_BLE &&
        free_slot && !active_scan && waiting_for_joycon_mate();
    const ConnectionPolicyState desired_state =
        !free_slot
            ? ConnectionPolicyState::Paused
            : (active_scan ? ConnectionPolicyState::Open
                           : ConnectionPolicyState::Passive);
    if (g_connection_policy_state == desired_state &&
        g_background_scan_active == background_scan) {
        return;
    }

    // Leave low-duty LE explicitly before the aggregate stop, which otherwise
    // does nothing when its own scanning flag is already clear.
    stop_background_scan();
    uni_bt_stop_scanning_unsafe();

    if (!free_slot) {
        uni_bt_allow_incoming_connections(false);
        g_connection_policy_state = ConnectionPolicyState::Paused;
        return;
    }

    // Passive mode permits incoming Classic reconnects, with LE discovery
    // limited to a remembered opposite half for a ready solo Joy-Con2.
    uni_bt_allow_incoming_connections(SWITCH_PICO_ENABLE_CLASSIC != 0);
    if (active_scan) {
        if (SWITCH_PICO_ENABLE_BLE) {
            uni_bt_le_set_background_scan(false);
        }
        uni_bt_start_scanning_and_autoconnect_unsafe();
        g_connection_policy_state = ConnectionPolicyState::Open;
    } else {
        g_connection_policy_state = ConnectionPolicyState::Passive;
        if (background_scan) {
            uni_bt_le_set_background_scan(true);
            uni_bt_le_scan_start();
            g_background_scan_active = true;
        }
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

void apply_joycon_configuration(const ConfigurationServiceSnapshot& configuration);
void process_joycon_gestures(uint32_t now_ms);

void process_configuration_timer(btstack_timer_source_t* timer) {
    __atomic_add_fetch(
        &g_configuration_timer_ticks, 1, __ATOMIC_RELAXED);
    btstack_run_loop_set_timer(timer, kConfigurationPollIntervalMs);
    btstack_run_loop_add_timer(timer);
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    apply_radio_connection_policy();
    configuration_service_task_on_storage_core(now_ms);
    profile_service_task_on_storage_core(now_ms);
    ConfigurationServiceSnapshot configuration{};
    configuration_service_snapshot(&configuration);
    apply_joycon_configuration(configuration);
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    if (configuration.state == ConfigurationServiceState::kReady) {
        uint8_t previously_owned = 0;
        for (uint8_t i = 0; i < kSlotCount; ++i)
            if (switch_native_output_owns(g_slots[i].device)) previously_owned |= 1u << i;
        switch_native_output_configure(configuration.configuration, configuration.generation);
        for (uint8_t i = 0; i < kSlotCount; ++i) {
            if ((previously_owned & (1u << i)) || !switch_native_output_owns(g_slots[i].device))
                continue;
            RumbleEnvelope retained{};
            critical_section_enter_blocking(&g_state_lock);
            const BackendSlot& current = g_slots[i];
            retained = current.pending_rumble;
            const bool valid = current.active && retained.slot == i &&
                retained.connection_generation == current.connection_generation &&
                retained.duration_ms == host_rumble_duration_ms();
            critical_section_exit(&g_state_lock);
            if (valid) switch_native_output_submit(i, retained.connection_generation,
                retained.received_us, retained.rumble,
                retained.duration_ms == kXInputHostRumbleDurationMs);
        }
    }
#endif
}

void dispatch_rumble(uni_hid_device_t* device, uint16_t duration_ms,
                     uint8_t weak, uint8_t strong) {
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    if (switch_native_output_feedback(device, strong, weak, duration_ms)) return;
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    if (haptics_experiment_feedback(device, strong, weak, duration_ms)) {
        return;
    }
#endif
    device->report_parser.play_dual_rumble(device, 0, duration_ms, weak, strong);
}

// Core 1 only. The mailbox carries values, never a parser pointer supplied by
// Core 0. Revalidate after lifecycle/topology work and before touching the parser.
void process_wii_orientation(uint8_t slot_index) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (!slot.wii_orientation_pending) {
        critical_section_exit(&g_state_lock);
        return;
    }
    const WiiOrientationRequest request = slot.pending_wii_orientation;
    slot.wii_orientation_pending = false;
    slot.pending_wii_orientation = {};
    if (!is_solo_wii_remote(slot) ||
        slot.connection_generation != request.connection_generation ||
        !controller_identity_equal(slot.identity, request.identity)) {
        critical_section_exit(&g_state_lock);
        return;
    }
    uni_hid_device_t* device = slot.device;
    // Subtype publication can lag set_mode during extension discovery. Even a
    // reselection must reach the parser to replace a deferred opposite choice.
    // A new logical epoch retires profile macros, hotkey holds, capture and
    // feedback without touching this connection's identity or saved profiles.
    invalidate_slot(slot);
    slot.gamepad = {};
    slot.extra_buttons = 0;
    critical_section_exit(&g_state_lock);

    // The setter can synchronously re-enter the platform ready callback, so
    // release the lock first. Lifecycle and parser callbacks share this core.
    if (device->report_parser.play_dual_rumble != nullptr) {
        dispatch_rumble(device, 0, 0, 0);
    }
    uni_hid_parser_wii_set_mode(
        device, request.vertical ? WII_MODE_VERTICAL : WII_MODE_HORIZONTAL);
    apply_slot_lighting(slot_index, device);
}

uint8_t xbox_trigger_magnitude(const SwitchHapticsActuatorFrame& frame) {
    uint16_t peak = 0;
    for (uint8_t i = 0; i < frame.sample_count && i < 3; ++i) {
        if (frame.samples[i].high_amplitude_q15 > peak)
            peak = frame.samples[i].high_amplitude_q15;
    }
    // Impulse triggers are amplitude-only ERMs, not HD actuators. Keep their
    // extra response at half scale, including after profile amplification.
    if (peak > 32767) peak = 32767;
    return static_cast<uint8_t>((static_cast<uint32_t>(peak) * 127u) / 32767u);
}

void dispatch_host_rumble(uni_hid_device_t* device, uint16_t duration_ms,
                          const ControllerRumbleOutput& rumble) {
    const uint8_t weak = rumble.high_frequency_magnitude;
    const uint8_t strong = rumble.low_frequency_magnitude;
    if (device->vendor_id == 0x045e &&
        device->report_parser.play_dual_rumble ==
            uni_hid_parser_xboxone_play_dual_rumble) {
        const bool hd = rumble.hd.actuators[0].sample_count != 0 ||
                        rumble.hd.actuators[1].sample_count != 0;
        const uint8_t left = hd ? xbox_trigger_magnitude(rumble.hd.actuators[0])
                                : weak / 2u;
        const uint8_t right = hd ? xbox_trigger_magnitude(rumble.hd.actuators[1])
                                 : weak / 2u;
        const bool stop = (weak | strong | left | right) == 0;
        xboxone_play_quad_rumble(device, 0, stop ? 0 : duration_ms,
                                 left, right, weak, strong);
        return;
    }
    dispatch_rumble(device, (weak | strong) == 0 ? 0 : duration_ms, weak, strong);
}

#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
void seed_native_host_rumble() {
    HapticsExperimentDiagnostics native;
    haptics_experiment_snapshot(&native);
    if (native.mode != 1 || native.slot >= kSlotCount ||
        native.run_id == g_seeded_native_run_id ||
        (native.state != HapticsExperimentState::kPending &&
         native.state != HapticsExperimentState::kRunning)) {
        return;
    }
    g_seeded_native_run_id = native.run_id;
    RumbleEnvelope retained{};
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[native.slot];
    const bool valid = slot.active && slot.device != nullptr &&
                       slot.retained_host_rumble_valid &&
                       slot.connection_generation == native.connection_generation &&
                       slot.retained_host_rumble.connection_generation ==
                           native.connection_generation &&
                       slot.retained_host_rumble.slot == native.slot &&
                       slot.retained_host_rumble.duration_ms ==
                           kXInputHostRumbleDurationMs;
    if (valid) {
        retained = slot.retained_host_rumble;
        // Arming cancels compatibility output even when its mailbox was
        // already consumed. Keep that held state available for the next Stop.
        slot.pending_rumble = retained;
        slot.rumble_pending = true;
    }
    critical_section_exit(&g_state_lock);
    if (valid) {
        // Replay once on arm, not on a watchdog cadence. The original timestamp
        // keeps a raced newer USB command authoritative in the host timeline.
        haptics_experiment_submit_rumble(
            native.slot, native.connection_generation, retained.received_us,
            retained.rumble.low_frequency_magnitude,
            retained.rumble.high_frequency_magnitude);
    }
}
#endif

void process_rumble_timer(btstack_timer_source_t* timer) {
    __atomic_add_fetch(&g_rumble_timer_ticks, 1, __ATOMIC_RELAXED);
    uint32_t now_ms = btstack_run_loop_get_time_ms();

    process_clear_pairings(now_ms);
    process_pairing_snapshot_request();
    process_joycon_gestures(now_ms);
    now_ms = btstack_run_loop_get_time_ms();
    const bool wake_identity_ready =
        switch2_wake_ready_for_connections();
    if (g_connection_policy_state ==
            ConnectionPolicyState::Uninitialized &&
        wake_identity_ready) {
        recompute_connection_status();
    }
    if (update_pairing_window(now_ms) && wake_identity_ready) {
        apply_connection_policy();
    }
    const bool xinput_host_mode =
        host_rumble_duration_ms() == kXInputHostRumbleDurationMs;
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    if (xinput_host_mode) seed_native_host_rumble();
    haptics_experiment_poll();
#endif

    for (uint8_t slot_index = 0; slot_index < kSlotCount; ++slot_index) {
        process_wii_orientation(slot_index);
        drain_switch2_ingress(slot_index, now_ms);
        RumbleEnvelope envelope{};
        FeedbackEnvelope feedback{};
        ProfileFeedbackEnvelope profile_feedback{};
        uni_hid_device_t* device = nullptr;
        uni_hid_device_t* profile_lighting_device = nullptr;
        uint32_t profile_lighting_generation = 0;
        uni_hid_device_t* companion = nullptr;
        uint32_t dispatch_generation = 0;
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
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
        if (haptics_experiment_owns(slot.device) &&
            !haptics_experiment_gameplay_owns(slot.device)) {
            // Fixture/startup/restoration exclusively own output. Preserve
            // stateful XInput requests until compatibility restoration ends.
            if (!xinput_host_mode) slot.rumble_pending = false;
            critical_section_exit(&g_state_lock);
            continue;
        }
#endif
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
            !local_feedback_active && slot.rumble_pending
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
            && !(xinput_host_mode &&
                 haptics_experiment_gameplay_owns(slot.device))
#endif
        ) {
            envelope = slot.pending_rumble;
            slot.rumble_pending = false;
            host_dispatch =
                envelope.slot == slot_index && slot.active &&
                envelope.duration_ms == host_rumble_duration_ms() &&
                slot.device != nullptr &&
                envelope.connection_generation ==
                    slot.connection_generation;
            if (host_dispatch) {
                device = slot.device;
            }
        }
        companion = slot.companion;
        dispatch_generation = slot.connection_generation;
        critical_section_exit(&g_state_lock);
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
        if (host_dispatch && switch_native_output_owns(device))
            host_dispatch = false;  // The timestamped native queue already owns this command.
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
        if (host_dispatch && haptics_experiment_gameplay_owns(device)) {
            // Switch commands have already entered the timestamped timeline.
            // Consume their finite fallback, never turn it into a PCM overlay
            // or emit compatibility reports while gameplay owns the device.
            host_dispatch = false;
        }
#endif

        uni_hid_device_t* lighting_targets[] = {
            profile_lighting_device, companion};
        for (uni_hid_device_t* target : lighting_targets) {
            if (!lighting_target_is_current(
                    slot_index, profile_lighting_generation, target)) {
                continue;
            }
            if (profile_lighting_restore) {
                apply_slot_lighting(slot_index, target);
            }
            if (profile_lighting_dispatch) {
                apply_profile_lighting(
                    profile_feedback.active_profile_number, target);
            }
        }
        uni_hid_device_t* rumble_targets[] = {device, companion};
        for (uni_hid_device_t* target : rumble_targets) {
            if (!lighting_target_is_current(
                    slot_index, dispatch_generation, target) ||
                target->report_parser.play_dual_rumble == nullptr) {
                continue;
            }
            if (profile_rumble_dispatch) {
                __atomic_add_fetch(
                    &g_rumble_dispatches, 1, __ATOMIC_RELAXED);
                dispatch_rumble(
                    target, kProfileFeedbackPhaseDurationMs,
                    kProfileFeedbackWeakMagnitude, kProfileFeedbackStrongMagnitude);
            } else if (feedback_dispatch) {
                __atomic_add_fetch(
                    &g_rumble_dispatches, 1, __ATOMIC_RELAXED);
                dispatch_rumble(
                    target, feedback.duration_ms,
                    feedback.weak_magnitude, feedback.strong_magnitude);
            } else if (host_dispatch) {
                __atomic_add_fetch(
                    &g_rumble_dispatches, 1, __ATOMIC_RELAXED);
                dispatch_host_rumble(target, envelope.duration_ms, envelope.rumble);
            }
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

void forget_joycon_pair_hint(uni_hid_device_t* device) {
    const int physical_index = physical_index_for_device(device);
    if (physical_index < 0) return;
    for (JoyConPairHint& hint : g_joycon_pair_hints) {
        if (hint.mate == device) hint = {};
    }
    g_joycon_pair_hints[physical_index] = {};
}

// Replacing an explicit association does not revoke its former partner's solo
// choice. Only the new current association participates in disconnect reset.
void set_joycon_override(uni_hid_device_t* left, uni_hid_device_t* right,
                         JoyConGroupingOverride mode) {
    uni_hid_device_t* devices[] = {left, right};
    for (uni_hid_device_t* device : devices) {
        for (JoyConConnectionOverride& current : g_joycon_overrides) {
            if (current.mate == device) current.mate = nullptr;
        }
    }
    g_joycon_overrides[physical_index_for_device(left)] = {mode, right};
    g_joycon_overrides[physical_index_for_device(right)] = {mode, left};
}

// Caller holds the state lock; used for disconnect and physical index reuse.
void reset_joycon_connection(uni_hid_device_t* device) {
    const int index = physical_index_for_device(device);
    if (index < 0) return;
    bool changed = g_joycon_overrides[index].mode !=
        JoyConGroupingOverride::Default;
    for (JoyConConnectionOverride& current : g_joycon_overrides) {
        if (current.mate == device) {
            current = {};
            changed = true;
        }
    }
    g_joycon_overrides[index] = {};
    if (changed) g_joycon_reconcile_requested = true;
    block_joycon_gesture(g_joycon_gestures[index].participants);
    g_joycon_gestures[index] = {};
    for (JoyConGesture& gesture : g_joycon_gestures) {
        gesture.participants &= ~(1u << index);
    }
    forget_joycon_pair_hint(device);
}

bool joycon_gesture_mature(uni_hid_device_t* first,
                           uni_hid_device_t* second, bool joining,
                           uint32_t now_ms) {
    const int first_index = physical_index_for_device(first);
    const int second_index = physical_index_for_device(second);
    if (first_index < 0 || second_index < 0 || first_index == second_index ||
        joycon_side(first) == 0 || joycon_side(first) != -joycon_side(second)) {
        return false;
    }
    const uint8_t participants = (1u << first_index) | (1u << second_index);
    const JoyConGesture& a = g_joycon_gestures[first_index];
    const JoyConGesture& b = g_joycon_gestures[second_index];
    if (a.device != first || b.device != second ||
        a.started_ms != b.started_ms) return false;
    const int indices[] = {first_index, second_index};
    for (int index : indices) {
        const JoyConGesture& gesture = g_joycon_gestures[index];
        if (!joycon_gesture_live(index) ||
            gesture.participants != participants ||
            gesture.joining != joining || !gesture.held ||
            now_ms - gesture.last_report_ms > kJoyConGestureFreshMs ||
            static_cast<int32_t>(gesture.last_report_ms - gesture.started_ms) <
                static_cast<int32_t>(kJoyConGestureHoldMs)) return false;
    }
    const int a_slot = slot_for_device(first);
    const int b_slot = slot_for_device(second);
    return joining
        ? a_slot != b_slot && g_slots[a_slot].companion == nullptr &&
              g_slots[b_slot].companion == nullptr
        : a_slot == b_slot && g_slots[a_slot].companion != nullptr;
}

// Caller holds the state lock. Prefer the last live pair's exact members,
// otherwise preserve the existing first-ready / lowest-slot admission order.
int joycon_partner_slot(uni_hid_device_t* device, int slot_index) {
    const int side = joycon_side(device);
    if (side == 0 || !joycon_default_pairing_allowed(device)) return -1;
    const auto& hint = g_joycon_pair_hints[physical_index_for_device(device)];
    int first = -1;
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        const BackendSlot& candidate = g_slots[index];
        if (index == slot_index || !candidate.active ||
            candidate.companion != nullptr ||
            !joycon_default_pairing_allowed(candidate.device) ||
            joycon_side(candidate.device) != -side) continue;
        if (candidate.device == hint.mate) return index;
        const auto& candidate_hint =
            g_joycon_pair_hints[physical_index_for_device(candidate.device)];
        if (candidate_hint.mate != nullptr && candidate_hint.mate != device) continue;
        if (first < 0) first = index;
    }
    return first;
}

void stop_joycon_output(uni_hid_device_t* device) {
    if (device->report_parser.play_dual_rumble != nullptr) {
        dispatch_rumble(device, 0, 0, 0);
    }
}

// Core 1 only; shared by ready admission, saved defaults and explicit gestures.
// Pair enrollment is atomic and idempotent, and always precedes topology
// changes with no cross-core input lock held during storage I/O.
bool merge_joycon_slots(int owner_index, int joining_index,
                       uni_hid_device_t* joining_device,
                       bool gesture = false) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& owner = g_slots[owner_index];
    BackendSlot& joining = g_slots[joining_index];
    uni_hid_device_t* const owner_device = owner.device;
    const uint32_t owner_generation = owner.connection_generation;
    const uint32_t joining_generation = joining.connection_generation;
    const bool joining_active = joining.active;
    const int side = joycon_side(joining_device);
    const bool admission = gesture
        ? joining.active && joycon_gesture_mature(
              owner_device, joining_device, true, btstack_run_loop_get_time_ms())
        : joycon_default_pairing_allowed(owner_device) &&
              joycon_default_pairing_allowed(joining_device);
    const bool eligible = admission &&
        owner.active && owner.companion == nullptr &&
        joining.companion == nullptr && side != 0 &&
        joycon_side(owner_device) == -side &&
        reserve_device_slot(joining_device) == joining_index;
    const ControllerIdentity owner_identity = identity_for_device(owner_device);
    const ControllerIdentity joining_identity = identity_for_device(joining_device);
    critical_section_exit(&g_state_lock);
    ControllerIdentity pair_identity{};
    if (!eligible ||
        !controller_identity_make_joycon_pair(
            side < 0 ? joining_identity : owner_identity,
            side < 0 ? owner_identity : joining_identity, &pair_identity) ||
        !profile_service_observe_joycon_pair_on_storage_core(pair_identity)) {
        return false;
    }

    critical_section_enter_blocking(&g_state_lock);
    const bool still_admitted = gesture
        ? joycon_gesture_mature(
              owner_device, joining_device, true, btstack_run_loop_get_time_ms())
        : joycon_default_pairing_allowed(owner_device) &&
              joycon_default_pairing_allowed(joining_device);
    if (!still_admitted ||
        !owner.active || owner.device != owner_device ||
        owner.companion != nullptr ||
        owner.connection_generation != owner_generation ||
        joining.active != joining_active || joining.companion != nullptr ||
        joining.connection_generation != joining_generation ||
        reserve_device_slot(joining_device) != joining_index) {
        critical_section_exit(&g_state_lock);
        return false;
    }
    invalidate_slot(owner);
    invalidate_slot(joining);
    // Clear both parser epochs and local motor feedback before publishing
    // either the new pair or its neutral retired output.
    stop_joycon_output(owner_device);
    stop_joycon_output(joining_device);
    if (side < 0) {
        owner.companion = owner.device;
        owner.companion_gamepad = owner.gamepad;
        owner.companion_extra_buttons = owner.extra_buttons;
        owner.device = joining_device;
        owner.gamepad = joining.gamepad;
        owner.extra_buttons = joining.extra_buttons;
    } else {
        owner.companion = joining_device;
        owner.companion_gamepad = joining.gamepad;
        owner.companion_extra_buttons = joining.extra_buttons;
    }
    owner.identity = pair_identity;
    refresh_topology_input(owner);
    joining.identity = controller_identity_global();
    joining.device = nullptr;
    joining.gamepad = {};
    joining.extra_buttons = 0;
    joining.active = false;
    forget_joycon_pair_hint(owner.device);
    forget_joycon_pair_hint(owner.companion);
    if (gesture) {
        set_joycon_override(owner.device, owner.companion,
                            JoyConGroupingOverride::Paired);
        queue_local_feedback(owner, kJoyConGestureFeedbackMs,
                             kProfileFeedbackWeakMagnitude,
                             kProfileFeedbackStrongMagnitude);
    }
    g_joycon_pair_hints[physical_index_for_device(owner.device)] =
        {owner.companion, static_cast<uint8_t>(owner_index)};
    g_joycon_pair_hints[physical_index_for_device(owner.companion)] =
        {owner.device, static_cast<uint8_t>(owner_index)};
    critical_section_exit(&g_state_lock);
    apply_slot_lighting(static_cast<uint8_t>(owner_index), owner.device);
    apply_slot_lighting(static_cast<uint8_t>(owner_index), owner.companion);
    return true;
}

bool split_joycon_slot(uint8_t owner_index, bool gesture = false) {
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& owner = g_slots[owner_index];
    const int physical_index = physical_index_for_device(owner.device);
    if (!owner.active || owner.companion == nullptr || physical_index < 0 ||
        (!gesture && g_joycon_overrides[physical_index].mode ==
                         JoyConGroupingOverride::Paired) ||
        (gesture && !joycon_gesture_mature(
            owner.device, owner.companion, false,
            btstack_run_loop_get_time_ms()))) {
        critical_section_exit(&g_state_lock);
        return false;
    }
    // Keep the pair's left member at its existing player index. Prefer the
    // right member's physical index, falling back to the lowest free output.
    int right_index = physical_index_for_device(owner.companion);
    if (right_index < 0 || g_slots[right_index].device != nullptr) {
        right_index = -1;
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            if (g_slots[index].device == nullptr) {
                right_index = index;
                break;
            }
        }
    }
    if (right_index < 0) {
        critical_section_exit(&g_state_lock);
        return false;
    }
    BackendSlot& right = g_slots[right_index];
    ControllerIdentity left_identity{};
    ControllerIdentity right_identity{};
    // The enrolled owner is authoritative even if a later identity-resolution
    // event has temporarily cleared a member's transport mapping.
    if (!controller_identity_joycon_pair_members(
            owner.identity, &left_identity, &right_identity)) {
        critical_section_exit(&g_state_lock);
        return false;
    }
    invalidate_slot(owner);
    invalidate_slot(right);
    stop_joycon_output(owner.device);
    stop_joycon_output(owner.companion);
    right.device = owner.companion;
    right.identity = right_identity;
    right.gamepad = owner.companion_gamepad;
    right.extra_buttons = owner.companion_extra_buttons;
    right.active = true;
    owner.identity = left_identity;
    owner.companion = nullptr;
    owner.companion_gamepad = {};
    owner.companion_extra_buttons = 0;
    refresh_topology_input(owner);
    refresh_topology_input(right);
    if (gesture) {
        set_joycon_override(owner.device, right.device,
                            JoyConGroupingOverride::Individual);
        queue_local_feedback(owner, kJoyConGestureFeedbackMs,
                             kProfileFeedbackWeakMagnitude,
                             kProfileFeedbackStrongMagnitude);
        queue_local_feedback(right, kJoyConGestureFeedbackMs,
                             kProfileFeedbackWeakMagnitude,
                             kProfileFeedbackStrongMagnitude);
    }
    critical_section_exit(&g_state_lock);
    apply_slot_lighting(owner_index, owner.device);
    apply_slot_lighting(static_cast<uint8_t>(right_index), right.device);
    return true;
}

void process_joycon_gestures(uint32_t now_ms) {
    for (uint8_t index = 0; index < kSlotCount; ++index) {
        now_ms = btstack_run_loop_get_time_ms();
        critical_section_enter_blocking(&g_state_lock);
        refresh_joycon_gestures(now_ms);
        const JoyConGesture& gesture = g_joycon_gestures[index];
        if (gesture.blocked || joycon_side(gesture.device) >= 0) {
            critical_section_exit(&g_state_lock);
            continue;
        }
        uni_hid_device_t* right = nullptr;
        for (uint8_t mate = 0; mate < kSlotCount; ++mate) {
            if (mate != index && (gesture.participants & (1u << mate))) {
                right = g_joycon_gestures[mate].device;
            }
        }
        const bool joining = gesture.joining;
        const int owner_slot = slot_for_device(gesture.device);
        const int right_slot = slot_for_device(right);
        const bool mature = joycon_gesture_mature(
            gesture.device, right, joining, now_ms);
        if (mature) block_joycon_gesture(gesture.participants);
        critical_section_exit(&g_state_lock);
        if (!mature) continue;
        // Latch success AND failure before any enrollment I/O. A failed seed
        // must not retry at the timer cadence or undo either participant.
        // Player-slot ownership is independent of physical handedness.
        // Keep the lower occupied slot, even when the left half owns the higher one.
        const bool keep_left_slot = owner_slot < right_slot;
        const bool changed = joining
            ? merge_joycon_slots(
                  keep_left_slot ? owner_slot : right_slot,
                  keep_left_slot ? right_slot : owner_slot,
                  keep_left_slot ? right : gesture.device, true)
            : split_joycon_slot(static_cast<uint8_t>(owner_slot), true);
        if (changed) recompute_connection_status();
    }
}

void apply_joycon_configuration(const ConfigurationServiceSnapshot& configuration) {
    if (configuration.state != ConfigurationServiceState::kReady) return;
    const JoyConMode requested = configuration.configuration.joycon_mode;
    if (requested != g_joycon_mode) {
        critical_section_enter_blocking(&g_state_lock);
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            g_joycon_overrides[index] = {};
            block_joycon_gesture(g_joycon_gestures[index].participants);
        }
        g_joycon_mode = requested;
        g_joycon_reconcile_requested = true;
        critical_section_exit(&g_state_lock);
    }
    if (!g_joycon_reconcile_requested) return;
    g_joycon_reconcile_requested = false;
    if (g_joycon_mode == JoyConMode::kIndividual) {
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            split_joycon_slot(index);
        }
    } else {
        uint8_t attempted = 0;
        for (uint8_t index = 0; index < kSlotCount; ++index) {
            critical_section_enter_blocking(&g_state_lock);
            const BackendSlot& slot = g_slots[index];
            int partner = slot.active && slot.companion == nullptr &&
                                  !(attempted & (1u << index))
                              ? joycon_partner_slot(slot.device, index) : -1;
            if (partner < 0 || (attempted & (1u << partner))) {
                critical_section_exit(&g_state_lock);
                continue;
            }
            int owner = index;
            int joining = partner;
            const auto& hint =
                g_joycon_pair_hints[physical_index_for_device(slot.device)];
            if (hint.mate == g_slots[partner].device && hint.owner_slot == partner) {
                owner = partner;
                joining = index;
            }
            uni_hid_device_t* joining_device = g_slots[joining].device;
            attempted |= (1u << index) | (1u << partner);
            critical_section_exit(&g_state_lock);
            // Failed seeds/invalid identities leave both live solos intact.
            // Retry only on a mode change or fresh identity/ready event, never
            // at the 50 ms poll cadence or for unrelated configuration edits.
            merge_joycon_slots(owner, joining, joining_device);
        }
    }
    recompute_connection_status();
}

void platform_init(int argc, const char** argv) {
    (void)argc;
    (void)argv;
}

void platform_on_init_complete() {
    if (SWITCH_PICO_ENABLE_CLASSIC) {
        gap_set_link_supervision_timeout(kClassicLinkSupervisionTimeout);
        gap_ssp_set_auto_accept(false);
        g_pairing_event_callback.callback = handle_btstack_event;
        hci_add_event_handler(&g_pairing_event_callback);
    }
    gap_set_bondable_mode(false);
    if (SWITCH_PICO_ENABLE_BLE) {
        // Bluepad32 does not initialize SM in Classic-only mode.
        sm_set_accepted_stk_generation_methods(0);
        g_identity_event_callback.callback = handle_btstack_event;
        sm_add_event_handler(&g_identity_event_callback);
    }
    switch2_wake_initialize();
    refresh_pairing_snapshot();
    btstack_run_loop_set_timer_handler(&g_rumble_timer, process_rumble_timer);
    btstack_run_loop_set_timer(&g_rumble_timer, kRumblePollIntervalMs);
    btstack_run_loop_add_timer(&g_rumble_timer);
    btstack_run_loop_set_timer_handler(
        &g_configuration_timer, process_configuration_timer);
    btstack_run_loop_set_timer(
        &g_configuration_timer, kConfigurationPollIntervalMs);
    btstack_run_loop_add_timer(&g_configuration_timer);
    __atomic_store_n(&g_initialization_stage, 6, __ATOMIC_RELEASE);
    ConfigurationServiceSnapshot configuration{};
    configuration_service_snapshot(&configuration);
    apply_joycon_configuration(configuration);
    if (switch2_wake_ready_for_connections()) {
        recompute_connection_status();
    }
}

bool device_transport_enabled(const uni_hid_device_t* device) {
    if (device == nullptr) {
        return false;
    }
    if (SWITCH_PICO_ENABLE_BLE && SWITCH_PICO_ENABLE_CLASSIC) {
        return true;
    }
    // GAP describes the actual live link. The protocol hint is also available
    // before an outgoing connection has an HCI handle.
    switch (gap_get_connection_type(device->conn.handle)) {
        case GAP_CONNECTION_ACL:
            return SWITCH_PICO_ENABLE_CLASSIC != 0;
        case GAP_CONNECTION_LE:
            return SWITCH_PICO_ENABLE_BLE != 0;
        default:
            break;
    }
    switch (device->conn.protocol) {
        case UNI_BT_CONN_PROTOCOL_BR_EDR:
            return SWITCH_PICO_ENABLE_CLASSIC != 0;
        case UNI_BT_CONN_PROTOCOL_BLE:
            return SWITCH_PICO_ENABLE_BLE != 0;
        default:
            // Preserve mixed-mode admission; single-transport builds cannot
            // safely admit a connection whose transport is still unknown.
            return SWITCH_PICO_ENABLE_BLE && SWITCH_PICO_ENABLE_CLASSIC;
    }
}

uni_error_t platform_on_device_discovered(bd_addr_t addr, const char* name,
                                          uint16_t cod, uint8_t rssi) {
    (void)name;
    (void)cod;
    (void)rssi;
    if (!has_free_slot()) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    const uni_hid_device_t* candidate =
        (g_connection_policy_state == ConnectionPolicyState::Passive ||
         !SWITCH_PICO_ENABLE_BLE || !SWITCH_PICO_ENABLE_CLASSIC)
            ? uni_hid_device_get_instance_for_address(addr)
            : nullptr;
    if (candidate != nullptr && !device_transport_enabled(candidate)) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    // First discovery can precede device creation, so the transport-specific
    // Bluepad32 discovery handlers must enforce the mode before this callback.
    if (g_connection_policy_state == ConnectionPolicyState::Open) {
        return UNI_ERROR_SUCCESS;
    }
    if (!SWITCH_PICO_ENABLE_BLE ||
        g_connection_policy_state != ConnectionPolicyState::Passive) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    const int side = joycon_side(candidate);
    uint8_t address_type = BD_ADDR_TYPE_UNKNOWN;
    return side != 0 && waiting_for_joycon_mate(side) &&
                   uni_hid_parser_switch2_identity_address_type(
                       candidate, &address_type) &&
                   uni_switch2_pairing_known(address_type, addr)
               ? UNI_ERROR_SUCCESS
               : UNI_ERROR_IGNORE_DEVICE;
}

void platform_on_device_connected(uni_hid_device_t* device) {
    if (device == nullptr) {
        return;
    }
    if (!device_transport_enabled(device) ||
        (g_connection_policy_state != ConnectionPolicyState::Open &&
         g_connection_policy_state != ConnectionPolicyState::Passive)) {
        uni_hid_device_disconnect(device);
        return;
    }
    const ControllerIdentity connection_identity = identity_for_device(device);
    critical_section_enter_blocking(&g_state_lock);
    const int physical_index = physical_index_for_device(device);
    if (physical_index >= 0) {
        g_retired_devices[physical_index] = nullptr;
        g_switch2_interval_requests[physical_index] = {};
        if (slot_for_device(device) < 0) reset_joycon_connection(device);
    }
    const int slot_index = reserve_device_slot(device);
    if (slot_index >= 0) {
        BackendSlot& slot = g_slots[slot_index];
        if (slot.device == nullptr) {
            slot.device = device;
            slot.identity = connection_identity;
            slot.rumble_pending = false;
            reset_slot_hotkeys(slot);
        }
    }
    critical_section_exit(&g_state_lock);
    if (slot_index >= 0) {
        recompute_connection_status();
    } else {
        uni_hid_device_disconnect(device);
    }
}

void platform_on_device_disconnected(uni_hid_device_t* device) {
    const int slot_index = slot_for_device(device);
    if (slot_index < 0) {
        return;
    }
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    switch_native_output_detach(device);
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    haptics_experiment_detach(device);
#endif
    uni_hid_device_t* survivor = nullptr;
    ControllerIdentity survivor_identity{};
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    g_retired_devices[physical_index_for_device(device)] = device;
    reset_joycon_connection(device);
    if (slot.companion != nullptr) {
        invalidate_slot(slot);
        if (slot.device == device) {
            slot.device = slot.companion;
            slot.gamepad = slot.companion_gamepad;
            slot.extra_buttons = slot.companion_extra_buttons;
        }
        slot.companion = nullptr;
        slot.companion_gamepad = {};
        slot.companion_extra_buttons = 0;
        survivor = slot.device;
        slot.identity = identity_for_device(survivor);
        survivor_identity = slot.identity;
        refresh_topology_input(slot);
    } else {
        release_slot(slot);
    }
    critical_section_exit(&g_state_lock);
    clear_ble_identity_for_device(device);
    if (survivor != nullptr) {
        if (survivor->report_parser.play_dual_rumble != nullptr) {
            dispatch_rumble(survivor, 0, 0, 0);
        }
        apply_slot_lighting(static_cast<uint8_t>(slot_index), survivor);
        if (survivor_identity.stable) {
            profile_service_observe_identity_on_storage_core(survivor_identity);
        }
    }
    // Losing a half frees transport capacity, not another logical player.
    if (g_connection_policy_state != ConnectionPolicyState::FailedClosed) {
        g_connection_policy_state = ConnectionPolicyState::Uninitialized;
    }
    recompute_connection_status();
}

uni_error_t platform_on_device_ready(uni_hid_device_t* device) {
    if (!device_transport_enabled(device) ||
        !uni_hid_device_is_gamepad(device)) {
        return UNI_ERROR_INVALID_CONTROLLER;
    }
    if (g_connection_policy_state == ConnectionPolicyState::FailedClosed) {
        return UNI_ERROR_NO_SLOTS;
    }

    bool became_active = false;
    bool paired = false;
    uint32_t lighting_generation = 0;
    uni_hid_device_t* owner = device;
    uni_hid_device_t* companion = nullptr;
    ControllerIdentity connection_identity = identity_for_device(device);
    critical_section_enter_blocking(&g_state_lock);
    int slot_index = reserve_device_slot(device);
    if (slot_index < 0) {
        critical_section_exit(&g_state_lock);
        return UNI_ERROR_NO_SLOTS;
    }
    BackendSlot& pending = g_slots[slot_index];
    if (!pending.active) {
        const int partner_index = joycon_partner_slot(device, slot_index);
        if (partner_index >= 0) {
            critical_section_exit(&g_state_lock);
            if (!merge_joycon_slots(partner_index, slot_index, device)) {
                return UNI_ERROR_INIT_FAILED;
            }
            critical_section_enter_blocking(&g_state_lock);
            slot_index = partner_index;
            paired = true;
        } else {
            pending.identity = connection_identity;
            pending.device = device;
            pending.state = make_neutral_state();
            pending.active = true;
            pending.rumble_pending = false;
            reset_slot_hotkeys(pending);
            ++pending.state_generation;
        }
        became_active = true;
    }
    const BackendSlot& current = g_slots[slot_index];
    owner = current.device;
    companion = current.companion;
    lighting_generation = current.connection_generation;
    connection_identity = current.identity;
    critical_section_exit(&g_state_lock);
    if (became_active) {
        apply_radio_connection_policy();
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
        if (!paired) {
            switch_native_output_attach(static_cast<uint8_t>(slot_index),
                                        lighting_generation, device, connection_identity);
        }
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
        if (!paired && !uni_hid_parser_switch2_is_ble_device(device)) {
            haptics_experiment_attach(
                static_cast<uint8_t>(slot_index), lighting_generation, device);
        }
#ifdef SWITCH_PICO_HD_RUMBLE
        if (connection_identity.vendor_id == 0x054c &&
            (connection_identity.product_id == 0x0ce6 ||
             connection_identity.product_id == 0x0df2)) {
            haptics_experiment_request(2, static_cast<uint8_t>(slot_index));
        }
#endif
#endif
        if (!paired && lighting_target_is_current(
                static_cast<uint8_t>(slot_index), lighting_generation, owner)) {
            apply_slot_lighting(static_cast<uint8_t>(slot_index), owner);
            if (companion != nullptr) {
                apply_slot_lighting(static_cast<uint8_t>(slot_index), companion);
            }
        }
        if (!paired && connection_identity.stable) {
            profile_service_observe_identity_on_storage_core(
                connection_identity);
        }
        if (!paired && joycon_side(device) != 0 &&
            g_joycon_mode == JoyConMode::kPaired) {
            g_joycon_reconcile_requested = true;
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
    __atomic_add_fetch(&g_controller_reports, 1, __ATOMIC_RELAXED);

    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (!slot.active) {
        critical_section_exit(&g_state_lock);
        return;
    }
#ifdef SWITCH_PICO_WII_IR
    uni_wii_ir_snapshot_t infrared{};
    const bool have_infrared = uni_hid_parser_wii_ir_snapshot(device, &infrared);
#ifdef SWITCH_PICO_WII_IR_GYRO
    observe_wii_aim_chord(
        slot, device, controller->gamepad,
        have_infrared ? &infrared : nullptr, time_us_32());
#endif
    if (have_infrared) {
        const bool nunchuk_c =
            (device->controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK ||
             device->controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK_ACCEL) &&
            (controller->gamepad.buttons & BUTTON_X) != 0;
        wii_ir_pointer_observe(static_cast<uint8_t>(slot_index),
                             slot.connection_generation, infrared.sequence,
                             infrared.buttons, infrared.x, infrared.y,
                             infrared.valid_mask, nunchuk_c);
    }
#endif
    const uint8_t extras = uni_hid_parser_switch2_extra_buttons(device);
    if (slot.companion == device) {
        slot.companion_gamepad = controller->gamepad;
        slot.companion_extra_buttons = extras;
    } else {
        slot.gamepad = controller->gamepad;
        slot.extra_buttons = extras;
    }
    observe_joycon_gesture(
        device, controller->gamepad, btstack_run_loop_get_time_ms());
    if (device->controller_type == CONTROLLER_TYPE_WiiController) {
        int32_t acceleration[3];
        uint32_t sequence;
        if (!uni_hid_parser_wii_accel_snapshot(device, acceleration, &sequence)) {
            slot.accelerometer = {};
        } else if (!slot.accelerometer.valid || sequence != slot.accelerometer.sequence) {
            slot.accelerometer = {
                convert_accel(-static_cast<int64_t>(acceleration[2])),
                convert_accel(-static_cast<int64_t>(acceleration[0])),
                convert_accel(acceleration[1]), sequence,
                btstack_run_loop_get_time_ms(), true};
        }
    }
    const uni_gamepad_t gamepad = logical_gamepad(slot);
    uni_hid_device_t* owner = slot.device;
    const bool fresh_motion =
        slot.companion == nullptr || slot.companion == device;
    const uint8_t merged_extras =
        slot.extra_buttons | slot.companion_extra_buttons;
    critical_section_exit(&g_state_lock);
    const uint16_t pre_hotkey_button_mask = logical_button_mask(gamepad);
    if (wake_chord_rising_edge(
            static_cast<uint8_t>(slot_index), owner, pre_hotkey_button_mask)) {
        switch2_wake_request();
    }
    const HotkeyDecision hotkeys = update_controller_hotkeys(
        static_cast<uint8_t>(slot_index), owner);
    ControllerState state = map_gamepad(
        gamepad, hotkeys.motion_enabled && fresh_motion, pre_hotkey_button_mask);
    state.extra_buttons = merged_extras;
    publish_device_state(
        static_cast<uint8_t>(slot_index), owner, pre_hotkey_button_mask, state);
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
    __atomic_store_n(&g_initialization_stage, 2, __ATOMIC_RELEASE);
    configuration_service_initialize_on_storage_core();
    profile_service_initialize_on_storage_core();
    ConfigurationServiceSnapshot configuration{};
    configuration_service_snapshot(&configuration);
    if (configuration.state == ConfigurationServiceState::kReady) {
        // Load before uni_init can deliver even the first ready callback.
        g_joycon_mode = configuration.configuration.joycon_mode;
    }
    __atomic_store_n(&g_initialization_stage, 3, __ATOMIC_RELEASE);
    if (cyw43_arch_init() != 0) {
        halt_wireless_backend();
    }
    __atomic_store_n(&g_initialization_stage, 4, __ATOMIC_RELEASE);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    g_status_led_on = true;

    uni_platform_set_custom(get_platform());
    if (uni_init(0, nullptr) != 0) {
        halt_wireless_backend();
    }
    __atomic_store_n(&g_initialization_stage, 5, __ATOMIC_RELEASE);

    btstack_run_loop_execute();
    while (true) {
        tight_loop_contents();
    }
}

}  // namespace

extern "C" bool switch_pico_switch2_pairing_allowed(void) {
    return SWITCH_PICO_ENABLE_BLE && g_initialized &&
           pairing_window_active_at(btstack_run_loop_get_time_ms());
}

extern "C" void __real_sm_request_pairing(hci_con_handle_t handle);
extern "C" void __wrap_sm_request_pairing(hci_con_handle_t handle) {
    uni_hid_device_t* device =
        uni_hid_device_get_instance_for_connection_handle(handle);
    if (!SWITCH_PICO_ENABLE_BLE) {
        if (device != nullptr) {
            uni_hid_device_disconnect(device);
        }
        return;
    }
    if (uni_hid_parser_switch2_is_ble_device(device)) {
        // GATT's implicit authentication retry must not enter standard SMP for
        // this proprietary protocol. Retain storage until HCI teardown.
        uni_hid_device_disconnect(device);
        return;
    }
    __real_sm_request_pairing(handle);
}

#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
extern "C" bool uni_platform_on_l2cap_can_send_now(
        uni_hid_device_t* device, uint16_t cid) {
    bool block_generic = false;
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    block_generic = haptics_experiment_blocks_generic(device);
#endif
    const bool consumed = native_output_scheduler_on_can_send_now(device, cid);
    return consumed || block_generic;
}
#endif

bool bluepad32_input_backend_capture_start(
    uint8_t slot, uint32_t connection_generation, const CaptureOptions& options) {
    if (!g_initialized || slot >= kSlotCount) return false;
    critical_section_enter_blocking(&g_state_lock);
    const BackendSlot& current = g_slots[slot];
    const bool accepted = current.active &&
        current.connection_generation == connection_generation &&
        g_macro_capture.start(slot, connection_generation, options,
                              time_us_32(), current.state);
    critical_section_exit(&g_state_lock);
    return accepted;
}

bool bluepad32_input_backend_capture_stop(uint32_t run_id) {
    if (!g_initialized || run_id == 0) return false;
    critical_section_enter_blocking(&g_state_lock);
    const bool matches = run_id == g_macro_capture.run_id();
    if (matches) g_macro_capture.stop(time_us_32());
    critical_section_exit(&g_state_lock);
    return matches;
}

bool bluepad32_input_backend_capture_page(
    uint32_t run_id, uint16_t first_index, Bluepad32CaptureSnapshot* output) {
    if (!g_initialized || output == nullptr) return false;
    critical_section_enter_blocking(&g_state_lock);
    g_macro_capture.tick(time_us_32());
    if ((run_id != 0 && run_id != g_macro_capture.run_id()) ||
        first_index > g_macro_capture.event_count()) {
        critical_section_exit(&g_state_lock);
        return false;
    }
    *output = {};
    output->run_id = g_macro_capture.run_id();
    output->connection_generation = g_macro_capture.generation();
    output->elapsed_us = g_macro_capture.elapsed_us(time_us_32());
    output->slot = g_macro_capture.slot();
    output->state = g_macro_capture.state();
    output->options = g_macro_capture.options();
    output->total_events = g_macro_capture.event_count();
    output->first_index = first_index;
    const uint16_t remaining = output->total_events - first_index;
    output->event_count = remaining < BLUEPAD32_CAPTURE_PAGE_EVENTS
                              ? remaining : BLUEPAD32_CAPTURE_PAGE_EVENTS;
    for (uint8_t index = 0; index < output->event_count; ++index) {
        g_macro_capture.event(first_index + index, &output->events[index]);
    }
    critical_section_exit(&g_state_lock);
    return true;
}

void bluepad32_input_backend_init() {
    if (g_initialized) {
        return;
    }

    critical_section_init(&g_state_lock);
#ifdef SWITCH_PICO_WII_IR
    wii_ir_pointer_init();
#endif
    configuration_service_prepare();
    profile_service_prepare();
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
    native_output_scheduler_prepare();
#endif
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    switch_native_output_prepare();
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    haptics_experiment_prepare();
#endif
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
        g_joycon_pair_hints[slot_index] = {};
        g_joycon_gestures[slot_index] = {};
        g_joycon_overrides[slot_index] = {};
    }
    g_joycon_mode = JoyConMode::kPaired;
    g_joycon_reconcile_requested = false;
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
    g_background_scan_active = false;
    g_pairing_window_deadline_ms = 0;
    g_pairing_window_duration_ms =
        kDefaultPairingWindowDurationMs;
    g_pairing_reset_feedback_deadline_ms = 0;
    g_pairing_window_open = false;
    g_initialized = true;
    __atomic_store_n(&g_initialization_stage, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&g_rumble_timer_ticks, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_configuration_timer_ticks, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_controller_reports, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_host_rumble_requests, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_local_feedback_requests, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_rumble_dispatches, 0, __ATOMIC_RELAXED);
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
    multicore_launch_core1_with_stack(
        core1_main, g_core1_stack, sizeof(g_core1_stack));
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
    if (g_pairing_snapshot.status != Bluepad32PairingSnapshotStatus::kFailed) {
        g_pairing_snapshot.status = Bluepad32PairingSnapshotStatus::kPending;
    }
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


void bluepad32_input_backend_diagnostics(
    Bluepad32BackendDiagnostics* out) {
    if (out == nullptr) {
        return;
    }
    *out = {};
    out->initialization_stage =
        __atomic_load_n(&g_initialization_stage, __ATOMIC_ACQUIRE);
    out->rumble_timer_ticks =
        __atomic_load_n(&g_rumble_timer_ticks, __ATOMIC_RELAXED);
    out->configuration_timer_ticks =
        __atomic_load_n(&g_configuration_timer_ticks, __ATOMIC_RELAXED);
    out->controller_reports =
        __atomic_load_n(&g_controller_reports, __ATOMIC_RELAXED);
    out->host_rumble_requests =
        __atomic_load_n(&g_host_rumble_requests, __ATOMIC_RELAXED);
    out->local_feedback_requests =
        __atomic_load_n(&g_local_feedback_requests, __ATOMIC_RELAXED);
    out->rumble_dispatches =
        __atomic_load_n(&g_rumble_dispatches, __ATOMIC_RELAXED);
    out->switch2_ingress_drops =
        __atomic_load_n(&g_switch2_ingress_drops, __ATOMIC_RELAXED);
    out->switch2_output_drops = uni_hid_parser_switch2_haptics_dropped();

    critical_section_enter_blocking(&g_state_lock);
    for (const BackendSlot& slot : g_slots) {
        if (slot.active) {
            ++out->active_slots;
        }
        if (slot.active && slot.device != nullptr &&
            slot.device->report_parser.play_dual_rumble != nullptr) {
            ++out->rumble_capable_slots;
        }
        if (slot.feedback_pending || slot.profile_feedback.active ||
            slot.pending_profile_feedback_count != 0) {
            ++out->feedback_pending_slots;
        }
        if (slot.rumble_pending || slot.switch2_ingress.count != 0) {
            ++out->rumble_pending_slots;
        }
    }
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
    out->accelerometer = slot.accelerometer;
    const uint32_t state_generation = slot.state_generation;
    critical_section_exit(&g_state_lock);

    if (state_generation == g_consumed_generation[slot_index]) {
        out->state.motion_sample_count = 0;
    }
    g_last_snapshot_generation[slot_index] = state_generation;
}

void bluepad32_input_backend_playtest_snapshot(
    uint8_t slot_index, Bluepad32PlaytestSnapshot* out) {
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
    out->state_generation = slot.state_generation;
    out->identity = slot.identity;
    out->physical_button_mask = slot.pre_hotkey_button_mask;
    out->state = slot.state;
    if (slot.device != nullptr) {
        out->battery = slot.device->controller.battery;
        out->capabilities =
            (slot.device->report_parser.play_dual_rumble != nullptr ? 1u : 0u) |
            (slot.device->report_parser.set_lightbar_color != nullptr ? 2u : 0u) |
            (slot.device->report_parser.set_player_leds != nullptr ? 4u : 0u) |
            (slot.state.motion_sample_count != 0 ? 8u : 0u);
        if (slot.active) {
            const int side = joycon_side(slot.device);
            if (side != 0) {
                out->controller_layout = slot.companion != nullptr
                    ? Bluepad32ControllerLayout::kJoyCon2MergedPair
                    : side < 0
                        ? Bluepad32ControllerLayout::kJoyCon2LeftSolo
                        : Bluepad32ControllerLayout::kJoyCon2RightSolo;
            } else {
                switch (slot.device->controller_subtype) {
                    case CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL:
                    case CONTROLLER_SUBTYPE_WIIMOTE_ACCEL:
                        out->controller_layout =
                            Bluepad32ControllerLayout::kWiiHorizontal;
                        break;
                    case CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL:
                        out->controller_layout =
                            Bluepad32ControllerLayout::kWiiVertical;
                        break;
                    case CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK:
                    case CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK_ACCEL:
                        out->controller_layout =
                            Bluepad32ControllerLayout::kWiiNunchuk;
                        break;
                    default:
                        break;
                }
            }
        }
    }
    critical_section_exit(&g_state_lock);
}

bool bluepad32_input_backend_toggle_motion(
    uint8_t slot_index, uint32_t connection_generation) {
    if (!g_initialized || !valid_slot(slot_index)) {
        return false;
    }
    bool toggled = false;
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active &&
        slot.connection_generation == connection_generation) {
        slot.motion_enabled = !slot.motion_enabled;
#ifdef SWITCH_PICO_WII_IR_GYRO
        if (slot.device != nullptr &&
            slot.device->controller_type == CONTROLLER_TYPE_WiiController) {
            const ControllerMotionSample sample =
                slot.state.motion_sample_count != 0
                    ? slot.state.motion_samples[0] : ControllerMotionSample{};
            wii_ir_gyro_update_motion(
                slot_index, connection_generation, slot.motion_enabled, sample);
            if (!slot.motion_enabled) {
                slot.state.motion_sample_count = 0;
            }
        }
#endif
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
        toggled = true;
    }
    critical_section_exit(&g_state_lock);
    return toggled;
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

#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
    const uint64_t received_us = time_us_64();
    uint32_t native_generation = 0;
    bool native_candidate = false;
#endif
    const uint16_t duration_ms = host_rumble_duration_ms();
    const uint32_t received_ms = btstack_run_loop_get_time_ms();
    critical_section_enter_blocking(&g_state_lock);
    BackendSlot& slot = g_slots[slot_index];
    if (slot.active && slot.device != nullptr) {
        if (uni_hid_parser_switch2_is_ble_device(slot.device)) {
            Switch2Ingress& ingress = slot.switch2_ingress;
            const uint8_t host_mode = switch2_host_mode();
            if (ingress.host_mode != host_mode) {
                clear_switch2_ingress(slot);
                ingress.host_mode = host_mode;
            }
            if (switch2_host_stop(rumble)) {
                // Host stop is a barrier, not a local-feedback cancellation.
                ingress.head = 0;
                ingress.count = 0;
            } else if (ingress.count == kSwitch2IngressCapacity) {
                ingress.head = (ingress.head + 1u) % kSwitch2IngressCapacity;
                --ingress.count;
                __atomic_add_fetch(&g_switch2_ingress_drops, 1, __ATOMIC_RELAXED);
            }
            Switch2HostCommand& command =
                ingress.commands[(ingress.head + ingress.count) % kSwitch2IngressCapacity];
            command = {};
            command.envelope.slot = slot_index;
            command.envelope.connection_generation = slot.connection_generation;
            command.envelope.rumble = rumble;
            command.envelope.duration_ms = duration_ms;
            command.envelope.received_ms = received_ms;
            command.generation = ingress.generation;
            ++ingress.count;
            __atomic_add_fetch(&g_host_rumble_requests, 1, __ATOMIC_RELAXED);
            critical_section_exit(&g_state_lock);
            return;
        }
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
        native_generation = slot.connection_generation;
        native_candidate = true;
#endif
        const RumbleEnvelope envelope{
            slot_index, slot.connection_generation, rumble,
            duration_ms, received_ms
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) || defined(SWITCH_PICO_NATIVE_SWITCH_RUMBLE)
            , received_us
#endif
        };
        slot.pending_rumble = envelope;
        slot.rumble_pending = true;
        __atomic_add_fetch(
            &g_host_rumble_requests, 1, __ATOMIC_RELAXED);
        if (duration_ms == kXInputHostRumbleDurationMs) {
            slot.retained_host_rumble = envelope;
            slot.retained_host_rumble_valid = true;
        } else {
            slot.retained_host_rumble = {};
            slot.retained_host_rumble_valid = false;
        }
    }
    critical_section_exit(&g_state_lock);
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    if (native_candidate)
        switch_native_output_submit(slot_index, native_generation, received_us, rumble,
                                    duration_ms == kXInputHostRumbleDurationMs);
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    if (native_candidate) {
        if (duration_ms == kXInputHostRumbleDurationMs) {
            haptics_experiment_submit_rumble(
                slot_index, native_generation, received_us,
                rumble.low_frequency_magnitude, rumble.high_frequency_magnitude);
        } else {
            haptics_experiment_submit(
                slot_index, native_generation, received_us, rumble.hd);
        }
    }
#endif
}

bool bluepad32_input_backend_set_wii_orientation(
    const ControllerIdentity& identity, uint32_t connection_generation,
    bool vertical) {
    if (!g_initialized || !identity.stable ||
        controller_identity_is_global(identity)) {
        return false;
    }
    bool queued = false;
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        if (!is_solo_wii_remote(slot) ||
            slot.connection_generation != connection_generation ||
            !controller_identity_equal(slot.identity, identity)) {
            continue;
        }
        slot.pending_wii_orientation = {identity, connection_generation, vertical};
        slot.wii_orientation_pending = true;
        queued = true;
        break;
    }
    critical_section_exit(&g_state_lock);
    return queued;
}

bool bluepad32_input_backend_identify(
    const ControllerIdentity& identity) {
    if (!g_initialized || !identity.stable ||
        controller_identity_is_global(identity)) {
        return false;
    }
    bool queued = false;
    critical_section_enter_blocking(&g_state_lock);
    for (BackendSlot& slot : g_slots) {
        if (!slot.active ||
            !controller_identity_equal(slot.identity, identity)) {
            continue;
        }
        const ProfileFeedbackEnvelope feedback{
            slot.connection_generation, 1,
            ControllerProfileConfirmationPolicy::kRumbleAndLed};
        queue_profile_feedback(slot, feedback);
        queued = true;
        break;
    }
    critical_section_exit(&g_state_lock);
    return queued;
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
        queue_profile_feedback(slot, feedback);
    }
    critical_section_exit(&g_state_lock);
}
