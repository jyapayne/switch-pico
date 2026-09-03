#include "adapter_mode_controller.h"
#include "adapter_reboot.h"

#include "adapter_configuration.h"
#include "adapter_host_probe.h"
#include "bluepad32_input_backend.h"
#include "configuration_service.h"
#include "controller_profile.h"
#include "controller_profile_runtime.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "usb_output_driver.h"

namespace {

constexpr uint32_t kInternalTransactionBit =
    CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK;
constexpr uint32_t kInternalTransactionValueMask =
    ~kInternalTransactionBit;
constexpr uint32_t kFeedbackPhaseMs = 75;
constexpr uint32_t kFeedbackGuardMs = 75;
static_assert(
    ADAPTER_MODE_CHORD_BUTTON_MASK ==
        static_cast<uint16_t>(
            (1u << static_cast<uint8_t>(
                 ControllerProfileLogicalButton::kLeftShoulder)) |
            (1u << static_cast<uint8_t>(
                 ControllerProfileLogicalButton::kRightShoulder)) |
            (1u << static_cast<uint8_t>(
                 ControllerProfileLogicalButton::kSelect)) |
            (1u << static_cast<uint8_t>(
                 ControllerProfileLogicalButton::kStart)) |
            (1u << static_cast<uint8_t>(
                 ControllerProfileLogicalButton::kSystem))),
    "raw mode chord must track the logical pre-hotkey mask");

struct ModeChordSlot {
    uint32_t connection_generation;
    uint32_t hold_started_ms;
    bool generation_valid;
    bool holding;
    bool triggered;
};

enum class ModeOperationKind : uint8_t {
    kNone,
    kChord,
    kRecovery,
};

enum class ModeOperationPhase : uint8_t {
    kSubmit,
    kWaitForCommit,
    kAcknowledge,
};

struct ModeOperation {
    ModeOperationKind kind;
    ModeOperationPhase phase;
    uint32_t transaction_id;
    AdapterRequestedMode target_mode;
    uint32_t feedback_deadline_ms;
    uint32_t connection_generation;
    uint8_t slot;
};

ModeChordSlot g_chord_slots[ADAPTER_MODE_CONTROLLER_SLOT_COUNT]{};
ModeOperation g_operation{};
uint32_t g_reboot_transaction_id = 0;
bool g_reboot_scheduled = false;
bool g_correlated_reboot_scheduled = false;
AdapterRequestedMode g_requested_mode = AdapterRequestedMode::kAuto;
uint32_t g_next_internal_transaction_value = 1;
bool g_recovery_requested = false;
bool g_recovery_failed = false;
uint32_t g_recovery_clear_pairings_token = 0;

bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}


bool successful_status(ConfigurationTransactionStatus status) {
    return status == ConfigurationTransactionStatus::kCommitted ||
           status == ConfigurationTransactionStatus::kUnchanged;
}

bool pending_status(ConfigurationTransactionStatus status) {
    return status == ConfigurationTransactionStatus::kReceiving ||
           status == ConfigurationTransactionStatus::kPending;
}


uint32_t next_internal_transaction_id() {
    const uint32_t transaction_id =
        kInternalTransactionBit | g_next_internal_transaction_value;
    if (g_next_internal_transaction_value ==
        kInternalTransactionValueMask) {
        g_next_internal_transaction_value = 1;
    } else {
        ++g_next_internal_transaction_value;
    }
    return transaction_id;
}

AdapterRequestedMode next_mode(AdapterRequestedMode mode) {
    switch (mode) {
        case AdapterRequestedMode::kAuto:
            return AdapterRequestedMode::kSwitch;
        case AdapterRequestedMode::kSwitch:
            return AdapterRequestedMode::kXInput;
        case AdapterRequestedMode::kXInput:
            return AdapterRequestedMode::kDInput;
        case AdapterRequestedMode::kDInput:
            return AdapterRequestedMode::kMac;
        case AdapterRequestedMode::kMac:
            return AdapterRequestedMode::kAuto;
    }
    return AdapterRequestedMode::kAuto;
}

// Profile numbers select both a bounded pulse count and the existing color.
uint8_t feedback_profile_number(AdapterRequestedMode mode) {
    switch (mode) {
        case AdapterRequestedMode::kAuto:
            return 1;
        case AdapterRequestedMode::kSwitch:
            return 2;
        case AdapterRequestedMode::kXInput:
            return 3;
        case AdapterRequestedMode::kDInput:
        case AdapterRequestedMode::kMac:
            return 4;
    }
    return 1;
}

// Mac reuses the fourth bounded pulse/color identity but is LED-only, making
// its confirmation tuple distinct from DInput without exceeding profile bounds.
ControllerProfileConfirmationPolicy feedback_policy(
    AdapterRequestedMode mode) {
    return mode == AdapterRequestedMode::kMac
               ? ControllerProfileConfirmationPolicy::kLed
               : ControllerProfileConfirmationPolicy::kRumbleAndLed;
}

void clear_mode_chord(ControllerState* state) {
    if (state == nullptr) {
        return;
    }
    state->button_left_shoulder = false;
    state->button_right_shoulder = false;
    state->button_select = false;
    state->button_start = false;
    state->button_system = false;
}

void begin_operation(ModeOperationKind kind, AdapterRequestedMode target_mode,
                     uint8_t slot = 0,
                     uint32_t connection_generation = 0) {
    g_operation = {};
    g_operation.kind = kind;
    g_operation.phase = ModeOperationPhase::kSubmit;
    g_operation.transaction_id = next_internal_transaction_id();
    g_operation.target_mode = target_mode;
    g_operation.slot = slot;
    g_operation.connection_generation = connection_generation;
}

void begin_recovery_operation_if_ready() {
    if (!g_recovery_requested || g_recovery_failed ||
        g_operation.kind != ModeOperationKind::kNone) {
        return;
    }

    Bluepad32PairingSnapshot pairing{};
    bluepad32_input_backend_pairing_snapshot(&pairing);
    if (!bluepad32_input_backend_clear_pairings_completed(
            pairing, g_recovery_clear_pairings_token)) {
        return;
    }

    begin_operation(ModeOperationKind::kRecovery,
                    AdapterRequestedMode::kAuto);
}

void reboot_now();

void finish_failed_operation() {
    const bool recovery =
        g_operation.kind == ModeOperationKind::kRecovery;
    g_operation = {};
    if (recovery) {
        g_recovery_failed = true;
    } else {
        begin_recovery_operation_if_ready();
    }
}

void finish_successful_mode_write(uint32_t now_ms) {
    g_requested_mode = g_operation.target_mode;

    if (g_recovery_requested ||
        g_operation.kind == ModeOperationKind::kRecovery) {
        if (g_operation.kind != ModeOperationKind::kRecovery) {
            g_operation = {};
            begin_recovery_operation_if_ready();
            return;
        }
        if (configuration_service_mode_transaction_reboot_ready(
                g_operation.transaction_id)) {
            reboot_now();
            return;
        }

        // Reboot only while this remains the latest accepted mode mutation.
        // Retry Auto if its correlation was displaced before Core 0 observed
        // the terminal result.
        begin_operation(ModeOperationKind::kRecovery,
                        AdapterRequestedMode::kAuto);
        return;
    }

    controller_profile_runtime_reset();
    const uint8_t feedback_profile =
        feedback_profile_number(g_operation.target_mode);
    bluepad32_input_backend_queue_profile_feedback(
        g_operation.slot, g_operation.connection_generation,
        feedback_profile, feedback_policy(g_operation.target_mode));
    g_operation.feedback_deadline_ms =
        now_ms + static_cast<uint32_t>(feedback_profile) * 2u *
                     kFeedbackPhaseMs +
        kFeedbackGuardMs;
    g_operation.phase = ModeOperationPhase::kAcknowledge;
}

void reboot_now() {
    if (g_reboot_scheduled) {
        return;
    }
    g_reboot_scheduled = true;
    controller_profile_runtime_reset();
    watchdog_reboot(0, 0, 0);
}

void advance_mode_write(uint32_t now_ms) {
    if (g_operation.phase == ModeOperationPhase::kSubmit) {
        const ConfigurationTransactionStatus status =
            configuration_service_set_mode_internal(
                g_operation.transaction_id, g_operation.target_mode,
                adapter_usb_mode_availability());
        if (status == ConfigurationTransactionStatus::kBusy) {
            return;
        }
        if (successful_status(status)) {
            finish_successful_mode_write(now_ms);
            return;
        }
        if (pending_status(status)) {
            g_operation.phase = ModeOperationPhase::kWaitForCommit;
            return;
        }
        finish_failed_operation();
        return;
    }

    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kIdle;
    if (!configuration_service_mode_transaction_status(
            g_operation.transaction_id, &status)) {
        finish_failed_operation();
        return;
    }
    if (successful_status(status)) {
        finish_successful_mode_write(now_ms);
    } else if (!pending_status(status)) {
        finish_failed_operation();
    }
}

}  // namespace

const AdapterModeAvailability& adapter_usb_mode_availability() {
    static constexpr AdapterModeAvailability kAvailability{
        true, true, true, true};
    return kAvailability;
}

void adapter_mode_controller_initialize_usb() {
    for (ModeChordSlot& slot : g_chord_slots) {
        slot = {};
    }
    g_operation = {};
    g_reboot_scheduled = false;
    g_reboot_transaction_id = 0;
    g_next_internal_transaction_value = 1;
    g_recovery_requested = false;
    g_recovery_failed = false;
    g_recovery_clear_pairings_token = 0;
    g_correlated_reboot_scheduled = false;

    configuration_service_initialize_pre_usb();
    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    g_requested_mode = snapshot.configuration.requested_mode;
    adapter_host_probe_init(g_requested_mode);
    usb_output_driver_init(adapter_host_probe_mode());
    tusb_init();
}

AdapterRequestedMode adapter_mode_controller_requested_mode() {
    return g_requested_mode;
}

void adapter_mode_controller_process_input(
    uint8_t slot_index, bool active, uint32_t connection_generation,
    uint16_t* pre_hotkey_button_mask, uint32_t now_ms,
    ControllerState* state) {
    if (slot_index >= ADAPTER_MODE_CONTROLLER_SLOT_COUNT) {
        return;
    }

    ModeChordSlot& slot = g_chord_slots[slot_index];
    if (!active) {
        slot = {};
        return;
    }
    if (!slot.generation_valid ||
        slot.connection_generation != connection_generation) {
        slot = {};
        slot.connection_generation = connection_generation;
        slot.generation_valid = true;
    }

    const uint16_t raw_button_mask =
        pre_hotkey_button_mask == nullptr ? 0 : *pre_hotkey_button_mask;
    const bool chord_held =
        (raw_button_mask & ADAPTER_MODE_CHORD_BUTTON_MASK) ==
        ADAPTER_MODE_CHORD_BUTTON_MASK;
    if (!chord_held) {
        slot.holding = false;
        slot.triggered = false;
        return;
    }

    if (pre_hotkey_button_mask != nullptr) {
        *pre_hotkey_button_mask = static_cast<uint16_t>(
            *pre_hotkey_button_mask &
            ~ADAPTER_MODE_CHORD_BUTTON_MASK);
    }
    clear_mode_chord(state);
    if (!slot.holding) {
        slot.holding = true;
        slot.hold_started_ms = now_ms;
        return;
    }
    if (slot.triggered ||
        !deadline_reached(now_ms,
                          slot.hold_started_ms +
                              ADAPTER_MODE_CHORD_HOLD_MS)) {
        return;
    }

    // Latch once per continuous hold even if another slot or recovery already
    // owns the single serialized internal mutation.
    slot.triggered = true;
    if (g_operation.kind == ModeOperationKind::kNone &&
        !g_recovery_requested) {
        ConfigurationServiceSnapshot snapshot{};
        configuration_service_snapshot(&snapshot);
        g_requested_mode = snapshot.configuration.requested_mode;
        begin_operation(ModeOperationKind::kChord,
                        next_mode(g_requested_mode), slot_index,
                        connection_generation);
    }
}

void adapter_mode_controller_task(uint32_t now_ms) {
    if (g_reboot_scheduled) {
        return;
    }
    begin_recovery_operation_if_ready();
    if (g_operation.kind == ModeOperationKind::kNone) {
        return;
    }

    if (g_operation.phase == ModeOperationPhase::kSubmit ||
        g_operation.phase == ModeOperationPhase::kWaitForCommit) {
        advance_mode_write(now_ms);
        return;
    }

    if (g_operation.phase == ModeOperationPhase::kAcknowledge) {
        if (g_recovery_requested) {
            g_operation = {};
            begin_recovery_operation_if_ready();
            return;
        }
        if (deadline_reached(now_ms,
                             g_operation.feedback_deadline_ms)) {
            if (configuration_service_mode_transaction_reboot_ready(
                    g_operation.transaction_id)) {
                reboot_now();
            } else {
                finish_failed_operation();
            }
        }
    }
}

void adapter_mode_controller_begin_recovery() {
    if (g_recovery_requested) {
        return;
    }

    configuration_service_reserve_for_recovery();
    g_recovery_requested = true;
    g_recovery_clear_pairings_token =
        bluepad32_input_backend_clear_pairings();
    g_recovery_failed =
        g_recovery_clear_pairings_token == 0;
    g_operation = {};
}

bool adapter_reboot_for_mode_transaction(uint32_t transaction_id) {
    if (g_reboot_scheduled) {
        return g_correlated_reboot_scheduled &&
               g_reboot_transaction_id == transaction_id;
    }
    if ((transaction_id & kInternalTransactionBit) != 0) {
        return false;
    }
    if (g_recovery_requested) {
        return false;
    }
    if (!configuration_service_mode_transaction_reboot_ready(
            transaction_id)) {
        return false;
    }
    g_correlated_reboot_scheduled = true;
    g_reboot_transaction_id = transaction_id;
    reboot_now();
    return true;
}
