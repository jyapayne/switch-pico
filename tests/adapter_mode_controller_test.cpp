#include "adapter/adapter_mode_controller.h"
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "configuration/adapter_configuration.h"
#include "adapter/adapter_reboot.h"
#include "input/bluepad32_input_backend.h"
#include "configuration/configuration_service.h"
#include "profile/controller_profile.h"

namespace {

enum class Call : uint8_t {
    kPreload,
    kSnapshot,
    kProbeInit,
    kOutputInit,
    kTinyUsbInit,
    kSetMode,
    kQueryMode,
    kRebootReady,
    kRuntimeReset,
    kFeedback,
    kReserveRecovery,
    kClearPairings,
    kPairingSnapshot,
    kWatchdogReboot,
    kBootselReboot,
};

std::vector<Call> calls;
AdapterRequestedMode stored_mode = AdapterRequestedMode::kAuto;
AdapterRequestedMode probed_requested_mode = AdapterRequestedMode::kAuto;
AdapterUsbMode probed_active_mode = AdapterUsbMode::kSwitchProbe;
AdapterUsbMode initialized_output_mode = AdapterUsbMode::kSwitchProbe;
std::vector<ConfigurationTransactionStatus> set_results;
size_t next_set_result = 0;
std::vector<uint32_t> set_transaction_ids;
std::vector<AdapterRequestedMode> set_modes;
bool query_matches = true;
ConfigurationTransactionStatus query_status =
    ConfigurationTransactionStatus::kPending;
uint32_t last_query_transaction_id = 0;
uint32_t last_reboot_ready_transaction_id = 0;
bool reboot_ready = true;
std::vector<bool> reboot_ready_results;
size_t next_reboot_ready_result = 0;
Bluepad32PairingSnapshotStatus pairing_status =
    Bluepad32PairingSnapshotStatus::kPending;
uint32_t pairing_generation = 0;
uint32_t pairing_completed_clear_token = 0;
uint32_t clear_pairings_result_token = 1;
uint8_t feedback_slot = 0;
uint32_t feedback_generation = 0;
uint8_t feedback_pulses = 0;
ControllerProfileConfirmationPolicy feedback_policy =
    ControllerProfileConfirmationPolicy::kNone;
int reboot_count = 0;
int runtime_reset_count = 0;
int clear_pairings_count = 0;
int bootsel_reboot_count = 0;
uint32_t bootsel_gpio_mask = UINT32_MAX;
uint32_t bootsel_disable_mask = UINT32_MAX;
bool recovery_reserved = false;
bool abandoned_host_receive = false;
bool abandoned_host_receive_canceled = false;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void reset_harness(AdapterRequestedMode requested_mode =
                       AdapterRequestedMode::kAuto) {
    calls.clear();
    stored_mode = requested_mode;
    probed_requested_mode = AdapterRequestedMode::kAuto;
    probed_active_mode = AdapterUsbMode::kSwitchProbe;
    initialized_output_mode = AdapterUsbMode::kSwitchProbe;
    set_results.clear();
    next_set_result = 0;
    set_transaction_ids.clear();
    set_modes.clear();
    query_matches = true;
    query_status = ConfigurationTransactionStatus::kPending;
    last_query_transaction_id = 0;
    last_reboot_ready_transaction_id = 0;
    reboot_ready = true;
    reboot_ready_results.clear();
    next_reboot_ready_result = 0;
    pairing_status = Bluepad32PairingSnapshotStatus::kPending;
    pairing_generation = 0;
    pairing_completed_clear_token = 0;
    clear_pairings_result_token = 1;
    feedback_slot = 0;
    feedback_generation = 0;
    feedback_pulses = 0;
    feedback_policy = ControllerProfileConfirmationPolicy::kNone;
    reboot_count = 0;
    runtime_reset_count = 0;
    clear_pairings_count = 0;
    bootsel_reboot_count = 0;
    bootsel_gpio_mask = UINT32_MAX;
    bootsel_disable_mask = UINT32_MAX;
    recovery_reserved = false;
    abandoned_host_receive = false;
    abandoned_host_receive_canceled = false;
    adapter_mode_controller_initialize_usb();
    calls.clear();
}

ControllerState held_state() {
    ControllerState state{};
    state.button_left_shoulder = true;
    state.button_right_shoulder = true;
    state.button_select = true;
    state.button_start = true;
    state.button_system = true;
    return state;
}

uint16_t sample(uint8_t slot, uint32_t generation, uint16_t mask,
                uint32_t now_ms, ControllerState* state) {
    adapter_mode_controller_process_input(slot, true, generation, &mask,
                                          now_ms, state);
    return mask;
}

void begin_hold(uint8_t slot, uint32_t generation, uint32_t start_ms) {
    ControllerState state = held_state();
    sample(slot, generation, ADAPTER_MODE_CHORD_BUTTON_MASK, start_ms,
           &state);
}

void finish_hold(uint8_t slot, uint32_t generation, uint32_t start_ms) {
    ControllerState state = held_state();
    sample(slot, generation, ADAPTER_MODE_CHORD_BUTTON_MASK,
           start_ms + ADAPTER_MODE_CHORD_HOLD_MS, &state);
}

void test_pre_tusb_ordering_and_configured_selection() {
    calls.clear();
    stored_mode = AdapterRequestedMode::kXInput;
    probed_active_mode = AdapterUsbMode::kXInput;
    adapter_mode_controller_initialize_usb();
    require(calls == std::vector<Call>({
                         Call::kPreload, Call::kSnapshot,
                         Call::kProbeInit, Call::kOutputInit,
                         Call::kTinyUsbInit}),
            "persistent mode was not consumed before output init and tusb");
    require(probed_requested_mode == AdapterRequestedMode::kXInput &&
                initialized_output_mode == AdapterUsbMode::kXInput &&
                adapter_mode_controller_requested_mode() ==
                    AdapterRequestedMode::kXInput,
            "preloaded configured mode did not reach the frozen driver");
}

void test_mode_availability_has_one_stable_value() {
    const AdapterModeAvailability& first =
        adapter_usb_mode_availability();
    const AdapterModeAvailability& second =
        adapter_usb_mode_availability();
    require(&first == &second && first.switch_mode &&
                first.xinput_mode && first.dinput_mode &&
                first.mac_mode,
            "mode availability was not the shared five-mode value");
}

void test_exact_hold_release_wrap_and_consumption() {
    reset_harness();
    ControllerState state = held_state();
    state.button_south = true;
    const uint16_t consumed_mask = sample(
        0, 7, ADAPTER_MODE_CHORD_BUTTON_MASK, 100, &state);
    require(consumed_mask == 0 &&
                !state.button_left_shoulder &&
                !state.button_right_shoulder && !state.button_select &&
                !state.button_start && !state.button_system &&
                state.button_south,
            "held raw mode chord was not consumed before profile processing");

    state = held_state();
    sample(0, 7, ADAPTER_MODE_CHORD_BUTTON_MASK, 3099, &state);
    adapter_mode_controller_task(3099);
    require(set_transaction_ids.empty(),
            "mode chord fired before exactly three seconds");
    state = {};
    sample(0, 7, 0, 3100, &state);
    begin_hold(0, 7, 4000);
    finish_hold(0, 7, 4000);
    set_results = {ConfigurationTransactionStatus::kBusy};
    adapter_mode_controller_task(7000);
    require(set_transaction_ids.size() == 1,
            "release did not rearm a full three-second hold");

    reset_harness();
    constexpr uint32_t kWrapStart = UINT32_MAX - 999u;
    begin_hold(0, 9, kWrapStart);
    ControllerState wrap_state = held_state();
    sample(0, 9, ADAPTER_MODE_CHORD_BUTTON_MASK, 1999, &wrap_state);
    adapter_mode_controller_task(1999);
    require(set_transaction_ids.empty(),
            "wrap-safe hold fired one millisecond early");
    wrap_state = held_state();
    sample(0, 9, ADAPTER_MODE_CHORD_BUTTON_MASK, 2000, &wrap_state);
    set_results = {ConfigurationTransactionStatus::kBusy};
    adapter_mode_controller_task(2000);
    require(set_transaction_ids.size() == 1,
            "three-second hold failed across uint32 wrap");
}

AdapterRequestedMode triggered_target(AdapterRequestedMode initial) {
    reset_harness(initial);
    begin_hold(0, 1, 0);
    finish_hold(0, 1, 0);
    set_results = {ConfigurationTransactionStatus::kBusy};
    adapter_mode_controller_task(ADAPTER_MODE_CHORD_HOLD_MS);
    require(set_modes.size() == 1, "cycle did not submit a mode request");
    return set_modes[0];
}

void test_cycle_and_slot_isolation() {
    require(triggered_target(AdapterRequestedMode::kAuto) ==
                AdapterRequestedMode::kSwitch,
            "Auto did not cycle to Switch");
    require(triggered_target(AdapterRequestedMode::kSwitch) ==
                AdapterRequestedMode::kXInput,
            "Switch did not cycle to XInput");
    require(triggered_target(AdapterRequestedMode::kXInput) ==
                AdapterRequestedMode::kDInput,
            "XInput did not cycle to DInput");
    require(triggered_target(AdapterRequestedMode::kDInput) ==
                AdapterRequestedMode::kMac,
            "DInput did not cycle to Mac");
    require(triggered_target(AdapterRequestedMode::kMac) ==
                AdapterRequestedMode::kAuto,
            "Mac did not cycle to Auto");

    reset_harness();
    begin_hold(0, 10, 0);
    begin_hold(1, 20, 1000);
    ControllerState released{};
    sample(0, 10, 0, 2999, &released);
    finish_hold(1, 20, 1000);
    set_results = {ConfigurationTransactionStatus::kBusy};
    adapter_mode_controller_task(4000);
    require(set_transaction_ids.size() == 1,
            "one slot's release reset another slot's hold");

    reset_harness();
    begin_hold(2, 30, 0);
    ControllerState state = held_state();
    sample(2, 31, ADAPTER_MODE_CHORD_BUTTON_MASK, 3000, &state);
    adapter_mode_controller_task(3000);
    require(set_transaction_ids.empty(),
            "connection generation change inherited a prior hold");
    uint16_t inactive_mask = ADAPTER_MODE_CHORD_BUTTON_MASK;
    adapter_mode_controller_process_input(
        2, false, 31, &inactive_mask, 6000, &state);
    sample(2, 31, ADAPTER_MODE_CHORD_BUTTON_MASK, 6001, &state);
    adapter_mode_controller_task(6001);
    require(set_transaction_ids.empty(),
            "inactive slot retained mode chord state");
}

void test_busy_retry_and_one_shot() {
    reset_harness();
    begin_hold(0, 1, 0);
    finish_hold(0, 1, 0);
    set_results = {
        ConfigurationTransactionStatus::kBusy,
        ConfigurationTransactionStatus::kBusy,
        ConfigurationTransactionStatus::kPending,
    };
    adapter_mode_controller_task(3000);
    adapter_mode_controller_task(3001);
    adapter_mode_controller_task(3002);
    require(set_transaction_ids.size() == 3 &&
                set_transaction_ids[0] == set_transaction_ids[1] &&
                set_transaction_ids[1] == set_transaction_ids[2] &&
                (set_transaction_ids[0] & 0x80000000u) != 0,
            "busy retry did not preserve one high-bit transaction ID");

    query_status = ConfigurationTransactionStatus::kStorageError;
    adapter_mode_controller_task(3003);
    require(last_query_transaction_id == set_transaction_ids[0],
            "commit polling lost internal transaction correlation");
    ControllerState state = held_state();
    sample(0, 1, ADAPTER_MODE_CHORD_BUTTON_MASK, 9000, &state);
    adapter_mode_controller_task(9000);
    require(set_transaction_ids.size() == 3 && reboot_count == 0,
            "continuous hold retriggered after a failed transaction");

    state = {};
    sample(0, 1, 0, 9001, &state);
    begin_hold(0, 1, 10000);
    finish_hold(0, 1, 10000);
    set_results.push_back(ConfigurationTransactionStatus::kStorageError);
    adapter_mode_controller_task(13000);
    require(set_transaction_ids.size() == 4 &&
                set_transaction_ids[3] != set_transaction_ids[0],
            "release did not create exactly one new internal transaction");
}

void test_commit_feedback_then_reboot() {
    reset_harness(AdapterRequestedMode::kAuto);
    begin_hold(3, 44, 0);
    finish_hold(3, 44, 0);
    set_results = {ConfigurationTransactionStatus::kPending};
    adapter_mode_controller_task(3000);
    query_status = ConfigurationTransactionStatus::kCommitted;
    adapter_mode_controller_task(3001);
    require(runtime_reset_count == 1 && feedback_slot == 3 &&
                feedback_generation == 44 && feedback_pulses == 2 &&
                feedback_policy ==
                    ControllerProfileConfirmationPolicy::kRumbleAndLed &&
                reboot_count == 0 &&
                calls[calls.size() - 2] == Call::kRuntimeReset &&
                calls.back() == Call::kFeedback,
            "commit did not cancel synthetic state then acknowledge Switch");

    adapter_mode_controller_task(3375);
    require(reboot_count == 0,
            "reboot occurred before bounded mode feedback completed");
    adapter_mode_controller_task(3376);
    require(reboot_count == 1 && runtime_reset_count == 2 &&
                calls[calls.size() - 2] == Call::kRuntimeReset &&
                calls.back() == Call::kWatchdogReboot,
            "committed mode did not reset synthetic state at watchdog reboot");
    adapter_mode_controller_task(4000);
    require(reboot_count == 1 && runtime_reset_count == 2,
            "scheduled watchdog reboot looped");
}

void test_feedback_distinguishes_all_modes() {
    struct FeedbackCase {
        AdapterRequestedMode initial_mode;
        AdapterRequestedMode target_mode;
        uint8_t pulses;
        ControllerProfileConfirmationPolicy policy;
    };
    constexpr FeedbackCase cases[] = {
        {AdapterRequestedMode::kAuto, AdapterRequestedMode::kSwitch, 2,
         ControllerProfileConfirmationPolicy::kRumbleAndLed},
        {AdapterRequestedMode::kSwitch, AdapterRequestedMode::kXInput, 3,
         ControllerProfileConfirmationPolicy::kRumbleAndLed},
        {AdapterRequestedMode::kXInput, AdapterRequestedMode::kDInput, 4,
         ControllerProfileConfirmationPolicy::kRumbleAndLed},
        {AdapterRequestedMode::kDInput, AdapterRequestedMode::kMac, 4,
         ControllerProfileConfirmationPolicy::kLed},
        {AdapterRequestedMode::kMac, AdapterRequestedMode::kAuto, 1,
         ControllerProfileConfirmationPolicy::kRumbleAndLed},
    };

    for (const FeedbackCase& expected : cases) {
        reset_harness(expected.initial_mode);
        begin_hold(1, 23, 0);
        finish_hold(1, 23, 0);
        set_results = {ConfigurationTransactionStatus::kCommitted};
        adapter_mode_controller_task(ADAPTER_MODE_CHORD_HOLD_MS);
        require(set_modes.size() == 1 &&
                    set_modes[0] == expected.target_mode &&
                    feedback_slot == 1 && feedback_generation == 23 &&
                    feedback_pulses == expected.pulses &&
                    feedback_policy == expected.policy &&
                    reboot_count == 0,
                "mode feedback tuple did not uniquely identify its target");

        const uint32_t feedback_duration =
            static_cast<uint32_t>(expected.pulses) * 2u * 75u + 75u;
        adapter_mode_controller_task(
            ADAPTER_MODE_CHORD_HOLD_MS + feedback_duration - 1u);
        require(reboot_count == 0,
                "mode feedback deadline was shorter than its bounded pulse sequence");
        adapter_mode_controller_task(
            ADAPTER_MODE_CHORD_HOLD_MS + feedback_duration);
        require(reboot_count == 1,
                "mode feedback did not reboot at its bounded deadline");
    }
}

void test_configuration_failure_and_correlated_reboot() {
    reset_harness();
    begin_hold(0, 1, 0);
    finish_hold(0, 1, 0);
    set_results = {ConfigurationTransactionStatus::kStorageError};
    adapter_mode_controller_task(3000);
    require(runtime_reset_count == 0 && feedback_pulses == 0 &&
                reboot_count == 0,
            "failed internal mode write rebooted or acknowledged");

    reset_harness();
    reboot_ready = false;
    require(!adapter_reboot_for_mode_transaction(12) &&
                runtime_reset_count == 0 && reboot_count == 0,
            "non-current host mode transaction rebooted");
    reboot_ready = true;
    require(!adapter_reboot_for_mode_transaction(0x80000001u) &&
                reboot_count == 0,
            "internal transaction used the management reboot API");
    require(adapter_reboot_for_mode_transaction(12) &&
                last_reboot_ready_transaction_id == 12 &&
                runtime_reset_count == 1 && reboot_count == 1 &&
                calls[calls.size() - 2] == Call::kRuntimeReset &&
                calls.back() == Call::kWatchdogReboot,
            "current committed host transaction did not reset then reboot");
    require(adapter_reboot_for_mode_transaction(12) &&
                reboot_count == 1 && runtime_reset_count == 1,
            "duplicate normal reboot was not latched");
}

void test_recovery_auto_wins_host_mode_interleaving() {
    reset_harness(AdapterRequestedMode::kXInput);
    begin_hold(0, 5, 0);
    finish_hold(0, 5, 0);
    calls.clear();
    pairing_status = Bluepad32PairingSnapshotStatus::kReady;
    pairing_generation = 41;
    clear_pairings_result_token = 17;
    abandoned_host_receive = true;
    adapter_mode_controller_begin_recovery();
    adapter_mode_controller_begin_recovery();
    require(clear_pairings_count == 1 && recovery_reserved &&
                abandoned_host_receive_canceled &&
                !abandoned_host_receive && calls.size() >= 2 &&
                calls[0] == Call::kReserveRecovery &&
                calls[1] == Call::kClearPairings,
            "BOOTSEL recovery must reserve and cancel abandoned host receive "
            "before requesting one pairing clear");
    require(!adapter_reboot_for_mode_transaction(77) &&
                last_reboot_ready_transaction_id == 0 &&
                reboot_count == 0,
            "host mode traffic rebooted during active recovery");

    set_results = {
        ConfigurationTransactionStatus::kBusy,
        ConfigurationTransactionStatus::kPending,
        ConfigurationTransactionStatus::kPending,
    };
    adapter_mode_controller_task(0);
    pairing_status = Bluepad32PairingSnapshotStatus::kReady;
    pairing_generation = 42;
    adapter_mode_controller_task(1);
    require(set_modes.empty(),
            "unrelated pairing refresh advanced recovery");

    pairing_completed_clear_token = clear_pairings_result_token - 1;
    adapter_mode_controller_task(2);
    require(set_modes.empty(),
            "pre-request pairing clear completion advanced recovery");

    pairing_status = Bluepad32PairingSnapshotStatus::kPending;
    adapter_mode_controller_task(3);
    require(set_modes.empty(),
            "pending recovery pairing clear advanced recovery");

    // Recovery clear N completed, then host clear N+1 was accepted and
    // completed before recovery observed N. N+1 must still acknowledge N.
    pairing_status = Bluepad32PairingSnapshotStatus::kReady;
    pairing_completed_clear_token = clear_pairings_result_token + 1;
    adapter_mode_controller_task(4);
    adapter_mode_controller_task(5);
    require(set_modes.size() == 2 &&
                set_modes[0] == AdapterRequestedMode::kAuto &&
                set_modes[1] == AdapterRequestedMode::kAuto &&
                set_transaction_ids[0] == set_transaction_ids[1],
            "later clear completion did not busy-retry recovery Auto");
    const size_t first_set_call = static_cast<size_t>(
        std::find(calls.begin(), calls.end(), Call::kSetMode) -
        calls.begin());
    const size_t clear_call = static_cast<size_t>(
        std::find(calls.begin(), calls.end(), Call::kClearPairings) -
        calls.begin());
    require(clear_call < first_set_call,
            "recovery submitted Auto before requesting bond clear");

    // Model a stale mode commit result. Reboot readiness rejects it, so
    // recovery submits a new correlated Auto transaction.
    reboot_ready_results = {false, true};
    query_status = ConfigurationTransactionStatus::kCommitted;
    adapter_mode_controller_task(6);
    require(reboot_count == 0 && set_transaction_ids.size() == 2,
            "superseded recovery Auto rebooted");

    adapter_mode_controller_task(7);
    require(set_transaction_ids.size() == 3 &&
                set_modes[2] == AdapterRequestedMode::kAuto &&
                set_transaction_ids[2] != set_transaction_ids[1] &&
                reboot_count == 0,
            "superseded Auto did not yield to a fresh recovery transaction");
    adapter_mode_controller_task(8);
    require(last_reboot_ready_transaction_id ==
                    set_transaction_ids[2] &&
                adapter_mode_controller_requested_mode() ==
                    AdapterRequestedMode::kAuto &&
                feedback_pulses == 0 && runtime_reset_count == 1 &&
                reboot_count == 1 &&
                calls[calls.size() - 2] == Call::kRuntimeReset &&
                calls.back() == Call::kWatchdogReboot,
            "recovery did not make Auto final and reboot immediately");
}

void test_failed_recovery_never_reboots() {
    reset_harness(AdapterRequestedMode::kXInput);
    clear_pairings_result_token = 0;
    abandoned_host_receive = true;
    adapter_mode_controller_begin_recovery();
    pairing_status = Bluepad32PairingSnapshotStatus::kReady;
    pairing_completed_clear_token = 1;
    adapter_mode_controller_task(0);
    require(recovery_reserved && abandoned_host_receive_canceled &&
                clear_pairings_count == 1 && set_modes.empty() &&
                !adapter_reboot_for_mode_transaction(91) &&
                last_reboot_ready_transaction_id == 0 &&
                runtime_reset_count == 0 && reboot_count == 0,
            "failed pairing clear released recovery ownership or rebooted");

    reset_harness(AdapterRequestedMode::kXInput);
    clear_pairings_result_token = UINT32_MAX;
    adapter_mode_controller_begin_recovery();
    pairing_status = Bluepad32PairingSnapshotStatus::kReady;
    pairing_completed_clear_token = 1;
    set_results = {ConfigurationTransactionStatus::kStorageError};
    adapter_mode_controller_task(1);
    adapter_mode_controller_task(2);
    require(set_modes.size() == 1 &&
                set_modes[0] == AdapterRequestedMode::kAuto &&
                !adapter_reboot_for_mode_transaction(92) &&
                last_reboot_ready_transaction_id == 0 &&
                runtime_reset_count == 0 && feedback_pulses == 0 &&
                reboot_count == 0,
            "wrapped clear completion did not start exactly one recovery Auto, "
            "or failed recovery acknowledged, released ownership, or retried");
}

void test_auto_xinput_disconnect_reboots_to_probe() {
    reset_harness(AdapterRequestedMode::kAuto);
    probed_active_mode = AdapterUsbMode::kXInput;
    adapter_mode_controller_on_usb_unmounted();
    adapter_mode_controller_on_usb_unmounted();
    require(runtime_reset_count == 1 && reboot_count == 1,
            "Auto XInput unmount did not schedule exactly one probe reboot");

    reset_harness(AdapterRequestedMode::kXInput);
    probed_active_mode = AdapterUsbMode::kXInput;
    adapter_mode_controller_on_usb_unmounted();
    require(runtime_reset_count == 0 && reboot_count == 0,
            "manual XInput mode rebooted after USB unmount");

    reset_harness(AdapterRequestedMode::kAuto);
    probed_active_mode = AdapterUsbMode::kSwitchProbe;
    adapter_mode_controller_on_usb_unmounted();
    require(runtime_reset_count == 0 && reboot_count == 0,
            "Auto Switch probe rebooted after USB unmount");
}

void test_bootsel_reboot_is_delayed_and_idempotent() {
    reset_harness(AdapterRequestedMode::kAuto);
    require(adapter_reboot_to_bootsel() &&
                adapter_reboot_to_bootsel() &&
                runtime_reset_count == 1 &&
                bootsel_reboot_count == 0 && reboot_count == 0,
            "BOOTSEL request was not accepted exactly once");

    adapter_mode_controller_task(UINT32_MAX - 25u);
    adapter_mode_controller_task(23);
    require(bootsel_reboot_count == 0,
            "BOOTSEL reboot occurred before the control-transfer guard");
    adapter_mode_controller_task(24);
    adapter_mode_controller_task(25);
    require(bootsel_reboot_count == 1 &&
                bootsel_gpio_mask == 0 &&
                bootsel_disable_mask == 0 &&
                reboot_count == 0 &&
                !adapter_reboot_for_mode_transaction(91),
            "BOOTSEL reboot was not delayed, one-shot, or isolated from "
            "the correlated watchdog path");
}

}  // namespace

void configuration_service_initialize_pre_usb() {
    calls.push_back(Call::kPreload);
}

void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    calls.push_back(Call::kSnapshot);
    *output = {};
    output->state = ConfigurationServiceState::kReady;
    output->configuration.requested_mode = stored_mode;
}

void configuration_service_reserve_for_recovery() {
    calls.push_back(Call::kReserveRecovery);
    recovery_reserved = true;
    if (abandoned_host_receive) {
        abandoned_host_receive = false;
        abandoned_host_receive_canceled = true;
    }
}

ConfigurationTransactionStatus configuration_service_set_mode_internal(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    require(!recovery_reserved ||
                requested_mode == AdapterRequestedMode::kAuto,
            "recovery reservation admitted a non-Auto internal mode");
    calls.push_back(Call::kSetMode);
    require(availability.switch_mode && availability.xinput_mode &&
                availability.dinput_mode && availability.mac_mode,
            "mode controller did not advertise all compiled drivers");
    set_transaction_ids.push_back(transaction_id);
    set_modes.push_back(requested_mode);
    if (next_set_result >= set_results.size()) {
        return ConfigurationTransactionStatus::kBusy;
    }
    return set_results[next_set_result++];
}

bool configuration_service_mode_transaction_status(
    uint32_t transaction_id, ConfigurationTransactionStatus* output) {
    calls.push_back(Call::kQueryMode);
    last_query_transaction_id = transaction_id;
    if (!query_matches) {
        return false;
    }
    *output = query_status;
    return true;
}

bool configuration_service_mode_transaction_reboot_ready(
    uint32_t transaction_id) {
    calls.push_back(Call::kRebootReady);
    last_reboot_ready_transaction_id = transaction_id;
    if (next_reboot_ready_result < reboot_ready_results.size()) {
        return reboot_ready_results[next_reboot_ready_result++];
    }
    return reboot_ready;
}

void adapter_host_probe_init(AdapterRequestedMode requested_mode) {
    calls.push_back(Call::kProbeInit);
    probed_requested_mode = requested_mode;
}

AdapterUsbMode adapter_host_probe_mode() { return probed_active_mode; }

void tusb_init() {
    calls.push_back(Call::kTinyUsbInit);
}
void usb_output_driver_init(AdapterUsbMode mode) {
    calls.push_back(Call::kOutputInit);
    initialized_output_mode = mode;
}

void controller_profile_runtime_reset() {
    calls.push_back(Call::kRuntimeReset);
    ++runtime_reset_count;
}

void bluepad32_input_backend_queue_profile_feedback(
    uint8_t slot, uint32_t connection_generation,
    uint8_t active_profile_number,
    ControllerProfileConfirmationPolicy policy) {
    calls.push_back(Call::kFeedback);
    feedback_slot = slot;
    feedback_generation = connection_generation;
    feedback_pulses = active_profile_number;
    feedback_policy = policy;
}

uint32_t bluepad32_input_backend_clear_pairings() {
    calls.push_back(Call::kClearPairings);
    require(recovery_reserved,
            "pairing clear was requested before recovery reservation");
    ++clear_pairings_count;
    pairing_status = Bluepad32PairingSnapshotStatus::kPending;
    return clear_pairings_result_token;
}

void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* output) {
    calls.push_back(Call::kPairingSnapshot);
    *output = {};
    output->status = pairing_status;
    output->generation = pairing_generation;
    output->completed_clear_pairings_token =
        pairing_completed_clear_token;
}

void watchdog_reboot(uint32_t, uint32_t, uint32_t) {
    calls.push_back(Call::kWatchdogReboot);
    ++reboot_count;
}

void reset_usb_boot(uint32_t usb_activity_gpio_pin_mask,
                    uint32_t disable_interface_mask) {
    calls.push_back(Call::kBootselReboot);
    ++bootsel_reboot_count;
    bootsel_gpio_mask = usb_activity_gpio_pin_mask;
    bootsel_disable_mask = disable_interface_mask;
}

int main() {
    test_pre_tusb_ordering_and_configured_selection();
    test_mode_availability_has_one_stable_value();
    test_exact_hold_release_wrap_and_consumption();
    test_cycle_and_slot_isolation();
    test_busy_retry_and_one_shot();
    test_commit_feedback_then_reboot();
    test_feedback_distinguishes_all_modes();
    test_configuration_failure_and_correlated_reboot();
    test_recovery_auto_wins_host_mode_interleaving();
    test_failed_recovery_never_reboots();
    test_auto_xinput_disconnect_reboots_to_probe();
    test_bootsel_reboot_is_delayed_and_idempotent();
    return 0;
}
