#include "controller_profile_runtime.h"

#include "controller_identity.h"
#include "controller_profile.h"
#include "profile_service.h"

#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeProfileRow {
    ControllerIdentity identity{};
    uint8_t active_profile = 0;
    ControllerProfile profiles[CONTROLLER_PROFILE_COUNT]{};
};

std::array<FakeProfileRow, CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT> rows{};
uint32_t database_generation = 7;
uint32_t configuration_reset_generation = 3;
unsigned active_snapshot_count = 0;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

ControllerIdentity make_identity(uint8_t value) {
    ControllerIdentity identity{};
    identity.stable = true;
    identity.transport = ControllerTransport::kClassic;
    identity.address[0] = value;
    identity.address[5] = static_cast<uint8_t>(value + 0x40u);
    identity.vendor_id = static_cast<uint16_t>(0x1000u + value);
    identity.product_id = static_cast<uint16_t>(0x2000u + value);
    return identity;
}

void prepare_profiles() {
    database_generation = 7;
    configuration_reset_generation = 3;
    active_snapshot_count = 0;
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        FakeProfileRow& row = rows[slot];
        row = {};
        row.identity = make_identity(static_cast<uint8_t>(slot + 1u));
        for (uint8_t profile_index = 0;
             profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
            row.profiles[profile_index] =
                controller_profile_default(row.identity, profile_index);
        }
        row.profiles[0].triggers[0].digital_threshold =
            static_cast<uint16_t>(1000u + slot);
        row.profiles[0].triggers[1].digital_threshold =
            static_cast<uint16_t>(5000u + slot);
        row.profiles[0].confirmation_policy =
            slot == 0 ? ControllerProfileConfirmationPolicy::kLed
                      : ControllerProfileConfirmationPolicy::kRumble;
    }
    rows[0].profiles[0].strong_rumble_scale = 0;
    rows[0].profiles[0].weak_rumble_scale = UINT8_MAX;
    rows[1].profiles[0].strong_rumble_scale = UINT8_MAX;
    rows[1].profiles[0].weak_rumble_scale = 0;
    controller_profile_runtime_reset();
}

Bluepad32SlotSnapshot make_snapshot(uint8_t slot,
                                    uint32_t connection_generation = 1) {
    Bluepad32SlotSnapshot snapshot{};
    snapshot.active = true;
    snapshot.connection_generation = connection_generation;
    snapshot.identity = rows[slot].identity;
    snapshot.state = controller_neutral_state();
    return snapshot;
}

ControllerProfileTransformResult runtime_transform(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    uint32_t now_ms = 0,
    AdapterUsbMode output_mode = AdapterUsbMode::kSwitchProbe) {
    return controller_profile_runtime_transform(slot, snapshot, now_ms,
                                                output_mode);
}

bool motion_equal(const ControllerState& first,
                  const ControllerState& second) {
    return first.motion_sample_count == second.motion_sample_count &&
           std::memcmp(first.motion_samples, second.motion_samples,
                       sizeof(first.motion_samples)) == 0;
}

void test_four_slot_cache_and_unchanged_generation() {
    prepare_profiles();
    std::array<Bluepad32SlotSnapshot,
               CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT>
        snapshots{};
    std::array<ControllerProfileTransformResult,
               CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT>
        transformed{};
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        snapshots[slot] = make_snapshot(slot);
        transformed[slot] =
            runtime_transform(slot, snapshots[slot]);
        require(transformed[slot].left_trigger_digital_threshold ==
                        static_cast<uint16_t>(1000u + slot) &&
                    transformed[slot].right_trigger_digital_threshold ==
                        static_cast<uint16_t>(5000u + slot),
                "a slot did not receive its own cached profile");
    }
    require(active_snapshot_count == CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT,
            "initial slot loads did not fetch exactly one profile each");

    rows[0].profiles[0].triggers[0].digital_threshold = 65000;
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        transformed[slot] =
            runtime_transform(slot, snapshots[slot]);
    }
    require(active_snapshot_count == CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT,
            "unchanged generations copied profiles on the report path");
    require(transformed[0].left_trigger_digital_threshold == 1000,
            "an unchanged generation bypassed the slot cache");

    snapshots[2].connection_generation = 2;
    transformed[2] =
        runtime_transform(2, snapshots[2]);
    require(active_snapshot_count ==
                CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT + 1 &&
                transformed[2].left_trigger_digital_threshold == 1002,
            "one slot connection generation did not refresh in isolation");

    snapshots[3].identity = rows[1].identity;
    transformed[3] =
        runtime_transform(3, snapshots[3]);
    require(active_snapshot_count ==
                CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT + 2 &&
                transformed[3].left_trigger_digital_threshold == 1001,
            "an exact identity change did not refresh only its slot");
}

void test_activation_disconnect_and_default_preservation() {
    prepare_profiles();
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    snapshot.state.button_south = true;
    ControllerProfileTransformResult transformed =
        runtime_transform(0, snapshot);
    require(transformed.state.button_south &&
                transformed.left_trigger_digital_threshold == 1000,
            "initial active profile was not applied");

    rows[0].active_profile = 1;
    rows[0].profiles[1].button_map[
        static_cast<uint8_t>(ControllerProfileLogicalButton::kSouth)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kNorth);
    rows[0].profiles[1].triggers[0].digital_threshold = 12345;
    ++database_generation;
    transformed = runtime_transform(0, snapshot);
    require(!transformed.state.button_south && transformed.state.button_north &&
                transformed.left_trigger_digital_threshold == 12345,
            "profile activation generation did not refresh the slot cache");

    const unsigned reads_before_disconnect = active_snapshot_count;
    snapshot.active = false;
    transformed = runtime_transform(0, snapshot);
    require(!transformed.state.button_south && !transformed.state.button_north &&
                transformed.left_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                active_snapshot_count == reads_before_disconnect,
            "disconnect did not neutralize output without fetching a profile");

    transformed = runtime_transform(0, snapshot);
    require(active_snapshot_count == reads_before_disconnect,
            "an unchanged inactive slot repeatedly cleared or fetched state");
    snapshot.active = true;
    transformed = runtime_transform(0, snapshot);
    require(active_snapshot_count == reads_before_disconnect + 1 &&
                transformed.state.button_north,
            "reconnection did not reload a cleared slot cache");

    Bluepad32SlotSnapshot default_snapshot = make_snapshot(2);
    rows[2].profiles[0] =
        controller_profile_default(rows[2].identity, 0);
    ++database_generation;
    default_snapshot.state.button_east = true;
    default_snapshot.state.dpad_left = true;
    default_snapshot.state.left_trigger = 32123;
    default_snapshot.state.right_trigger = 54321;
    default_snapshot.state.left_stick_x = -12345;
    default_snapshot.state.right_stick_y = 23456;
    default_snapshot.state.motion_sample_count = 2;
    default_snapshot.state.motion_samples[0] = {1, 2, 3, 4, 5, 6};
    default_snapshot.state.motion_samples[1] = {-1, -2, -3, -4, -5, -6};
    transformed = runtime_transform(2, default_snapshot);
    require(transformed.state.button_east && transformed.state.dpad_left &&
                transformed.state.left_trigger == 32123 &&
                transformed.state.right_trigger == 54321 &&
                transformed.state.left_stick_x == -12345 &&
                transformed.state.right_stick_y == 23456 &&
                motion_equal(transformed.state, default_snapshot.state) &&
                transformed.left_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                transformed.right_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            "default runtime profile changed serializer input or motion");
}

void test_analog_thresholds_rumble_and_local_confirmation() {
    prepare_profiles();
    Bluepad32SlotSnapshot first = make_snapshot(0);
    Bluepad32SlotSnapshot second = make_snapshot(1);
    first.state.left_trigger = second.state.left_trigger = 23456;
    first.state.right_trigger = second.state.right_trigger = 45678;
    const ControllerProfileTransformResult first_output =
        runtime_transform(0, first);
    const ControllerProfileTransformResult second_output =
        runtime_transform(1, second);
    require(first_output.state.left_trigger ==
                    second_output.state.left_trigger &&
                first_output.state.right_trigger ==
                    second_output.state.right_trigger &&
                first_output.left_trigger_digital_threshold !=
                    second_output.left_trigger_digital_threshold,
            "digital thresholds changed XInput-visible analog trigger state");

    const ControllerRumbleOutput host{91, 73};
    const ControllerRumbleOutput first_host =
        controller_profile_runtime_scale_host_rumble(0, first, host);
    const ControllerRumbleOutput second_host =
        controller_profile_runtime_scale_host_rumble(1, second, host);
    require(first_host.low_frequency_magnitude == 0 &&
                first_host.high_frequency_magnitude == 73 &&
                second_host.low_frequency_magnitude == 91 &&
                second_host.high_frequency_magnitude == 0,
            "host rumble was not scaled through each slot profile");

    const ControllerProfileRuntimeLocalConfirmation confirmation =
        controller_profile_runtime_local_confirmation(0, first, host);
    require(confirmation.rumble.low_frequency_magnitude == 91 &&
                confirmation.rumble.high_frequency_magnitude == 73 &&
                confirmation.policy ==
                    ControllerProfileConfirmationPolicy::kLed,
            "local confirmation was scaled or lost its profile policy");
}

void configure_synthetic_profile(uint8_t slot) {
    ControllerProfile& profile = rows[slot].profiles[0];
    profile = controller_profile_default(rows[slot].identity, 0);
    profile.macro_trigger =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kSouth);
    profile.macro_cancel =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kCapture);
    profile.macro_step_count = 2;
    profile.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 1000;
    profile.macro_steps[0].output_button_mask = static_cast<uint16_t>(
        1u << static_cast<uint8_t>(
            ControllerProfileLogicalButton::kNorth));
    profile.macro_steps[1] = {};
    profile.macro_steps[1].type =
        ControllerProfileMacroStepType::kEnd;
    profile.turbo_modes[static_cast<uint8_t>(
        ControllerProfileLogicalButton::kEast)] =
        ControllerProfileTurboMode::kAutoBurst;
}

void start_macro(Bluepad32SlotSnapshot* snapshot, uint32_t now_ms,
                 AdapterUsbMode mode = AdapterUsbMode::kSwitchProbe) {
    snapshot->state = controller_neutral_state();
    (void)runtime_transform(0, *snapshot, now_ms, mode);
    snapshot->state.button_south = true;
    const ControllerProfileTransformResult started =
        runtime_transform(0, *snapshot, now_ms + 1u, mode);
    require(started.state.button_north && !started.state.button_south,
            "runtime fixture did not start its macro");
    snapshot->state = controller_neutral_state();
    const ControllerProfileTransformResult held =
        runtime_transform(0, *snapshot, now_ms + 2u, mode);
    require(held.state.button_north,
            "runtime fixture macro did not remain active");
}

void start_auto_burst(Bluepad32SlotSnapshot* snapshot, uint32_t now_ms,
                      AdapterUsbMode mode =
                          AdapterUsbMode::kSwitchProbe) {
    snapshot->state = controller_neutral_state();
    (void)runtime_transform(0, *snapshot, now_ms, mode);
    snapshot->state.button_east = true;
    const ControllerProfileTransformResult started =
        runtime_transform(0, *snapshot, now_ms + 1u, mode);
    require(started.state.button_east,
            "runtime fixture did not start Auto Burst");
    snapshot->state = controller_neutral_state();
    const ControllerProfileTransformResult latched =
        runtime_transform(0, *snapshot, now_ms + 2u, mode);
    require(latched.state.button_east,
            "runtime fixture Auto Burst did not latch");
}

void require_no_synthetic_output(
    const ControllerProfileTransformResult& output,
    const char* message) {
    require(!output.state.button_north && !output.state.button_east &&
                !output.state.button_south &&
                !output.state.button_capture,
            message);
}

void test_all_runtime_cancellation_causes() {
    prepare_profiles();
    configure_synthetic_profile(0);
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    start_macro(&snapshot, 10);
    snapshot.active = false;
    ControllerProfileTransformResult output =
        runtime_transform(0, snapshot, 13);
    require_no_synthetic_output(
        output, "disconnect did not cancel synthetic output");
    snapshot.active = true;
    output = runtime_transform(0, snapshot, 14);
    require_no_synthetic_output(
        output, "reconnection restored stale synthetic output");

    prepare_profiles();
    configure_synthetic_profile(0);
    snapshot = make_snapshot(0);
    start_macro(&snapshot, 20);
    ++snapshot.connection_generation;
    output = runtime_transform(0, snapshot, 23);
    require_no_synthetic_output(
        output, "connection replacement did not cancel before output");

    prepare_profiles();
    configure_synthetic_profile(0);
    snapshot = make_snapshot(0);
    start_macro(&snapshot, 30);
    rows[0].active_profile = 1;
    ++database_generation;
    output = runtime_transform(0, snapshot, 33);
    require_no_synthetic_output(
        output, "profile/database generation change did not cancel");

    prepare_profiles();
    configure_synthetic_profile(0);
    snapshot = make_snapshot(0);
    start_auto_burst(&snapshot, 40);
    output = runtime_transform(0, snapshot, 43,
                               AdapterUsbMode::kXInput);
    require_no_synthetic_output(
        output, "output-mode change did not cancel Auto Burst");
    output = runtime_transform(0, snapshot, 44,
                               AdapterUsbMode::kXInput);
    require_no_synthetic_output(
        output, "output-mode cancellation left a stuck output");

    prepare_profiles();
    configure_synthetic_profile(0);
    snapshot = make_snapshot(0);
    start_auto_burst(&snapshot, 50);
    ++configuration_reset_generation;
    output = runtime_transform(0, snapshot, 53);
    require_no_synthetic_output(
        output, "configuration reset did not cancel Auto Burst");
    output = runtime_transform(0, snapshot, 54);
    require_no_synthetic_output(
        output, "configuration reset cancellation left a stuck output");
}

void test_runtime_slot_synthetic_isolation() {
    prepare_profiles();
    configure_synthetic_profile(0);
    configure_synthetic_profile(1);
    Bluepad32SlotSnapshot first = make_snapshot(0);
    Bluepad32SlotSnapshot second = make_snapshot(1);
    (void)runtime_transform(0, first, 0);
    (void)runtime_transform(1, second, 0);
    first.state.button_east = true;
    second.state.button_east = true;
    require(runtime_transform(0, first, 1).state.button_east &&
                runtime_transform(1, second, 1).state.button_east,
            "runtime slots did not activate independently");
    first.state = controller_neutral_state();
    second.state = controller_neutral_state();
    require(runtime_transform(0, first, 2).state.button_east &&
                runtime_transform(1, second, 2).state.button_east,
            "runtime Auto Burst state did not remain isolated");

    first.state.button_capture = true;
    const ControllerProfileTransformResult cancelled =
        runtime_transform(0, first, 3);
    const ControllerProfileTransformResult untouched =
        runtime_transform(1, second, 3);
    require_no_synthetic_output(
        cancelled, "slot-local configured cancel did not clear its output");
    require(untouched.state.button_east,
            "slot-local configured cancel affected another slot");
}

}  // namespace

uint32_t profile_service_database_generation() {
    return database_generation;
}

uint32_t configuration_service_reset_generation() {
    return configuration_reset_generation;
}

void profile_service_active_profile_snapshot(
    const ControllerIdentity& identity,
    ProfileServiceActiveProfileSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    ++active_snapshot_count;
    *output = {};
    output->metadata.state = ProfileServiceState::kReady;
    output->metadata.generation = database_generation;
    for (const FakeProfileRow& row : rows) {
        if (!controller_identity_equal(row.identity, identity)) {
            continue;
        }
        output->valid = true;
        output->profile_index = row.active_profile;
        output->profile = row.profiles[row.active_profile];
        return;
    }
}

int main() {
    test_four_slot_cache_and_unchanged_generation();
    test_activation_disconnect_and_default_preservation();
    test_analog_thresholds_rumble_and_local_confirmation();
    test_all_runtime_cancellation_causes();
    test_runtime_slot_synthetic_isolation();
    return 0;
}
