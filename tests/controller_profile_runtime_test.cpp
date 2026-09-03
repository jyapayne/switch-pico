#include "profile/controller_profile_runtime.h"
#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_service.h"
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
struct ActivationAttempt {
    uint32_t transaction_id = 0;
    ControllerIdentity identity{};
    uint8_t profile_index = 0;
};


std::array<FakeProfileRow, CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT> rows{};
uint32_t database_generation = 7;
uint32_t configuration_reset_generation = 3;
unsigned active_snapshot_count = 0;
std::array<ActivationAttempt, 32> activation_attempts{};
size_t activation_attempt_count = 0;
unsigned activation_busy_attempts = 0;

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
    activation_attempts = {};
    activation_attempt_count = 0;
    activation_busy_attempts = 0;
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
void apply_button_masks(uint16_t pre_hotkey_mask,
                        uint16_t output_mask,
                        Bluepad32SlotSnapshot* snapshot) {
    snapshot->pre_hotkey_button_mask = pre_hotkey_mask;
    snapshot->state = controller_neutral_state();
    controller_profile_apply_button_mask(output_mask, &snapshot->state);
}
void apply_button_mask(uint16_t mask, Bluepad32SlotSnapshot* snapshot) {
    apply_button_masks(mask, mask, snapshot);
}

ControllerProfileRuntimeProfileChangeEvent take_profile_change(
    uint8_t slot, bool* available) {
    ControllerProfileRuntimeProfileChangeEvent event{};
    *available =
        controller_profile_runtime_take_profile_change(slot, &event);
    return event;
}

ControllerProfileRuntimeProfileChangeEvent
take_initial_profile_indication(uint8_t slot, bool* available) {
    ControllerProfileRuntimeProfileChangeEvent event{};
    *available =
        controller_profile_runtime_take_initial_profile_indication(
            slot, &event);
    return event;
}

constexpr uint16_t logical_button_bit(
    ControllerProfileLogicalButton button) {
    return static_cast<uint16_t>(
        1u << static_cast<uint8_t>(button));
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

void test_initial_profile_indication_once_per_connection() {
    prepare_profiles();
    std::array<Bluepad32SlotSnapshot,
               CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT>
        snapshots{};
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        rows[slot].active_profile = slot;
        rows[slot].profiles[slot].confirmation_policy =
            ControllerProfileConfirmationPolicy::kRumbleAndLed;
        snapshots[slot] = make_snapshot(slot);
        (void)runtime_transform(slot, snapshots[slot]);
    }

    constexpr uint8_t kTakeOrder[CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT] = {
        2, 0, 3, 1};
    for (const uint8_t slot : kTakeOrder) {
        bool available = false;
        const ControllerProfileRuntimeProfileChangeEvent event =
            take_initial_profile_indication(slot, &available);
        require(available &&
                    event.connection_generation == 1 &&
                    event.database_generation == database_generation &&
                    event.active_profile_number == slot + 1u &&
                    event.policy ==
                        ControllerProfileConfirmationPolicy::kLed,
                "initial profile 1..4 indication was not isolated, "
                "LED-only, or generation-bound");
        (void)take_initial_profile_indication(slot, &available);
        require(!available,
                "initial profile indication repeated without a new "
                "connection");
        (void)take_profile_change(slot, &available);
        require(!available,
                "initial profile resolution published switch feedback");
    }

    ++database_generation;
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        (void)runtime_transform(slot, snapshots[slot]);
        bool available = true;
        (void)take_initial_profile_indication(slot, &available);
        require(!available,
                "database refresh repeated initial profile indication");
    }

    rows[2].identity = controller_identity_global();
    snapshots[0].identity = controller_identity_global();
    snapshots[0].connection_generation = 2;
    (void)runtime_transform(0, snapshots[0]);
    bool available = false;
    const ControllerProfileRuntimeProfileChangeEvent unresolved_event =
        take_initial_profile_indication(0, &available);
    require(available &&
                unresolved_event.connection_generation == 2 &&
                unresolved_event.active_profile_number == 3,
            "first committed unresolved-identity profile was not "
            "indicated");

    snapshots[0].identity = rows[1].identity;
    (void)runtime_transform(0, snapshots[0]);
    (void)take_initial_profile_indication(0, &available);
    require(!available,
            "identity promotion repeated initial profile indication");

    snapshots[0].connection_generation = 3;
    (void)runtime_transform(0, snapshots[0]);
    const ControllerProfileRuntimeProfileChangeEvent reconnect_event =
        take_initial_profile_indication(0, &available);
    require(available &&
                reconnect_event.connection_generation == 3 &&
                reconnect_event.active_profile_number == 2 &&
                reconnect_event.policy ==
                    ControllerProfileConfirmationPolicy::kLed,
            "true reconnection did not publish one fresh LED-only "
            "profile indication");

    rows[1].profiles[1].confirmation_policy =
        ControllerProfileConfirmationPolicy::kRumble;
    snapshots[1].connection_generation = 2;
    (void)runtime_transform(1, snapshots[1]);
    (void)take_initial_profile_indication(1, &available);
    require(!available,
            "rumble-only initial policy disturbed controller feedback");

    rows[3].profiles[3].confirmation_policy =
        ControllerProfileConfirmationPolicy::kNone;
    snapshots[3].connection_generation = 2;
    (void)runtime_transform(3, snapshots[3]);
    (void)take_initial_profile_indication(3, &available);
    require(!available,
            "disabled initial policy disturbed controller feedback");
}

void test_default_switching_retry_commit_and_feedback() {
    prepare_profiles();
    ControllerProfile& initial_profile = rows[0].profiles[0];
    initial_profile.macro_trigger = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kLeftShoulder);
    initial_profile.macro_step_count = 2;
    initial_profile.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    initial_profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    initial_profile.macro_steps[0].duration_ms = 1000;
    initial_profile.macro_steps[0].output_button_mask =
        logical_button_bit(ControllerProfileLogicalButton::kNorth);
    initial_profile.macro_steps[1].type =
        ControllerProfileMacroStepType::kEnd;

    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    bool event_available = true;
    (void)take_profile_change(0, &event_available);
    require(!event_available,
            "initial profile load published confirmation feedback");

    activation_busy_attempts = 2;
    const uint16_t held_mask = static_cast<uint16_t>(
        CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD |
        logical_button_bit(ControllerProfileLogicalButton::kSouth));
    apply_button_mask(held_mask, &snapshot);
    ControllerProfileTransformResult output =
        runtime_transform(0, snapshot, 1);
    require(activation_attempt_count == 1 &&
                rows[0].active_profile == 0 &&
                output.state.button_south &&
                !output.state.button_left_shoulder &&
                !output.state.button_right_shoulder &&
                !output.state.button_select &&
                !output.state.button_start &&
                !output.state.button_north,
            "default chord was not consumed before a busy activation and synthetic trigger");
    output = runtime_transform(0, snapshot, 2);
    output = runtime_transform(0, snapshot, 3);
    require(activation_attempt_count == 3 &&
                activation_attempts[0].transaction_id ==
                    activation_attempts[1].transaction_id &&
                activation_attempts[1].transaction_id ==
                    activation_attempts[2].transaction_id &&
                (activation_attempts[0].transaction_id & 0x80000000u) != 0 &&
                activation_attempts[0].transaction_id != 0 &&
                activation_attempts[0].profile_index == 1 &&
                output.state.button_south && !output.state.button_north,
            "held busy chord did not retry one internal activation transaction");

    (void)runtime_transform(0, snapshot, 4);
    require(activation_attempt_count == 3,
            "accepted activation repeated while the chord remained held");
    (void)take_profile_change(0, &event_available);
    require(!event_available,
            "activation feedback was published before storage commit");

    rows[0].active_profile = 1;
    rows[0].profiles[1].button_map[
        static_cast<uint8_t>(ControllerProfileLogicalButton::kSouth)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kNorth);
    rows[0].profiles[1].confirmation_policy =
        ControllerProfileConfirmationPolicy::kLed;
    ++database_generation;
    output = runtime_transform(0, snapshot, 5);
    const ControllerProfileRuntimeProfileChangeEvent event =
        take_profile_change(0, &event_available);
    require(event_available && event.connection_generation == 1 &&
                event.database_generation == database_generation &&
                event.active_profile_number == 2 &&
                event.policy == ControllerProfileConfirmationPolicy::kLed &&
                output.state.button_north &&
                !output.state.button_south &&
                !output.state.button_left_shoulder &&
                activation_attempt_count == 3,
            "committed activation did not cancel, transform, and publish exactly one event");
    (void)take_profile_change(0, &event_available);
    require(!event_available,
            "committed profile change feedback was published twice");

    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 6);
    apply_button_mask(CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD,
                      &snapshot);
    (void)runtime_transform(0, snapshot, 7);
    require(activation_attempt_count == 4 &&
                activation_attempts[3].profile_index == 2 &&
                activation_attempts[3].transaction_id !=
                    activation_attempts[0].transaction_id &&
                (activation_attempts[3].transaction_id & 0x80000000u) != 0,
            "release did not re-arm one activation for the next profile");
}

void test_identity_promotion_preserves_held_switching() {
    prepare_profiles();
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    snapshot.identity = controller_identity_global();
    apply_button_mask(
        CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD, &snapshot);

    const ControllerProfileTransformResult unresolved =
        runtime_transform(0, snapshot, 0);
    require(activation_attempt_count == 1 &&
                controller_identity_is_global(
                    activation_attempts[0].identity) &&
                unresolved.left_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            "unresolved BLE identity did not begin one activation");

    snapshot.identity = rows[0].identity;
    const ControllerProfileTransformResult promoted =
        runtime_transform(0, snapshot, 1);
    bool event_available = true;
    (void)take_profile_change(0, &event_available);
    require(activation_attempt_count == 1 &&
                promoted.left_trigger_digital_threshold == 1000 &&
                !event_available,
            "identity-only BLE promotion re-armed a held activation or "
            "failed to refresh its profile");

    (void)runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 1,
            "promoted BLE identity repeated an accepted held activation");

    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 3);
    apply_button_mask(
        CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD, &snapshot);
    (void)runtime_transform(0, snapshot, 4);
    require(activation_attempt_count == 2 &&
                controller_identity_equal(
                    activation_attempts[1].identity,
                    rows[0].identity),
            "physical release did not re-arm the promoted identity");
}

void configure_motion_suppression_probe(ControllerProfile* profile) {
    profile->macro_trigger = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kDpadUp);
    profile->macro_step_count = 2;
    profile->macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    profile->macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile->macro_steps[0].duration_ms = 1000;
    profile->macro_steps[0].output_button_mask =
        logical_button_bit(ControllerProfileLogicalButton::kNorth);
    profile->macro_steps[1].type =
        ControllerProfileMacroStepType::kEnd;
}

void test_switching_uses_pre_hotkey_buttons_only() {
    constexpr uint16_t kMotionHotkeyLogicalMask =
        static_cast<uint16_t>(
            logical_button_bit(
                ControllerProfileLogicalButton::kDpadUp) |
            logical_button_bit(
                ControllerProfileLogicalButton::kRightShoulder) |
            logical_button_bit(
                ControllerProfileLogicalButton::kStart));
    constexpr uint16_t kCustomChord =
        static_cast<uint16_t>(
            kMotionHotkeyLogicalMask |
            logical_button_bit(
                ControllerProfileLogicalButton::kSouth));

    for (uint8_t custom = 0; custom < 2; ++custom) {
        prepare_profiles();
        ControllerProfile& profile = rows[0].profiles[0];
        configure_motion_suppression_probe(&profile);
        const uint16_t switching_chord =
            custom != 0 ? kCustomChord
                        : CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD;
        profile.switching_chord = custom != 0 ? kCustomChord : 0;

        Bluepad32SlotSnapshot snapshot = make_snapshot(0);
        (void)runtime_transform(0, snapshot, 0);
        const uint16_t pre_hotkey_mask = static_cast<uint16_t>(
            switching_chord | kMotionHotkeyLogicalMask);
        const uint16_t output_mask = static_cast<uint16_t>(
            pre_hotkey_mask & ~kMotionHotkeyLogicalMask);
        apply_button_masks(
            pre_hotkey_mask, output_mask, &snapshot);
        ControllerProfileTransformResult output =
            runtime_transform(0, snapshot, 1);
        require(activation_attempt_count == 1 &&
                    !output.state.dpad_up &&
                    !output.state.button_right_shoulder &&
                    !output.state.button_start &&
                    !output.state.button_left_shoulder &&
                    !output.state.button_select &&
                    !output.state.button_south &&
                    !output.state.button_north,
                custom != 0
                    ? "custom chord hidden by motion suppression"
                    : "default chord hidden by motion suppression");

        output = runtime_transform(0, snapshot, 2);
        require(activation_attempt_count == 1 &&
                    !output.state.button_north,
                "held pre-hotkey chord repeated activation or leaked "
                "into synthetic input");
    }
}

void test_custom_switching_chord_and_wrap() {
    prepare_profiles();
    constexpr uint16_t kCustomChord =
        static_cast<uint16_t>(
            logical_button_bit(ControllerProfileLogicalButton::kSouth) |
            logical_button_bit(ControllerProfileLogicalButton::kCapture));
    rows[0].profiles[0].switching_chord = kCustomChord;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);

    apply_button_mask(CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD,
                      &snapshot);
    ControllerProfileTransformResult output =
        runtime_transform(0, snapshot, 1);
    require(activation_attempt_count == 0 &&
                output.state.button_left_shoulder &&
                output.state.button_right_shoulder &&
                output.state.button_select && output.state.button_start,
            "nonzero switching chord did not replace the default");

    apply_button_mask(kCustomChord, &snapshot);
    output = runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 1 &&
                !output.state.button_south &&
                !output.state.button_capture,
            "custom switching chord was not consumed or activated");

    prepare_profiles();
    rows[0].active_profile = 3;
    rows[0].profiles[3].switching_chord = kCustomChord;
    snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 10);
    bool event_available = true;
    (void)take_profile_change(0, &event_available);
    require(!event_available,
            "initial profile 4 load published a change event");
    apply_button_mask(kCustomChord, &snapshot);
    (void)runtime_transform(0, snapshot, 11);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 0,
            "profile switching did not wrap profile 4 to profile 1");
}
void test_switching_slot_isolation() {
    prepare_profiles();
    std::array<Bluepad32SlotSnapshot,
               CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT>
        snapshots{};
    for (uint8_t slot = 0;
         slot < CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT; ++slot) {
        snapshots[slot] = make_snapshot(slot);
        (void)runtime_transform(slot, snapshots[slot], 0);
    }

    apply_button_mask(CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD,
                      &snapshots[2]);
    (void)runtime_transform(2, snapshots[2], 1);
    snapshots[0].state.button_south = true;
    const ControllerProfileTransformResult untouched =
        runtime_transform(0, snapshots[0], 1);
    require(activation_attempt_count == 1 &&
                controller_identity_equal(
                    activation_attempts[0].identity,
                    rows[2].identity) &&
                untouched.state.button_south,
            "one slot's switching chord affected another slot");

    apply_button_mask(CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD,
                      &snapshots[3]);
    (void)runtime_transform(2, snapshots[2], 2);
    (void)runtime_transform(3, snapshots[3], 2);
    require(activation_attempt_count == 2 &&
                controller_identity_equal(
                    activation_attempts[1].identity,
                    rows[3].identity),
            "held switching state was shared between controller slots");
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
    bool event_available = false;
    const ControllerProfileRuntimeProfileChangeEvent cancellation_event =
        take_profile_change(0, &event_available);
    require(event_available &&
                cancellation_event.active_profile_number == 2,
            "profile commit feedback was not observed with cancellation");

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
ConfigurationTransactionStatus profile_service_activate_internal(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index) {
    require(activation_attempt_count < activation_attempts.size(),
            "activation attempt fixture overflow");
    activation_attempts[activation_attempt_count++] = {
        transaction_id, identity, profile_index};
    if (activation_busy_attempts != 0) {
        --activation_busy_attempts;
        return ConfigurationTransactionStatus::kBusy;
    }
    return ConfigurationTransactionStatus::kPending;
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
    test_initial_profile_indication_once_per_connection();
    test_default_switching_retry_commit_and_feedback();
    test_identity_promotion_preserves_held_switching();
    test_switching_uses_pre_hotkey_buttons_only();
    test_custom_switching_chord_and_wrap();
    test_switching_slot_isolation();
    test_all_runtime_cancellation_causes();
    test_runtime_slot_synthetic_isolation();
    return 0;
}
