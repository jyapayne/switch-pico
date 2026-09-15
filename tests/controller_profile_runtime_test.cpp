#include "profile/controller_profile_runtime.h"
#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_service.h"
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <cmath>

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
ConfigurationTransactionStatus activation_result =
    ConfigurationTransactionStatus::kPending;
uint8_t last_motion_toggle_slot = 0;
uint32_t last_motion_toggle_connection_generation = 0;
unsigned motion_toggle_count = 0;

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
    activation_result = ConfigurationTransactionStatus::kPending;
    last_motion_toggle_slot = 0;
    last_motion_toggle_connection_generation = 0;
    motion_toggle_count = 0;
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
    initial_profile.macros[0].trigger_mask =
        logical_button_bit(
            ControllerProfileLogicalButton::kLeftShoulder);
    initial_profile.macros[0].first_step = 0;
    initial_profile.macros[0].step_count = 1;
    initial_profile.macro_step_count = 1;
    initial_profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    initial_profile.macro_steps[0].duration_ms = 1000;
    initial_profile.macro_steps[0].output_button_mask =
        logical_button_bit(ControllerProfileLogicalButton::kNorth);

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
    profile->macros[0].trigger_mask =
        logical_button_bit(
            ControllerProfileLogicalButton::kDpadUp);
    profile->macros[0].first_step = 0;
    profile->macros[0].step_count = 1;
    profile->macro_step_count = 1;
    profile->macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile->macro_steps[0].duration_ms = 1000;
    profile->macro_steps[0].output_button_mask =
        logical_button_bit(ControllerProfileLogicalButton::kNorth);
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
    rows[0].active_profile = 7;
    rows[0].profiles[7].switching_chord = kCustomChord;
    snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 10);
    bool event_available = true;
    (void)take_profile_change(0, &event_available);
    require(!event_available,
            "initial profile 8 load published a change event");
    apply_button_mask(kCustomChord, &snapshot);
    (void)runtime_transform(0, snapshot, 11);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 0,
            "profile switching did not wrap profile 8 to profile 1");
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


void test_profile_motion_toggle_supports_trigger_chords() {
    prepare_profiles();
    const uint32_t custom_chord =
        logical_button_bit(ControllerProfileLogicalButton::kSouth) |
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL);
    rows[0].profiles[0].motion_toggle_chord = custom_chord;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0, 9);

    (void)runtime_transform(0, snapshot, 0);
    require(motion_toggle_count == 0,
            "profile refresh toggled motion without a chord");

    apply_button_mask(
        logical_button_bit(ControllerProfileLogicalButton::kSouth),
        &snapshot);
    snapshot.state.left_trigger = UINT16_MAX;
    ControllerProfileTransformResult output =
        runtime_transform(0, snapshot, 1);
    require(motion_toggle_count == 1 &&
                last_motion_toggle_slot == 0 &&
                last_motion_toggle_connection_generation == 9 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.state.left_trigger == 0,
            "trigger-backed motion chord did not toggle and consume inputs");

    (void)runtime_transform(0, snapshot, 2);
    require(motion_toggle_count == 1,
            "held trigger-backed motion chord toggled twice");
    snapshot = make_snapshot(0, 9);
    (void)runtime_transform(0, snapshot, 3);
    apply_button_mask(
        logical_button_bit(ControllerProfileLogicalButton::kSouth),
        &snapshot);
    snapshot.state.left_trigger = UINT16_MAX;
    (void)runtime_transform(0, snapshot, 4);
    require(motion_toggle_count == 2,
            "released trigger-backed motion chord did not re-arm");
}

void configure_synthetic_profile(uint8_t slot) {
    ControllerProfile& profile = rows[slot].profiles[0];
    profile = controller_profile_default(rows[slot].identity, 0);
    profile.macros[0].trigger_mask =
        logical_button_bit(ControllerProfileLogicalButton::kSouth);
    profile.macros[0].cancel_control =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kCapture);
    profile.macros[0].first_step = 0;
    profile.macros[0].step_count = 1;
    profile.macro_step_count = 1;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 1000;
    profile.macro_steps[0].output_button_mask = static_cast<uint16_t>(
        1u << static_cast<uint8_t>(
            ControllerProfileLogicalButton::kNorth));
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

void test_shortcut_arbitration_latching_and_commit_feedback() {
    prepare_profiles();
    ControllerProfile& profile = rows[0].profiles[0];
    constexpr uint16_t modifier = 1u << 9;
    constexpr uint16_t chord = modifier | 1u;
    profile.shortcuts.modifier = 9;
    profile.shortcuts.selectors[2] = 0;
    profile.shortcuts.selectors[3] = 1;
    profile.switching_chord = chord;
    profile.motion_toggle_chord = chord;
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    profile.shift.modifier = 9;
    profile.shift.button_map[2] = 3;
    profile.macros[0] = {chord, CONTROLLER_PROFILE_NO_BUTTON, 0, 1};
    profile.macro_step_count = 1;
    profile.macro_steps[0] = {kControllerProfileOverrideButtons, 100, 8};
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);

    apply_button_mask(chord | 2u, &snapshot);
    auto output = runtime_transform(0, snapshot, 1);
    require(activation_attempt_count == 0 && motion_toggle_count == 0 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "ambiguous direct selectors triggered a lower-priority action");
    apply_button_mask(chord, &snapshot);
    output = runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 0 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "ambiguous chord chose a target when one selector was released");
    apply_button_mask(modifier, &snapshot);
    (void)runtime_transform(0, snapshot, 3);
    apply_button_mask(chord | 4u, &snapshot);
    output = runtime_transform(0, snapshot, 4);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 2 &&
                motion_toggle_count == 0 && output.state.button_west &&
                !output.state.button_north && !output.state.button_south &&
                !output.state.button_capture,
            "direct shortcut did not outrank cycle, motion, Shift and macro");
    bool available = true;
    (void)take_profile_change(0, &available);
    require(!available, "accepted but uncommitted shortcut produced feedback");

    ControllerProfile& committed = rows[0].profiles[2];
    committed.shortcuts.modifier = 9;
    committed.shortcuts.selectors[4] = 0;
    committed.button_map[2] = 3;
    committed.confirmation_policy = ControllerProfileConfirmationPolicy::kLed;
    rows[0].active_profile = 2;
    ++database_generation;
    output = runtime_transform(0, snapshot, 5);
    const auto event = take_profile_change(0, &available);
    require(available && event.active_profile_number == 3 &&
                event.policy == ControllerProfileConfirmationPolicy::kLed &&
                activation_attempt_count == 1 && output.state.button_north &&
                !output.state.button_capture && !output.state.button_south,
            "committed shortcut redirected a held chord or lost commit feedback");
    (void)take_profile_change(0, &available);
    require(!available, "shortcut published duplicate commit feedback");
    apply_button_mask(modifier, &snapshot);
    (void)runtime_transform(0, snapshot, 6);
    activation_result = ConfigurationTransactionStatus::kStorageError;
    apply_button_mask(chord, &snapshot);
    output = runtime_transform(0, snapshot, 7);
    (void)runtime_transform(0, snapshot, 8);
    (void)take_profile_change(0, &available);
    require(activation_attempt_count == 2 &&
                activation_attempts[1].profile_index == 4 && !available &&
                controller_profile_extract_button_mask(output.state) == 0,
            "failed activation retried, leaked its chord, or produced feedback");

    prepare_profiles();
    rows[0].profiles[0].shortcuts.modifier = 9;
    rows[0].profiles[0].shortcuts.selectors[0] = 0;
    snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    apply_button_mask(chord, &snapshot);
    output = runtime_transform(0, snapshot, 1);
    (void)take_profile_change(0, &available);
    require(activation_attempt_count == 0 && !available &&
                controller_profile_extract_button_mask(output.state) == 0,
            "already-active direct target was not a consumed no-op");
}

void test_busy_shortcut_target_and_generation_isolation() {
    prepare_profiles();
    constexpr uint16_t chord = (1u << 9) | 1u;
    rows[0].profiles[0].shortcuts.modifier = 9;
    rows[0].profiles[0].shortcuts.selectors[3] = 0;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    activation_busy_attempts = 3;
    apply_button_mask(chord, &snapshot);
    (void)runtime_transform(0, snapshot, 1);
    rows[0].active_profile = 1;
    rows[0].profiles[1].shortcuts.modifier = 9;
    rows[0].profiles[1].shortcuts.selectors[6] = 0;
    ++database_generation;
    (void)runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 2 &&
                activation_attempts[1].profile_index == 3 &&
                activation_attempts[0].transaction_id ==
                    activation_attempts[1].transaction_id,
            "profile refresh changed a busy shortcut's latched target");
    ++snapshot.connection_generation;
    auto output = runtime_transform(0, snapshot, 3);
    require(activation_attempt_count == 2 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "new connection generation inherited or retriggered held activation");
    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 4);
    apply_button_mask(chord, &snapshot);
    (void)runtime_transform(0, snapshot, 5);
    require(activation_attempt_count == 3 &&
                activation_attempts[2].profile_index == 6 &&
                activation_attempts[2].transaction_id !=
                    activation_attempts[0].transaction_id,
            "new generation could not rearm with its own target");
    output = runtime_transform(0, snapshot, 6, AdapterUsbMode::kXInput);
    (void)runtime_transform(0, snapshot, 7, AdapterUsbMode::kXInput);
    require(activation_attempt_count == 3 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "output-mode reset leaked a pending activation retry");
}

void test_cycle_motion_arbitration_and_held_refresh() {
    prepare_profiles();
    constexpr uint16_t chord = (1u << 9) | 1u;
    rows[0].profiles[0].switching_chord = chord;
    rows[0].profiles[0].motion_toggle_chord = chord;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    apply_button_mask(chord, &snapshot);
    auto output = runtime_transform(0, snapshot, 1);
    require(activation_attempt_count == 1 && motion_toggle_count == 0 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "one cycle chord also toggled motion");
    apply_button_mask(1u, &snapshot);
    (void)runtime_transform(0, snapshot, 2);
    apply_button_mask(chord, &snapshot);
    (void)runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 2 && motion_toggle_count == 0,
            "re-completing a released cycle chord did not rearm");
    apply_button_mask(1u, &snapshot);
    (void)runtime_transform(0, snapshot, 2);
    rows[0].profiles[0].motion_toggle_chord = 1u;
    rows[0].profiles[0].switching_chord = 2u;
    ++database_generation;
    output = runtime_transform(0, snapshot, 3);
    require(motion_toggle_count == 0 && !output.state.button_south,
            "partial held chord became a new action after profile refresh");
    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 4);
    apply_button_mask(1u, &snapshot);
    output = runtime_transform(0, snapshot, 5);
    require(motion_toggle_count == 1 && !output.state.button_south,
            "motion did not rearm after the higher-priority chord released");
    ++database_generation;
    output = runtime_transform(0, snapshot, 6);
    require(motion_toggle_count == 1 && !output.state.button_south,
            "profile refresh phantom-toggled held motion chord");
}

void test_shift_slot_and_context_resets() {
    prepare_profiles();
    std::array<Bluepad32SlotSnapshot, 4> snapshots{};
    constexpr uint8_t outputs[4] = {1, 2, 3, 12};
    for (uint8_t slot = 0; slot < 4; ++slot) {
        auto& profile = rows[slot].profiles[0];
        profile.shift.mode = ControllerProfileShiftMode::kToggle;
        profile.shift.modifier = 9;
        profile.shift.button_map[0] = outputs[slot];
        snapshots[slot] = make_snapshot(slot);
        (void)runtime_transform(slot, snapshots[slot], 0);
        apply_button_mask((1u << 9) | 1u, &snapshots[slot]);
        const auto output = runtime_transform(slot, snapshots[slot], 1);
        require(controller_profile_extract_button_mask(output.state) ==
                    (1u << outputs[slot]),
                "Shift activation was not physical, consumed and slot-local");
    }
    ++snapshots[0].connection_generation;
    auto output = runtime_transform(0, snapshots[0], 2);
    require(output.state.button_south && !output.state.button_capture,
            "connection reset phantom-toggled held Shift");
    output = runtime_transform(1, snapshots[1], 2);
    require(output.state.button_west && !output.state.button_south,
            "one slot's Shift reset disturbed another slot");
    output = runtime_transform(1, snapshots[1], 3, AdapterUsbMode::kXInput);
    require(output.state.button_south && !output.state.button_west,
            "mode reset retained or retriggered toggle Shift");
    rows[2].profiles[1].shift = rows[2].profiles[0].shift;
    rows[2].active_profile = 1;
    ++database_generation;
    output = runtime_transform(2, snapshots[2], 4);
    require(output.state.button_south && !output.state.button_north &&
                !output.state.button_capture,
            "profile activation retained or phantom-toggled Shift");
    output = runtime_transform(3, snapshots[3], 4);
    require(output.state.button_south && !output.state.dpad_up,
            "database refresh did not reset the remaining Shift slot");
    apply_button_mask(1u, &snapshots[3]);
    (void)runtime_transform(3, snapshots[3], 5);
    apply_button_mask((1u << 9) | 1u, &snapshots[3]);
    output = runtime_transform(3, snapshots[3], 6);
    require(output.state.dpad_up && !output.state.button_south,
            "Shift could not rearm after reset and physical release");
    ++configuration_reset_generation;
    output = runtime_transform(3, snapshots[3], 7);
    require(output.state.button_south && !output.state.dpad_up,
            "configuration reset retained Shift toggle state");
}

void test_held_synthetic_sources_and_disconnect_rearming() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.macros[0] = {1u, 9, 0, 1};
    profile.macros[0].mode = ControllerProfileMacroMode::kToggle;
    profile.macro_step_count = 1;
    profile.macro_steps[0].override_flags = kControllerProfileOverrideLeftStick;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].left_stick_x = 12345;
    profile.turbo_modes[1] = ControllerProfileTurboMode::kBurst;
    profile.turbo_modes[2] = ControllerProfileTurboMode::kAutoBurst;
    profile.shortcuts.modifier = 9;
    profile.shortcuts.selectors[2] = 3;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    apply_button_mask(7u, &snapshot);
    auto output = runtime_transform(0, snapshot, 1);
    require(output.state.left_stick_x == 12345 &&
                output.state.button_east && output.state.button_west &&
                !output.state.button_south,
            "independent looping macro and Burst sources did not start");
    ++database_generation;
    output = runtime_transform(0, snapshot, 2);
    require(output.state.left_stick_x == 0 && output.state.button_south &&
                !output.state.button_east && !output.state.button_west,
            "refresh phantom-restarted a held edge-triggered synthetic source");
    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 3);
    apply_button_mask(7u, &snapshot);
    output = runtime_transform(0, snapshot, 4);
    require(output.state.left_stick_x == 12345 &&
                output.state.button_east && output.state.button_west,
            "synthetic sources could not rearm after refresh and release");
    snapshot.active = false;
    (void)runtime_transform(0, snapshot, 5);
    snapshot.active = true;
    apply_button_mask((1u << 9) | 8u, &snapshot);
    output = runtime_transform(0, snapshot, 6);
    require(activation_attempt_count == 0 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.state.left_stick_x == 0,
            "reconnect treated an already-held shortcut as a new command");
    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 7);
    apply_button_mask((1u << 9) | 8u, &snapshot);
    (void)runtime_transform(0, snapshot, 8);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 2,
            "reconnected shortcut did not rearm on physical release");
}

void test_shortcut_selector_rollover_without_modifier_release() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.shortcuts.modifier = 9;
    profile.shortcuts.selectors[1] = 0;
    profile.shortcuts.selectors[2] = 1;
    auto snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    apply_button_mask((1u << 9) | 1u, &snapshot);
    (void)runtime_transform(0, snapshot, 1);
    apply_button_mask((1u << 9) | 2u, &snapshot);
    const auto output = runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 2 &&
                activation_attempts[0].profile_index == 1 &&
                activation_attempts[1].profile_index == 2 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "swapping shortcut selectors while holding the modifier lost the new action");
}

void test_extra_hotkeys_consume_mappings_and_rearm() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kLeftSolo;
    profile.switching_chord = (1u << 18) | (1u << 24);
    profile.motion_toggle_chord = (1u << 19) | (1u << 16);
    profile.shortcuts.modifier = 20;
    profile.shortcuts.selectors[7] = 0;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.extra_button_map[1] = 1;
    profile.extra_button_map[2] = 2;
    profile.extra_button_map[6] = CONTROLLER_PROFILE_LEFT_SR_OUTPUT;
    profile.triggers[0].output = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    Bluepad32SlotSnapshot snapshot = make_snapshot(0, 9);
    (void)runtime_transform(0, snapshot, 0);
    snapshot.state.extra_buttons = 1;
    auto output = runtime_transform(0, snapshot, 1);
    require(activation_attempt_count == 0 && output.state.extra_buttons == 0x08 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "partial extra switching chord lost its rail route or active layout");
    snapshot.state.extra_buttons = 0x41;
    output = runtime_transform(0, snapshot, 2);
    require(activation_attempt_count == 1 &&
                activation_attempts[0].profile_index == 1 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.state.right_trigger == 0 && output.state.extra_buttons == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "extra switching chord leaked rails or lost the active layout while suppressed");
    (void)runtime_transform(0, snapshot, 3);
    require(activation_attempt_count == 1, "held extra switching chord retriggered");
    snapshot.state = controller_neutral_state();
    (void)runtime_transform(0, snapshot, 4);
    snapshot.state.extra_buttons = 2;
    snapshot.state.left_trigger = UINT16_MAX;
    output = runtime_transform(0, snapshot, 5);
    require(motion_toggle_count == 1 && last_motion_toggle_slot == 0 &&
                last_motion_toggle_connection_generation == 9 &&
                !output.state.button_east && output.state.left_trigger == 0 &&
                output.state.extra_buttons == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "extra/trigger motion chord leaked a mapped rail or lost slot/layout routing");
    (void)runtime_transform(0, snapshot, 6);
    require(motion_toggle_count == 1, "held extra motion chord retriggered");
    snapshot.state = controller_neutral_state();
    (void)runtime_transform(0, snapshot, 7);
    apply_button_mask(1, &snapshot);
    snapshot.state.extra_buttons = 4;
    output = runtime_transform(0, snapshot, 8);
    require(activation_attempt_count == 2 &&
                activation_attempts[1].profile_index == 7 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "extra shortcut modifier failed to select profile eight or leaked inputs");
    apply_button_mask(0, &snapshot);
    (void)runtime_transform(0, snapshot, 9);
    snapshot.state.extra_buttons = 0x41;
    (void)runtime_transform(0, snapshot, 10);
    require(activation_attempt_count == 3 &&
                activation_attempts[2].profile_index == 1,
            "released extra switching chord did not rearm");
}

void test_shift_rails_reset_with_live_profile_layout_changes() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kRightSolo;
    profile.button_map[0] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_SR_OUTPUT;
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    profile.shift.modifier = 9;
    profile.shift.button_map[0] = CONTROLLER_PROFILE_RIGHT_SL_OUTPUT;
    profile.shift.extra_button_map[0] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.shift.button_map[9] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    auto snapshot = make_snapshot(0);
    auto output = runtime_transform(0, snapshot, 0);
    require(output.state.extra_buttons == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kRightSolo,
            "neutral active input lost its solo layout or created a rail press");

    apply_button_mask(1u | (1u << 9), &snapshot);
    snapshot.state.extra_buttons = 1;
    output = runtime_transform(0, snapshot, 1);
    require(output.state.extra_buttons == 0x60 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "Shift failed to route both ordinary and extra sources or leaked its modifier");
    apply_button_mask(1u, &snapshot);
    snapshot.state.extra_buttons = 1;
    output = runtime_transform(0, snapshot, 2);
    require(output.state.extra_buttons == 0x60,
            "toggle Shift dropped its rail routes when the modifier was released");
    ++configuration_reset_generation;
    output = runtime_transform(0, snapshot, 3);
    require(output.state.extra_buttons == 0x18 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kRightSolo,
            "configuration reset left stale Shift rails or reset profile-owned layout");
    apply_button_mask(1u | (1u << 9), &snapshot);
    snapshot.state.extra_buttons = 1;
    output = runtime_transform(0, snapshot, 4);
    require(output.state.extra_buttons == 0x60,
            "Shift rail routes did not rearm after reset");

    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kLeftSolo;
    profile.button_map[0] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_NO_BUTTON;
    ++database_generation;
    output = runtime_transform(0, snapshot, 5);
    require(output.state.extra_buttons == 0x40 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "same-profile live edit retained old Shift rails or stale native layout");
    rows[0].active_profile = 1;
    ++database_generation;
    output = runtime_transform(0, snapshot, 6);
    require(output.state.extra_buttons == 0 && output.state.button_south &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kPaired,
            "activating a normal profile retained the previous profile's rails or layout");

    snapshot.active = false;
    output = runtime_transform(0, snapshot, 7);
    require(output.state.extra_buttons == 0 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kPaired,
            "disconnect retained mapped rails or active-only routing metadata");
}

void test_stick_swap_precedes_final_macro_overrides_in_all_output_modes() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.swap_sticks = true;
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kRightSolo;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.macros[0] = {1u, CONTROLLER_PROFILE_NO_BUTTON, 0, 1};
    profile.macro_step_count = 1;
    auto& step = profile.macro_steps[0];
    step.override_flags =
        kControllerProfileOverrideButtons | kControllerProfileOverrideLeftStick;
    step.output_button_mask =
        logical_button_bit(ControllerProfileLogicalButton::kLeftStick);
    step.left_stick_x = 1234;
    step.left_stick_y = -5678;
    step.duration_ms = 10;
    auto snapshot = make_snapshot(0);
    snapshot.state.left_stick_x = 111;
    snapshot.state.left_stick_y = -222;
    snapshot.state.right_stick_x = 333;
    snapshot.state.right_stick_y = -444;
    snapshot.state.button_left_stick = true;
    snapshot.state.extra_buttons = 1;
    auto output = runtime_transform(0, snapshot, 0, AdapterUsbMode::kXInput);
    require(output.state.left_stick_x == 333 && output.state.left_stick_y == -444 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == -222 &&
                !output.state.button_left_stick && output.state.button_right_stick,
            "stick axes and clicks were not swapped for a non-native output mode");
    snapshot.state.button_south = true;
    output = runtime_transform(0, snapshot, 1, AdapterUsbMode::kXInput);
    require(output.state.left_stick_x == 1234 && output.state.left_stick_y == -5678 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == -222 &&
                controller_profile_extract_button_mask(output.state) ==
                    logical_button_bit(ControllerProfileLogicalButton::kLeftStick) &&
                output.state.extra_buttons == 0x40,
            "stick swap moved final macro outputs or broadened its standard-button override");
    output = runtime_transform(0, snapshot, 11, AdapterUsbMode::kXInput);
    require(output.state.left_stick_x == 333 && output.state.left_stick_y == -444 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == -222 &&
                !output.state.button_left_stick && output.state.button_right_stick,
            "macro completion failed to restore the live swapped sticks and clicks");

    profile.swap_sticks = false;
    ++database_generation;
    output = runtime_transform(0, snapshot, 12, AdapterUsbMode::kXInput);
    require(output.state.left_stick_x == 111 && output.state.left_stick_y == -222 &&
                output.state.right_stick_x == 333 && output.state.right_stick_y == -444 &&
                output.state.button_left_stick && !output.state.button_right_stick,
            "live swap edit retained old output pairs or restarted a held macro");
}

void test_shift_directions_select_maps_and_consume_modifiers() {
    for (const auto mode : {ControllerProfileShiftMode::kHold,
                            ControllerProfileShiftMode::kToggle}) {
        prepare_profiles();
        auto& profile = rows[0].profiles[0];
        profile.button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
        profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_DOWN_OUTPUT;
        profile.shift.mode = mode;
        profile.shift.modifier = 9;
        profile.button_map[9] = CONTROLLER_PROFILE_LEFT_STICK_LEFT_OUTPUT;
        profile.shift.button_map[9] = CONTROLLER_PROFILE_LEFT_STICK_LEFT_OUTPUT;
        profile.shift.button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
        profile.shift.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
        auto snapshot = make_snapshot(0);
        (void)runtime_transform(0, snapshot, 0);
        apply_button_mask(1u, &snapshot);
        auto output = runtime_transform(0, snapshot, 1);
        require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767,
                "inactive Shift did not use the base direction map");
        apply_button_mask(1u | (1u << 9), &snapshot);
        snapshot.state.extra_buttons = 1;
        output = runtime_transform(0, snapshot, 2);
        require(output.state.left_stick_x == 32767 && output.state.left_stick_y == 0 &&
                    output.state.extra_buttons == 0 &&
                    controller_profile_extract_button_mask(output.state) == 0,
                "Shift did not OR ordinary and extra directions or leaked its consumed modifier");
        apply_button_mask(1u << 9, &snapshot);
        snapshot.state.extra_buttons = 1;
        output = runtime_transform(0, snapshot, 3);
        require(output.state.left_stick_x == 32767 && output.state.left_stick_y == 0,
                "releasing the ordinary source cleared a held Shift extra direction");
        apply_button_mask(0, &snapshot);
        snapshot.state.extra_buttons = 1;
        output = runtime_transform(0, snapshot, 4);
        const bool toggled = mode == ControllerProfileShiftMode::kToggle;
        require(output.state.left_stick_x == (toggled ? 32767 : 0) &&
                    output.state.left_stick_y == (toggled ? 0 : 32767),
                "modifier release failed to retain toggle Shift or restore hold Shift's base map");
        snapshot.state.extra_buttons = 0;
        output = runtime_transform(0, snapshot, 5);
        require(output.state.left_stick_x == 0 && output.state.left_stick_y == 0,
                "Shift retained a direction after its last source released");
    }
}

void test_consumed_direction_sources_and_live_layout_changes() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kRightSolo;
    profile.motion_toggle_chord = (1u << 18) | (1u << 16);
    profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
    profile.triggers[0].output = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
    profile.triggers[0].digital_threshold = 12345;
    profile.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_LEFT_OUTPUT;
    auto snapshot = make_snapshot(0);
    (void)runtime_transform(0, snapshot, 0);
    snapshot.state.extra_buttons = 1;
    auto output = runtime_transform(0, snapshot, 1);
    require(output.state.left_stick_x == 32767 && output.state.left_stick_y == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kRightSolo,
            "partial extra hotkey lost its direction or native-layout metadata");
    apply_button_mask(1u << 12, &snapshot);
    snapshot.state.extra_buttons = 1;
    snapshot.state.left_trigger = 12345;
    output = runtime_transform(0, snapshot, 2);
    require(motion_toggle_count == 1 &&
                output.state.left_stick_x == -32767 && output.state.left_stick_y == 0 &&
                output.state.left_trigger == 0 && output.state.extra_buttons == 0 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kRightSolo,
            "consumed extra/trigger hotkey leaked directions or suppressed unrelated movement/layout");
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kLeftSolo;
    profile.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_DOWN_OUTPUT;
    ++database_generation;
    output = runtime_transform(0, snapshot, 3);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == 32767 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "live profile edit retained an old direction or native layout");
    snapshot.active = false;
    output = runtime_transform(0, snapshot, 4);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kPaired,
            "disconnect retained digital movement or active-only native-layout metadata");
}

void test_direction_fallback_precedes_final_macro_override() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.swap_sticks = true;
    profile.native_joycon_layout = ControllerProfileNativeJoyconLayout::kLeftSolo;
    profile.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
    profile.macros[0] = {1u, CONTROLLER_PROFILE_NO_BUTTON, 0, 1};
    profile.macro_step_count = 1;
    auto& step = profile.macro_steps[0];
    step.override_flags = kControllerProfileOverrideLeftStick;
    step.left_stick_x = 1234;
    step.left_stick_y = -5678;
    step.duration_ms = 10;
    auto snapshot = make_snapshot(0);
    snapshot.state.left_stick_x = 111;
    snapshot.state.dpad_up = true;
    auto output = runtime_transform(0, snapshot, 0);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == 0,
            "runtime applied direction fallback before the physical stick swap");
    snapshot.state.button_south = true;
    output = runtime_transform(0, snapshot, 1);
    require(output.state.left_stick_x == 1234 && output.state.left_stick_y == -5678 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == 0 &&
                output.native_joycon_layout == ControllerProfileNativeJoyconLayout::kLeftSolo,
            "digital directions replaced final macro output or lost native-layout metadata");
    output = runtime_transform(0, snapshot, 11);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767 &&
                output.state.right_stick_x == 111 && output.state.right_stick_y == 0,
            "macro completion failed to restore a still-held mapped direction");
}

void test_accelerometer_swing_requires_evidence_and_settle() {
    WiiSwingDetector detector;
    WiiAccelerometerSample sample{};
    sample.valid = true;
    uint32_t now = UINT32_MAX - 100u;
    auto feed = [&](int16_t x, int16_t y, int16_t z) {
        now += 10;
        sample = {x, y, z, sample.sequence + 1, now, true};
        return detector.update(sample, now, 1, true);
    };
    for (int i = 0; i < 20; ++i)
        require(!feed(0, 0, 4096), "resting gravity armed a swing output");
    for (int degrees = 0; degrees <= 90; degrees += 3) {
        const double angle = degrees * 0.017453292519943;
        require(!feed(static_cast<int16_t>(4096 * std::sin(angle)), 0,
                      static_cast<int16_t>(4096 * std::cos(angle))),
                "ordinary rotation of gravity triggered a sword swing");
    }
    for (int i = 0; i < 40; ++i) feed(4096, 0, 0);
    require(!feed(4096, 10000, 0), "a single acceleration spike triggered a swing");
    for (int i = 0; i < 5; ++i) {
        now += 2;
        require(!detector.update(sample, now, 1, true),
                "repeated reads of one sample accumulated swing evidence");
    }
    require(feed(4096, 10000, 0), "sustained acceleration did not produce a swing");
    for (int i = 0; i < 50; ++i) {
        const bool pressed = feed(4096, 10000, 0);
        if (i >= 8) require(!pressed, "continuous shaking retriggered without settling");
    }
    for (int i = 0; i < 35; ++i)
        require(!feed(4096, 0, 0), "settling generated a second button pulse");
    require(!feed(4096, -10000, 0) && feed(4096, -10000, 0),
            "settled detector did not accept an opposite-direction swing");
    require(!detector.update(sample, now + 151, 1, true),
            "stale acceleration retained a button press");
}

void test_swing_output_isolated_from_motion_remaps_and_profiles() {
    prepare_profiles();
    rows[0].profiles[0].swing.button = 2;  // Final logical west / Switch Y.
    rows[0].profiles[0].button_map[2] = 1;
    rows[1].profiles[0].swing.button = 2;
    auto snapshot = make_snapshot(0);
    snapshot.state.button_south = true;
    snapshot.pre_hotkey_button_mask = 1;
    uint32_t now = 0;
    auto feed = [&](int16_t x) {
        now += 10;
        snapshot.accelerometer = {x, 0, 4096, snapshot.accelerometer.sequence + 1, now, true};
        return runtime_transform(0, snapshot, now);
    };
    for (int i = 0; i < 20; ++i) feed(0);
    feed(10000);
    auto output = feed(10000);
    require(output.state.button_west && output.state.button_south &&
                !output.state.button_east && output.state.motion_sample_count == 0,
            "accelerometer swing required gyro output, remapped its target, or lost physical input");
    require(!runtime_transform(1, make_snapshot(1), now).state.button_west,
            "a swing leaked to another controller slot");
    snapshot.state.button_west = true;
    rows[0].profiles[0].button_map[2] = 2;
    ++database_generation;
    output = runtime_transform(0, snapshot, now + 1);
    require(output.state.button_west, "gesture cancellation released a physical target button");
    snapshot.state.button_west = false;
    rows[0].profiles[0].swing.button = 3;
    ++database_generation;
    output = runtime_transform(0, snapshot, now + 2);
    require(!output.state.button_west && !output.state.button_north,
            "profile refresh replayed a swing onto old or new target");
}

void test_swing_modifier_release_cancels_and_requires_fresh_settle() {
    prepare_profiles();
    rows[0].profiles[0].swing = {2, 1, 0};
    auto snapshot = make_snapshot(0);
    uint32_t now = 0;
    auto feed = [&](int16_t x, bool held) {
        now += 10;
        apply_button_mask(held ? 1 : 0, &snapshot);
        snapshot.accelerometer = {x, 0, 4096, snapshot.accelerometer.sequence + 1, now, true};
        return runtime_transform(0, snapshot, now);
    };
    for (int i = 0; i < 20; ++i) feed(0, true);
    feed(10000, true);
    require(feed(10000, true).state.button_west, "held modifier failed to allow a swing");
    require(!feed(10000, false).state.button_west, "modifier release retained swing output");
    require(!feed(10000, true).state.button_west, "repressing modifier during motion retriggered");
    for (int i = 0; i < 60; ++i) feed(0, true);
    feed(10000, true);
    require(feed(10000, true).state.button_west, "settling after modifier release did not rearm");
    snapshot.active = false;
    require(!runtime_transform(0, snapshot, now + 1).state.button_west,
            "disconnect retained gesture output");
}

void test_back_and_forth_swings_do_not_require_full_stops() {
    prepare_profiles();
    rows[0].profiles[0].swing.button = 2;
    auto snapshot = make_snapshot(0);
    uint32_t now = UINT32_MAX - 100u;
    unsigned presses = 0;
    bool held = false;
    auto feed = [&](int16_t force) {
        now += 10;
        snapshot.accelerometer = {force, 0, 4096, snapshot.accelerometer.sequence + 1, now, true};
        const bool pressed = runtime_transform(0, snapshot, now).state.button_west;
        if (pressed && !held) ++presses;
        held = pressed;
    };
    for (int i = 0; i < 20; ++i) feed(0);
    for (int stroke = 0; stroke < 6; ++stroke) {
        const int direction = stroke % 2 == 0 ? 1 : -1;
        for (int i = 0; i < 24; ++i) feed(direction * 10000);
        // An 80ms lower-force transition, not a 120ms full stop.
        for (int i = 0; i < 8; ++i) feed(direction * 2000);
        require(presses == static_cast<unsigned>(stroke + 1),
                "each back-and-forth stroke must press once without coming to rest");
    }
}

void test_early_rebound_cannot_become_a_delayed_swing() {
    WiiSwingDetector detector;
    WiiAccelerometerSample sample{};
    uint32_t now = 0;
    unsigned presses = 0;
    bool held = false;
    auto feed = [&](int16_t force) {
        now += 10;
        sample = {force, 0, 4096, sample.sequence + 1, now, true};
        const bool pressed = detector.update(sample, now, 1, true);
        if (pressed && !held) ++presses;
        held = pressed;
    };
    for (int i = 0; i < 20; ++i) feed(0);
    for (int i = 0; i < 3; ++i) feed(10000);
    require(presses == 1, "initial swing must trigger");
    for (int i = 0; i < 5; ++i) feed(0);
    for (int i = 0; i < 40; ++i) feed(-10000);
    require(presses == 1,
            "early rebound must not fire after cooldown while its force remains high");
    for (int i = 0; i < 5; ++i) feed(0);
    for (int i = 0; i < 3; ++i) feed(10000);
    require(presses == 2, "a new stroke after cooldown and release must trigger");
}

void test_combined_swings_have_priority_at_window_boundary() {
    for (bool nunchuk_first : {false, true}) {
        for (unsigned delay : {0u, 100u, 110u}) {
            WiiSwingGestures gestures;
            uint32_t now = UINT32_MAX - 100u;
            uint32_t sequence = 0;
            unsigned counts[3]{};
            auto feed = [&](int16_t first, int16_t second) {
                now += 10;
                ++sequence;
                WiiAccelerometerSample remote{
                    nunchuk_first ? second : first, 0, 4096, sequence, now, true};
                WiiAccelerometerSample nunchuk{
                    nunchuk_first ? first : second, 0, 4096, sequence, now, true};
                auto result = gestures.update(remote, nunchuk, now, 1, 1, 7, 100);
                for (unsigned i = 0; i < 3; ++i)
                    if (result.started & (1u << i)) ++counts[i];
                return result;
            };
            for (unsigned i = 0; i < 20; ++i) feed(0, 0);
            for (unsigned step = 0; step < 40; ++step) {
                auto result = feed(10000, step * 10 >= delay ? 10000 : 0);
                if (delay <= 100)
                    require((result.active & 3u) == 0, "combined gesture leaked individual buttons");
            }
            if (delay <= 100)
                require(counts[2] == 1 && counts[0] == 0 && counts[1] == 0,
                        "simultaneous/in-window strokes must start only the combined action");
            else
                require(counts[2] == 0 && counts[0] == 1 && counts[1] == 1,
                        "out-of-window strokes must remain two individual actions");
        }
    }
}

void test_combined_pending_cancels_on_detach_and_modifier_release() {
    for (bool detach : {false, true}) {
        WiiSwingGestures gestures;
        uint32_t now = 0, sequence = 0;
        auto feed = [&](int16_t force, uint8_t allowed, bool nunchuk_valid) {
            now += 10;
            ++sequence;
            WiiAccelerometerSample remote{force, 0, 4096, sequence, now, true};
            WiiAccelerometerSample nunchuk{0, 0, 4096, sequence, now, nunchuk_valid};
            return gestures.update(remote, nunchuk, now, 1, 1, allowed, 100);
        };
        for (unsigned i = 0; i < 20; ++i) feed(0, 7, true);
        feed(10000, 7, true);
        require(feed(10000, 7, true).started == 0, "single must wait while combined is eligible");
        for (unsigned i = 0; i < 20; ++i)
            require(feed(10000, detach ? 7 : 3, !detach).started == 0,
                    "detaching or releasing combined modifier replayed a pending action");
    }
    WiiSwingGestures remote_only;
    WiiAccelerometerSample missing{};
    WiiAccelerometerSample sample{0, 0, 4096, 0, 0, true};
    for (unsigned i = 0; i < 20; ++i) {
        sample.timestamp_ms += 10;
        ++sample.sequence;
        remote_only.update(sample, missing, sample.timestamp_ms, 1, 1, 7, 100);
    }
    sample.x = 10000;
    ++sample.sequence;
    sample.timestamp_ms += 10;
    remote_only.update(sample, missing, sample.timestamp_ms, 1, 1, 7, 100);
    ++sample.sequence;
    sample.timestamp_ms += 10;
    require(remote_only.update(sample, missing, sample.timestamp_ms, 1, 1, 7, 100).started == 1,
            "absent Nunchuk must not delay the Remote gesture");
}

void test_nunchuk_button_and_combined_macro_runtime() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.swing.button = 2;
    profile.nunchuk_swing.button = 1;
    profile.combined_swing.macro = 0;
    profile.macros[0].step_count = 1;
    profile.macros[0].mode = ControllerProfileMacroMode::kWhileHeld;
    profile.macros[0].trigger_mask = 1;  // Not held during the combined gesture.
    profile.macro_step_count = 1;
    profile.macro_steps[0].override_flags = kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 100;
    profile.macro_steps[0].output_button_mask = 8;
    for (unsigned i = 1; i < CONTROLLER_PROFILE_MACRO_COUNT; ++i)
        profile.macros[i].first_step = 1;
    auto snapshot = make_snapshot(0);
    uint32_t now = 0, sequence = 0;
    auto feed = [&](int16_t remote, int16_t nunchuk) {
        now += 10;
        ++sequence;
        snapshot.accelerometer = {remote, 0, 4096, sequence, now, true};
        snapshot.nunchuk_accelerometer = {nunchuk, 0, 4096, sequence, now, true};
        return runtime_transform(0, snapshot, now);
    };
    for (unsigned i = 0; i < 20; ++i) feed(0, 0);
    feed(10000, 10000);
    auto result = feed(10000, 10000);
    require(result.state.button_north && !result.state.button_west && !result.state.button_east,
            "combined gesture must start its macro instead of both individual buttons");
    for (unsigned i = 0; i < 8; ++i)
        require(feed(0, 0).state.button_north, "gesture macro stopped when physical trigger was absent");
    feed(0, 0);
    require(!feed(0, 0).state.button_north, "gesture macro must finish exactly one cycle");
    for (unsigned i = 0; i < 30; ++i) feed(0, 0);
    feed(0, 10000);
    require(!feed(0, 10000).state.button_east, "Nunchuk alone must wait for combination window");
    for (unsigned i = 0; i < 10; ++i) feed(0, 10000);
    result = feed(0, 10000);
    require(result.state.button_east && !result.state.button_west && !result.state.button_north,
            "Nunchuk-only gesture must produce its configured button after the window");
}

void test_gesture_macros_rearm_during_continuous_swings() {
    prepare_profiles();
    auto& profile = rows[0].profiles[0];
    profile.swing.macro = 0;
    profile.macros[0].step_count = 1;
    profile.macros[0].mode = ControllerProfileMacroMode::kRepeat;
    profile.macros[0].repeat_count = 5;
    profile.macro_step_count = 1;
    profile.macro_steps[0].override_flags = kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 100;
    profile.macro_steps[0].output_button_mask = 8;
    auto snapshot = make_snapshot(0);
    uint32_t now = 0;
    unsigned starts = 0;
    bool held = false;
    auto feed = [&](int16_t force) {
        now += 10;
        snapshot.accelerometer = {force, 0, 4096, snapshot.accelerometer.sequence + 1, now, true};
        const bool pressed = runtime_transform(0, snapshot, now).state.button_north;
        if (pressed && !held) ++starts;
        held = pressed;
    };
    for (unsigned i = 0; i < 20; ++i) feed(0);
    for (unsigned stroke = 0; stroke < 4; ++stroke) {
        const int direction = stroke % 2 ? -1 : 1;
        for (unsigned i = 0; i < 24; ++i) feed(direction * 10000);
        for (unsigned i = 0; i < 8; ++i) feed(direction * 2000);
        require(starts == stroke + 1 && !held,
                "gesture macro must play once per stroke without requiring a full stop");
    }
}

struct CombinedConfirmationRig {
    WiiSwingGestures gestures;
    uint32_t now = 1000, sequence = 0;
    unsigned counts[3]{};
    CombinedConfirmationRig() {
        for (unsigned i = 0; i < 20; ++i) feed(0, 0);
    }
    WiiSwingGestureResult feed(int16_t remote_force, int16_t nunchuk_force,
                               uint8_t allowed = 7) {
        now += 10;
        ++sequence;
        WiiAccelerometerSample remote{remote_force, 0, 4096, sequence, now, true};
        WiiAccelerometerSample nunchuk{nunchuk_force, 0, 4096, sequence, now, true};
        const auto result = gestures.update(remote, nunchuk, now, 1, 1, allowed, 100);
        for (unsigned i = 0; i < 3; ++i)
            if (result.started & (1u << i)) ++counts[i];
        return result;
    }
};

void test_combined_accepts_smaller_motion_before_or_after_full_swing() {
    for (bool weak_nunchuk : {false, true}) {
        for (int delay : {-60, 0, 60}) {
            CombinedConfirmationRig rig;
            for (int step = 0; step < 40; ++step) {
                const int strong_start = delay < 0 ? -delay : 0;
                const int weak_start = delay > 0 ? delay : 0;
                const int16_t strong = step * 10 >= strong_start ? 10000 : 0;
                const int16_t weak = step * 10 >= weak_start ? 4500 : 0;
                const auto result = rig.feed(weak_nunchuk ? strong : weak,
                                             weak_nunchuk ? weak : strong);
                require((result.active & 3) == 0, "confirmed combined motion leaked a single action");
            }
            require(rig.counts[2] == 1 && rig.counts[0] == 0 && rig.counts[1] == 0,
                    "one full swing plus smaller companion motion must combine in either order");
        }
    }
}

void test_confirmation_does_not_weaken_individuals_or_accept_noise() {
    for (uint8_t allowed : {uint8_t{3}, uint8_t{7}}) {
        CombinedConfirmationRig rig;
        for (unsigned i = 0; i < 40; ++i) rig.feed(4500, 4500, allowed);
        require(rig.counts[0] == 0 && rig.counts[1] == 0 && rig.counts[2] == 0,
                "two subthreshold movements must not generate any action");
    }
    for (bool spike : {false, true}) {
        CombinedConfirmationRig rig;
        for (unsigned i = 0; i < 40; ++i)
            rig.feed(10000, spike && i == 2 ? 4500 : 0);
        require(rig.counts[0] == 1 && rig.counts[1] == 0 && rig.counts[2] == 0,
                "a stationary companion or one noisy sample must not confirm a combined swing");
    }
}

void test_consumed_confirmation_suppresses_late_full_swing() {
    CombinedConfirmationRig rig;
    for (unsigned i = 0; i < 45; ++i)
        rig.feed(10000, i < 8 ? 4500 : 10000);
    require(rig.counts[2] == 1 && rig.counts[0] == 0 && rig.counts[1] == 0,
            "a confirmation growing into a full swing must not leak a later individual action");
}

void test_expired_or_discarded_confirmation_cannot_be_reused() {
    for (bool discard : {false, true}) {
        CombinedConfirmationRig rig;
        for (unsigned i = 0; i < 3; ++i) rig.feed(0, 4500);
        if (discard) rig.gestures.discard_actions();
        else for (unsigned i = 0; i < 12; ++i) rig.feed(0, 0);
        for (unsigned i = 0; i < 40; ++i) rig.feed(10000, 0);
        require(rig.counts[0] == 1 && rig.counts[1] == 0 && rig.counts[2] == 0,
                "expired or macro-discarded evidence must not confirm a later swing");
    }
}

}  // namespace

bool bluepad32_input_backend_toggle_motion(
    uint8_t slot, uint32_t connection_generation) {
    last_motion_toggle_slot = slot;
    last_motion_toggle_connection_generation = connection_generation;
    ++motion_toggle_count;
    return true;
}

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
    return activation_result;
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
    test_profile_motion_toggle_supports_trigger_chords();
    test_switching_slot_isolation();
    test_all_runtime_cancellation_causes();
    test_runtime_slot_synthetic_isolation();
    test_shortcut_arbitration_latching_and_commit_feedback();
    test_busy_shortcut_target_and_generation_isolation();
    test_cycle_motion_arbitration_and_held_refresh();
    test_shift_slot_and_context_resets();
    test_held_synthetic_sources_and_disconnect_rearming();
    test_shortcut_selector_rollover_without_modifier_release();
    test_extra_hotkeys_consume_mappings_and_rearm();
    test_shift_rails_reset_with_live_profile_layout_changes();
    test_stick_swap_precedes_final_macro_overrides_in_all_output_modes();
    test_shift_directions_select_maps_and_consume_modifiers();
    test_consumed_direction_sources_and_live_layout_changes();
    test_direction_fallback_precedes_final_macro_override();
    test_accelerometer_swing_requires_evidence_and_settle();
    test_swing_output_isolated_from_motion_remaps_and_profiles();
    test_swing_modifier_release_cancels_and_requires_fresh_settle();
    test_back_and_forth_swings_do_not_require_full_stops();
    test_early_rebound_cannot_become_a_delayed_swing();
    test_combined_swings_have_priority_at_window_boundary();
    test_combined_pending_cancels_on_detach_and_modifier_release();
    test_nunchuk_button_and_combined_macro_runtime();
    test_gesture_macros_rearm_during_continuous_swings();
    test_combined_accepts_smaller_motion_before_or_after_full_swing();
    test_confirmation_does_not_weaken_individuals_or_accept_noise();
    test_consumed_confirmation_suppresses_late_full_swing();
    test_expired_or_discarded_confirmation_cannot_be_reused();
    return 0;
}
