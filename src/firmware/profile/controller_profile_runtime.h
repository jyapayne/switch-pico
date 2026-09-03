#pragma once

#include <stdint.h>

#include "adapter/adapter_usb_mode.h"

#include "input/bluepad32_input_backend.h"
#include "profile/controller_profile_transform.h"

constexpr uint8_t CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT = 4;
constexpr uint16_t CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD =
    static_cast<uint16_t>(
        (1u << static_cast<uint8_t>(
             ControllerProfileLogicalButton::kLeftShoulder)) |
        (1u << static_cast<uint8_t>(
             ControllerProfileLogicalButton::kRightShoulder)) |
        (1u << static_cast<uint8_t>(
             ControllerProfileLogicalButton::kSelect)) |
        (1u << static_cast<uint8_t>(
             ControllerProfileLogicalButton::kStart)));

struct ControllerProfileRuntimeProfileChangeEvent {
    uint32_t connection_generation = 0;
    uint32_t database_generation = 0;
    uint8_t active_profile_number = 0;
    ControllerProfileConfirmationPolicy policy =
        ControllerProfileConfirmationPolicy::kNone;
};

// Take the one-shot LED-only indication for the first committed profile
// resolved on a connection. Identity promotion and database refresh do not
// publish it again.
bool controller_profile_runtime_take_initial_profile_indication(
    uint8_t slot, ControllerProfileRuntimeProfileChangeEvent* output);

struct ControllerProfileRuntimeLocalConfirmation {
    ControllerRumbleOutput rumble{};
    ControllerProfileConfirmationPolicy policy =
        ControllerProfileConfirmationPolicy::kRumbleAndLed;
};

// Reset all four fixed slot caches to the default profile.
void controller_profile_runtime_reset();

// Refresh a slot when its identity, connection generation, or database
// generation changes; identity-only promotion preserves a held switching
// transaction. Consume pre-hotkey switching chords while transforming the
// backend-suppressed state through the shared synthetic pipeline. Inactive
// snapshots return neutral output and invalidate the slot.
ControllerProfileTransformResult controller_profile_runtime_transform(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot, uint32_t now_ms,
    AdapterUsbMode output_mode);

// Take the single committed profile-index change observed by the slot. Initial
// profile loads, identity promotions, and connection replacements do not
// publish an event.
bool controller_profile_runtime_take_profile_change(
    uint8_t slot, ControllerProfileRuntimeProfileChangeEvent* output);

// Refresh from the current slot snapshot and scale host-originated rumble.
ControllerRumbleOutput controller_profile_runtime_scale_host_rumble(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble);

// Preserve local confirmation rumble exactly while exposing profile policy.
ControllerProfileRuntimeLocalConfirmation
controller_profile_runtime_local_confirmation(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble);
