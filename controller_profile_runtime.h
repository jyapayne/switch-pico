#pragma once

#include <stdint.h>

#include "bluepad32_input_backend.h"
#include "controller_profile_transform.h"

constexpr uint8_t CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT = 4;

struct ControllerProfileRuntimeLocalConfirmation {
    ControllerRumbleOutput rumble{};
    ControllerProfileConfirmationPolicy policy =
        ControllerProfileConfirmationPolicy::kRumbleAndLed;
};

// Reset all four fixed slot caches to the default profile.
void controller_profile_runtime_reset();

// Refresh a slot only when its connection key or database generation changes,
// then transform the raw snapshot. Inactive snapshots return neutral output and
// invalidate the slot immediately.
ControllerProfileTransformResult controller_profile_runtime_transform(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot);

// Refresh from the current slot snapshot and scale host-originated rumble.
ControllerRumbleOutput controller_profile_runtime_scale_host_rumble(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble);

// Preserve local confirmation rumble exactly while exposing profile policy.
ControllerProfileRuntimeLocalConfirmation
controller_profile_runtime_local_confirmation(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble);
