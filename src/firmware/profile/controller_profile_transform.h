#pragma once

#include <stdint.h>

#include "profile/controller_profile.h"
#include "core/controller_state.h"
#include "usb/switch/switch_haptics.h"

struct ControllerProfileTransformResult {
    ControllerState state{};
    uint16_t left_trigger_digital_threshold =
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
    uint16_t right_trigger_digital_threshold =
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
};

uint16_t controller_profile_extract_button_mask(const ControllerState& state);
void controller_profile_apply_button_mask(uint16_t button_mask,
                                          ControllerState* state);
uint16_t controller_profile_map_button_mask(
    uint16_t input_button_mask, const ControllerProfile& profile);

ControllerProfileTransformResult controller_profile_transform(
    const ControllerState& input, const ControllerProfile& profile);

uint8_t controller_profile_scale_rumble_magnitude(uint8_t magnitude,
                                                  uint8_t scale);
ControllerRumbleOutput controller_profile_scale_host_rumble(
    const ControllerRumbleOutput& input, const ControllerProfile& profile);
ControllerProfileConfirmationPolicy controller_profile_confirmation_policy(
    const ControllerProfile& profile);
