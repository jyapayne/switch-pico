#include "profile/controller_profile_transform.h"
#include <limits.h>

namespace {

constexpr uint32_t kQ16One = 1u << 16;

constexpr uint16_t button_bit(ControllerProfileLogicalButton button) {
    return static_cast<uint16_t>(
        1u << static_cast<uint8_t>(button));
}

int32_t clamp_centered_axis(int16_t value, int16_t center) {
    const int32_t adjusted =
        static_cast<int32_t>(value) - static_cast<int32_t>(center);
    if (adjusted < INT16_MIN) {
        return INT16_MIN;
    }
    if (adjusted > INT16_MAX) {
        return INT16_MAX;
    }
    return adjusted;
}

uint32_t absolute_axis(int32_t value) {
    return value < 0 ? static_cast<uint32_t>(-value)
                     : static_cast<uint32_t>(value);
}

// This endpoint-preserving rational curve is x / (c + (1 - c) * x),
// evaluated with x in Q16 and c in Q8.8. Values above 1.0 reduce the
// response below the linear curve; values below 1.0 increase it.
uint32_t apply_curve_q16(uint32_t input_q16, uint16_t curve_q8_8) {
    if (input_q16 == 0 || input_q16 == kQ16One || curve_q8_8 == 256) {
        return input_q16;
    }

    const uint64_t denominator =
        static_cast<uint64_t>(curve_q8_8) *
            (kQ16One - input_q16) +
        static_cast<uint64_t>(256) * input_q16;
    const uint64_t numerator =
        static_cast<uint64_t>(input_q16) * 256u * kQ16One;
    const uint64_t curved = (numerator + denominator / 2u) / denominator;
    return curved > kQ16One ? kQ16One
                            : static_cast<uint32_t>(curved);
}

bool is_default_stick_configuration(
    const ControllerProfileStickConfiguration& configuration) {
    return configuration.center_x == 0 && configuration.center_y == 0 &&
           configuration.inner_deadzone == 0 &&
           configuration.outer_saturation == 32767 &&
           configuration.curve_q8_8 == 256 && !configuration.invert_x &&
           !configuration.invert_y;
}

bool is_default_trigger_configuration(
    const ControllerProfileTriggerConfiguration& configuration) {
    return configuration.lower_deadzone == 0 &&
           configuration.upper_saturation == UINT16_MAX &&
           configuration.curve_q8_8 == 256;
}

int16_t scale_stick_axis(int32_t adjusted_axis, uint32_t magnitude,
                         uint32_t response_q16, bool invert) {
    if (adjusted_axis == 0 || magnitude == 0 || response_q16 == 0) {
        return 0;
    }

    bool negative = adjusted_axis < 0;
    if (invert) {
        negative = !negative;
    }
    const uint32_t axis_magnitude = absolute_axis(adjusted_axis);
    const uint32_t output_limit =
        negative ? static_cast<uint32_t>(-static_cast<int32_t>(INT16_MIN))
                 : static_cast<uint32_t>(INT16_MAX);
    const uint64_t numerator =
        static_cast<uint64_t>(axis_magnitude) * output_limit *
        response_q16;
    const uint64_t denominator =
        static_cast<uint64_t>(magnitude) * kQ16One;
    uint32_t output_magnitude = static_cast<uint32_t>(
        (numerator + denominator / 2u) / denominator);
    if (output_magnitude > output_limit) {
        output_magnitude = output_limit;
    }

    if (!negative) {
        return static_cast<int16_t>(output_magnitude);
    }
    if (output_magnitude ==
        static_cast<uint32_t>(-static_cast<int32_t>(INT16_MIN))) {
        return INT16_MIN;
    }
    return static_cast<int16_t>(-static_cast<int32_t>(output_magnitude));
}

void transform_stick(const ControllerProfileStickConfiguration& configuration,
                     int16_t input_x, int16_t input_y, int16_t* output_x,
                     int16_t* output_y) {
    if (is_default_stick_configuration(configuration)) {
        *output_x = input_x;
        *output_y = input_y;
        return;
    }

    const int32_t adjusted_x =
        clamp_centered_axis(input_x, configuration.center_x);
    const int32_t adjusted_y =
        clamp_centered_axis(input_y, configuration.center_y);
    const uint32_t magnitude_x = absolute_axis(adjusted_x);
    const uint32_t magnitude_y = absolute_axis(adjusted_y);
    const uint32_t magnitude =
        magnitude_x > magnitude_y ? magnitude_x : magnitude_y;

    uint32_t response_q16 = 0;
    if (magnitude <= configuration.inner_deadzone) {
        response_q16 = 0;
    } else if (magnitude >= configuration.outer_saturation) {
        response_q16 = kQ16One;
    } else {
        const uint32_t input_range =
            static_cast<uint32_t>(configuration.outer_saturation) -
            configuration.inner_deadzone;
        const uint32_t input_offset =
            magnitude - configuration.inner_deadzone;
        const uint32_t normalized_q16 = static_cast<uint32_t>(
            (static_cast<uint64_t>(input_offset) * kQ16One +
             input_range / 2u) /
            input_range);
        response_q16 =
            apply_curve_q16(normalized_q16, configuration.curve_q8_8);
    }

    *output_x = scale_stick_axis(adjusted_x, magnitude, response_q16,
                                 configuration.invert_x);
    *output_y = scale_stick_axis(adjusted_y, magnitude, response_q16,
                                 configuration.invert_y);
}

uint16_t transform_trigger(
    uint16_t input,
    const ControllerProfileTriggerConfiguration& configuration) {
    if (is_default_trigger_configuration(configuration)) {
        return input;
    }
    if (input <= configuration.lower_deadzone) {
        return 0;
    }
    if (input >= configuration.upper_saturation) {
        return UINT16_MAX;
    }

    const uint32_t input_range =
        static_cast<uint32_t>(configuration.upper_saturation) -
        configuration.lower_deadzone;
    const uint32_t input_offset =
        static_cast<uint32_t>(input) - configuration.lower_deadzone;
    if (configuration.curve_q8_8 == 256) {
        return static_cast<uint16_t>(
            (static_cast<uint64_t>(input_offset) * UINT16_MAX +
             input_range / 2u) /
            input_range);
    }

    const uint32_t normalized_q16 = static_cast<uint32_t>(
        (static_cast<uint64_t>(input_offset) * kQ16One +
         input_range / 2u) /
        input_range);
    const uint32_t curved_q16 =
        apply_curve_q16(normalized_q16, configuration.curve_q8_8);
    return static_cast<uint16_t>(
        (static_cast<uint64_t>(curved_q16) * UINT16_MAX +
         kQ16One / 2u) /
        kQ16One);
}

}  // namespace

uint16_t controller_profile_extract_button_mask(const ControllerState& state) {
    uint16_t mask = 0;
    mask |= state.button_south
                ? button_bit(ControllerProfileLogicalButton::kSouth)
                : 0;
    mask |= state.button_east
                ? button_bit(ControllerProfileLogicalButton::kEast)
                : 0;
    mask |= state.button_west
                ? button_bit(ControllerProfileLogicalButton::kWest)
                : 0;
    mask |= state.button_north
                ? button_bit(ControllerProfileLogicalButton::kNorth)
                : 0;
    mask |= state.button_left_shoulder
                ? button_bit(ControllerProfileLogicalButton::kLeftShoulder)
                : 0;
    mask |= state.button_right_shoulder
                ? button_bit(ControllerProfileLogicalButton::kRightShoulder)
                : 0;
    mask |= state.button_select
                ? button_bit(ControllerProfileLogicalButton::kSelect)
                : 0;
    mask |= state.button_start
                ? button_bit(ControllerProfileLogicalButton::kStart)
                : 0;
    mask |= state.button_system
                ? button_bit(ControllerProfileLogicalButton::kSystem)
                : 0;
    mask |= state.button_capture
                ? button_bit(ControllerProfileLogicalButton::kCapture)
                : 0;
    mask |= state.button_left_stick
                ? button_bit(ControllerProfileLogicalButton::kLeftStick)
                : 0;
    mask |= state.button_right_stick
                ? button_bit(ControllerProfileLogicalButton::kRightStick)
                : 0;
    mask |= state.dpad_up
                ? button_bit(ControllerProfileLogicalButton::kDpadUp)
                : 0;
    mask |= state.dpad_down
                ? button_bit(ControllerProfileLogicalButton::kDpadDown)
                : 0;
    mask |= state.dpad_left
                ? button_bit(ControllerProfileLogicalButton::kDpadLeft)
                : 0;
    mask |= state.dpad_right
                ? button_bit(ControllerProfileLogicalButton::kDpadRight)
                : 0;
    return mask;
}

void controller_profile_apply_button_mask(uint16_t button_mask,
                                          ControllerState* state) {
    if (state == nullptr) {
        return;
    }
    state->button_south =
        (button_mask & button_bit(ControllerProfileLogicalButton::kSouth)) != 0;
    state->button_east =
        (button_mask & button_bit(ControllerProfileLogicalButton::kEast)) != 0;
    state->button_west =
        (button_mask & button_bit(ControllerProfileLogicalButton::kWest)) != 0;
    state->button_north =
        (button_mask & button_bit(ControllerProfileLogicalButton::kNorth)) != 0;
    state->button_left_shoulder =
        (button_mask &
         button_bit(ControllerProfileLogicalButton::kLeftShoulder)) != 0;
    state->button_right_shoulder =
        (button_mask &
         button_bit(ControllerProfileLogicalButton::kRightShoulder)) != 0;
    state->button_select =
        (button_mask & button_bit(ControllerProfileLogicalButton::kSelect)) != 0;
    state->button_start =
        (button_mask & button_bit(ControllerProfileLogicalButton::kStart)) != 0;
    state->button_system =
        (button_mask & button_bit(ControllerProfileLogicalButton::kSystem)) != 0;
    state->button_capture =
        (button_mask & button_bit(ControllerProfileLogicalButton::kCapture)) != 0;
    state->button_left_stick =
        (button_mask &
         button_bit(ControllerProfileLogicalButton::kLeftStick)) != 0;
    state->button_right_stick =
        (button_mask &
         button_bit(ControllerProfileLogicalButton::kRightStick)) != 0;
    state->dpad_up =
        (button_mask & button_bit(ControllerProfileLogicalButton::kDpadUp)) != 0;
    state->dpad_down =
        (button_mask & button_bit(ControllerProfileLogicalButton::kDpadDown)) != 0;
    state->dpad_left =
        (button_mask & button_bit(ControllerProfileLogicalButton::kDpadLeft)) != 0;
    state->dpad_right =
        (button_mask & button_bit(ControllerProfileLogicalButton::kDpadRight)) != 0;
}

uint16_t controller_profile_map_button_mask(
    uint16_t input_button_mask, const ControllerProfile& profile) {
    uint16_t output_button_mask = 0;
    for (uint8_t input = 0;
         input < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++input) {
        if ((input_button_mask & static_cast<uint16_t>(1u << input)) == 0) {
            continue;
        }
        const uint8_t output = profile.button_map[input];
        if (output < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT) {
            output_button_mask |= static_cast<uint16_t>(1u << output);
        }
    }
    return output_button_mask;
}

ControllerProfileTransformResult controller_profile_transform(
    const ControllerState& input, const ControllerProfile& profile) {
    ControllerProfileTransformResult result{};
    result.state = input;
    const uint16_t input_button_mask =
        controller_profile_extract_button_mask(input);
    controller_profile_apply_button_mask(
        controller_profile_map_button_mask(input_button_mask, profile),
        &result.state);

    transform_stick(profile.sticks[0], input.left_stick_x,
                    input.left_stick_y, &result.state.left_stick_x,
                    &result.state.left_stick_y);
    transform_stick(profile.sticks[1], input.right_stick_x,
                    input.right_stick_y, &result.state.right_stick_x,
                    &result.state.right_stick_y);
    result.state.left_trigger = transform_trigger(input.left_trigger,
                                                  profile.triggers[0]);
    result.state.right_trigger = transform_trigger(input.right_trigger,
                                                   profile.triggers[1]);
    result.left_trigger_digital_threshold =
        profile.triggers[0].digital_threshold;
    result.right_trigger_digital_threshold =
        profile.triggers[1].digital_threshold;
    return result;
}

uint8_t controller_profile_scale_rumble_magnitude(uint8_t magnitude,
                                                  uint8_t scale) {
    const uint32_t scaled =
        (static_cast<uint32_t>(magnitude) * scale + UINT8_MAX / 2u) /
        UINT8_MAX;
    return scaled > UINT8_MAX ? UINT8_MAX
                              : static_cast<uint8_t>(scaled);
}

ControllerRumbleOutput controller_profile_scale_host_rumble(
    const ControllerRumbleOutput& input, const ControllerProfile& profile) {
    return {
        controller_profile_scale_rumble_magnitude(
            input.low_frequency_magnitude, profile.strong_rumble_scale),
        controller_profile_scale_rumble_magnitude(
            input.high_frequency_magnitude, profile.weak_rumble_scale),
    };
}

ControllerProfileConfirmationPolicy controller_profile_confirmation_policy(
    const ControllerProfile& profile) {
    return profile.confirmation_policy;
}
