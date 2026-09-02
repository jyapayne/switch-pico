#include "controller_identity.h"
#include "controller_profile.h"
#include "controller_profile_transform.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits.h>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

ControllerProfile default_profile() {
    return controller_profile_default(controller_identity_global(), 0);
}

bool states_equal(const ControllerState& left, const ControllerState& right) {
    return left.dpad_up == right.dpad_up &&
           left.dpad_down == right.dpad_down &&
           left.dpad_left == right.dpad_left &&
           left.dpad_right == right.dpad_right &&
           left.button_south == right.button_south &&
           left.button_east == right.button_east &&
           left.button_west == right.button_west &&
           left.button_north == right.button_north &&
           left.button_left_shoulder == right.button_left_shoulder &&
           left.button_right_shoulder == right.button_right_shoulder &&
           left.button_select == right.button_select &&
           left.button_start == right.button_start &&
           left.button_system == right.button_system &&
           left.button_capture == right.button_capture &&
           left.button_left_stick == right.button_left_stick &&
           left.button_right_stick == right.button_right_stick &&
           left.left_trigger == right.left_trigger &&
           left.right_trigger == right.right_trigger &&
           left.left_stick_x == right.left_stick_x &&
           left.left_stick_y == right.left_stick_y &&
           left.right_stick_x == right.right_stick_x &&
           left.right_stick_y == right.right_stick_y &&
           left.motion_sample_count == right.motion_sample_count &&
           std::memcmp(left.motion_samples, right.motion_samples,
                       sizeof(left.motion_samples)) == 0;
}

ControllerProfileTransformResult transform_left_stick(
    const ControllerProfileStickConfiguration& configuration, int16_t x,
    int16_t y) {
    ControllerProfile profile = default_profile();
    profile.sticks[0] = configuration;
    ControllerState state{};
    state.left_stick_x = x;
    state.left_stick_y = y;
    return controller_profile_transform(state, profile);
}

uint16_t transform_left_trigger(
    const ControllerProfileTriggerConfiguration& configuration,
    uint16_t value) {
    ControllerProfile profile = default_profile();
    profile.triggers[0] = configuration;
    ControllerState state{};
    state.left_trigger = value;
    return controller_profile_transform(state, profile).state.left_trigger;
}

void test_button_masks_and_direct_mapping() {
    ControllerState state{};
    controller_profile_apply_button_mask(UINT16_MAX, &state);
    require(controller_profile_extract_button_mask(state) == UINT16_MAX,
            "button mask application omitted a logical button");
    controller_profile_apply_button_mask(0, &state);
    require(controller_profile_extract_button_mask(state) == 0,
            "zero button mask did not clear every logical button");
    controller_profile_apply_button_mask(UINT16_MAX, nullptr);

    for (uint8_t input = 0;
         input < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++input) {
        ControllerProfile profile = default_profile();
        const uint8_t output = static_cast<uint8_t>(
            CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT - 1u - input);
        profile.button_map[input] = output;
        state = {};
        controller_profile_apply_button_mask(
            static_cast<uint16_t>(1u << input), &state);
        const ControllerProfileTransformResult transformed =
            controller_profile_transform(state, profile);
        require(controller_profile_extract_button_mask(transformed.state) ==
                    static_cast<uint16_t>(1u << output),
                "a logical button did not map directly to its output");
    }

    ControllerProfile profile = default_profile();
    profile.button_map[0] = 1;
    profile.button_map[1] = 2;
    require(controller_profile_map_button_mask(1u, profile) == 2u,
            "button mapping recursively remapped an output");
    profile.button_map[1] = 1;
    require(controller_profile_map_button_mask(3u, profile) == 2u,
            "duplicate mapped outputs were not combined");
    profile.button_map[0] = CONTROLLER_PROFILE_NO_BUTTON;
    require(controller_profile_map_button_mask(1u, profile) == 0,
            "disabled button mapping still produced output");

    ControllerProfile invalid = default_profile();
    invalid.button_map[0] = CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT;
    require(!controller_profile_validate(invalid),
            "logical output 16 was accepted");
    invalid.button_map[0] = 0xfe;
    require(!controller_profile_validate(invalid),
            "logical output 0xfe was accepted");
    invalid.button_map[0] = CONTROLLER_PROFILE_NO_BUTTON;
    require(controller_profile_validate(invalid),
            "disabled logical output 0xff was rejected");
}

void test_stick_center_boundaries_and_inversion() {
    ControllerProfileStickConfiguration configuration{};
    configuration.center_x = 1234;
    configuration.center_y = -2345;
    configuration.inner_deadzone = 1000;
    configuration.outer_saturation = 20000;
    configuration.curve_q8_8 = 256;
    ControllerProfileTransformResult transformed = transform_left_stick(
        configuration, configuration.center_x, configuration.center_y);
    require(transformed.state.left_stick_x == 0 &&
                transformed.state.left_stick_y == 0,
            "center calibration did not precede stick shaping");

    transformed = transform_left_stick(
        configuration,
        static_cast<int16_t>(configuration.center_x +
                             configuration.inner_deadzone),
        configuration.center_y);
    require(transformed.state.left_stick_x == 0 &&
                transformed.state.left_stick_y == 0,
            "inner deadzone boundary was not neutral");
    transformed = transform_left_stick(
        configuration,
        static_cast<int16_t>(configuration.center_x +
                             configuration.inner_deadzone + 1),
        configuration.center_y);
    require(transformed.state.left_stick_x > 0,
            "first value outside inner deadzone stayed neutral");

    configuration.center_x = 0;
    configuration.center_y = 0;
    configuration.inner_deadzone = 0;
    configuration.outer_saturation = 20000;
    transformed = transform_left_stick(configuration, 19999, 0);
    require(transformed.state.left_stick_x > 0 &&
                transformed.state.left_stick_x < INT16_MAX,
            "value below outer saturation reached an endpoint");
    transformed = transform_left_stick(configuration, 20000, 0);
    require(transformed.state.left_stick_x == INT16_MAX,
            "positive outer saturation boundary missed endpoint");
    transformed = transform_left_stick(configuration, -20000, 0);
    require(transformed.state.left_stick_x == INT16_MIN,
            "negative outer saturation boundary missed endpoint");

    configuration.outer_saturation = 32767;
    configuration.invert_x = true;
    configuration.invert_y = true;
    transformed = transform_left_stick(configuration, INT16_MIN, 0);
    require(transformed.state.left_stick_x == INT16_MAX,
            "negative stick endpoint did not invert to positive endpoint");
    transformed = transform_left_stick(configuration, 0, INT16_MAX);
    require(transformed.state.left_stick_y == INT16_MIN,
            "positive stick endpoint did not invert to negative endpoint");

    configuration.invert_x = false;
    configuration.invert_y = false;
    configuration.outer_saturation = 20000;
    transformed = transform_left_stick(configuration, 15000, 15000);
    require(transformed.state.left_stick_x < INT16_MAX &&
                transformed.state.left_stick_y < INT16_MAX,
            "stick magnitude was not Chebyshev magnitude");
}

void test_stick_curves_and_monotonicity() {
    ControllerProfileStickConfiguration linear{};
    linear.inner_deadzone = 0;
    linear.outer_saturation = 32767;
    linear.curve_q8_8 = 256;
    ControllerProfileStickConfiguration slow = linear;
    slow.curve_q8_8 = 512;
    ControllerProfileStickConfiguration fast = linear;
    fast.curve_q8_8 = 128;

    const int16_t linear_mid =
        transform_left_stick(linear, 16384, 0).state.left_stick_x;
    const int16_t slow_mid =
        transform_left_stick(slow, 16384, 0).state.left_stick_x;
    const int16_t fast_mid =
        transform_left_stick(fast, 16384, 0).state.left_stick_x;
    require(slow_mid < linear_mid && linear_mid < fast_mid,
            "stick curve directions are reversed or ineffective");
    require(transform_left_stick(slow, 0, 0).state.left_stick_x == 0 &&
                transform_left_stick(fast, 32767, 0)
                        .state.left_stick_x == INT16_MAX,
            "stick response curve did not preserve endpoints");

    const uint16_t curves[] = {1, 128, 256, 512, UINT16_MAX};
    for (uint16_t curve : curves) {
        ControllerProfileStickConfiguration configuration = linear;
        configuration.curve_q8_8 = curve;
        int16_t previous = 0;
        for (int32_t input = 0; input <= INT16_MAX; ++input) {
            const int16_t output = transform_left_stick(
                                       configuration,
                                       static_cast<int16_t>(input), 0)
                                       .state.left_stick_x;
            require(output >= previous,
                    "stick response was not monotonic");
            previous = output;
        }
        require(previous == INT16_MAX,
                "monotonic stick response missed positive endpoint");
    }
}

void test_trigger_boundaries_curves_and_thresholds() {
    ControllerProfileTriggerConfiguration configuration{};
    configuration.lower_deadzone = 1000;
    configuration.upper_saturation = 60000;
    configuration.curve_q8_8 = 256;
    configuration.digital_threshold = 32000;
    require(transform_left_trigger(configuration, 999) == 0 &&
                transform_left_trigger(configuration, 1000) == 0,
            "trigger lower deadzone boundary was not zero");
    require(transform_left_trigger(configuration, 1001) > 0,
            "first trigger value above lower deadzone stayed zero");
    require(transform_left_trigger(configuration, 59999) < UINT16_MAX &&
                transform_left_trigger(configuration, 60000) == UINT16_MAX &&
                transform_left_trigger(configuration, UINT16_MAX) ==
                    UINT16_MAX,
            "trigger upper saturation boundary missed full scale");

    ControllerProfileTriggerConfiguration slow = configuration;
    slow.curve_q8_8 = 512;
    ControllerProfileTriggerConfiguration fast = configuration;
    fast.curve_q8_8 = 128;
    const uint16_t midpoint = static_cast<uint16_t>(
        (static_cast<uint32_t>(configuration.lower_deadzone) +
         configuration.upper_saturation) /
        2u);
    const uint16_t linear_mid =
        transform_left_trigger(configuration, midpoint);
    const uint16_t slow_mid = transform_left_trigger(slow, midpoint);
    const uint16_t fast_mid = transform_left_trigger(fast, midpoint);
    require(slow_mid < linear_mid && linear_mid < fast_mid,
            "trigger curve directions are reversed or ineffective");
    require(transform_left_trigger(slow, configuration.lower_deadzone) == 0 &&
                transform_left_trigger(fast,
                                       configuration.upper_saturation) ==
                    UINT16_MAX,
            "trigger response curve did not preserve endpoints");

    uint16_t previous = 0;
    for (uint32_t input = 0; input <= UINT16_MAX; ++input) {
        const uint16_t output = transform_left_trigger(
            slow, static_cast<uint16_t>(input));
        require(output >= previous,
                "trigger response was not monotonic");
        previous = output;
    }

    ControllerProfile profile = default_profile();
    profile.triggers[0] = slow;
    profile.triggers[0].digital_threshold = 12345;
    profile.triggers[1].digital_threshold = 54321;
    ControllerState state{};
    state.left_trigger = midpoint;
    const ControllerProfileTransformResult transformed =
        controller_profile_transform(state, profile);
    require(transformed.left_trigger_digital_threshold == 12345 &&
                transformed.right_trigger_digital_threshold == 54321,
            "profile-owned digital thresholds were not returned");
}

void test_default_whole_state_equivalence() {
    ControllerState input{};
    input.dpad_up = true;
    input.dpad_right = true;
    input.button_south = true;
    input.button_west = true;
    input.button_left_shoulder = true;
    input.button_select = true;
    input.button_system = true;
    input.button_capture = true;
    input.button_right_stick = true;
    input.left_trigger = 0;
    input.right_trigger = UINT16_MAX;
    input.left_stick_x = INT16_MIN;
    input.left_stick_y = INT16_MAX;
    input.right_stick_x = INT16_MAX;
    input.right_stick_y = INT16_MIN;
    input.motion_sample_count = CONTROLLER_MOTION_SAMPLE_CAPACITY;
    input.motion_samples[0] =
        {INT16_MIN, -30000, -1, 0, 1, INT16_MAX};
    input.motion_samples[1] = {1, 2, 3, 4, 5, 6};
    input.motion_samples[2] =
        {INT16_MAX, 30000, 1, 0, -1, INT16_MIN};

    const ControllerProfile profile = default_profile();
    const ControllerProfileTransformResult transformed =
        controller_profile_transform(input, profile);
    require(states_equal(input, transformed.state),
            "default profile changed whole controller state or motion");
    require(transformed.left_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                transformed.right_trigger_digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD == 22934,
            "default digital threshold changed");

    for (uint32_t trigger = 0; trigger <= UINT16_MAX; ++trigger) {
        ControllerState trigger_state{};
        trigger_state.left_trigger = static_cast<uint16_t>(trigger);
        trigger_state.right_trigger = static_cast<uint16_t>(trigger);
        const ControllerState output =
            controller_profile_transform(trigger_state, profile).state;
        require(output.left_trigger == trigger_state.left_trigger &&
                    output.right_trigger == trigger_state.right_trigger,
                "default profile lost full trigger analog precision");
    }
}

void test_rumble_scaling_and_confirmation_policy() {
    ControllerProfile profile = default_profile();
    for (uint16_t magnitude = 0; magnitude <= UINT8_MAX; ++magnitude) {
        const ControllerRumbleOutput input{
            static_cast<uint8_t>(magnitude),
            static_cast<uint8_t>(UINT8_MAX - magnitude),
        };
        const ControllerRumbleOutput output =
            controller_profile_scale_host_rumble(input, profile);
        require(output.low_frequency_magnitude ==
                        input.low_frequency_magnitude &&
                    output.high_frequency_magnitude ==
                        input.high_frequency_magnitude,
                "default rumble scaling was not bit-exact identity");
    }

    profile.strong_rumble_scale = 0;
    profile.weak_rumble_scale = 0;
    ControllerRumbleOutput output = controller_profile_scale_host_rumble(
        {UINT8_MAX, UINT8_MAX}, profile);
    require(output.low_frequency_magnitude == 0 &&
                output.high_frequency_magnitude == 0,
            "zero rumble scales did not mute both bands");

    profile.strong_rumble_scale = 128;
    profile.weak_rumble_scale = 64;
    output = controller_profile_scale_host_rumble({200, 201}, profile);
    require(output.low_frequency_magnitude == 100 &&
                output.high_frequency_magnitude == 50,
            "strong/weak mid-scale rumble mapping was incorrect");
    require(controller_profile_scale_rumble_magnitude(UINT8_MAX,
                                                      UINT8_MAX) ==
                UINT8_MAX,
            "full rumble scaling did not saturate at uint8 maximum");

    profile.confirmation_policy = ControllerProfileConfirmationPolicy::kLed;
    require(controller_profile_confirmation_policy(profile) ==
                ControllerProfileConfirmationPolicy::kLed,
            "confirmation policy was not exposed unchanged");
}

}  // namespace

int main() {
    test_button_masks_and_direct_mapping();
    test_stick_center_boundaries_and_inversion();
    test_stick_curves_and_monotonicity();
    test_trigger_boundaries_curves_and_thresholds();
    test_default_whole_state_equivalence();
    test_rumble_scaling_and_confirmation_policy();
    return 0;
}
