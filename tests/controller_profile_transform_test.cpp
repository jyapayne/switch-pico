#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/controller_profile_transform.h"
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
           left.extra_buttons == right.extra_buttons &&
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
    profile.triggers[0].output =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
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
    invalid.button_map[0] = CONTROLLER_PROFILE_OUTPUT_CONTROL_COUNT;
    require(!controller_profile_validate(invalid),
            "out-of-range logical output was accepted");
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
    profile.triggers[0].output =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
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

void test_trigger_and_button_cross_mapping() {
    ControllerProfile profile = default_profile();
    profile.triggers[0].output =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kSouth);
    profile.triggers[0].digital_threshold = 30000;
    profile.triggers[1].output =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.triggers[1].digital_threshold = 40000;
    profile.button_map[
        static_cast<uint8_t>(ControllerProfileLogicalButton::kEast)] =
        CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL;

    ControllerState input{};
    input.left_trigger = 29999;
    input.right_trigger = 45000;
    input.button_east = true;
    ControllerProfileTransformResult output =
        controller_profile_transform(input, profile);
    require(!output.state.button_south &&
                output.state.left_trigger == 45000 &&
                output.state.right_trigger == UINT16_MAX &&
                output.left_trigger_digital_threshold == 40000,
            "cross-mapped triggers lost analog or button output");

    input.left_trigger = 30000;
    output = controller_profile_transform(input, profile);
    require(output.state.button_south,
            "trigger threshold did not produce its mapped button");

    profile.triggers[0].output =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    require(!controller_profile_validate(profile),
            "two analog triggers targeting one output were accepted");
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

    ControllerRumbleOutput hd_input{};
    hd_input.hd.actuators[0].sample_count = 2;
    hd_input.hd.actuators[0].samples[0] = {48, 96, 18000, 10000};
    hd_input.hd.actuators[0].samples[1] = {49, 97, 31000, 0};
    hd_input.hd.actuators[1].sample_count = 1;
    hd_input.hd.actuators[1].samples[0] = {20, 70, 12000, 10000};
    ControllerProfile hd_profile = profile;
    hd_profile.strong_rumble_scale = 0;
    hd_profile.weak_rumble_scale = 128;
    const auto hd_output = controller_profile_scale_host_rumble(hd_input, hd_profile);
    require(hd_output.hd.actuators[0].sample_count == 2 &&
                hd_output.hd.actuators[1].sample_count == 1 &&
                hd_output.hd.actuators[0].samples[1].low_frequency_index == 49 &&
                hd_output.hd.actuators[1].samples[0].high_frequency_index == 70,
            "profile gain lost HD substeps or side-specific frequencies");
    require(hd_output.hd.actuators[0].samples[0].low_amplitude_q15 == 0 &&
                hd_output.hd.actuators[1].samples[0].low_amplitude_q15 == 0 &&
                hd_output.hd.actuators[0].samples[0].high_amplitude_q15 == 5020 &&
                hd_output.hd.actuators[1].samples[0].high_amplitude_q15 == 5020 &&
                hd_output.hd.actuators[0].samples[1].high_amplitude_q15 == 0,
            "profile band gains were not applied to linear HD amplitudes");

    profile.confirmation_policy = ControllerProfileConfirmationPolicy::kLed;
    require(controller_profile_confirmation_policy(profile) ==
                ControllerProfileConfirmationPolicy::kLed,
            "confirmation policy was not exposed unchanged");
}

void test_extra_sources_preserve_standard_routes_and_suppress_unmapped_inputs() {
    ControllerProfile profile = default_profile();
    ControllerState input{};
    input.extra_buttons = 0x7f;
    require(states_equal(
                controller_profile_transform(input, profile).state,
                controller_neutral_state()),
            "unmapped extra inputs leaked into console output");
    require(controller_profile_extract_control_mask(input, profile) ==
                (0x7fu << CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL),
            "extra inputs were omitted or overlapped the trigger controls");
    profile.extra_button_map[0] = 0;
    profile.extra_button_map[1] = CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.extra_button_map[2] = CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL;
    profile.extra_button_map[3] = 12;
    profile.extra_button_map[4] = 13;
    profile.extra_button_map[5] = 14;
    profile.extra_button_map[6] = 15;
    input.button_south = true;
    input.left_trigger = 12345;
    auto output = controller_profile_transform(input, profile);
    require(controller_profile_extract_button_mask(output.state) == 0xf001 &&
                output.state.left_trigger == UINT16_MAX &&
                output.state.right_trigger == UINT16_MAX &&
                output.state.extra_buttons == 0,
            "extra mappings lost rail buttons, trigger output, or shared contributors");
    controller_profile_remove_control_mask(
        (1u << 18) | (1u << 20) | (1u << 24) | (1u << 16), &input);
    output = controller_profile_transform(input, profile);
    require(controller_profile_extract_button_mask(output.state) == 0x7001 &&
                output.state.left_trigger == UINT16_MAX &&
                output.state.right_trigger == 0 &&
                input.extra_buttons == 0x3a && input.left_trigger == 0,
            "consumed extra sources leaked, removed a shared button, or lost another trigger source");
    input.extra_buttons = 0x80;
    input.button_south = false;
    require(controller_profile_extract_control_mask(input, profile) == 0 &&
                states_equal(controller_profile_transform(input, profile).state,
                             controller_neutral_state()),
            "reserved extra bit became a control or output");
}

void test_native_extra_destinations_combine_mapped_sources() {
    ControllerProfile profile = default_profile();
    profile.button_map[0] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.button_map[1] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.extra_button_map[6] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.triggers[0].output = CONTROLLER_PROFILE_LEFT_SL_OUTPUT;
    profile.triggers[0].digital_threshold = 30000;
    ControllerState input{};
    input.button_south = true;
    input.button_east = true;
    input.extra_buttons = 0x41;
    input.left_trigger = 30000;
    auto output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x48 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.state.left_trigger == 0,
            "native destinations lost contributors or leaked their original outputs");

    input.button_south = false;
    input.button_east = false;
    input.extra_buttons = 0;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x08,
            "releasing mapped buttons cleared an independently held trigger rail");
    input.left_trigger = 29999;
    input.extra_buttons = 1;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x08,
            "a below-threshold trigger cleared an independently held extra-source rail");
    input.extra_buttons = 0;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0,
            "released extra destinations retained stale mapped presses");
}

void test_trigger_rails_use_transformed_source_thresholds() {
    ControllerProfile profile = default_profile();
    profile.triggers[0].output = CONTROLLER_PROFILE_LEFT_SR_OUTPUT;
    profile.triggers[0].lower_deadzone = 1000;
    profile.triggers[0].upper_saturation = 11000;
    profile.triggers[0].digital_threshold = 32768;
    profile.triggers[1].output = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.triggers[1].lower_deadzone = 2000;
    profile.triggers[1].upper_saturation = 22000;
    profile.triggers[1].digital_threshold = UINT16_MAX;
    ControllerState input{};
    input.left_trigger = 5999;
    input.right_trigger = 21999;
    auto output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0 &&
                output.state.left_trigger == 0 && output.state.right_trigger == 0,
            "trigger rails activated below their transformed thresholds");
    input.left_trigger = 6000;
    input.right_trigger = 22000;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x50,
            "trigger rails missed their calibrated threshold or saturation boundary");
    profile.triggers[0].curve_q8_8 = 512;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x40,
            "trigger-to-rail routing ignored the source response curve");
    input.left_trigger = 11000;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x50,
            "curved trigger rail did not preserve the saturated endpoint");

    profile.triggers[0].digital_threshold = 0;
    profile.triggers[1].digital_threshold = 0;
    input.left_trigger = 1000;
    input.right_trigger = 2000;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0,
            "zero-threshold rails activated inside calibrated trigger deadzones");
    input = {};
    require(controller_profile_transform(input, profile).state.extra_buttons == 0,
            "neutral triggers created zero-threshold rail presses");
    input.left_trigger = 1001;
    output = controller_profile_transform(input, profile);
    require(output.state.extra_buttons == 0x10,
            "first nonzero transformed trigger failed its zero-threshold rail route");
}

void test_stick_swap_keeps_physical_calibration_and_mapped_clicks() {
    ControllerProfile profile = default_profile();
    profile.swap_sticks = true;
    profile.sticks[0].center_x = 1200;
    profile.sticks[0].center_y = -900;
    profile.sticks[0].inner_deadzone = 500;
    profile.sticks[0].outer_saturation = 20000;
    profile.sticks[0].invert_x = true;
    profile.sticks[1].center_x = -3000;
    profile.sticks[1].center_y = 2000;
    profile.sticks[1].inner_deadzone = 2000;
    profile.sticks[1].outer_saturation = 10000;
    profile.sticks[1].invert_y = true;
    ControllerState input{};
    input.left_stick_x = 1700;
    input.left_stick_y = -900;
    input.right_stick_x = -3000;
    input.right_stick_y = 4000;
    auto output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == 0 &&
                output.state.right_stick_x == 0 && output.state.right_stick_y == 0,
            "stick swap moved calibration or inner deadzones off their physical sticks");

    input.left_stick_x = 21200;
    input.right_stick_y = 12000;
    input.button_left_stick = true;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 &&
                output.state.left_stick_y == INT16_MIN &&
                output.state.right_stick_x == INT16_MIN &&
                output.state.right_stick_y == 0 &&
                !output.state.button_left_stick && output.state.button_right_stick,
            "stick swap separated clicks from calibrated saturated axis pairs");

    profile.button_map[10] = 0;
    profile.button_map[0] = 11;
    input.button_south = true;
    output = controller_profile_transform(input, profile);
    require(output.state.button_south && output.state.button_left_stick &&
                !output.state.button_right_stick,
            "stick clicks were swapped before normal output mapping");
    profile.swap_sticks = false;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == INT16_MIN &&
                output.state.left_stick_y == 0 &&
                output.state.right_stick_x == 0 &&
                output.state.right_stick_y == INT16_MIN &&
                !output.state.button_left_stick && output.state.button_right_stick,
            "disabling stick swap did not restore the calibrated normal mappings");
}

void test_left_stick_directions_cancel_and_normalize_without_button_leaks() {
    ControllerProfile profile = default_profile();
    profile.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
    profile.button_map[13] = CONTROLLER_PROFILE_LEFT_STICK_DOWN_OUTPUT;
    profile.button_map[14] = CONTROLLER_PROFILE_LEFT_STICK_LEFT_OUTPUT;
    profile.button_map[15] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
    const struct {
        uint16_t directions;
        int16_t x;
        int16_t y;
    } cases[] = {
        {0, 0, 0},
        {1, 0, -32767}, {2, 0, 32767}, {4, -32767, 0}, {8, 32767, 0},
        {5, -23169, -23169}, {9, 23169, -23169},
        {6, -23169, 23169}, {10, 23169, 23169},
        {3, 0, 0}, {12, 0, 0}, {15, 0, 0},
        {7, -32767, 0}, {11, 32767, 0},
        {13, 0, -32767}, {14, 0, 32767},
    };
    for (const auto& entry : cases) {
        ControllerState input{};
        controller_profile_apply_button_mask(entry.directions << 12, &input);
        input.right_stick_x = 123;
        input.right_stick_y = -456;
        const auto output = controller_profile_transform(input, profile);
        require(output.state.left_stick_x == entry.x &&
                    output.state.left_stick_y == entry.y,
                "direction combination lost a cardinal, diagonal, or independent cancellation");
        const int32_t x = output.state.left_stick_x;
        const int32_t y = output.state.left_stick_y;
        require(x * x + y * y <= 32767 * 32767,
                "digital left-stick vector exceeded the unit radius");
        require(controller_profile_extract_button_mask(output.state) == 0 &&
                    output.state.extra_buttons == 0 &&
                    output.state.left_trigger == 0 && output.state.right_trigger == 0 &&
                    output.state.right_stick_x == 123 &&
                    output.state.right_stick_y == -456,
                "direction mapping leaked Dpad, clicks, rails, triggers, or right-stick movement");
    }
}

void test_left_stick_direction_sources_combine_and_release_independently() {
    ControllerProfile profile = default_profile();
    profile.button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
    profile.extra_button_map[0] = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
    profile.extra_button_map[6] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
    profile.triggers[0].output = CONTROLLER_PROFILE_LEFT_STICK_RIGHT_OUTPUT;
    profile.triggers[0].digital_threshold = 30000;
    ControllerState input{};
    input.button_south = true;
    input.extra_buttons = 0x41;
    input.left_trigger = 30000;
    auto output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 32767 && output.state.left_stick_y == 0 &&
                output.state.extra_buttons == 0x40 &&
                controller_profile_extract_button_mask(output.state) == 0 &&
                output.state.left_trigger == 0,
            "direction contributors accumulated magnitude or corrupted an independent rail");
    input.button_south = false;
    input.left_trigger = 29999;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 32767 && output.state.extra_buttons == 0x40,
            "releasing button and trigger contributors cleared a held extra direction");
    input.extra_buttons = 0;
    input.left_trigger = 30000;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 32767 && output.state.extra_buttons == 0,
            "trigger direction depended on a button contributor or retained a released rail");
    input.left_trigger = 0;
    require(states_equal(controller_profile_transform(input, profile).state,
                         controller_neutral_state()),
            "releasing every direction contributor left stale output");
}

void test_trigger_directions_use_transformed_threshold_and_nonzero_guard() {
    ControllerProfile profile = default_profile();
    profile.triggers[0].output = CONTROLLER_PROFILE_LEFT_STICK_LEFT_OUTPUT;
    profile.triggers[0].lower_deadzone = 1000;
    profile.triggers[0].upper_saturation = 11000;
    profile.triggers[0].digital_threshold = 32768;
    profile.triggers[1].output = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
    profile.triggers[1].lower_deadzone = 2000;
    profile.triggers[1].upper_saturation = 22000;
    profile.triggers[1].digital_threshold = UINT16_MAX;
    ControllerState input{};
    input.left_trigger = 5999;
    input.right_trigger = 21999;
    auto output = controller_profile_transform(input, profile);
    require(states_equal(output.state, controller_neutral_state()),
            "trigger directions activated below transformed thresholds");
    input.left_trigger = 6000;
    input.right_trigger = 22000;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == -23169 && output.state.left_stick_y == -23169 &&
                output.state.left_trigger == 0 && output.state.right_trigger == 0 &&
                output.state.extra_buttons == 0 &&
                controller_profile_extract_button_mask(output.state) == 0,
            "trigger directions missed threshold equality, normalization, or leaked source output");
    profile.triggers[0].curve_q8_8 = 512;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767,
            "trigger direction threshold ignored the response curve");
    input.left_trigger = 11000;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == -23169 && output.state.left_stick_y == -23169,
            "curved trigger direction lost its saturated endpoint");
    profile.triggers[0].digital_threshold = 0;
    profile.triggers[1].digital_threshold = 0;
    input.left_trigger = 1000;
    input.right_trigger = 2000;
    require(states_equal(controller_profile_transform(input, profile).state,
                         controller_neutral_state()),
            "zero-threshold directions activated inside trigger deadzones");
    input = {};
    require(states_equal(controller_profile_transform(input, profile).state,
                         controller_neutral_state()),
            "neutral zero-threshold triggers synthesized directions");
    input.left_trigger = 1001;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == -32767 && output.state.left_stick_y == 0,
            "first nonzero curved trigger value missed a zero-threshold direction");
}

void test_left_stick_directions_yield_to_whole_calibrated_swapped_vector() {
    ControllerProfile profile = default_profile();
    profile.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
    ControllerState input{};
    input.dpad_up = true;
    input.left_stick_x = 1;
    auto output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 1 && output.state.left_stick_y == 0,
            "digital direction blended into the neutral axis of a live analog vector");
    input.left_stick_x = 0;
    input.left_stick_y = 1;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == 1,
            "digital direction replaced a nonzero analog Y axis");

    profile.sticks[0].center_x = 1000;
    profile.sticks[0].center_y = -2000;
    profile.sticks[0].inner_deadzone = 500;
    input.left_stick_x = 1500;
    input.left_stick_y = -2000;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767,
            "calibration or deadzone was applied after digital fallback");
    input.left_stick_x = 1501;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x > 0 && output.state.left_stick_y == 0,
            "first analog value outside the deadzone did not own the whole vector");

    profile.swap_sticks = true;
    profile.sticks[1].center_x = -3000;
    profile.sticks[1].center_y = 2000;
    profile.sticks[1].inner_deadzone = 1000;
    input.right_stick_x = -2000;
    input.right_stick_y = 2000;
    input.button_left_stick = true;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x == 0 && output.state.left_stick_y == -32767 &&
                output.state.right_stick_x > 0 && output.state.right_stick_y == 0 &&
                !output.state.button_left_stick && output.state.button_right_stick,
            "directions followed physical left or changed swapped analog movement and clicks");
    input.right_stick_x = -1999;
    output = controller_profile_transform(input, profile);
    require(output.state.left_stick_x > 0 && output.state.left_stick_y == 0 &&
                output.state.right_stick_x > 0 && output.state.right_stick_y == 0,
            "mapped-left analog priority ignored physical-right calibration after swapping");
}

}  // namespace

int main() {
    test_button_masks_and_direct_mapping();
    test_stick_center_boundaries_and_inversion();
    test_stick_curves_and_monotonicity();
    test_trigger_boundaries_curves_and_thresholds();
    test_trigger_and_button_cross_mapping();
    test_extra_sources_preserve_standard_routes_and_suppress_unmapped_inputs();
    test_native_extra_destinations_combine_mapped_sources();
    test_trigger_rails_use_transformed_source_thresholds();
    test_stick_swap_keeps_physical_calibration_and_mapped_clicks();
    test_left_stick_directions_cancel_and_normalize_without_button_leaks();
    test_left_stick_direction_sources_combine_and_release_independently();
    test_trigger_directions_use_transformed_threshold_and_nonzero_guard();
    test_left_stick_directions_yield_to_whole_calibrated_swapped_vector();
    test_default_whole_state_equivalence();
    test_rumble_scaling_and_confirmation_policy();
    return 0;
}
