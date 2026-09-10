#include "profile/controller_synthetic_input.h"
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

constexpr uint8_t button_index(ControllerProfileLogicalButton button) {
    return static_cast<uint8_t>(button);
}

constexpr uint16_t button_bit(ControllerProfileLogicalButton button) {
    return static_cast<uint16_t>(1u << button_index(button));
}

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

ControllerState state_with_buttons(uint16_t mask) {
    ControllerState state = controller_neutral_state();
    controller_profile_apply_button_mask(mask, &state);
    return state;
}

bool has_button(const ControllerProfileTransformResult& result,
                ControllerProfileLogicalButton button) {
    return (controller_profile_extract_button_mask(result.state) &
            button_bit(button)) != 0;
}

ControllerProfile profile_with_macro(
    ControllerProfileLogicalButton trigger,
    ControllerProfileLogicalButton cancel =
        ControllerProfileLogicalButton::kCapture) {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.macros[0].trigger_mask = button_bit(trigger);
    profile.macros[0].cancel_control = button_index(cancel);
    profile.macros[0].first_step = 0;
    return profile;
}


void test_immediate_press_release_dpad_and_explicit_end() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSouth);
    profile.macro_step_count = 2;
    profile.macros[0].step_count = 2;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth) |
        button_bit(ControllerProfileLogicalButton::kDpadUp);
    profile.macro_steps[1].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[1].duration_ms = 20;
    profile.macro_steps[1].output_button_mask = 0;

    ControllerSyntheticInputContext context{};
    ControllerState input =
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kSouth));
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, input, profile, 100);
    require(has_button(output, ControllerProfileLogicalButton::kNorth) &&
                has_button(output, ControllerProfileLogicalButton::kDpadUp) &&
                !has_button(output, ControllerProfileLogicalButton::kSouth),
            "macro step zero was not immediate or its trigger leaked");

    input = controller_neutral_state();
    output = controller_synthetic_input_apply(&context, input, profile, 109);
    require(has_button(output, ControllerProfileLogicalButton::kNorth) &&
                has_button(output, ControllerProfileLogicalButton::kDpadUp),
            "macro press state ended before its scheduled deadline");
    output = controller_synthetic_input_apply(&context, input, profile, 110);
    require(controller_profile_extract_button_mask(output.state) == 0,
            "macro release state did not replace the full button mask");

    input =
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kWest));
    output = controller_synthetic_input_apply(&context, input, profile, 129);
    require(!has_button(output, ControllerProfileLogicalButton::kWest),
            "active macro release state did not override physical buttons");
    output = controller_synthetic_input_apply(&context, input, profile, 130);
    require(has_button(output, ControllerProfileLogicalButton::kWest) &&
                !context.macro_active,
            "explicit end did not clear overrides and restore physical input");
}

void test_macro_trigger_chord_requires_every_button() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSouth);
    profile.macros[0].trigger_mask =
        button_bit(ControllerProfileLogicalButton::kSouth) |
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL);
    profile.macros[0].cancel_control =
        CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL;
    profile.macro_step_count = 1;
    profile.macros[0].step_count = 1;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 100;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);

    ControllerSyntheticInputContext context{};
    ControllerState input =
        state_with_buttons(
            button_bit(ControllerProfileLogicalButton::kSouth));
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, input, profile, 0);
    require(!context.macro_active &&
                has_button(output, ControllerProfileLogicalButton::kSouth),
            "partial macro chord triggered or consumed a normal button");

    input = state_with_buttons(
        button_bit(ControllerProfileLogicalButton::kSouth));
    input.left_trigger = UINT16_MAX;
    output = controller_synthetic_input_apply(
        &context, input, profile, 1);
    require(context.macro_active &&
                has_button(output, ControllerProfileLogicalButton::kNorth) &&
                !has_button(output, ControllerProfileLogicalButton::kSouth) &&
                output.state.left_trigger == 0,
            "completed trigger-backed macro chord did not consume inputs");

    input = controller_neutral_state();
    input.right_trigger = UINT16_MAX;
    output = controller_synthetic_input_apply(
        &context, input, profile, 2);
    require(!context.macro_active &&
                output.state.right_trigger == 0,
            "trigger-backed macro cancellation leaked or stayed active");
}

void test_optional_field_overrides_and_motion_preservation() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSelect);
    profile.macro_step_count = 1;
    profile.macros[0].step_count = 1;
    ControllerProfileMacroStep& step = profile.macro_steps[0];
    step.override_flags = kControllerProfileOverrideLeftStick |
                          kControllerProfileOverrideRightStick |
                          kControllerProfileOverrideLeftTrigger |
                          kControllerProfileOverrideRightTrigger;
    step.duration_ms = 100;
    step.left_stick_x = INT16_MIN;
    step.left_stick_y = 1234;
    step.right_stick_x = -2345;
    step.right_stick_y = INT16_MAX;
    step.left_trigger = 0;
    step.right_trigger = UINT16_MAX;

    ControllerState input = state_with_buttons(
        button_bit(ControllerProfileLogicalButton::kSelect) |
        button_bit(ControllerProfileLogicalButton::kSouth));
    input.left_stick_x = 10;
    input.left_stick_y = 20;
    input.right_stick_x = 30;
    input.right_stick_y = 40;
    input.left_trigger = 111;
    input.right_trigger = 222;
    input.motion_sample_count = 2;
    input.motion_samples[0] = {1, 2, 3, 4, 5, 6};
    input.motion_samples[1] = {-1, -2, -3, -4, -5, -6};

    ControllerSyntheticInputContext context{};
    const ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, input, profile, 5);
    require(has_button(output, ControllerProfileLogicalButton::kSouth) &&
                !has_button(output, ControllerProfileLogicalButton::kSelect),
            "a trigger-only macro override changed buttons or leaked trigger");
    require(output.state.left_stick_x == INT16_MIN &&
                output.state.left_stick_y == 1234 &&
                output.state.right_stick_x == -2345 &&
                output.state.right_stick_y == INT16_MAX &&
                output.state.left_trigger == 0 &&
                output.state.right_trigger == UINT16_MAX,
            "one or more optional macro fields were not overridden");
    require(output.state.motion_sample_count == input.motion_sample_count &&
                std::memcmp(output.state.motion_samples,
                            input.motion_samples,
                            sizeof(input.motion_samples)) == 0,
            "synthetic processing changed motion samples");
}

void test_zero_max_wait_and_scheduled_catch_up() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSouth);
    profile.macro_step_count = 3;
    profile.macros[0].step_count = 3;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 0;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    profile.macro_steps[1].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[1].duration_ms = CONTROLLER_PROFILE_MAX_WAIT_MS;
    profile.macro_steps[1].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kEast);
    profile.macro_steps[2].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[2].duration_ms = 0;
    profile.macro_steps[2].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kWest);

    ControllerSyntheticInputContext context{};
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(
            &context,
            state_with_buttons(
                button_bit(ControllerProfileLogicalButton::kSouth)),
            profile, 0);
    require(has_button(output, ControllerProfileLogicalButton::kNorth),
            "zero-wait step zero was skipped on its trigger report");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 0);
    require(has_button(output, ControllerProfileLogicalButton::kEast),
            "zero wait did not advance on the next scheduler observation");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile,
        CONTROLLER_PROFILE_MAX_WAIT_MS - 1u);
    require(has_button(output, ControllerProfileLogicalButton::kEast),
            "maximum legal wait expired early");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile,
        CONTROLLER_PROFILE_MAX_WAIT_MS);
    require(controller_profile_extract_button_mask(output.state) == 0 &&
                !context.macro_active,
            "zero-duration catch-up did not reach the explicit end");

    profile.macro_step_count = 3;
    profile.macros[0].step_count = 3;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    profile.macro_steps[1].duration_ms = 20;
    profile.macro_steps[1].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kEast);
    profile.macro_steps[2].duration_ms = 30;
    profile.macro_steps[2].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kWest);
    context = {};
    (void)controller_synthetic_input_apply(
        &context,
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kSouth)),
        profile, 100);
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 145);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "catch-up used observation time instead of prior deadlines");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 1000);
    require(!context.macro_active &&
                controller_profile_extract_button_mask(output.state) == 0,
            "large time jump did not finish the bounded macro");

    profile.macro_step_count = 1;
    profile.macros[0].step_count = 1;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    context = {};
    constexpr uint32_t macro_near_wrap = UINT32_MAX - 5u;
    output = controller_synthetic_input_apply(
        &context,
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kSouth)),
        profile, macro_near_wrap);
    require(has_button(output, ControllerProfileLogicalButton::kNorth),
            "macro did not start immediately near uint32 wrap");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 3);
    require(has_button(output, ControllerProfileLogicalButton::kNorth),
            "macro deadline expired early across uint32 wrap");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 4);
    require(!context.macro_active &&
                controller_profile_extract_button_mask(output.state) == 0,
            "macro deadline was not uint32-wrap safe");
}

void test_consumption_cancel_precedence_and_duplicate_contributors() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSelect,
                           ControllerProfileLogicalButton::kCapture);
    profile.macro_step_count = 1;
    profile.macros[0].step_count = 1;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 100;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kDpadLeft);
    profile.button_map[button_index(ControllerProfileLogicalButton::kSouth)] =
        button_index(ControllerProfileLogicalButton::kNorth);
    profile.button_map[button_index(ControllerProfileLogicalButton::kEast)] =
        button_index(ControllerProfileLogicalButton::kNorth);
    profile.turbo_modes[button_index(ControllerProfileLogicalButton::kSouth)] =
        ControllerProfileTurboMode::kTurbo;

    ControllerSyntheticInputContext context{};
    ControllerState input = state_with_buttons(
        button_bit(ControllerProfileLogicalButton::kSouth) |
        button_bit(ControllerProfileLogicalButton::kEast));
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, input, profile, 0);
    require(has_button(output, ControllerProfileLogicalButton::kNorth),
            "duplicate mapped contributors were not ORed");
    output = controller_synthetic_input_apply(&context, input, profile, 34);
    require(has_button(output, ControllerProfileLogicalButton::kNorth),
            "Turbo gating erased a duplicate physical contributor");

    input = state_with_buttons(
        button_bit(ControllerProfileLogicalButton::kSouth) |
        button_bit(ControllerProfileLogicalButton::kSelect));
    output = controller_synthetic_input_apply(&context, input, profile, 35);
    require(has_button(output, ControllerProfileLogicalButton::kDpadLeft) &&
                !has_button(output, ControllerProfileLogicalButton::kNorth) &&
                !has_button(output, ControllerProfileLogicalButton::kSelect),
            "macro button override did not outrank Turbo or consume trigger");

    input = state_with_buttons(
        button_bit(ControllerProfileLogicalButton::kSouth) |
        button_bit(ControllerProfileLogicalButton::kCapture));
    output = controller_synthetic_input_apply(&context, input, profile, 36);
    require(!context.macro_active &&
                !has_button(output, ControllerProfileLogicalButton::kCapture) &&
                has_button(output, ControllerProfileLogicalButton::kNorth),
            "configured cancel leaked or failed to clear macro state");
}

void test_turbo_rate_release_and_uint32_wrap() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    const uint8_t south =
        button_index(ControllerProfileLogicalButton::kSouth);
    profile.turbo_modes[south] = ControllerProfileTurboMode::kTurbo;
    const ControllerState pressed =
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kSouth));
    ControllerSyntheticInputContext context{};
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, pressed, profile, 0);
    require(has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo did not begin in its ON phase");

    constexpr std::array<uint32_t, 4> deltas = {17, 29, 11, 23};
    uint32_t now_ms = 0;
    size_t delta_index = 0;
    bool previous_on = true;
    unsigned completed_activations = 0;
    while (now_ms < 1000) {
        uint32_t delta = deltas[delta_index++ % deltas.size()];
        if (delta > 1000 - now_ms) {
            delta = 1000 - now_ms;
        }
        now_ms += delta;
        output = controller_synthetic_input_apply(
            &context, pressed, profile, now_ms);
        const bool on =
            has_button(output, ControllerProfileLogicalButton::kSouth);
        if (previous_on && !on) {
            ++completed_activations;
        }
        previous_on = on;
    }
    require(completed_activations == 15 && previous_on,
            "irregular ticks did not produce exactly 15 activations per second");

    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 1001);
    require(!has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo release left a stuck output");
    output = controller_synthetic_input_apply(&context, pressed, profile, 1002);
    require(has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo repress did not restart in the ON phase");

    context = {};
    constexpr uint32_t near_wrap = UINT32_MAX - 10u;
    output = controller_synthetic_input_apply(
        &context, pressed, profile, near_wrap);
    require(has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo wrap test did not start ON");
    output = controller_synthetic_input_apply(&context, pressed, profile, 23);
    require(!has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo phase accumulation was not uint32-wrap safe");
    output = controller_synthetic_input_apply(&context, pressed, profile, 56);
    require(has_button(output, ControllerProfileLogicalButton::kSouth),
            "Turbo remainder was lost across uint32 wrap");
}

void test_auto_burst_toggle_cancel_and_external_cancel() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSelect,
                           ControllerProfileLogicalButton::kEast);
    const uint8_t west = button_index(ControllerProfileLogicalButton::kWest);
    profile.turbo_modes[west] = ControllerProfileTurboMode::kAutoBurst;
    const ControllerState pressed =
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kWest));
    ControllerSyntheticInputContext context{};

    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(&context, pressed, profile, 0);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst did not toggle on in the ON phase");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 1);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst stopped when its physical input was released");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 34);
    require(!has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst did not enter its OFF phase");
    output = controller_synthetic_input_apply(&context, pressed, profile, 35);
    require(!has_button(output, ControllerProfileLogicalButton::kWest) &&
                !context.bindings[west].active,
            "second Auto Burst rising press did not toggle it off");

    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 36);
    output = controller_synthetic_input_apply(&context, pressed, profile, 40);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst did not toggle on a second time");
    output = controller_synthetic_input_apply(
        &context,
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kEast)),
        profile, 41);
    require(!has_button(output, ControllerProfileLogicalButton::kWest) &&
                !has_button(output, ControllerProfileLogicalButton::kEast) &&
                !context.bindings[west].active,
            "macro cancel did not clear all Auto Burst state or was not consumed");

    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 42);
    output = controller_synthetic_input_apply(&context, pressed, profile, 50);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst could not restart after configured cancellation");
    controller_synthetic_input_cancel(
        &context, button_bit(ControllerProfileLogicalButton::kWest));
    output = controller_synthetic_input_apply(&context, pressed, profile, 51);
    require(!has_button(output, ControllerProfileLogicalButton::kWest),
            "external cancel retriggered an already-held Auto Burst input");
    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 52);
    output = controller_synthetic_input_apply(&context, pressed, profile, 53);
    require(has_button(output, ControllerProfileLogicalButton::kWest),
            "Auto Burst did not restart after release following cancellation");
}

void test_multiple_macro_bindings_share_step_pool() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.macros[0] = {
        button_bit(ControllerProfileLogicalButton::kSouth),
        CONTROLLER_PROFILE_NO_BUTTON, 0, 1};
    profile.macros[1] = {
        button_bit(ControllerProfileLogicalButton::kEast),
        CONTROLLER_PROFILE_NO_BUTTON, 1, 1};
    profile.macros[2].first_step = 2;
    profile.macros[3].first_step = 2;
    profile.macro_step_count = 2;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 1;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    profile.macro_steps[1].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[1].duration_ms = 1;
    profile.macro_steps[1].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kWest);

    ControllerSyntheticInputContext context{};
    ControllerProfileTransformResult output =
        controller_synthetic_input_apply(
            &context,
            state_with_buttons(
                button_bit(ControllerProfileLogicalButton::kEast)),
            profile, 0);
    require(context.macro_active && context.macro_index == 1 &&
                has_button(output, ControllerProfileLogicalButton::kWest),
            "second macro descriptor did not execute its shared step");
    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 1);
    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 2);
    output = controller_synthetic_input_apply(
        &context,
        state_with_buttons(
            button_bit(ControllerProfileLogicalButton::kSouth)),
        profile, 3);
    require(context.macro_active && context.macro_index == 0 &&
                has_button(output, ControllerProfileLogicalButton::kNorth),
            "first macro descriptor did not execute independently");
}

void test_four_contexts_are_isolated() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.turbo_modes[button_index(ControllerProfileLogicalButton::kSouth)] =
        ControllerProfileTurboMode::kAutoBurst;
    std::array<ControllerSyntheticInputContext, 4> contexts{};
    const ControllerState pressed =
        state_with_buttons(button_bit(ControllerProfileLogicalButton::kSouth));

    ControllerProfileTransformResult slot0 =
        controller_synthetic_input_apply(&contexts[0], pressed, profile, 0);
    ControllerProfileTransformResult slot1 = controller_synthetic_input_apply(
        &contexts[1], controller_neutral_state(), profile, 0);
    require(has_button(slot0, ControllerProfileLogicalButton::kSouth) &&
                !has_button(slot1, ControllerProfileLogicalButton::kSouth),
            "synthetic activation leaked into another slot");
    (void)controller_synthetic_input_apply(
        &contexts[0], controller_neutral_state(), profile, 1);
    slot1 = controller_synthetic_input_apply(&contexts[1], pressed, profile, 10);
    controller_synthetic_input_cancel(&contexts[0]);
    slot0 = controller_synthetic_input_apply(
        &contexts[0], controller_neutral_state(), profile, 11);
    slot1 = controller_synthetic_input_apply(
        &contexts[1], controller_neutral_state(), profile, 11);
    require(!has_button(slot0, ControllerProfileLogicalButton::kSouth) &&
                has_button(slot1, ControllerProfileLogicalButton::kSouth),
            "cancelling one slot changed another slot's Auto Burst state");
}

void test_parameterized_turbo_and_finite_burst() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.turbo_defaults = {10, 25, 3};
    profile.turbo_modes[0] = ControllerProfileTurboMode::kBurst;
    profile.turbo_modes[1] = ControllerProfileTurboMode::kTurbo;
    profile.turbo_override_mask = 1u << 1;
    profile.turbo_overrides[1] = {20, 80, 1};
    profile.button_map[0] = 3;
    profile.button_map[1] = 2;
    ControllerSyntheticInputContext context{};
    const ControllerState pressed = state_with_buttons(3);
    auto output = controller_synthetic_input_apply(&context, pressed, profile, 0);
    require(output.state.button_north && output.state.button_west,
            "parameterized bindings did not start immediately");
    output = controller_synthetic_input_apply(&context, pressed, profile, 24);
    require(output.state.button_north && output.state.button_west,
            "duty window ended early");
    output = controller_synthetic_input_apply(&context, pressed, profile, 25);
    require(!output.state.button_north && output.state.button_west,
            "defaults or physical-source override selected the wrong duty");
    output = controller_synthetic_input_apply(&context, pressed, profile, 40);
    require(!output.state.button_north && !output.state.button_west,
            "override duty boundary remained ON");
    output = controller_synthetic_input_apply(&context, pressed, profile, 50);
    require(!output.state.button_north && output.state.button_west,
            "override period followed the default rate");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 100);
    require(output.state.button_north && !output.state.button_west,
            "finite Burst release cancelled its windows or hold Turbo latched");
    output = controller_synthetic_input_apply(&context, pressed, profile, 101);
    require(output.state.button_north, "fresh Burst press did not restart");
    output = controller_synthetic_input_apply(&context, pressed, profile, 325);
    require(output.state.button_north, "third Burst ON window ended early");
    output = controller_synthetic_input_apply(&context, pressed, profile, 326);
    require(!output.state.button_north, "Burst did not stop at final ON end");
    output = controller_synthetic_input_apply(&context, pressed, profile, 401);
    require(!output.state.button_north, "held completed Burst rearmed itself");

    context = {};
    profile.turbo_defaults.burst_count = 1;
    constexpr uint32_t start = UINT32_MAX - 10u;
    (void)controller_synthetic_input_apply(&context, pressed, profile, start);
    output = controller_synthetic_input_apply(&context, pressed, profile, 13);
    require(output.state.button_north, "one-window Burst ended early at wrap");
    output = controller_synthetic_input_apply(&context, pressed, profile, 14);
    require(!output.state.button_north, "one-window Burst crossed its deadline");
    context = {};
    profile.turbo_defaults = {1, 99, 255};
    (void)controller_synthetic_input_apply(&context, pressed, profile, 0);
    output = controller_synthetic_input_apply(&context, pressed, profile, 254989);
    require(output.state.button_north, "maximum Burst lost its final window");
    output = controller_synthetic_input_apply(&context, pressed, profile, 254990);
    require(!output.state.button_north, "maximum Burst count overflowed");
    context = {};
    (void)controller_synthetic_input_apply(&context, pressed, profile, 0);
    output = controller_synthetic_input_apply(
        &context, pressed, profile, 4000000000u);
    require(!output.state.button_north && output.state.button_west,
            "long-gap catch-up replayed missed Burst pulses or overflowed Turbo");
    context = {};
    profile.turbo_modes[0] = ControllerProfileTurboMode::kTurbo;
    profile.turbo_defaults = {30, 1, 1};
    output = controller_synthetic_input_apply(&context, pressed, profile, 0);
    require(output.state.button_north, "narrow Turbo duty did not start ON");
    output = controller_synthetic_input_apply(&context, pressed, profile, 1);
    require(!output.state.button_north,
            "narrow duty was silently clamped to the polling cadence");
    output = controller_synthetic_input_apply(&context, pressed, profile, 100);
    require(output.state.button_north, "narrow duty lost its absolute cycle phase");
    context = {};
    profile.turbo_defaults = {30, 99, 1};
    (void)controller_synthetic_input_apply(&context, pressed, profile, 0);
    output = controller_synthetic_input_apply(&context, pressed, profile, 33);
    require(!output.state.button_north, "99-percent duty included its OFF boundary");
    output = controller_synthetic_input_apply(&context, pressed, profile, 34);
    require(output.state.button_north, "fractional period accumulated polling drift");
}

void test_shift_maps_consumption_and_physical_bindings() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSelect);
    profile.shift.mode = ControllerProfileShiftMode::kHold;
    profile.shift.modifier = CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.shift.button_map[0] = 3;
    profile.button_map[0] = CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL;
    profile.sticks[0].invert_x = true;
    profile.triggers[1].lower_deadzone = 1000;
    profile.turbo_modes[0] = ControllerProfileTurboMode::kTurbo;
    profile.turbo_defaults = {10, 20, 3};
    ControllerState input = state_with_buttons(1);
    input.left_trigger = UINT16_MAX;
    input.right_trigger = 32768;
    input.left_stick_x = 12345;
    const auto base = controller_profile_transform(input, profile);
    ControllerSyntheticInputContext context{};
    auto output = controller_synthetic_input_apply(&context, input, profile, 0);
    require(output.state.button_north && !output.state.button_south &&
                output.state.left_trigger == 0 &&
                output.state.right_trigger < UINT16_MAX &&
                output.state.left_stick_x == base.state.left_stick_x,
            "Shift did not consume analog modifier or replaced base analog tuning");
    const uint16_t tuned_trigger = output.state.right_trigger;
    output = controller_synthetic_input_apply(&context, input, profile, 20);
    require(!output.state.button_north &&
                output.state.right_trigger == tuned_trigger,
            "Shift moved Turbo settings to the mapped output");
    input.left_trigger = 0;
    output = controller_synthetic_input_apply(&context, input, profile, 100);
    require(!output.state.button_north &&
                output.state.right_trigger == UINT16_MAX,
            "releasing hold Shift did not restore base button-to-trigger mapping");

    context = {};
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    profile.shift.modifier = button_index(ControllerProfileLogicalButton::kSelect);
    profile.turbo_modes[0] = ControllerProfileTurboMode::kOff;
    profile.macros[0].step_count = profile.macro_step_count = 1;
    profile.macro_steps[0] = {kControllerProfileOverrideButtons, 100, 2};
    input = state_with_buttons(1u | (1u << profile.shift.modifier));
    output = controller_synthetic_input_apply(&context, input, profile, 0);
    require(output.state.button_north && !output.state.button_select &&
                !output.state.button_east,
            "Shift modifier leaked into mapping or its competing macro");
    output = controller_synthetic_input_apply(&context, input, profile, 1);
    require(output.state.button_north, "held toggle Shift retriggered");
    output = controller_synthetic_input_apply(
        &context, state_with_buttons(1), profile, 2);
    require(output.state.button_north, "toggle Shift did not latch after release");
    output = controller_synthetic_input_apply(&context, input, profile, 3);
    require(!output.state.button_north && output.state.right_trigger == UINT16_MAX,
            "second Shift rising edge did not restore base mapping");
    controller_synthetic_input_cancel(
        &context, controller_profile_extract_control_mask(input, profile));
    output = controller_synthetic_input_apply(&context, input, profile, 4);
    require(!output.state.button_north, "cancel phantom-toggled a held modifier");
    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 5);
    output = controller_synthetic_input_apply(&context, input, profile, 6, 0, true);
    require(!output.state.button_north, "winning hotkey did not suppress Shift");
    output = controller_synthetic_input_apply(&context, input, profile, 7);
    require(!output.state.button_north, "suppression release phantom-toggled Shift");
}

void test_macro_playback_modes_and_bounded_cycle_skips() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSouth);
    profile.macros[0].trigger_mask |= 1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.macros[0].step_count = profile.macro_step_count = 3;
    profile.macro_steps[0] = {kControllerProfileOverrideButtons, 10, 8};
    profile.macro_steps[1] = {kControllerProfileOverrideButtons, 0, 2};
    profile.macro_steps[2] = {kControllerProfileOverrideButtons, 20, 4};
    ControllerState held = state_with_buttons(1);
    held.left_trigger = UINT16_MAX;
    ControllerSyntheticInputContext context{};
    profile.macros[0].mode = ControllerProfileMacroMode::kWhileHeld;
    auto output = controller_synthetic_input_apply(&context, held, profile, 0);
    require(output.state.button_north && output.state.left_trigger == 0,
            "while-held macro did not consume its entire physical trigger");
    output = controller_synthetic_input_apply(&context, held, profile, 10);
    require(output.state.button_west && !output.state.button_east,
            "zero-duration interior step delayed the next timed state");
    output = controller_synthetic_input_apply(&context, held, profile, 30);
    require(output.state.button_north, "while-held macro did not repeat");
    output = controller_synthetic_input_apply(
        &context, state_with_buttons(1), profile, 31);
    require(!output.state.button_north && !output.state.button_west,
            "partial trigger release did not stop while-held playback");
    output = controller_synthetic_input_apply(&context, held, profile, 32);
    require(output.state.button_north, "completed trigger did not rearm playback");

    context = {};
    profile.macros[0].mode = ControllerProfileMacroMode::kRepeat;
    profile.macros[0].repeat_count = 3;
    (void)controller_synthetic_input_apply(&context, held, profile, 0);
    output = controller_synthetic_input_apply(&context, held, profile, 89);
    require(output.state.button_west, "repeat count omitted its last cycle");
    output = controller_synthetic_input_apply(&context, held, profile, 90);
    require(!output.state.button_west && !output.state.button_north,
            "finite macro repeated past its exact cycle count");
    output = controller_synthetic_input_apply(&context, held, profile, 120);
    require(!output.state.button_west && !output.state.button_north,
            "held finite macro phantom-restarted after completing");

    context = {};
    profile.macros[0].mode = ControllerProfileMacroMode::kToggle;
    constexpr uint32_t start = UINT32_MAX - 5u;
    (void)controller_synthetic_input_apply(&context, held, profile, start);
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 4);
    require(output.state.button_west, "toggle macro lost phase over clock wrap");
    output = controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 3999999994u);
    require(output.state.button_west,
            "long-gap toggle macro overflowed or iterated missed cycles");
    output = controller_synthetic_input_apply(
        &context, held, profile, 3999999995u);
    require(!output.state.button_north && !output.state.button_west &&
                output.state.left_trigger == 0,
            "matching rising trigger did not toggle active macro off");
    (void)controller_synthetic_input_apply(
        &context, controller_neutral_state(), profile, 3999999996u);
    (void)controller_synthetic_input_apply(
        &context, held, profile, 3999999997u);
    output = controller_synthetic_input_apply(
        &context, state_with_buttons(1u << 9), profile, 3999999998u);
    require(controller_profile_extract_button_mask(output.state) == 0,
            "configured cancel did not kill looping macro output");
}

void test_extra_shift_and_macro_sources_are_consumed() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.shift.mode = ControllerProfileShiftMode::kHold;
    profile.shift.modifier = 18;
    profile.extra_button_map[0] = 3;
    profile.shift.extra_button_map[0] = 3;
    profile.extra_button_map[1] = CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    profile.shift.extra_button_map[1] = 1;
    ControllerSyntheticInputContext context{};
    ControllerState input{};
    input.extra_buttons = 3;
    auto output = controller_synthetic_input_apply(&context, input, profile, 0);
    require(output.state.button_east && !output.state.button_north &&
                output.state.left_trigger == 0 && output.state.extra_buttons == 0,
            "extra Shift modifier leaked or failed to select the extra mapping layer");
    input.extra_buttons = 2;
    output = controller_synthetic_input_apply(&context, input, profile, 1);
    require(!output.state.button_east && output.state.left_trigger == UINT16_MAX,
            "extra hold-Shift release did not restore the base mapping");
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    context = {};
    input.extra_buttons = 3;
    (void)controller_synthetic_input_apply(&context, input, profile, 2);
    input.extra_buttons = 2;
    output = controller_synthetic_input_apply(&context, input, profile, 3);
    require(output.state.button_east && output.state.left_trigger == 0,
            "extra toggle-Shift did not latch after modifier release");
    input.extra_buttons = 3;
    output = controller_synthetic_input_apply(&context, input, profile, 4);
    require(!output.state.button_east && !output.state.button_north &&
                output.state.left_trigger == UINT16_MAX,
            "inactive toggle layer leaked its extra modifier");

    profile.shift.mode = ControllerProfileShiftMode::kOff;
    profile.macros[0] = {(1u << 19) | (1u << 24), 20, 0, 1,
                         ControllerProfileMacroMode::kWhileHeld, 1};
    profile.macro_step_count = 1;
    profile.macro_steps[0] = {kControllerProfileOverrideRightTrigger, 100,
                            0, 0, 0, 0, 0, 0, 45000};
    for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
        profile.macros[index].first_step = 1;
    }
    profile.extra_button_map[2] = 3;
    profile.extra_button_map[6] = 0;
    context = {};
    input.extra_buttons = 2;
    output = controller_synthetic_input_apply(&context, input, profile, 5);
    require(output.state.left_trigger == UINT16_MAX && output.state.right_trigger == 0,
            "partial extra macro chord activated");
    input.extra_buttons = 0x42;
    output = controller_synthetic_input_apply(&context, input, profile, 6);
    require(output.state.left_trigger == 0 && output.state.right_trigger == 45000 &&
                !output.state.button_south && output.state.extra_buttons == 0,
            "extra macro trigger chord leaked its mapped outputs");
    input.extra_buttons = 0x46;
    output = controller_synthetic_input_apply(&context, input, profile, 7);
    require(output.state.right_trigger == 0 && !output.state.button_north,
            "extra macro cancellation did not win or leaked its mapped output");
    input.extra_buttons = 0x42;
    output = controller_synthetic_input_apply(&context, input, profile, 8);
    require(output.state.right_trigger == 0,
            "cancel release retriggered an already-held extra macro chord");
    input.extra_buttons = 0;
    (void)controller_synthetic_input_apply(&context, input, profile, 9);
    input.extra_buttons = 0x42;
    output = controller_synthetic_input_apply(&context, input, profile, 10);
    require(output.state.right_trigger == 45000,
            "extra macro trigger did not rearm after release");
    input.extra_buttons = 2;
    output = controller_synthetic_input_apply(&context, input, profile, 11);
    require(output.state.right_trigger == 0,
            "while-held macro survived release of an extra trigger source");
}

void test_gesture_macros_play_once_and_preserve_trigger_inputs() {
    for (auto mode : {ControllerProfileMacroMode::kWhileHeld,
                      ControllerProfileMacroMode::kToggle,
                      ControllerProfileMacroMode::kRepeat}) {
        auto profile = profile_with_macro(ControllerProfileLogicalButton::kSouth);
        profile.macro_step_count = 1;
        profile.macros[0].step_count = 1;
        profile.macros[0].mode = mode;
        profile.macros[0].repeat_count = 5;
        profile.macro_steps[0].duration_ms = 100;
        profile.macro_steps[0].override_flags = kControllerProfileOverrideLeftStick;
        profile.macro_steps[0].left_stick_x = 12345;
        ControllerSyntheticInputContext context{};
        ControllerState input{};
        auto output = controller_synthetic_input_apply(&context, input, profile, 1000, 0, false, 0);
        require(output.state.left_stick_x == 12345, "gesture must start macro immediately");
        input.button_south = true;
        output = controller_synthetic_input_apply(&context, input, profile, 1050);
        require(output.state.left_stick_x == 12345 && output.state.button_south,
                "gesture macro consumed its unrelated physical trigger button");
        input.button_south = false;
        output = controller_synthetic_input_apply(&context, input, profile, 1090);
        require(output.state.left_stick_x == 12345, "gesture macro stopped on trigger release");
        output = controller_synthetic_input_apply(&context, input, profile, 1100);
        require(output.state.left_stick_x == 0 && !context.macro_active,
                "gesture macro repeated according to physical playback mode");
        output = controller_synthetic_input_apply(&context, input, profile, 1200, 0, false, 0);
        require(output.state.left_stick_x == 12345, "later gesture could not restart completed macro");
        input.button_capture = true;
        output = controller_synthetic_input_apply(&context, input, profile, 1210, 0, false, 0);
        require(!context.macro_active && output.state.left_stick_x == 0,
                "explicit cancel must beat a simultaneous gesture request");
    }
}

void test_physical_macro_precedes_simultaneous_gesture_request() {
    auto profile = controller_profile_default(controller_identity_global(), 0);
    profile.macro_step_count = 2;
    for (unsigned i = 0; i < 2; ++i) {
        profile.macros[i].first_step = i;
        profile.macros[i].step_count = 1;
        profile.macro_steps[i].override_flags = kControllerProfileOverrideButtons;
        profile.macro_steps[i].duration_ms = 100;
        profile.macro_steps[i].output_button_mask = i == 0 ? 4 : 8;
    }
    profile.macros[1].trigger_mask = 1;
    ControllerState input{};
    input.button_south = true;
    ControllerSyntheticInputContext context{};
    const auto output = controller_synthetic_input_apply(&context, input, profile, 100, 0, false, 0);
    require(output.state.button_north && !output.state.button_west,
            "gesture request stole a simultaneous physical macro trigger");
}

}  // namespace

int main() {
    test_immediate_press_release_dpad_and_explicit_end();
    test_macro_trigger_chord_requires_every_button();
    test_optional_field_overrides_and_motion_preservation();
    test_zero_max_wait_and_scheduled_catch_up();
    test_consumption_cancel_precedence_and_duplicate_contributors();
    test_turbo_rate_release_and_uint32_wrap();
    test_multiple_macro_bindings_share_step_pool();
    test_auto_burst_toggle_cancel_and_external_cancel();
    test_four_contexts_are_isolated();
    test_parameterized_turbo_and_finite_burst();
    test_shift_maps_consumption_and_physical_bindings();
    test_macro_playback_modes_and_bounded_cycle_skips();
    test_extra_shift_and_macro_sources_are_consumed();
    test_gesture_macros_play_once_and_preserve_trigger_inputs();
    test_physical_macro_precedes_simultaneous_gesture_request();
    return 0;
}
