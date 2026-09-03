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
    profile.macro_trigger = button_index(trigger);
    profile.macro_cancel = button_index(cancel);
    return profile;
}

void set_end(ControllerProfile* profile, uint8_t index) {
    profile->macro_steps[index] = {};
    profile->macro_steps[index].type =
        ControllerProfileMacroStepType::kEnd;
}

void test_immediate_press_release_dpad_and_explicit_end() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSouth);
    profile.macro_step_count = 3;
    profile.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth) |
        button_bit(ControllerProfileLogicalButton::kDpadUp);
    profile.macro_steps[1].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[1].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[1].duration_ms = 20;
    profile.macro_steps[1].output_button_mask = 0;
    set_end(&profile, 2);

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

void test_optional_field_overrides_and_motion_preservation() {
    ControllerProfile profile =
        profile_with_macro(ControllerProfileLogicalButton::kSelect);
    profile.macro_step_count = 2;
    ControllerProfileMacroStep& step = profile.macro_steps[0];
    step.type = ControllerProfileMacroStepType::kState;
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
    set_end(&profile, 1);

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
    profile.macro_step_count = 4;
    profile.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 0;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    profile.macro_steps[1].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[1].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[1].duration_ms = CONTROLLER_PROFILE_MAX_WAIT_MS;
    profile.macro_steps[1].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kEast);
    profile.macro_steps[2].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[2].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[2].duration_ms = 0;
    profile.macro_steps[2].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kWest);
    set_end(&profile, 3);

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

    profile.macro_step_count = 4;
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

    profile.macro_step_count = 2;
    profile.macro_steps[0].duration_ms = 10;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kNorth);
    set_end(&profile, 1);
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
    profile.macro_step_count = 2;
    profile.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 100;
    profile.macro_steps[0].output_button_mask =
        button_bit(ControllerProfileLogicalButton::kDpadLeft);
    set_end(&profile, 1);
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

}  // namespace

int main() {
    test_immediate_press_release_dpad_and_explicit_end();
    test_optional_field_overrides_and_motion_preservation();
    test_zero_max_wait_and_scheduled_catch_up();
    test_consumption_cancel_precedence_and_duplicate_contributors();
    test_turbo_rate_release_and_uint32_wrap();
    test_auto_burst_toggle_cancel_and_external_cancel();
    test_four_contexts_are_isolated();
    return 0;
}
