#include "controller_synthetic_input.h"

namespace {

constexpr uint32_t kTurboTransitionsPerSecond = 30;
constexpr uint32_t kTurboPhaseUnitsPerTransition = 1000;

constexpr uint16_t button_bit(uint8_t button) {
    return static_cast<uint16_t>(1u << button);
}

bool is_bound_button(uint8_t button) {
    return button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT;
}

void clear_binding(ControllerSyntheticBindingState* binding) {
    *binding = {};
}

void start_binding(ControllerSyntheticBindingState* binding,
                   uint32_t now_ms) {
    binding->active = true;
    binding->phase_on = true;
    binding->phase_units = 0;
    binding->last_update_ms = now_ms;
}

void advance_binding(ControllerSyntheticBindingState* binding,
                     uint32_t now_ms) {
    const uint32_t elapsed_ms = now_ms - binding->last_update_ms;
    const uint64_t total_units =
        static_cast<uint64_t>(binding->phase_units) +
        static_cast<uint64_t>(elapsed_ms) * kTurboTransitionsPerSecond;
    const uint64_t transition_count =
        total_units / kTurboPhaseUnitsPerTransition;
    if ((transition_count & 1u) != 0) {
        binding->phase_on = !binding->phase_on;
    }
    binding->phase_units = static_cast<uint16_t>(
        total_units -
        transition_count * kTurboPhaseUnitsPerTransition);
    binding->last_update_ms = now_ms;
}

bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return now_ms - deadline_ms < (UINT32_MAX / 2u + 1u);
}

void stop_macro(ControllerSyntheticInputContext* context) {
    context->macro_active = false;
    context->macro_step_index = 0;
    context->macro_deadline_ms = 0;
}

bool start_macro(ControllerSyntheticInputContext* context,
                 const ControllerProfile& profile, uint32_t now_ms) {
    if (profile.macro_step_count == 0 ||
        profile.macro_steps[0].type !=
            ControllerProfileMacroStepType::kState) {
        stop_macro(context);
        return false;
    }
    context->macro_active = true;
    context->macro_step_index = 0;
    context->macro_deadline_ms =
        now_ms + profile.macro_steps[0].duration_ms;
    return true;
}

void advance_macro(ControllerSyntheticInputContext* context,
                   const ControllerProfile& profile, uint32_t now_ms) {
    for (uint8_t transition = 0;
         transition < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY;
         ++transition) {
        if (!context->macro_active ||
            context->macro_step_index >= profile.macro_step_count) {
            stop_macro(context);
            return;
        }
        const ControllerProfileMacroStep& current =
            profile.macro_steps[context->macro_step_index];
        if (current.type != ControllerProfileMacroStepType::kState) {
            stop_macro(context);
            return;
        }
        if (!deadline_reached(now_ms, context->macro_deadline_ms)) {
            return;
        }

        const uint8_t next_index =
            static_cast<uint8_t>(context->macro_step_index + 1u);
        if (next_index >= profile.macro_step_count ||
            profile.macro_steps[next_index].type ==
                ControllerProfileMacroStepType::kEnd) {
            stop_macro(context);
            return;
        }
        context->macro_step_index = next_index;
        context->macro_deadline_ms +=
            profile.macro_steps[next_index].duration_ms;
    }
}

void apply_macro_override(const ControllerProfileMacroStep& step,
                          ControllerState* output) {
    if ((step.override_flags & kControllerProfileOverrideButtons) != 0) {
        controller_profile_apply_button_mask(step.output_button_mask,
                                             output);
    }
    if ((step.override_flags & kControllerProfileOverrideLeftStick) != 0) {
        output->left_stick_x = step.left_stick_x;
        output->left_stick_y = step.left_stick_y;
    }
    if ((step.override_flags & kControllerProfileOverrideRightStick) != 0) {
        output->right_stick_x = step.right_stick_x;
        output->right_stick_y = step.right_stick_y;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftTrigger) != 0) {
        output->left_trigger = step.left_trigger;
    }
    if ((step.override_flags & kControllerProfileOverrideRightTrigger) != 0) {
        output->right_trigger = step.right_trigger;
    }
}

}  // namespace

void controller_synthetic_input_cancel(
    ControllerSyntheticInputContext* context,
    uint16_t current_input_button_mask) {
    if (context == nullptr) {
        return;
    }
    *context = {};
    context->previous_input_button_mask = current_input_button_mask;
}

ControllerProfileTransformResult controller_synthetic_input_apply(
    ControllerSyntheticInputContext* context, const ControllerState& input,
    const ControllerProfile& profile, uint32_t now_ms) {
    if (context == nullptr) {
        return controller_profile_transform(input, profile);
    }

    const uint16_t input_button_mask =
        controller_profile_extract_button_mask(input);
    uint16_t rising_button_mask = static_cast<uint16_t>(
        input_button_mask & ~context->previous_input_button_mask);
    const bool cancel_pressed =
        is_bound_button(profile.macro_cancel) &&
        (input_button_mask & button_bit(profile.macro_cancel)) != 0;
    if (cancel_pressed) {
        controller_synthetic_input_cancel(context, input_button_mask);
        rising_button_mask = 0;
    }

    bool macro_started = false;
    if (!cancel_pressed && is_bound_button(profile.macro_trigger) &&
        (rising_button_mask & button_bit(profile.macro_trigger)) != 0) {
        macro_started = start_macro(context, profile, now_ms);
    }
    if (!macro_started) {
        advance_macro(context, profile, now_ms);
    }

    uint16_t gated_input_button_mask = 0;
    for (uint8_t input_button = 0;
         input_button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT;
         ++input_button) {
        ControllerSyntheticBindingState& binding =
            context->bindings[input_button];
        if (input_button == profile.macro_trigger ||
            input_button == profile.macro_cancel) {
            clear_binding(&binding);
            continue;
        }

        const uint16_t bit = button_bit(input_button);
        const bool pressed = (input_button_mask & bit) != 0;
        const bool rising = (rising_button_mask & bit) != 0;
        switch (profile.turbo_modes[input_button]) {
            case ControllerProfileTurboMode::kOff:
                clear_binding(&binding);
                if (pressed) {
                    gated_input_button_mask |= bit;
                }
                break;
            case ControllerProfileTurboMode::kTurbo:
                if (!pressed) {
                    clear_binding(&binding);
                    break;
                }
                if (!binding.active) {
                    start_binding(&binding, now_ms);
                } else {
                    advance_binding(&binding, now_ms);
                }
                if (binding.phase_on) {
                    gated_input_button_mask |= bit;
                }
                break;
            case ControllerProfileTurboMode::kAutoBurst:
                if (rising) {
                    if (binding.active) {
                        clear_binding(&binding);
                    } else {
                        start_binding(&binding, now_ms);
                    }
                } else if (binding.active) {
                    advance_binding(&binding, now_ms);
                }
                if (binding.active && binding.phase_on) {
                    gated_input_button_mask |= bit;
                }
                break;
        }
    }

    ControllerState gated_input = input;
    controller_profile_apply_button_mask(gated_input_button_mask,
                                         &gated_input);
    ControllerProfileTransformResult result =
        controller_profile_transform(gated_input, profile);
    if (context->macro_active &&
        context->macro_step_index < profile.macro_step_count) {
        const ControllerProfileMacroStep& step =
            profile.macro_steps[context->macro_step_index];
        if (step.type == ControllerProfileMacroStepType::kState) {
            apply_macro_override(step, &result.state);
        } else {
            stop_macro(context);
        }
    }

    context->previous_input_button_mask = input_button_mask;
    return result;
}
