#include "profile/controller_synthetic_input.h"

namespace {

constexpr uint32_t kTurboCycleUnits = 1000;

constexpr uint16_t button_bit(uint8_t button) {
    return static_cast<uint16_t>(1u << button);
}

constexpr uint32_t control_bit(uint8_t control) {
    return static_cast<uint32_t>(1u << control);
}

bool is_bound_control(uint8_t control) {
    return control < CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT;
}

void clear_binding(ControllerSyntheticBindingState* binding) {
    *binding = {};
}

void start_binding(ControllerSyntheticBindingState* binding,
                   uint32_t now_ms, uint8_t burst_count = 0) {
    *binding = {};
    binding->active = true;
    binding->phase_on = true;
    binding->burst_windows_remaining = burst_count;
    binding->last_update_ms = now_ms;
}

void advance_binding(ControllerSyntheticBindingState* binding,
                     const ControllerProfileTurboSettings& settings,
                     uint32_t now_ms) {
    const uint32_t elapsed_ms = now_ms - binding->last_update_ms;
    const uint64_t total_units =
        static_cast<uint64_t>(binding->phase_units) +
        static_cast<uint64_t>(elapsed_ms) * settings.rate_hz;
    const uint64_t cycles = total_units / kTurboCycleUnits;
    binding->phase_units =
        static_cast<uint16_t>(total_units % kTurboCycleUnits);
    binding->phase_on = binding->phase_units <
                        static_cast<uint32_t>(settings.duty_percent) * 10u;
    binding->last_update_ms = now_ms;
    if (binding->burst_windows_remaining != 0) {
        if (cycles >= binding->burst_windows_remaining) {
            clear_binding(binding);
            return;
        }
        binding->burst_windows_remaining = static_cast<uint8_t>(
            binding->burst_windows_remaining - cycles);
        if (binding->burst_windows_remaining == 1 && !binding->phase_on) {
            clear_binding(binding);
        }
    }
}

void stop_macro(ControllerSyntheticInputContext* context) {
    context->macro_active = false;
    context->macro_index = 0;
    context->macro_step_index = 0;
    context->macro_cycle_duration_ms = 0;
    context->macro_cycle_elapsed_ms = 0;
    context->macro_last_update_ms = 0;
    context->macro_cycles_remaining = 0;
}

bool start_macro(ControllerSyntheticInputContext* context,
                 const ControllerProfile& profile, uint8_t macro_index,
                 uint32_t now_ms) {
    if (macro_index >= CONTROLLER_PROFILE_MACRO_COUNT) {
        return false;
    }
    const ControllerProfileMacro& macro = profile.macros[macro_index];
    if (macro.step_count == 0 ||
        macro.step_count > CONTROLLER_PROFILE_MACRO_STEPS_PER_MACRO ||
        macro.first_step + macro.step_count > profile.macro_step_count ||
        macro.first_step + macro.step_count >
            CONTROLLER_PROFILE_MACRO_STEP_CAPACITY) {
        return false;
    }
    uint32_t duration_ms = 0;
    for (uint8_t step = 0; step < macro.step_count; ++step) {
        duration_ms += profile.macro_steps[macro.first_step + step].duration_ms;
    }
    if (duration_ms == 0 && macro.mode != ControllerProfileMacroMode::kOnce) {
        return false;
    }
    context->macro_active = true;
    context->macro_index = macro_index;
    context->macro_step_index = 0;
    context->macro_cycle_duration_ms = duration_ms;
    context->macro_cycle_elapsed_ms = 0;
    context->macro_last_update_ms = now_ms;
    context->macro_cycles_remaining =
        macro.mode == ControllerProfileMacroMode::kOnce ? 1 :
        macro.mode == ControllerProfileMacroMode::kRepeat ? macro.repeat_count : 0;
    return true;
}

void advance_macro(ControllerSyntheticInputContext* context,
                   const ControllerProfile& profile, uint32_t now_ms) {
    if (!context->macro_active) {
        return;
    }
    const uint32_t duration_ms = context->macro_cycle_duration_ms;
    if (context->macro_index >= CONTROLLER_PROFILE_MACRO_COUNT ||
        duration_ms == 0) {
        stop_macro(context);
        return;
    }
    const uint32_t elapsed_ms = now_ms - context->macro_last_update_ms;
    const uint64_t total_ms =
        static_cast<uint64_t>(context->macro_cycle_elapsed_ms) + elapsed_ms;
    const uint64_t cycles = total_ms / duration_ms;
    if (context->macro_cycles_remaining != 0) {
        if (cycles >= context->macro_cycles_remaining) {
            stop_macro(context);
            return;
        }
        context->macro_cycles_remaining = static_cast<uint8_t>(
            context->macro_cycles_remaining - cycles);
    }
    context->macro_cycle_elapsed_ms =
        static_cast<uint32_t>(total_ms % duration_ms);
    context->macro_last_update_ms = now_ms;
    const ControllerProfileMacro& macro = profile.macros[context->macro_index];
    uint32_t remaining_ms = context->macro_cycle_elapsed_ms;
    for (uint8_t step = 0;
         step < macro.step_count &&
         step < CONTROLLER_PROFILE_MACRO_STEPS_PER_MACRO; ++step) {
        const uint16_t step_duration =
            profile.macro_steps[macro.first_step + step].duration_ms;
        if (remaining_ms < step_duration) {
            context->macro_step_index = step;
            return;
        }
        remaining_ms -= step_duration;
    }
    stop_macro(context);
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
    uint32_t current_input_control_mask) {
    if (context == nullptr) {
        return;
    }
    *context = {};
    context->previous_input_control_mask = current_input_control_mask;
}

ControllerProfileTransformResult controller_synthetic_input_apply(
    ControllerSyntheticInputContext* context, const ControllerState& input,
    const ControllerProfile& profile, uint32_t now_ms,
    uint32_t suppressed_control_mask, bool suppress_shift) {
    if (context == nullptr) {
        return controller_profile_transform(input, profile);
    }

    const uint32_t raw_control_mask =
        controller_profile_extract_control_mask(input, profile);
    uint32_t consumed_controls = suppressed_control_mask;
    const uint32_t raw_rising_mask =
        raw_control_mask & ~context->previous_input_control_mask;
    if (profile.shift.mode != ControllerProfileShiftMode::kOff &&
        is_bound_control(profile.shift.modifier)) {
        const uint32_t modifier = control_bit(profile.shift.modifier);
        const bool pressed = (raw_control_mask & modifier) != 0;
        const bool allowed = !suppress_shift &&
                             (suppressed_control_mask & modifier) == 0;
        if (profile.shift.mode == ControllerProfileShiftMode::kHold) {
            context->shift_active = pressed && allowed;
        } else if (allowed && (raw_rising_mask & modifier) != 0) {
            context->shift_active = !context->shift_active;
        }
        consumed_controls |= modifier;
    } else {
        context->shift_active = false;
    }
    const uint32_t input_control_mask = raw_control_mask & ~consumed_controls;
    uint32_t rising_control_mask = raw_rising_mask & ~consumed_controls;
    bool cancel_pressed = false;
    for (const ControllerProfileMacro& macro : profile.macros) {
        if (is_bound_control(macro.cancel_control) &&
            (input_control_mask & control_bit(macro.cancel_control)) != 0) {
            cancel_pressed = true;
            consumed_controls |= control_bit(macro.cancel_control);
        }
    }
    if (cancel_pressed) {
        controller_synthetic_input_cancel(context, raw_control_mask);
        rising_control_mask = 0;
    }

    bool macro_started = false;
    const bool macro_was_active = context->macro_active;
    if (macro_was_active) {
        const ControllerProfileMacro& macro =
            profile.macros[context->macro_index];
        const bool held = (input_control_mask & macro.trigger_mask) ==
                          macro.trigger_mask;
        consumed_controls |= macro.trigger_mask;
        if ((macro.mode == ControllerProfileMacroMode::kWhileHeld && !held) ||
            (macro.mode == ControllerProfileMacroMode::kToggle && held &&
             (rising_control_mask & macro.trigger_mask) != 0)) {
            stop_macro(context);
        }
    }
    if (!cancel_pressed && !macro_was_active) {
        for (uint8_t macro_index = 0;
             macro_index < CONTROLLER_PROFILE_MACRO_COUNT; ++macro_index) {
            const ControllerProfileMacro& macro =
                profile.macros[macro_index];
            const bool completed =
                macro.trigger_mask != 0 && macro.step_count != 0 &&
                (input_control_mask & macro.trigger_mask) ==
                    macro.trigger_mask &&
                (rising_control_mask & macro.trigger_mask) != 0;
            if (completed &&
                start_macro(context, profile, macro_index, now_ms)) {
                macro_started = true;
                consumed_controls |= macro.trigger_mask;
                break;
            }
        }
    }
    if (!macro_started) {
        advance_macro(context, profile, now_ms);
    }
    if (context->macro_active &&
        context->macro_index < CONTROLLER_PROFILE_MACRO_COUNT) {
        consumed_controls |=
            profile.macros[context->macro_index].trigger_mask;
    }

    uint16_t gated_input_button_mask = 0;
    for (uint8_t input_button = 0;
         input_button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT;
         ++input_button) {
        ControllerSyntheticBindingState& binding =
            context->bindings[input_button];
        const uint16_t bit = button_bit(input_button);
        if ((consumed_controls & bit) != 0) {
            clear_binding(&binding);
            continue;
        }
        const bool pressed = (input_control_mask & bit) != 0;
        const bool rising = (rising_control_mask & bit) != 0;
        const ControllerProfileTurboSettings& settings =
            (profile.turbo_override_mask & bit) != 0
                ? profile.turbo_overrides[input_button]
                : profile.turbo_defaults;
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
                    advance_binding(&binding, settings, now_ms);
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
                    advance_binding(&binding, settings, now_ms);
                }
                if (binding.active && binding.phase_on) {
                    gated_input_button_mask |= bit;
                }
                break;
            case ControllerProfileTurboMode::kBurst:
                if (rising) {
                    start_binding(&binding, now_ms, settings.burst_count);
                } else if (binding.active) {
                    advance_binding(&binding, settings, now_ms);
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
    controller_profile_remove_control_mask(
        consumed_controls, &gated_input);

    ControllerProfileTransformResult result =
        controller_profile_transform(
            gated_input, profile,
            context->shift_active ? profile.shift.button_map : nullptr);
    if (context->macro_active &&
        context->macro_index < CONTROLLER_PROFILE_MACRO_COUNT) {
        const ControllerProfileMacro& macro =
            profile.macros[context->macro_index];
        if (context->macro_step_index < macro.step_count) {
            apply_macro_override(
                profile.macro_steps[
                    macro.first_step + context->macro_step_index],
                &result.state);
        } else {
            stop_macro(context);
        }
    }

    context->previous_input_control_mask = raw_control_mask;
    return result;
}
