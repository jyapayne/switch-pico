#pragma once

#include <stdint.h>

#include "profile/controller_profile_transform.h"

struct ControllerSyntheticBindingState {
    bool active = false;
    bool phase_on = false;
    uint16_t phase_units = 0;
    uint8_t burst_windows_remaining = 0;
    uint32_t last_update_ms = 0;
};

struct ControllerSyntheticInputContext {
    bool macro_active = false;
    uint8_t macro_index = 0;
    uint8_t macro_step_index = 0;
    uint32_t macro_cycle_duration_ms = 0;
    uint32_t macro_cycle_elapsed_ms = 0;
    uint32_t macro_last_update_ms = 0;
    uint8_t macro_cycles_remaining = 0;
    bool shift_active = false;
    uint32_t previous_input_control_mask = 0;
    ControllerSyntheticBindingState
        bindings[CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT]{};
};

// Clear every synthetic source, including toggle Shift. Inputs already held at
// cancellation cannot restart edge-triggered Macro, Burst, Auto Burst or Shift
// until they are released and pressed again.
void controller_synthetic_input_cancel(
    ControllerSyntheticInputContext* context,
    uint32_t current_input_control_mask = 0);

// Consume reserved controls and Shift before physical macro/Turbo arbitration.
// Only the selected button map changes; base analog tuning and final macro
// overrides retain their precedence. Suppressed inputs still track raw edges.
ControllerProfileTransformResult controller_synthetic_input_apply(
    ControllerSyntheticInputContext* context, const ControllerState& input,
    const ControllerProfile& profile, uint32_t now_ms,
    uint32_t suppressed_control_mask = 0, bool suppress_shift = false);
