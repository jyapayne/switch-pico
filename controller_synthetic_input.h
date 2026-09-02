#pragma once

#include <stdint.h>

#include "controller_profile_transform.h"

struct ControllerSyntheticBindingState {
    bool active = false;
    bool phase_on = false;
    uint16_t phase_units = 0;
    uint32_t last_update_ms = 0;
};

struct ControllerSyntheticInputContext {
    bool macro_active = false;
    uint8_t macro_step_index = 0;
    uint32_t macro_deadline_ms = 0;
    uint16_t previous_input_button_mask = 0;
    ControllerSyntheticBindingState
        bindings[CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT]{};
};

// Clear every synthetic source. Inputs already held at cancellation remain
// consumed or physical, but edge-triggered Macro and Auto Burst bindings do not
// restart until they are released and pressed again.
void controller_synthetic_input_cancel(
    ControllerSyntheticInputContext* context,
    uint16_t current_input_button_mask = 0);

// Apply raw-input consumption, mapped physical contributions, Turbo/Auto Burst
// gating, and finally the active macro step's field overrides.
ControllerProfileTransformResult controller_synthetic_input_apply(
    ControllerSyntheticInputContext* context, const ControllerState& input,
    const ControllerProfile& profile, uint32_t now_ms);
