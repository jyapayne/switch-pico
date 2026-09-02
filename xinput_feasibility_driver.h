#pragma once

#include <stdint.h>

#include "controller_state.h"
#include "switch_haptics.h"

void xinput_feasibility_init(uint8_t instance);
void xinput_feasibility_set_rumble_callback(uint8_t instance,
                                            ControllerRumbleCallback callback);
void xinput_feasibility_set_input(uint8_t instance,
                                  const ControllerState& state);
bool xinput_feasibility_task(uint8_t instance);
bool xinput_feasibility_is_ready(uint8_t instance);
