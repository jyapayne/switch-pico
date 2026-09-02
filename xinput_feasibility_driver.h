#pragma once

#include <stdint.h>

#include "switch_pro_driver.h"

void xinput_feasibility_init(uint8_t instance);
void xinput_feasibility_set_rumble_callback(uint8_t instance,
                                            SwitchRumbleCallback callback);
void xinput_feasibility_set_input(uint8_t instance,
                                  const SwitchInputState &state);
bool xinput_feasibility_task(uint8_t instance);
bool xinput_feasibility_is_ready(uint8_t instance);
