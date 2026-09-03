#pragma once

#include <stdint.h>

#include "controller_state.h"
#include "device/usbd_pvt.h"
#include "switch_haptics.h"

void xinput_init(uint8_t instance);
void xinput_set_rumble_callback(uint8_t instance,
                                ControllerRumbleCallback callback);
void xinput_set_input(uint8_t instance, const ControllerState& state);
bool xinput_task(uint8_t instance);
bool xinput_is_ready(uint8_t instance);

usbd_class_driver_t const* xinput_class_driver();
