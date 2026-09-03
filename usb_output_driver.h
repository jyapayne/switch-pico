#pragma once

#include <stdint.h>

#include "adapter_usb_mode.h"
#include "controller_state.h"
#include "switch_haptics.h"

// Select and initialize the static USB output implementation. Call once before
// tusb_init(); the selected descriptors and class driver remain fixed for the
// lifetime of the USB device stack.
void usb_output_driver_init(AdapterUsbMode mode);

const char* usb_output_driver_mode_name();
AdapterUsbMode usb_output_driver_mode();
const char* usb_output_driver_name();

void usb_output_driver_set_input(uint8_t instance,
                                 const ControllerState& state,
                                 uint16_t left_trigger_threshold,
                                 uint16_t right_trigger_threshold);
bool usb_output_driver_task(uint8_t instance);
bool usb_output_driver_is_ready(uint8_t instance);
void usb_output_driver_set_rumble_callback(
    uint8_t instance, ControllerRumbleCallback callback);
