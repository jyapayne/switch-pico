#pragma once

#include <stdint.h>

#include "adapter_usb_mode.h"
#include "controller_state.h"
#include "switch_haptics.h"

// Mode capability bits published in USB management info. Input is present in
// every current mode; rumble/motion describe host-visible USB capabilities.
constexpr uint8_t USB_OUTPUT_CAPABILITY_INPUT = 1u << 0;
constexpr uint8_t USB_OUTPUT_CAPABILITY_RUMBLE = 1u << 1;
constexpr uint8_t USB_OUTPUT_CAPABILITY_MOTION = 1u << 2;
constexpr uint8_t USB_OUTPUT_CAPABILITY_MASK =
    USB_OUTPUT_CAPABILITY_INPUT | USB_OUTPUT_CAPABILITY_RUMBLE |
    USB_OUTPUT_CAPABILITY_MOTION;

// Select and initialize the static USB output implementation. Call once before
// tusb_init(); the selected descriptors and class driver remain fixed for the
// lifetime of the USB device stack.
void usb_output_driver_init(AdapterUsbMode mode);

const char* usb_output_driver_mode_name();
AdapterUsbMode usb_output_driver_mode();
const char* usb_output_driver_name();
uint8_t usb_output_driver_capabilities();

void usb_output_driver_set_input(uint8_t instance,
                                 const ControllerState& state,
                                 uint16_t left_trigger_threshold,
                                 uint16_t right_trigger_threshold);
bool usb_output_driver_task(uint8_t instance);
bool usb_output_driver_is_ready(uint8_t instance);
void usb_output_driver_set_rumble_callback(
    uint8_t instance, ControllerRumbleCallback callback);
