/*
 * Minimal Switch Pro controller emulation glue derived from the GP2040-CE
 * SwitchProDriver. The driver keeps the same descriptors/handshake while
 * exposing a simple API for feeding inputs.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "controller_color.h"
#include "controller_state.h"
#include "switch_haptics.h"
#include "tusb.h"
#include "switch_pro_descriptors.h"
// Preserve the pre-neutral-state 35%-of-1023 digital trigger boundary.
constexpr uint32_t SWITCH_PRO_LEGACY_TRIGGER_RANGE_MAXIMUM = 1023;
constexpr uint32_t SWITCH_PRO_LEGACY_TRIGGER_PRESS_THRESHOLD = 358;
constexpr uint16_t SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD =
    static_cast<uint16_t>(
        (static_cast<uint32_t>(CONTROLLER_TRIGGER_MAX) *
         SWITCH_PRO_LEGACY_TRIGGER_PRESS_THRESHOLD) /
        SWITCH_PRO_LEGACY_TRIGGER_RANGE_MAXIMUM);




// Initialize one HID instance before entering the main loop.
void switch_pro_init(uint8_t instance);

// Update the desired controller state and digital trigger thresholds for one
// HID instance.
void switch_pro_set_input(uint8_t instance, const ControllerState& state,
                          uint16_t left_trigger_threshold,
                          uint16_t right_trigger_threshold);

// Drive one Switch Pro USB state machine; returns true only when a regular
// 0x30 input report was successfully queued.
bool switch_pro_task(uint8_t instance);

// Convert a packed UART message into controller state (returns true if parsed).
bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length,
                                  ControllerState& out_state);

// Driver state helpers
bool switch_pro_is_ready(uint8_t instance);

void switch_pro_set_rumble_callback(uint8_t instance,
                                    ControllerRumbleCallback callback);

// TinyUSB-facing helpers called only by usb_output_driver.
uint16_t switch_pro_hid_get_report(uint8_t instance, uint8_t report_id,
                                   hid_report_type_t report_type,
                                   uint8_t* buffer,
                                   uint16_t requested_length);
void switch_pro_hid_set_report(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               const uint8_t* buffer, uint16_t buffer_size);
void switch_pro_hid_report_received(uint8_t instance, uint8_t report_id,
                                    const uint8_t* buffer,
                                    uint16_t buffer_size);
uint8_t const* switch_pro_hid_report_descriptor(uint8_t instance);
void switch_pro_mount();
void switch_pro_unmount();