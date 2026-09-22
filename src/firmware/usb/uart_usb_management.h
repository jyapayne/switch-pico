#pragma once

#include <stdint.h>

#include "tusb.h"

// Minimal EP0 vendor-request management for the regular UART firmware. It
// answers INFO and accepts BOOTSEL reboot using the same wire protocol as the
// AIO firmware, so `switch-pico-config reboot bootsel` works when the Pico's
// USB side is plugged into a PC. Every other operation is stalled.
bool uart_usb_management_vendor_control(uint8_t rhport, uint8_t stage,
                                        const tusb_control_request_t* request);

// Main-loop hook: performs a requested BOOTSEL reboot once the USB status
// stage has had time to complete.
void uart_usb_management_task();
