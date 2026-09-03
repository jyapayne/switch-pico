#pragma once

#include <stdint.h>

#include "adapter_usb_mode.h"
#include "tusb.h"

// Consume watchdog scratch and freeze the active mode. Call exactly once
// before usb_output_driver_init() and tusb_init().
void adapter_host_probe_init(AdapterRequestedMode requested_mode);
AdapterUsbMode adapter_host_probe_mode();
void adapter_host_probe_note_string_descriptor(uint8_t index);
bool adapter_host_probe_vendor_control(uint8_t rhport, uint8_t stage,
                                       tusb_control_request_t const *request);
