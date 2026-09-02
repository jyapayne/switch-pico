#pragma once

#include <stdint.h>

#include "tusb.h"

enum class AdapterUsbMode : uint8_t {
    kSwitchProbe,
    kXInput,
};

void adapter_host_probe_init();
AdapterUsbMode adapter_host_probe_mode();
void adapter_host_probe_note_string_descriptor(uint8_t index);
bool adapter_host_probe_vendor_control(uint8_t rhport, uint8_t stage,
                                       tusb_control_request_t const *request);
