#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Core 0: native root management plus private BOOTSEL on root and children.
// Non-hub probes retain their private BOOTSEL-only management surface.
bool probe_management_vendor_control(uint8_t rhport, uint8_t stage,
                                     const tusb_control_request_t* request);
// Core 0: service the existing reboot delay only after a validated status ACK.
void probe_bootsel_task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
