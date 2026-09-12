#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Native SIE hub: device slot 0 is the hub; controller instances 0/1 map to
// device slots 1/2 (right/left). The caller owns Bluetooth on Core 0; this
// transport owns Core 1. No external USB wiring is used.
// Recover a timed-out test firmware to BOOTSEL instead of rebooting forever.
void native_hub_startup_guard(void);
bool native_hub_init(void);
void native_hub_task(void);
bool native_hub_mounted(uint8_t instance);
bool native_hub_suspended(uint8_t instance);
bool native_hub_hid_ready(uint8_t instance);
bool native_hub_hid_report(uint8_t instance, uint8_t report_id,
                           const void* data, uint16_t length);
uint32_t native_hub_vendor_write_available(uint8_t instance);
uint32_t native_hub_vendor_write(uint8_t instance, const void* data, uint32_t length);
uint32_t native_hub_vendor_write_flush(uint8_t instance);
// OUT packets are delivered directly and once through tud_vendor_rx_cb;
// there is no second receive FIFO to drain in this backend.
bool native_hub_control_xfer(uint8_t device_slot,
                             const tusb_control_request_t* request,
                             void* buffer, uint16_t length);
bool native_hub_control_status(uint8_t device_slot,
                               const tusb_control_request_t* request);

// Supplied by the existing native Joy-Con protocol engine. Each returned
// descriptor is the standalone model, with interfaces 0/1 and EPs 1/2.
const uint8_t* native_joycon_device_descriptor(uint8_t instance);
const uint8_t* native_joycon_configuration_descriptor(uint8_t instance);
const uint16_t* native_joycon_string_descriptor(uint8_t instance, uint8_t index,
                                               uint16_t language_id);
void native_joycon_usb_reset(uint8_t instance);

#ifdef __cplusplus
}
#endif
