// SPDX-License-Identifier: Apache-2.0
#ifndef UNI_HID_PARSER_WII_IR_H
#define UNI_HID_PARSER_WII_IR_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#include "uni_hid_device.h"

#if SWITCH_PICO_WII_IR
// Bluetooth-thread snapshot, before orientation or controller button mapping.
// buttons uses the big-endian Wiimote mask: A=0x0008, B=0x0004.
// valid_mask bit i selects x[i]/y[i] (0..1023 / 0..767); invalid slots are zero.
// sequence advances only for a fresh IR report, even with no visible spots.
// Hotplug retains the sequence; reconnect/setup resets it to zero. The getter
// returns false while IR is unavailable or initializing, and for other devices.
typedef struct {
    uint32_t sequence;
    uint16_t buttons;
    uint16_t x[4];
    uint16_t y[4];
    uint8_t valid_mask;
} uni_wii_ir_snapshot_t;

bool uni_hid_parser_wii_ir_snapshot(uni_hid_device_t* d, uni_wii_ir_snapshot_t* out);
#endif

#ifdef __cplusplus
}
#endif

#endif  // UNI_HID_PARSER_WII_IR_H
