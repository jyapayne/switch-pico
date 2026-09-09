#pragma once

#ifdef SWITCH_PICO_WII_IR_MOUSE

#include <stdint.h>

#include "tusb.h"

// Called on the USB core. Mount/unmount discard movement from the old session.
void usb_wii_ir_mouse_reset();
void usb_wii_ir_mouse_task();
uint16_t usb_wii_ir_mouse_get_report(uint8_t report_id,
                                     hid_report_type_t report_type,
                                     uint8_t* buffer,
                                     uint16_t requested_length);

#endif
