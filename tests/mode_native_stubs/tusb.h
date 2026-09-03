#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    CONTROL_STAGE_SETUP = 0,
    CONTROL_STAGE_DATA = 1,
    CONTROL_STAGE_ACK = 2,
    TUSB_REQ_RCPT_DEVICE = 0,
    TUSB_DIR_OUT = 0,
    TUSB_REQ_TYPE_VENDOR = 2,
    TUSB_DIR_IN = 1,
};

struct tusb_request_type_bits_t {
    uint8_t recipient;
    uint8_t type;
    uint8_t direction;
};

struct tusb_control_request_t {
    tusb_request_type_bits_t bmRequestType_bit;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};
void tusb_init();

bool tud_control_xfer(uint8_t rhport,
                      const tusb_control_request_t* request,
                      void* buffer, uint16_t length);
