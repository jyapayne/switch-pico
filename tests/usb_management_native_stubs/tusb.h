#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    CONTROL_STAGE_SETUP = 0,
    CONTROL_STAGE_DATA = 1,
    CONTROL_STAGE_ACK = 2,
    TUSB_REQ_RCPT_DEVICE = 0,
    TUSB_DIR_OUT = 0,
    TUSB_DIR_IN = 1,
};

typedef struct {
    uint8_t recipient;
    uint8_t type;
    uint8_t direction;
} tusb_request_type_bits_t;

typedef struct {
    tusb_request_type_bits_t bmRequestType_bit;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} tusb_control_request_t;

#ifdef __cplusplus
extern "C" {
#endif

bool tud_control_xfer(uint8_t rhport,
                      const tusb_control_request_t* request,
                      void* buffer, uint16_t length);
bool tud_control_status(uint8_t rhport,
                        const tusb_control_request_t* request);
bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage,
    const tusb_control_request_t* request);
#ifdef __cplusplus
}
#endif
