#pragma once
#include <stdbool.h>
#include <stdint.h>

enum {
    CONTROL_STAGE_SETUP = 0, CONTROL_STAGE_DATA = 1, CONTROL_STAGE_ACK = 2,
    TUSB_REQ_RCPT_DEVICE = 0, TUSB_REQ_TYPE_VENDOR = 2,
    TUSB_DIR_OUT = 0, TUSB_DIR_IN = 1,
    TUSB_XFER_BULK = 2, TUSB_XFER_INTERRUPT = 3,
    TUSB_REQ_GET_STATUS = 0, TUSB_REQ_CLEAR_FEATURE = 1, TUSB_REQ_SET_FEATURE = 3,
    TUSB_REQ_SET_ADDRESS = 5, TUSB_REQ_GET_DESCRIPTOR = 6,
    TUSB_REQ_GET_CONFIGURATION = 8, TUSB_REQ_SET_CONFIGURATION = 9,
    TUSB_REQ_GET_INTERFACE = 10, TUSB_REQ_SET_INTERFACE = 11,
    TUSB_DESC_DEVICE = 1, TUSB_DESC_CONFIGURATION = 2, TUSB_DESC_STRING = 3,
    HID_REPORT_TYPE_OUTPUT = 2,
};
typedef uint8_t hid_report_type_t;
typedef struct __attribute__((packed)) {
    union {
        uint8_t bmRequestType;
        struct __attribute__((packed)) {
            uint8_t recipient : 5;
            uint8_t type : 2;
            uint8_t direction : 1;
        } bmRequestType_bit;
    };
    uint8_t bRequest;
    uint16_t wValue, wIndex, wLength;
} tusb_control_request_t;

#ifdef __cplusplus
extern "C" {
#endif
bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t* request, void* buffer, uint16_t length);
bool tud_control_status(uint8_t rhport, const tusb_control_request_t* request);
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request);
void tud_vendor_rx_cb(uint8_t instance, const uint8_t* buffer, uint16_t length);
void tud_vendor_tx_cb(uint8_t instance, uint32_t length);
const uint8_t* tud_hid_descriptor_report_cb(uint8_t instance);
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t* buffer, uint16_t length);
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, const uint8_t* buffer, uint16_t length);
void tud_hid_report_complete_cb(uint8_t instance, const uint8_t* buffer, uint16_t length);
#ifdef __cplusplus
}
#endif
