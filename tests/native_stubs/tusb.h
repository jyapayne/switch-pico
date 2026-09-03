#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HID_REPORT_TYPE_INVALID = 0,
    HID_REPORT_TYPE_INPUT = 1,
    HID_REPORT_TYPE_OUTPUT = 2,
    HID_REPORT_TYPE_FEATURE = 3,
} hid_report_type_t;

typedef enum {
    XFER_RESULT_SUCCESS = 0,
    XFER_RESULT_FAILED,
} xfer_result_t;

enum {
    TUSB_DIR_OUT = 0,
    TUSB_DIR_IN = 1,
    TUSB_DESC_ENDPOINT = 5,
    TUSB_DESC_STRING = 3,
};

#pragma pack(push, 1)
typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} tusb_desc_interface_t;

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} tusb_desc_endpoint_t;
#pragma pack(pop)

static inline uint8_t const* tu_desc_next(void const* descriptor) {
    uint8_t const* bytes = (uint8_t const*)descriptor;
    return bytes + bytes[0];
}

static inline uint8_t tu_desc_type(void const* descriptor) {
    return ((uint8_t const*)descriptor)[1];
}

static inline uint8_t tu_edpt_dir(uint8_t endpoint) {
    return (endpoint & 0x80u) != 0 ? TUSB_DIR_IN : TUSB_DIR_OUT;
}

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} tusb_control_request_t;

bool tud_hid_n_ready(uint8_t instance);
bool tud_hid_n_report(uint8_t instance, uint8_t report_id,
                      const void* report, uint16_t length);
bool tud_suspended(void);
bool tud_remote_wakeup(void);
bool tud_ready(void);

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t* buffer,
                               uint16_t requested_length);
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           const uint8_t* buffer, uint16_t buffer_size);
void tud_hid_report_received_cb(uint8_t instance, uint8_t report_id,
                                const uint8_t* buffer, uint16_t buffer_size);
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance);
void tud_mount_cb(void);
void tud_umount_cb(void);
uint8_t const* tud_descriptor_device_cb(void);
uint8_t const* tud_descriptor_configuration_cb(uint8_t index);
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);
bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request);
bool tud_control_request_cb(uint8_t rhport,
                            tusb_control_request_t const* request);

#ifdef __cplusplus
}
#endif
