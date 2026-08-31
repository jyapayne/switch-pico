#pragma once

#include <stdbool.h>
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

#ifdef __cplusplus
}
#endif
