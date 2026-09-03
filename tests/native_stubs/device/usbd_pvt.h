#pragma once

#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char const* name;
    void (*init)(void);
    bool (*deinit)(void);
    void (*reset)(uint8_t rhport);
    uint16_t (*open)(uint8_t rhport,
                     tusb_desc_interface_t const* interface_descriptor,
                     uint16_t max_length);
    bool (*control_xfer_cb)(uint8_t rhport, uint8_t stage,
                            tusb_control_request_t const* request);
    bool (*xfer_cb)(uint8_t rhport, uint8_t endpoint,
                    xfer_result_t result, uint32_t transferred);
    void (*sof)(uint8_t rhport, uint32_t frame_count);
} usbd_class_driver_t;

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count);

bool usbd_edpt_open(uint8_t rhport,
                    tusb_desc_endpoint_t const* endpoint_descriptor);
bool usbd_edpt_xfer(uint8_t rhport, uint8_t endpoint, uint8_t* buffer,
                    uint16_t total_bytes);
bool usbd_edpt_busy(uint8_t rhport, uint8_t endpoint);
bool usbd_edpt_claim(uint8_t rhport, uint8_t endpoint);
bool usbd_edpt_release(uint8_t rhport, uint8_t endpoint);

#ifdef __cplusplus
}
#endif
