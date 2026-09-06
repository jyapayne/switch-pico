#pragma once

#include <uni.h>

#ifdef __cplusplus
extern "C" {
#endif

bool uni_hid_parser_switch_native_info(uni_hid_device_t* device, uint8_t* type,
                                       uint8_t* firmware_hi, uint8_t* firmware_lo);
bool uni_hid_parser_switch_native_acquire(uni_hid_device_t* device);
bool uni_hid_parser_switch_native_send(uni_hid_device_t* device,
                                       const uint8_t rumble[8]);
void uni_hid_parser_switch_native_release(uni_hid_device_t* device);
#ifdef __cplusplus
}
#endif
