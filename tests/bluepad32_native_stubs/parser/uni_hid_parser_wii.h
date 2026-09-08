#pragma once

#include <uni.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum wii_flags {
    WII_MODE_HORIZONTAL = 0,
    WII_MODE_VERTICAL = 1,
    WII_MODE_ACCEL = 2,
} wii_mode_t;

void uni_hid_parser_wii_set_mode(uni_hid_device_t* device, wii_mode_t mode);

#ifdef __cplusplus
}
#endif
