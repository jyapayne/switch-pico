#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void reset_usb_boot(uint32_t usb_activity_gpio_pin_mask,
                    uint32_t disable_interface_mask);

#ifdef __cplusplus
}
#endif
