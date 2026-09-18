#pragma once

#include <stdint.h>

struct SystemClockStatus {
    uint32_t requested_sys_khz;
    uint32_t measured_sys_khz;
    uint32_t measured_usb_khz;
    uint32_t core_voltage_mv;
    uint32_t flash_clock_divider;
    uint32_t cyw43_pio_divider256;
    int32_t temperature_millicelsius;
};

#ifdef __cplusplus
extern "C" {
#endif

// Core 0, before board/peripheral initialization and before launching core 1.
void system_clock_initialize(void);
// Core 0 only; reads the dedicated on-chip temperature ADC channel.
struct SystemClockStatus system_clock_status(void);

#ifdef __cplusplus
}
#endif
