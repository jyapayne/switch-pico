#pragma once

#include <cstdint>

using io_rw_32 = volatile uint32_t;

enum gpio_override {
    GPIO_OVERRIDE_NORMAL = 0,
    GPIO_OVERRIDE_LOW = 2,
};

void bootsel_test_masked_write(io_rw_32* address, uint32_t values,
                               uint32_t mask);

inline void hw_write_masked(io_rw_32* address, uint32_t values,
                            uint32_t mask) {
    *address = (*address & ~mask) | (values & mask);
    bootsel_test_masked_write(address, values, mask);
}

#if PICO_RP2350
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB 14u
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS 0x0000c000u
#else
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB 12u
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS 0x00003000u
#endif
