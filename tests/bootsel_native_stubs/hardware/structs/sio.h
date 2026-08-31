#pragma once

#include <cstdint>

struct sio_hw_t {
    volatile uint32_t gpio_in;
    volatile uint32_t gpio_hi_in;
};

extern sio_hw_t* sio_hw;
