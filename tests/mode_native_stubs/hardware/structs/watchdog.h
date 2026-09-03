#pragma once

#include <stdint.h>

struct watchdog_hw_t {
    uint32_t scratch[8];
};

extern watchdog_hw_t* watchdog_hw;
