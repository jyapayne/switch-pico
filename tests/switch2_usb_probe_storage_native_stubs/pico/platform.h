#pragma once

#include "hardware/flash.h"

extern "C" {
extern uint8_t probe_test_flash[PICO_FLASH_SIZE_BYTES];
}
#define XIP_BASE (reinterpret_cast<uintptr_t>(probe_test_flash))
