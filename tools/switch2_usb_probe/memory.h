#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool probe_memory_read(uint32_t address, uint8_t* output, size_t length);
// Packed center, positive travel, negative travel (two12-bit axes each).
bool probe_memory_right_stick_calibration(uint8_t output[9]);
