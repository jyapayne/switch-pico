#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Invalid instances and unavailable ranges leave output unchanged.
bool probe_memory_read(uint8_t instance, uint32_t address, uint8_t* output, size_t length);
// Packed center, positive travel, negative travel (two12-bit axes each).
bool probe_memory_stick_calibration(uint8_t instance, uint8_t output[9]);
