#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool probe_memory_read(uint32_t address, uint8_t* output, size_t length);
bool probe_memory_right_stick_center(uint8_t output[3]);
