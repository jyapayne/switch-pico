#pragma once

#include <stdint.h>

uint64_t time_us_64();

// Match SDK section spelling so the RAM build compiles the real annotations.
#define __not_in_flash(group) __attribute__((section(".time_critical." group)))
#define __time_critical_func(name) __not_in_flash(#name) name
