#pragma once
#include <stdint.h>

using absolute_time_t = uint64_t;
absolute_time_t get_absolute_time();
uint32_t to_ms_since_boot(absolute_time_t value);
absolute_time_t make_timeout_time_ms(uint32_t timeout);
bool time_reached(absolute_time_t deadline);
void sleep_ms(uint32_t milliseconds);
