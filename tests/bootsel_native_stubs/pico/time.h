#pragma once

#include <cstdint>

using absolute_time_t = uint64_t;

absolute_time_t get_absolute_time();
uint64_t to_ms_since_boot(absolute_time_t time);
