#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t milliseconds;
} absolute_time_t;

absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);
absolute_time_t make_timeout_time_ms(uint32_t milliseconds);
bool time_reached(absolute_time_t time);

#ifdef __cplusplus
}
#endif
