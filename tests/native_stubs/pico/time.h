#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t milliseconds;
} absolute_time_t;

absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);

#ifdef __cplusplus
}
#endif
