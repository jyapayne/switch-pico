#pragma once

#include <stdint.h>

typedef int32_t alarm_id_t;
typedef uint64_t absolute_time_t;
typedef int64_t (*alarm_callback_t)(alarm_id_t alarm_id, void* user_data);

absolute_time_t get_absolute_time();
uint64_t to_ms_since_boot(absolute_time_t time);
alarm_id_t add_alarm_in_ms(int64_t delay_ms, alarm_callback_t callback,
                           void* user_data, bool fire_if_past);
