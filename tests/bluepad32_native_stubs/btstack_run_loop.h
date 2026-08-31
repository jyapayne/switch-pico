#pragma once

#include <stdint.h>

struct btstack_timer_source_t {
    void (*handler)(btstack_timer_source_t*);
    uint32_t timeout_ms;
};

inline void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer,
    void (*handler)(btstack_timer_source_t*)) {
    timer->handler = handler;
}

inline void btstack_run_loop_set_timer(btstack_timer_source_t* timer,
                                       uint32_t timeout_ms) {
    timer->timeout_ms = timeout_ms;
}

inline void btstack_run_loop_add_timer(btstack_timer_source_t*) {}
inline void btstack_run_loop_execute() {}
