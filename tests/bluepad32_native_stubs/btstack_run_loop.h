#pragma once

#include <stdint.h>

struct btstack_timer_source_t {
    void (*handler)(btstack_timer_source_t*);
    uint32_t timeout_ms;
    uint32_t add_count;
    uint64_t due_ms;
};

#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
void native_test_add_timer(btstack_timer_source_t* timer);
int btstack_run_loop_remove_timer(btstack_timer_source_t* timer);
#endif

inline void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer,
    void (*handler)(btstack_timer_source_t*)) {
    timer->handler = handler;
}

inline void btstack_run_loop_set_timer(btstack_timer_source_t* timer,
                                       uint32_t timeout_ms) {
    timer->timeout_ms = timeout_ms;
}

inline void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    ++timer->add_count;
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    native_test_add_timer(timer);
#endif
}
uint32_t btstack_run_loop_get_time_ms();
inline void btstack_run_loop_execute() {}
