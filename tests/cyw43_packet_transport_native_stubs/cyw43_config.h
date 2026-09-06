#pragma once

#include <stdint.h>

void cyw43_thread_enter(void);
void cyw43_thread_exit(void);
void cyw43_delay_ms(uint32_t milliseconds);
_Noreturn void panic(const char *message, ...);

#define CYW43_THREAD_ENTER cyw43_thread_enter();
#define CYW43_THREAD_EXIT cyw43_thread_exit();
