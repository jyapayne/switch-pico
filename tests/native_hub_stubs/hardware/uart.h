#pragma once
#include <stdbool.h>
#include <stdint.h>
#define uart0 ((void*)0)
bool uart_is_writable(void* uart);
void uart_putc_raw(void* uart, char value);
