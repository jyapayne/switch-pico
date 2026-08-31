#pragma once

#include <cstdint>

#define __no_inline_not_in_flash_func(function_name) function_name

constexpr int PICO_OK = 0;

int flash_safe_execute(void (*function)(void*), void* parameter,
                       uint32_t enter_exit_timeout_ms);
