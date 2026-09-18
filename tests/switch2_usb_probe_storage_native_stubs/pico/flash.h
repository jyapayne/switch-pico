#pragma once

#include <cstdint>

constexpr int PICO_OK = 0;
int flash_safe_execute(void (*function)(void*), void* parameter,
                       uint32_t enter_exit_timeout_ms);
