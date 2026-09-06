#pragma once

#include <stddef.h>
#include <stdint.h>
void multicore_launch_core1_with_stack(void (*entry)(), uint32_t* stack, size_t size);
