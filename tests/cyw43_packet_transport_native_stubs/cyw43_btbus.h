#pragma once

#include <stdint.h>

typedef struct _cyw43_ll_t {
    int unused;
} cyw43_ll_t;

int cyw43_btbus_init(cyw43_ll_t *self);
int cyw43_btbus_read(uint8_t *buffer, uint32_t capacity, uint32_t *size);
