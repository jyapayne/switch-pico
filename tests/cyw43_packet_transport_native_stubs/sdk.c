#include "cyw43_btbus.h"

int test_cyw43_btbus_init(cyw43_ll_t *self);

/* Separate translation unit: GNU --wrap only redirects unresolved references. */
int cyw43_btbus_init(cyw43_ll_t *self) {
    return test_cyw43_btbus_init(self);
}
