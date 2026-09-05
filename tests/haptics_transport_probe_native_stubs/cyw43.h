#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
int cyw43_bluetooth_hci_write(uint8_t* buffer, size_t length);
int cyw43_bluetooth_hci_read(uint8_t* buffer, uint32_t capacity, uint32_t* length);
#ifdef __cplusplus
}
#endif
