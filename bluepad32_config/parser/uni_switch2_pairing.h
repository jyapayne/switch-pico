// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Application-level trust, not SMP bonds or cryptographic authentication.
// Call on the Bluetooth/storage core after the BTstack TLV backend is ready.
#define UNI_SWITCH2_PAIRING_CAPACITY 16
bool uni_switch2_pairing_known(uint8_t address_type, const uint8_t address[6]);
bool uni_switch2_pairing_remember(uint8_t address_type, const uint8_t address[6]);
bool uni_switch2_pairing_clear(void);
bool uni_switch2_pairing_get(uint8_t index, uint8_t* address_type, uint8_t address[6]);

#ifdef __cplusplus
}
#endif
