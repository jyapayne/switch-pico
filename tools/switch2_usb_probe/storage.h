#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Synchronous, main-loop-only API for the single-core probe. Serialize calls.
// Blobs are opaque, nonempty, and at most 512 bytes. Load requires an exact
// length match and leaves output unchanged on failure; it never writes flash.
// Invalid instances fail before reading a bank or writing output.
bool probe_storage_load(uint8_t instance, uint8_t *output, size_t size);

// Success means an identical blob was already committed, or a replacement was
// committed and read back. Failure never authorizes a protocol acknowledgement.
bool probe_storage_save(uint8_t instance, const uint8_t *data, size_t size);

// Flash-relative offset of the instance's two-sector pairing bank, or UINT32_MAX
// for an invalid instance. Pair A's right bank remains immediately below profile
// storage and its left bank immediately below that; standalone selects the same
// bank for its side. Both original banks are always reserved. Four-child hubs
// reserve two more banks below pair A, ordered pair B right then left. Firmware
// must fit below the entire reserved range; load/save touch only the selected bank.
uint32_t probe_storage_offset(uint8_t instance);

#ifdef __cplusplus
}
#endif
