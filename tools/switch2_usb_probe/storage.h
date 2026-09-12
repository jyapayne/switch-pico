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
// for an invalid instance. The right bank remains immediately below profile
// storage; the left bank occupies the preceding two sectors. Both are reserved
// in every build, and load/save inspect and mutate only the selected bank.
uint32_t probe_storage_offset(uint8_t instance);

#ifdef __cplusplus
}
#endif
