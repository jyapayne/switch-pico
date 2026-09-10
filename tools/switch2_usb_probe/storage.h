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
bool probe_storage_load(uint8_t *output, size_t size);

// Success means an identical blob was already committed, or a replacement was
// committed and read back. Failure never authorizes a protocol acknowledgement.
bool probe_storage_save(const uint8_t *data, size_t size);

// Flash-relative offset of the two sectors immediately below profile storage.
uint32_t probe_storage_offset(void);

#ifdef __cplusplus
}
#endif
