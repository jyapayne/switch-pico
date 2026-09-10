#pragma once

// Pairing uses one AES-128 ECB block; no TLS, entropy service, or heap-backed cipher API.
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES
