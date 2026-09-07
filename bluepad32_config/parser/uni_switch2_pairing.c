// SPDX-License-Identifier: Apache-2.0
#include "parser/uni_switch2_pairing.h"

#include <stddef.h>
#include <string.h>
#include <btstack_tlv.h>

#define PAIRING_TAG 0x53325052u  // S2PR, separate from BTstack BTD/BTL records.
#define HEADER_SIZE 2u
#define RECORD_SIZE 7u
#define STORE_SIZE (HEADER_SIZE + UNI_SWITCH2_PAIRING_CAPACITY * RECORD_SIZE)

static bool stable_address(uint8_t type, const uint8_t address[6]) {
    if (address == NULL || type > 1) return false;
    unsigned any = 0;
    unsigned not_ff = 0;
    for (unsigned i = 0; i < 6; ++i) {
        any |= address[i];
        not_ff |= address[i] ^ 0xffu;
    }
    if (any == 0 || not_ff == 0) return false;
    if (type == 0) return true;
    if ((address[0] & 0xc0u) != 0xc0u) return false;
    // The random portion of a static random address cannot be all zero/one.
    any = address[0] & 0x3fu;
    not_ff = (address[0] & 0x3fu) ^ 0x3fu;
    for (unsigned i = 1; i < 6; ++i) {
        any |= address[i];
        not_ff |= address[i] ^ 0xffu;
    }
    return any != 0 && not_ff != 0;
}

static bool load(const btstack_tlv_t** tlv, void** context, uint8_t data[STORE_SIZE]) {
    btstack_tlv_get_instance(tlv, context);
    if (*tlv == NULL || (*tlv)->get_tag == NULL || (*tlv)->store_tag == NULL) return false;
    int size = (*tlv)->get_tag(*context, PAIRING_TAG, data, STORE_SIZE);
    if (size == 0) {
        data[0] = 1;
        data[1] = 0;
        return true;
    }
    if (size < (int)HEADER_SIZE || data[0] != 1 || data[1] > UNI_SWITCH2_PAIRING_CAPACITY ||
        size != (int)(HEADER_SIZE + data[1] * RECORD_SIZE)) return false;
    for (uint8_t i = 0; i < data[1]; ++i) {
        const uint8_t* row = data + HEADER_SIZE + i * RECORD_SIZE;
        if (!stable_address(row[0], row + 1)) return false;
        if (i != 0 && memcmp(row - RECORD_SIZE, row, RECORD_SIZE) >= 0) return false;
    }
    return true;
}

bool uni_switch2_pairing_known(uint8_t type, const uint8_t address[6]) {
    if (!stable_address(type, address)) return false;
    const btstack_tlv_t* tlv;
    void* context;
    uint8_t data[STORE_SIZE];
    if (!load(&tlv, &context, data)) return false;
    for (uint8_t i = 0; i < data[1]; ++i) {
        const uint8_t* row = data + HEADER_SIZE + i * RECORD_SIZE;
        if (row[0] == type && memcmp(row + 1, address, 6) == 0) return true;
    }
    return false;
}

bool uni_switch2_pairing_remember(uint8_t type, const uint8_t address[6]) {
    if (!stable_address(type, address)) return false;
    const btstack_tlv_t* tlv;
    void* context;
    uint8_t data[STORE_SIZE];
    if (!load(&tlv, &context, data)) return false;
    uint8_t record[RECORD_SIZE];
    record[0] = type;
    memcpy(record + 1, address, 6);
    uint8_t index = 0;
    for (; index < data[1]; ++index) {
        int order = memcmp(record, data + HEADER_SIZE + index * RECORD_SIZE, RECORD_SIZE);
        if (order == 0) return true;
        if (order < 0) break;
    }
    if (data[1] == UNI_SWITCH2_PAIRING_CAPACITY) return false;
    uint8_t* insertion = data + HEADER_SIZE + index * RECORD_SIZE;
    memmove(insertion + RECORD_SIZE, insertion, (data[1] - index) * RECORD_SIZE);
    memcpy(insertion, record, RECORD_SIZE);
    ++data[1];
    return tlv->store_tag(context, PAIRING_TAG, data, HEADER_SIZE + data[1] * RECORD_SIZE) == 0;
}

bool uni_switch2_pairing_clear(void) {
    const btstack_tlv_t* tlv;
    void* context;
    btstack_tlv_get_instance(&tlv, &context);
    if (tlv == NULL || tlv->store_tag == NULL) return false;
    // Store an empty record rather than the void delete API: callers must know
    // whether forgetting was persisted before acknowledging clear-pairings.
    const uint8_t empty[HEADER_SIZE] = {1, 0};
    return tlv->store_tag(context, PAIRING_TAG, empty, sizeof(empty)) == 0;
}

bool uni_switch2_pairing_get(uint8_t index, uint8_t* type, uint8_t address[6]) {
    if (type == NULL || address == NULL) return false;
    const btstack_tlv_t* tlv;
    void* context;
    uint8_t data[STORE_SIZE];
    if (!load(&tlv, &context, data) || index >= data[1]) return false;
    const uint8_t* row = data + HEADER_SIZE + index * RECORD_SIZE;
    *type = row[0];
    memcpy(address, row + 1, 6);
    return true;
}
