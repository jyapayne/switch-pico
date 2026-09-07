#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <btstack_tlv.h>
#include "parser/uni_switch2_pairing.h"

static struct {
    uint32_t tag;
    uint8_t data[256];
    uint32_t size;
    bool fail_write;
} storage;

static int get_tag(void* context, uint32_t tag, uint8_t* data, uint32_t capacity) {
    (void)context;
    if (tag != storage.tag) return 0;
    uint32_t copied = storage.size < capacity ? storage.size : capacity;
    memcpy(data, storage.data, copied);
    return (int)storage.size;
}
static int store_tag(void* context, uint32_t tag, const uint8_t* data, uint32_t size) {
    (void)context;
    if (storage.fail_write) return -1;
    assert(size <= sizeof(storage.data));
    storage.tag = tag;
    memcpy(storage.data, data, size);
    storage.size = size;
    return 0;
}
static const btstack_tlv_t tlv = {get_tag, store_tag, NULL};
void btstack_tlv_get_instance(const btstack_tlv_t** implementation, void** context) {
    *implementation = &tlv;
    *context = &storage;
}

int main(void) {
    uint8_t public_address[6] = {0x10, 2, 3, 4, 5, 6};
    uint8_t static_address[6] = {0xc1, 2, 3, 4, 5, 6};
    uint8_t private_address[6] = {0x41, 2, 3, 4, 5, 6};
    assert(!uni_switch2_pairing_known(0, public_address));
    assert(!uni_switch2_pairing_remember(1, private_address));
    assert(!uni_switch2_pairing_remember(2, public_address));
    assert(uni_switch2_pairing_remember(1, static_address));
    assert(uni_switch2_pairing_remember(0, public_address));
    assert(uni_switch2_pairing_known(0, public_address));
    assert(!uni_switch2_pairing_known(1, public_address));
    assert(uni_switch2_pairing_known(1, static_address));

    // Duplicate authorization remains valid without requiring another flash write.
    storage.fail_write = true;
    assert(uni_switch2_pairing_remember(0, public_address));
    assert(!uni_switch2_pairing_clear());
    assert(uni_switch2_pairing_known(0, public_address));
    public_address[5]++;
    assert(!uni_switch2_pairing_remember(0, public_address));
    assert(!uni_switch2_pairing_known(0, public_address));
    public_address[5]--;
    storage.fail_write = false;

    uint8_t type, observed[6];
    assert(uni_switch2_pairing_get(0, &type, observed));
    assert(type == 0 && memcmp(observed, public_address, 6) == 0);
    assert(uni_switch2_pairing_get(1, &type, observed));
    assert(type == 1 && memcmp(observed, static_address, 6) == 0);
    assert(!uni_switch2_pairing_get(2, &type, observed));
    assert(uni_switch2_pairing_clear());
    assert(!uni_switch2_pairing_known(0, public_address));
    assert(!uni_switch2_pairing_known(1, static_address));

    // Filling the bounded store never silently forgets an earlier controller.
    for (uint8_t i = 0; i < UNI_SWITCH2_PAIRING_CAPACITY; ++i) {
        public_address[5] = i;
        assert(uni_switch2_pairing_remember(0, public_address));
    }
    public_address[5] = UNI_SWITCH2_PAIRING_CAPACITY;
    assert(!uni_switch2_pairing_remember(0, public_address));
    public_address[5] = 0;
    assert(uni_switch2_pairing_known(0, public_address));

    // Corrupt persisted data is not an authorization and can be explicitly cleared.
    storage.data[1] = 255;
    assert(!uni_switch2_pairing_known(0, public_address));
    assert(!uni_switch2_pairing_remember(0, public_address));
    assert(uni_switch2_pairing_clear());
    assert(uni_switch2_pairing_remember(0, public_address));
    storage.size++;
    assert(!uni_switch2_pairing_known(0, public_address));
    return 0;
}
