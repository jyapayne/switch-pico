#include "hardware_stub.h"
#include <assert.h>
#include <stdio.h>

// The real router tables and token-header decision run on the host. Only the
// clock/pad registers and the SIE bank-selection receiver are modeled here;
// the timing loop is compiled but never run against a simulated USB wire.
#define PICO_RP2350 1
#undef SIO_GPIO_HI_IN_USB_DP_BITS
#undef SIO_GPIO_HI_IN_USB_DM_BITS
#define SIO_GPIO_HI_IN_USB_DP_BITS (1u << 24)
#define SIO_GPIO_HI_IN_USB_DM_BITS (1u << 25)
#define SIO_MTIME_CTRL_EN_BITS 1u
#define SIO_MTIME_CTRL_FULLSPEED_BITS 2u
#define __wfe() ((void)0)
#define __dsb() ((void)0)
#define __isb() ((void)0)
static struct {
    volatile uint32_t mtime, mtimeh, mtimecmp, mtimecmph, mtime_ctrl, gpio_hi_in;
} router_test_sio;
#undef sio_hw
#define sio_hw (&router_test_sio)
#include "router.c"

usb_hw_t native_test_usb;
uint32_t native_test_interrupt_mask;
static unsigned selections;
static uint8_t selected_address, selected_owner;
static bool accept_selection = true;

void native_test_service_interrupt(void) {}
bool native_hub_select_device(uint8_t address, uint8_t owner, uint32_t cutoff) {
    (void)cutoff;
    ++selections;
    selected_address = address;
    selected_owner = owner;
    return accept_selection;
}

static const routing_table* current_table(void) {
    uint32_t generation;
    return acquire_table(&generation);
}

static void expect_route(const routing_table* table, unsigned address, uint8_t owner) {
    selections = 0;
    raw_packet packet = {0};
    route_header(table,address,TOKEN_SETUP_SIGNATURE,127,100,&packet);
    uint32_t sequence;
    assert(probe_router_setup_slot(&sequence) == owner);
    probe_router_stats snapshot;
    probe_router_snapshot(&snapshot);
    assert(snapshot.last_setup_slot == owner && snapshot.last_setup_sequence == sequence);
    if (owner == PROBE_ROUTER_UNASSIGNED) {
        assert(selections == 0 && packet.retargets == 0);
    } else {
        assert(selections == 1 && selected_address == address && selected_owner == owner);
        assert(packet.retargets == (address != 127));
    }
}

static uint8_t address_wire(unsigned address, unsigned kind) {
    // Independent LSB-first NRZI encoder, starting after the token PID's K.
    unsigned wire = 0, line = 0, ones = kind ? 3u : 0u, bit_index = 0;
    for (unsigned symbol = 0; symbol < 8; ++symbol) {
        unsigned bit;
        if (ones == 6) {
            bit = 0;
        } else {
            bit = bit_index < 7 ? (address >> bit_index) & 1u : 0u;
            ++bit_index;
        }
        if (!bit) line ^= 1u;
        wire |= line << symbol;
        ones = bit ? ones + 1u : 0u;
    }
    return wire;
}

static unsigned raw_prefix(uint8_t wire) {
    unsigned prefix = 0;
    for (unsigned bit = 0; bit < 4; ++bit)
        prefix |= ((wire >> bit) & 1u ? LINE_J : LINE_K) << (2u * bit);
    return prefix;
}

static void expect_prefixes(const routing_table* table, const uint8_t* addresses) {
    for (unsigned kind = 0; kind < 2; ++kind) {
        for (uint8_t slot = 0; slot < PROBE_ROUTER_SLOTS; ++slot) {
            const uint8_t wire = address_wire(addresses[slot],kind);
            const unsigned prefix = raw_prefix(wire);
            unsigned matches = 0;
            for (uint8_t other = 0; other < PROBE_ROUTER_SLOTS; ++other)
                matches += raw_prefix(address_wire(addresses[other],kind)) == prefix;
            assert(table->early_address[kind][prefix] ==
                   (matches == 1 ? addresses[slot] : PROBE_ROUTER_UNASSIGNED));
            expect_route(table,address_decoder[kind][wire],slot);
        }
    }
}

int main(void) {
    probe_router_init(FS_CLOCK_HZ);
    // Simulate observer readiness, not USB timing; this enables the actual
    // routing decision without starting the hardware-bound sampling loop.
    counters.ready = 1;
    probe_router_enable(true);
    const routing_table* table = current_table();
    expect_route(table,0,0);
    for (unsigned address = 1; address < 128; ++address)
        expect_route(table,address,PROBE_ROUTER_UNASSIGNED);

    uint8_t addresses[PROBE_ROUTER_SLOTS];
    addresses[0] = 9;
    for (uint8_t slot = 1; slot < PROBE_ROUTER_SLOTS; ++slot) addresses[slot] = 17u * slot;
    probe_router_publish(addresses,PROBE_ROUTER_UNASSIGNED);
    table = current_table();
    expect_prefixes(table,addresses);
    expect_route(table,0,PROBE_ROUTER_UNASSIGNED);
    expect_route(table,128,PROBE_ROUTER_UNASSIGNED);
    expect_route(table,255,PROBE_ROUTER_UNASSIGNED);

    // Every child's address shares the first four symbols. No early owner may
    // be guessed, even though the full decoded addresses still route uniquely.
    for (uint8_t slot = 1; slot < PROBE_ROUTER_SLOTS; ++slot) addresses[slot] = 1u + 16u * slot;
    probe_router_publish(addresses,PROBE_ROUTER_UNASSIGNED);
    table = current_table();
    expect_prefixes(table,addresses);

    // Slot 4 must not collide with the invalid sentinel or sequence carry.
    setup_publication = SETUP_SEQUENCE_MASK - 1u;
    expect_route(table,addresses[PROBE_ROUTER_SLOTS - 1],PROBE_ROUTER_SLOTS - 1);
    uint32_t sequence;
    assert(probe_router_setup_slot(&sequence) == PROBE_ROUTER_SLOTS - 1 && sequence == SETUP_SEQUENCE_MASK);
    expect_route(table,addresses[PROBE_ROUTER_SLOTS - 1],PROBE_ROUTER_SLOTS - 1);
    assert(probe_router_setup_slot(&sequence) == PROBE_ROUTER_SLOTS - 1 && sequence == 0);
    expect_route(table,127,PROBE_ROUTER_UNASSIGNED);
    assert(probe_router_setup_slot(&sequence) == PROBE_ROUTER_UNASSIGNED && sequence == 0);

    accept_selection = false;
    selections = 0;
    raw_packet packet = {0};
    route_header(table,addresses[1],TOKEN_SETUP_SIGNATURE,127,100,&packet);
    assert(selections == 1 && packet.retargets == 0 &&
           probe_router_setup_slot(&sequence) == PROBE_ROUTER_UNASSIGNED);
    accept_selection = true;

    addresses[1] = addresses[2];
    probe_router_publish(addresses,PROBE_ROUTER_SLOTS - 1);
    table = current_table();
    expect_route(table,addresses[1],PROBE_ROUTER_UNASSIGNED);
    expect_route(table,0,PROBE_ROUTER_SLOTS - 1);
    for (unsigned kind = 0; kind < 2; ++kind)
        for (unsigned prefix = 0; prefix < 256; ++prefix)
            assert(table->early_address[kind][prefix] != addresses[1]);
    probe_router_publish(addresses,PROBE_ROUTER_UNASSIGNED);
    expect_route(current_table(),0,PROBE_ROUTER_UNASSIGNED);
    printf("native router ownership regressions passed for %u slots\n",PROBE_ROUTER_SLOTS);
    return 0;
}
