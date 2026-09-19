#include "hardware_stub.h"
#include "router.h"
#include <assert.h>
#include <stdio.h>

// The real router tables, deadline sampler and token-header decision run on
// the host. Scripted register reads exercise timing boundaries, not physical
// pad latency, instruction timing or USB signal integrity.
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
typedef struct {
    volatile uint32_t mtime, mtimeh, mtimecmp, mtimecmph, mtime_ctrl, gpio_hi_in;
} router_test_registers;
static router_test_registers router_test_sio;
enum { MANUAL_READS, IDLE_READS, DRAIN_READS };
static unsigned read_mode, register_reads, nonidle_read;
static uint32_t simulated_start, simulated_elapsed;

static router_test_registers* router_test_read_registers(void) {
    if (read_mode == IDLE_READS) {
        ++register_reads;
        router_test_sio.mtime = simulated_start +
            simulated_elapsed * (register_reads - 1u) / (FS_IDLE_POLLS + 1u);
        router_test_sio.gpio_hi_in = (register_reads == nonidle_read ? 2u : 1u) << 24;
    } else if (read_mode == DRAIN_READS) {
        ++register_reads;
        // SE0 is observed at zero and returns to J after the requested delay.
        // A subsequent K either becomes SOP after qualified EOP or remains
        // untrusted packet data until the real capture drain timeout expires.
        router_test_sio.mtime = register_reads < 4u ? 0u :
            register_reads < 9u ? simulated_elapsed : FS_CLOCK_HZ / 10000u;
        router_test_sio.gpio_hi_in =
            (register_reads < 4u ? 0u : register_reads < 7u ? 1u : 2u) << 24;
    }
    return &router_test_sio;
}
#undef sio_hw
#define sio_hw router_test_read_registers()
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

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
void native_hub_note_selected_token(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid) {
    (void)address; (void)owner; (void)cutoff; (void)pid;
}
void native_hub_note_failed_select(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid) {
    (void)address; (void)owner; (void)cutoff; (void)pid;
}
#endif

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

static void test_clock_and_phase_guards(void) {
    const uint32_t bad_clocks[] = {
        FS_CLOCK_HZ == 240000000u ? 300000000u : 240000000u,
        FS_CLOCK_HZ + 1u,
        150000000u,
    };
    for (unsigned i = 0; i < sizeof(bad_clocks) / sizeof(bad_clocks[0]); ++i) {
        probe_router_init(bad_clocks[i]);
        assert(!probe_router_set_phase(0));
        // Even a stale ready flag cannot arm a differently compiled receiver.
        counters.ready = 1;
        probe_router_enable(true);
        selections = 0;
        raw_packet packet = {0};
        route_header(current_table(),0,TOKEN_SETUP_SIGNATURE,127,100,&packet);
        assert(!selections && !packet.retargets);
    }

    probe_router_init(FS_CLOCK_HZ);
    assert(probe_router_set_phase(0));
    assert(probe_router_set_phase(FS_BIT_CYCLES - 1u));
    assert(!probe_router_set_phase(FS_BIT_CYCLES));
    assert(!probe_router_set_phase(UINT32_MAX));
    counters.ready = 1;
    probe_router_enable(true);
    assert(!probe_router_set_phase(0));
    probe_router_enable(false);
    assert(probe_router_set_phase(0));
}

static void test_sample_deadlines(void) {
    const uint32_t bit_cycles = FS_CLOCK_MHZ == 300u ? 25u : 20u;
    uint32_t deadline = 1000u, line = LINE_SE1;
    router_test_sio.mtime = deadline;
    router_test_sio.gpio_hi_in = LINE_J << 24;
    assert(sample_line(&deadline,&line));
    assert(line == LINE_J && deadline == 1000u + bit_cycles);

    router_test_sio.mtime = deadline + bit_cycles - 1u;
    router_test_sio.gpio_hi_in = LINE_K << 24;
    assert(sample_line(&deadline,&line));
    assert(line == LINE_K && deadline == 1000u + 2u * bit_cycles);

    router_test_sio.mtime = deadline + bit_cycles;
    router_test_sio.gpio_hi_in = LINE_J << 24;
    assert(!sample_line(&deadline,&line));
    assert(line == LINE_K && deadline == 1000u + 2u * bit_cycles);

    deadline = UINT32_MAX - bit_cycles + 1u;
    router_test_sio.mtime = deadline;
    assert(sample_line(&deadline,&line));
    assert(line == LINE_J && deadline == 0u);
    router_test_sio.mtime = bit_cycles - 1u;
    assert(sample_line(&deadline,&line) && deadline == bit_cycles);
}

static void test_drain_qualification(void) {
    const uint32_t bit_cycles = FS_CLOCK_MHZ == 300u ? 25u : 20u;
    read_mode = IDLE_READS;
    simulated_start = UINT32_MAX - 100u;
    simulated_elapsed = 8u * bit_cycles - 1u;
    register_reads = nonidle_read = 0;
    assert(!observe_idle_j());
    simulated_elapsed = 8u * bit_cycles;
    register_reads = 0;
    assert(observe_idle_j());
    register_reads = 0;
    nonidle_read = FS_IDLE_POLLS / 2u;
    assert(!observe_idle_j());

    read_mode = DRAIN_READS;
    simulated_elapsed = (bit_cycles + 1u) / 2u - 1u;
    register_reads = 0;
    raw_packet packet = capture_packet(PROBE_ROUTER_DEFAULT_PHASE,current_table(),true);
    assert(!packet.sop && packet.resync);
    simulated_elapsed = (bit_cycles + 1u) / 2u;
    register_reads = 0;
    packet = capture_packet(PROBE_ROUTER_DEFAULT_PHASE,current_table(),true);
    assert(packet.sop && packet.late);
    read_mode = MANUAL_READS;
}

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
static void test_root_response_attribution(void) {
    const uint32_t nak = 0x96a5a666u, data1 = 0x965aa666u;
    probe_router_stats snapshot;
    probe_router_init(FS_CLOCK_HZ);
    counters.ready = 1;
    probe_router_enable(true);
    uint8_t addresses[PROBE_ROUTER_SLOTS];
    for (unsigned i = 0; i < PROBE_ROUTER_SLOTS; ++i) addresses[i] = 5u+i;
    probe_router_publish(addresses,PROBE_ROUTER_UNASSIGNED);
    const routing_table* table = current_table();
    raw_packet packet = {0};

    // Two decisions for one physical token must retain a single observation.
    route_header(table,5,TOKEN_IN_SIGNATURE,0,1000,&packet);
    route_header(table,5,TOKEN_IN_SIGNATURE,0,1000,&packet);
    router_test_sio.mtime = 1100;
    observe_discarded_header(nak);
    root_observe_eop(1200,true);
    probe_router_snapshot(&snapshot);
    assert(snapshot.root_in_count == 1 && snapshot.root_header == nak);
    assert(snapshot.root_header_cycle == 1100 && snapshot.root_eop_cycle == 1200);

    // Recovery's own response cannot erase the pre-SETUP NAK observation.
    route_header(table,5,TOKEN_SETUP_SIGNATURE,0,2000,&packet);
    route_header(table,5,TOKEN_IN_SIGNATURE,0,2200,&packet);
    router_test_sio.mtime = 2250;
    observe_discarded_header(data1);
    root_observe_eop(2300,true);
    probe_router_snapshot(&snapshot);
    assert(snapshot.root_header == data1);
    assert(snapshot.before_setup_in_count == 1 && snapshot.before_setup_in_cutoff == 1000);
    assert(snapshot.before_setup_header == nak && snapshot.before_setup_header_cycle == 1100);
    assert(snapshot.before_setup_eop_cycle == 1200);

    // A child token, even a rejected or unmapped one, ends root attribution.
    for (unsigned kind = 0; kind < 3; ++kind) {
        route_header(table,5,TOKEN_IN_SIGNATURE,0,3000+kind*1000,&packet);
        accept_selection = kind != 2;
        route_header(table,kind == 1 ? 127 : 6,TOKEN_IN_SIGNATURE,0,3100+kind*1000,&packet);
        accept_selection = true;
        observe_discarded_header(nak);
        root_observe_eop(3200+kind*1000,true);
        probe_router_snapshot(&snapshot);
        assert(snapshot.root_header == 0 && snapshot.root_eop_cycle == 0);
    }

    // Idle qualification is not an observed EOP; a later packet cannot fill it.
    route_header(table,5,TOKEN_IN_SIGNATURE,0,6000,&packet);
    observe_discarded_header(nak);
    root_observe_eop(6100,false);
    root_observe_eop(6200,true);
    probe_router_snapshot(&snapshot);
    assert(snapshot.root_header == nak && snapshot.root_eop_cycle == 0);

    counters.root_in_count = UINT32_MAX;
    route_header(table,5,TOKEN_IN_SIGNATURE,0,7000,&packet);
    route_header(table,5,TOKEN_IN_SIGNATURE,0,7000,&packet);
    probe_router_snapshot(&snapshot);
    assert(snapshot.root_in_count == 0);

    probe_router_init(FS_CLOCK_HZ);
    observe_discarded_header(nak);
    probe_router_snapshot(&snapshot);
    assert(snapshot.root_header == 0 && snapshot.before_setup_header == 0);
}
#endif

int main(void) {
    test_clock_and_phase_guards();
    test_sample_deadlines();
    test_drain_qualification();
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    test_root_response_attribution();
#endif
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
