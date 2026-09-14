#include "router.h"

#include <string.h>

#include "pico.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"

#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
extern bool native_hub_select_device(uint8_t address, uint8_t owner, uint32_t cutoff);
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
#include "usb/native_hub/native_hub_trace.h"
#endif
#endif

#if !PICO_RP2350 || defined(__riscv)
#error "The native PHY observer requires an RP2350 Arm core"
#endif

// This sampler does not drive USB data. Main enables the native-pad input
// mux and attachment pull-up; the native SIE remains the USB transmitter.
// This isolated probe owns SIO MTIME, usable by a Secure Arm core. FULLSPEED
// makes it a zero-wait-state cycle counter next to the GPIO inputs, avoiding
// SysTick's PPB accesses and 24-bit down-counter arithmetic in every sample.
// Deadlines use modular 32-bit arithmetic for intervals below 2^31 cycles.
#define FS_CLOCK_HZ 240000000u
#define FS_BIT_CYCLES 20u
#define LINE_SE0 0u
#define LINE_J 1u
#define LINE_K 2u
#define LINE_SE1 3u
#define PID_OUT 0xe1u
#define PID_IN 0x69u
#define PID_SETUP 0x2du
// NRZI SYNC+PID words, with K=2 and J=1 packed into two-bit samples.
#define TOKEN_OUT_SIGNATURE 0xaa66a666u
#define TOKEN_IN_SIGNATURE 0x95a6a666u
#define TOKEN_SETUP_SIGNATURE 0x9a56a666u
#define NO_READER 2u
#define SETUP_SEQUENCE_MASK 0x3fffffffu
#define SETUP_SLOT_SHIFT 30u
#define SETUP_INVALID (3u << SETUP_SLOT_SHIFT)
#define RAW_BITS 40u

_Static_assert(SIO_GPIO_HI_IN_USB_DP_BITS == (1u << 24), "SIO USB DP layout");
_Static_assert(SIO_GPIO_HI_IN_USB_DM_BITS == (1u << 25), "SIO USB DM layout");
_Static_assert(PROBE_ROUTER_SLOTS == 3u, "Packed setup owner has three slots");

typedef struct {
    uint8_t owner[128];
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
    uint8_t early_address[2][256];
#endif
} routing_table;

// Complete physical NRZI SYNC+PID signatures. The PID's two distinguishing
// symbols index this table, but the entire signature must match.
static uint32_t token_words[16];

typedef struct {
    uint32_t words[3];
    uint32_t count;
    uint32_t retargets;
    bool eop;
    bool late;
    bool sop;
    bool resync;
} raw_packet;

static routing_table tables[2];
static probe_router_stats counters;
static uint32_t published_generation;
static uint32_t reader_index;
static uint32_t enabled;
static uint32_t phase_cycles;
static uint32_t setup_publication;
static uint32_t fatal_fault;
static bool valid_clock;
static uint8_t address_decoder[2][256];
static bool address_decoder_ready;

static __force_inline uint32_t atomic_read(const uint32_t* value) {
    return __atomic_load_n(value, __ATOMIC_RELAXED);
}

static __force_inline void atomic_write(uint32_t* value, uint32_t next) {
    __atomic_store_n(value, next, __ATOMIC_RELAXED);
}

// These counters have one writer (Core 1); only loads/stores, not exclusive
// read-modify-write loops, are needed. They are updated outside sample windows.
static __force_inline void count_one(uint32_t* counter) {
    atomic_write(counter, atomic_read(counter) + 1u);
}

static __force_inline void invalidate_setup(void) {
    const uint32_t previous = atomic_read(&setup_publication);
    __atomic_store_n(&setup_publication,
                     (previous & SETUP_SEQUENCE_MASK) | SETUP_INVALID,
                     __ATOMIC_RELEASE);
}

static __force_inline void publish_setup(uint8_t slot) {
    const uint32_t sequence = (atomic_read(&setup_publication) + 1u) & SETUP_SEQUENCE_MASK;
    const uint32_t owner = slot < PROBE_ROUTER_SLOTS ? slot : 3u;
    __atomic_store_n(&setup_publication, sequence | (owner << SETUP_SLOT_SHIFT),
                     __ATOMIC_RELEASE);
}


static void build_address_decoder(void) {
    if (address_decoder_ready) return;
    // C0 initializes once. Eight observed D+ symbols cover all seven address
    // bits plus at most one stuffed bit. All three token PIDs end in K.
    for (unsigned kind = 0; kind < 2; ++kind) {
        for (unsigned wire = 0; wire < 256; ++wire) {
            unsigned previous = 0, ones = kind ? 3u : 0u, bits = 0, address = 0;
            bool valid = true;
            for (unsigned n = 0; n < 8 && bits < 7; ++n) {
                const unsigned line = (wire >> n) & 1u;
                const unsigned bit = line == previous;
                previous = line;
                if (ones == 6u) {
                    if (bit != 0u) valid = false;
                    ones = 0;
                    continue;
                }
                address |= bit << bits++;
                ones = bit ? ones + 1u : 0u;
            }
            address_decoder[kind][wire] = valid && bits == 7 ?
                (uint8_t)address : PROBE_ROUTER_UNASSIGNED;
        }
    }
    address_decoder_ready = true;
}

static void build_table(routing_table* table,
                        const uint8_t addresses[PROBE_ROUTER_SLOTS], uint8_t default_slot) {
    build_address_decoder();
    memset(table->owner, PROBE_ROUTER_UNASSIGNED, sizeof(table->owner));
    if (default_slot < PROBE_ROUTER_SLOTS)
        table->owner[0] = default_slot;
    for (uint8_t slot = 0; slot < PROBE_ROUTER_SLOTS; ++slot) {
        const uint8_t address = addresses[slot];
        if (address == 0 || address >= 128) continue;
        bool unique = true;
        for (uint8_t other = 0; other < PROBE_ROUTER_SLOTS; ++other) {
            if (other != slot && addresses[other] == address)
                unique = false;
        }
        if (unique)
            table->owner[address] = slot;
    }
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
    // A unique observed prefix can preselect the SIE sooner. It still compares
    // the complete hardware address and CRC before accepting the transaction.
    // Index the four captured line pairs directly. Packing their D+ bits in
    // the sampling window delays bit 20 even when the prefix is ambiguous.
    memset(table->early_address, PROBE_ROUTER_UNASSIGNED, sizeof(table->early_address));
    for (unsigned kind = 0; kind < 2; ++kind) {
        for (unsigned prefix = 0; prefix < 16; ++prefix) {
            uint8_t candidate = PROBE_ROUTER_UNASSIGNED;
            for (unsigned suffix = 0; suffix < 16; ++suffix) {
                uint8_t address = address_decoder[kind][prefix | (suffix << 4)];
                if (address >= 128 || table->owner[address] >= PROBE_ROUTER_SLOTS) continue;
                if (candidate != PROBE_ROUTER_UNASSIGNED && candidate != address) {
                    candidate = PROBE_ROUTER_UNASSIGNED;
                    break;
                }
                candidate = address;
            }
            unsigned raw_prefix = 0u;
            for (unsigned bit = 0; bit < 4; ++bit)
                raw_prefix |= ((prefix >> bit) & 1u ? LINE_J : LINE_K) << (2u * bit);
            table->early_address[kind][raw_prefix] = candidate;
        }
    }
#endif
}

void probe_router_init(uint32_t system_clock_hz) {
    // Explicit SRAM data: Core1 must never fetch flash during durable saves.
    token_words[6] = TOKEN_OUT_SIGNATURE;
    token_words[10] = TOKEN_IN_SIGNATURE;
    token_words[5] = TOKEN_SETUP_SIGNATURE;
    const uint8_t addresses[PROBE_ROUTER_SLOTS] = {0u, PROBE_ROUTER_UNASSIGNED,
                                                PROBE_ROUTER_UNASSIGNED};
    memset(&counters, 0, sizeof(counters));
    published_generation = 0u;
    reader_index = NO_READER;
    enabled = 0u;
    phase_cycles = 0u;
    setup_publication = SETUP_INVALID;
    fatal_fault = 0u;
    valid_clock = system_clock_hz == FS_CLOCK_HZ;
    counters.cycles_per_bit = system_clock_hz / 12000000u;
    build_table(&tables[0], addresses, 0u);
}

void probe_router_publish(const uint8_t addresses[PROBE_ROUTER_SLOTS], uint8_t default_slot) {
    const uint32_t generation = atomic_read(&published_generation);
    const uint32_t next_index = (generation + 1u) & 1u;
    // A pointer swap alone is NOT safe double buffering: a second publication
    // could overwrite the table still in use by a packet. The reader's hazard
    // index protects that table until decoding finishes. Core 1 never waits.
    // Bound the writer's wait as well: an unexpectedly stopped observer must
    // not trap Core 0 or prevent the watchdog/reboot control path from running.
    uint32_t remaining = 1000000u;
    while (__atomic_load_n(&reader_index, __ATOMIC_SEQ_CST) == next_index) {
        if (--remaining == 0u) {
            atomic_write(&enabled, 0u);
            atomic_write(&fatal_fault, 1u);
            atomic_write(&counters.ready, 0u);
            return;
        }
    }
    build_table(&tables[next_index], addresses, default_slot);
    __atomic_store_n(&published_generation, generation + 1u, __ATOMIC_SEQ_CST);
}

void probe_router_enable(bool enable) {
    // ARM qualification (observed hub tokens/SETUPs) belongs to the control
    // request handler. This additionally prevents enabling a failed observer.
    __atomic_store_n(&enabled, enable && atomic_read(&counters.ready) != 0u &&
                     atomic_read(&fatal_fault) == 0u, __ATOMIC_RELEASE);
}

bool probe_router_set_phase(uint32_t cycles) {
    if (cycles >= atomic_read(&counters.cycles_per_bit) || atomic_read(&enabled) != 0u)
        return false;
    __atomic_store_n(&phase_cycles, cycles, __ATOMIC_RELEASE);
    return true;
}

void probe_router_snapshot(probe_router_stats* out) {
#define SNAPSHOT(member) out->member = atomic_read(&counters.member)
    SNAPSHOT(ready);
    SNAPSHOT(sops);
    SNAPSHOT(sync_ok);
    SNAPSHOT(valid_tokens);
    SNAPSHOT(valid_setups);
    SNAPSHOT(crc_errors);
    SNAPSHOT(late_samples);
    SNAPSHOT(retargets);
    for (uint32_t slot = 0u; slot < PROBE_ROUTER_SLOTS; ++slot)
        out->address_hits[slot] = atomic_read(&counters.address_hits[slot]);
    SNAPSHOT(cycles_per_bit);
    SNAPSHOT(last_pid);
    SNAPSHOT(last_address);
    for (uint32_t i = 0; i < 3; ++i)
        out->last_raw[i] = atomic_read(&counters.last_raw[i]);
    SNAPSHOT(last_raw_count);
    SNAPSHOT(last_raw_eop);
    SNAPSHOT(last_raw_late);
#undef SNAPSHOT
    const uint32_t setup = __atomic_load_n(&setup_publication, __ATOMIC_ACQUIRE);
    out->last_setup_sequence = setup & SETUP_SEQUENCE_MASK;
    const uint32_t slot = setup >> SETUP_SLOT_SHIFT;
    out->last_setup_slot = slot < PROBE_ROUTER_SLOTS ? slot : PROBE_ROUTER_UNASSIGNED;
}

uint8_t probe_router_setup_slot(uint32_t* sequence) {
    const uint32_t setup = __atomic_load_n(&setup_publication, __ATOMIC_ACQUIRE);
    *sequence = setup & SETUP_SEQUENCE_MASK;
    const uint32_t slot = setup >> SETUP_SLOT_SHIFT;
    return slot < PROBE_ROUTER_SLOTS ? (uint8_t)slot : PROBE_ROUTER_UNASSIGNED;
}

static __force_inline uint32_t cycles_now(void) {
    return sio_hw->mtime;
}

static __force_inline int32_t cycles_after(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline);
}

static __force_inline uint32_t receive_line(void) {
    // Native-mode measurements returned zero here while PHY_DIRECT saw traffic.
    // Main can select USBPHY_AS_GPIO to test the separate native-pad SIO path.
    return (sio_hw->gpio_hi_in >> 24) & 3u;
}

static __force_inline bool sample_line(uint32_t* deadline, uint32_t* line) {
    uint32_t now;
    do {
        now = cycles_now();
    } while (cycles_after(now, *deadline) < 0);
    // Reuse the wait-loop timestamp instead of a second timer access per bit.
    // A full-bit overrun is definitely a missed sample. Edge-poll timing still
    // needs calibration: the host correlates sampled headers with actual
    // hardware-accepted SETUP requests before enabling address writes.
    if (cycles_after(now, *deadline) >= (int32_t)FS_BIT_CYCLES)
        return false;
    *line = receive_line();
    *deadline += FS_BIT_CYCLES;
    // Keep one rolling deadline. GCC's unrolled affine expansion otherwise
    // retains SOP/phase and spills/rebuilds per-bit deadlines in the hot path.
    __asm volatile ("" : "+r"(*deadline));
    return true;
}

static __force_inline void route_header(const routing_table* table, uint32_t address,
                                        uint32_t signature, uint32_t initial_address,
                                        uint32_t cutoff, raw_packet* packet) {
    // TinyUSB clears SETUP_REC only AFTER copying the hardware-validated SETUP
    // into its event callback. Until then, preserve both address and owner.
    if (usb_hw->sie_status & USB_SIE_STATUS_SETUP_REC_BITS)
        return;
    invalidate_setup();
    if (address >= 128u || table->owner[address] >= PROBE_ROUTER_SLOTS)
        return;
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
    if (atomic_read(&enabled) != 0u) {
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
        const uint8_t pid = signature == TOKEN_OUT_SIGNATURE ? PID_OUT :
            signature == TOKEN_IN_SIGNATURE ? PID_IN : PID_SETUP;
        const bool selected = (pid == PID_OUT || (pid == PID_IN && table->owner[address] == 0))
            ? native_hub_select_device_traced((uint8_t)address, table->owner[address], cutoff, pid)
            : native_hub_select_device((uint8_t)address, table->owner[address], cutoff);
#else
        const bool selected = native_hub_select_device((uint8_t)address, table->owner[address], cutoff);
#endif
        if (!selected) {
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
            native_hub_note_failed_select((uint8_t)address, table->owner[address], cutoff, pid);
#endif
            return;
        }
        if (initial_address != address) ++packet->retargets;
    }
#else
    if (initial_address != address && atomic_read(&enabled) != 0u) {
        if (cycles_after(cycles_now(), cutoff) >= 0) {
            packet->late = true;
            return;
        }
        __dmb();
        usb_hw->dev_addr_ctrl = address;
        ++packet->retargets;
    }
#endif
    // Candidate observations qualify calibration only. Runtime ownership
    // comes from the hardware address frozen by SETUP_REC. A missed software
    // candidate must not reject a correctly addressed, hardware-accepted SETUP.
    if (signature == TOKEN_SETUP_SIGNATURE)
        publish_setup(table->owner[address]);
}

// The timing-critical path samples the complete address before selecting the
// native SIE. Hardware SETUP acceptance qualifies the candidate; opportunistic
// full-token CRC decoding below is diagnostic, not an ownership authority.
static bool observe_idle_j(void);

// Prepare before waiting for EOP: an ACK can be followed immediately by a poll.
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
static raw_packet __no_inline_not_in_flash_func(capture_packet)(
    uint32_t phase, const routing_table* table, bool draining) {
#else
static raw_packet __no_inline_not_in_flash_func(capture_packet)(
    uint32_t phase, const routing_table* table) {
#endif
    raw_packet result = {0};
    uint32_t word0 = LINE_K, word1 = 0u, word2 = 0u;
    const uint8_t* decoder = NULL;
    uint32_t expected_word = 0u;
    const uint32_t initial_address = usb_hw->dev_addr_ctrl;
    #if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
    const uint8_t* early_decoder = NULL;
    #endif
    // Complete capture preparation before looking for the edge. The first
    // hardware traces showed that preparing this state after SOP lost bit 1.
    // Later zero-valued accumulators must remain constants until first use;
    // forcing them into live registers adds spills and unnecessary ORs.
    __asm volatile ("" : "+r"(word0), "+m"(result) : : "memory");
    #if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
drain_prepared_capture:;
    if (draining) {
        const uint32_t stop = cycles_now() + FS_CLOCK_HZ / 10000u;
        bool saw_se0 = false;
        uint32_t se0_since = 0;
        for (;;) {
            uint32_t line = receive_line(), now = cycles_now();
            if (cycles_after(now,stop) >= 0) { result.resync = true; return result; }
            if (line == LINE_SE0) {
                if (!saw_se0) se0_since = now;
                saw_se0 = true;
            } else {
                // Half a bit rejects pad skew while allowing late ACK EOP entry.
                if (line == LINE_J && saw_se0 &&
                    cycles_after(now,se0_since) >= (int32_t)(FS_BIT_CYCLES / 2u)) break;
                if (line == LINE_J && observe_idle_j()) break;
                saw_se0 = false;
            }
        }
    }
    #endif
    uint32_t line = receive_line();
    if (line != LINE_J) {
        result.resync = true;
        return result;
    }
    // A falling D+ leaves full-speed idle. Inspect the complete captured pair
    // before accepting K; defer normalization until after the polling loop.
    // Eight straight polls amortize loop bookkeeping and reduce edge jitter.
    uint32_t pins;
#define POLL_IDLE() do { \
        pins = sio_hw->gpio_hi_in; \
        if ((pins & SIO_GPIO_HI_IN_USB_DP_BITS) == 0u) goto edge; \
    } while (0)
    #if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
    for (;;) {
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        // Keep the prepared frame while idle; leave only for a table update/fault.
        if ((atomic_read(&published_generation) & 1u) != atomic_read(&reader_index) ||
            atomic_read(&fatal_fault) != 0u) return result;
    }
    #else
    for (unsigned poll = 0; poll < 512u; ++poll) {
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
        POLL_IDLE(); POLL_IDLE(); POLL_IDLE(); POLL_IDLE();
    }
    #endif
#undef POLL_IDLE
    return result;
edge:
    line = (pins >> 24) & 3u;
    if (line != LINE_K) {
        result.resync = true;
        return result;
    }
    const uint32_t sop_time = cycles_now();
    // This timestamp follows the PHY read and edge-detection instructions.
    // Captures showed an extra full-bit delay skipped SYNC's second symbol.
    // Sweep the next sample relative to read completion, then keep 20-cycle
    // spacing; every stored line symbol is still physically observed.
    uint32_t deadline = sop_time + phase;
    result.sop = true;
    // The first stored K is the observed SOP above, not an invented SYNC bit.
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
#define SET_EARLY_DECODER(kind) (early_decoder = table->early_address[kind])
// PID rejection happens before any address/body samples. Reuse the prepared
// frame: rebuilding its stack image here can miss the end of a short ACK/NAK
// and the following token. All other result/address accumulators are still zero.
#define DISCARD_NON_TOKEN() do { \
        result.sop = false; word0 = LINE_K; draining = true; \
        goto drain_prepared_capture; \
    } while (0)
#define ROUTE_EARLY(bit, base) do { \
        if ((base) + (bit) == 19u && decoder != NULL) { \
            uint8_t candidate = early_decoder[word1 & 0xffu]; \
            if (candidate < 128u) \
                route_header(table, candidate, word0, initial_address, \
                             deadline + 11u * FS_BIT_CYCLES, &result); \
        } \
    } while (0)
#else
#define SET_EARLY_DECODER(kind) ((void)0)
#define DISCARD_NON_TOKEN() ((void)0)
#define ROUTE_EARLY(bit, base) ((void)0)
#endif
#define ROUTE_BITS(word, bit, base) do { \
        if ((base) + (bit) == 11u) { \
            const uint32_t index = (word0 >> 20) & 15u; \
            expected_word = token_words[index]; \
            decoder = address_decoder[index == 6u]; \
            SET_EARLY_DECODER(index == 6u); \
        } \
        if ((base) + (bit) == 15u && word0 != expected_word) { \
            decoder = NULL; \
            DISCARD_NON_TOKEN(); \
        } \
        ROUTE_EARLY(bit, base); \
        if ((base) + (bit) == 23u && decoder != NULL) { \
            /* Compact observed D+ bits only after all eight symbols exist. */ \
            uint32_t address_wire = word1 & 0x5555u; \
            address_wire = (address_wire | (address_wire >> 1u)) & 0x3333u; \
            address_wire = (address_wire | (address_wire >> 2u)) & 0x0f0fu; \
            address_wire = (address_wire | (address_wire >> 4u)) & 0xffu; \
            route_header(table, decoder[address_wire], word0, \
                         initial_address, deadline + 7u * FS_BIT_CYCLES, &result); \
            if (result.late) { result.count = (base) + (bit) + 1u; goto done; } \
        } \
    } while (0)
#define CAPTURE(word, bit, base) do { \
        if (!sample_line(&deadline, &line)) { \
            result.count = (base) + (bit); goto late; \
        } \
        if (line == LINE_SE0) { result.count = (base) + (bit); goto eop; } \
        (word) |= line << (2u * (bit)); \
        ROUTE_BITS(word, bit, base); \
    } while (0)
#define CAPTURE_16(word, base) \
    CAPTURE(word, 0u, base); CAPTURE(word, 1u, base); \
    CAPTURE(word, 2u, base); CAPTURE(word, 3u, base); \
    CAPTURE(word, 4u, base); CAPTURE(word, 5u, base); \
    CAPTURE(word, 6u, base); CAPTURE(word, 7u, base); \
    CAPTURE(word, 8u, base); CAPTURE(word, 9u, base); \
    CAPTURE(word, 10u, base); CAPTURE(word, 11u, base); \
    CAPTURE(word, 12u, base); CAPTURE(word, 13u, base); \
    CAPTURE(word, 14u, base); CAPTURE(word, 15u, base)
    CAPTURE(word0, 1u, 0u); CAPTURE(word0, 2u, 0u);
    CAPTURE(word0, 3u, 0u); CAPTURE(word0, 4u, 0u);
    CAPTURE(word0, 5u, 0u); CAPTURE(word0, 6u, 0u);
    CAPTURE(word0, 7u, 0u); CAPTURE(word0, 8u, 0u);
    CAPTURE(word0, 9u, 0u); CAPTURE(word0, 10u, 0u);
    CAPTURE(word0, 11u, 0u); CAPTURE(word0, 12u, 0u);
    CAPTURE(word0, 13u, 0u); CAPTURE(word0, 14u, 0u);
    CAPTURE(word0, 15u, 0u);
    CAPTURE_16(word1, 16u);
    CAPTURE(word2, 0u, 32u); CAPTURE(word2, 1u, 32u);
    CAPTURE(word2, 2u, 32u); CAPTURE(word2, 3u, 32u);
    CAPTURE(word2, 4u, 32u); CAPTURE(word2, 5u, 32u);
    CAPTURE(word2, 6u, 32u); CAPTURE(word2, 7u, 32u);
#undef CAPTURE_16
#undef CAPTURE
#undef ROUTE_EARLY
#undef SET_EARLY_DECODER
#undef DISCARD_NON_TOKEN
    result.count = RAW_BITS;
    goto done;
eop:
    // Full-speed EOP is two bit times of SE0 followed by one J bit. A reset,
    // truncated packet, or SE1 is not a token. Check all three samples.
    if (!sample_line(&deadline, &line))
        goto late;
    if (line != LINE_SE0)
        goto done;
    if (!sample_line(&deadline, &line))
        goto late;
    result.eop = line == LINE_J;
    goto done;
late:
    result.late = true;
done:
    result.words[0] = word0;
    result.words[1] = word1;
    result.words[2] = word2;
    return result;
}


static bool __not_in_flash_func(observe_idle_j)(void) {
    // Stuffing prohibits eight consecutive J bit times inside a packet.
    // Use tight PHY polling, not sparse timer-paced reads that could miss K.
    const uint32_t start = cycles_now();
    for (uint32_t i = 0; i < 64u; ++i) {
        if (receive_line() != LINE_J)
            return false;
    }
    return cycles_after(cycles_now(), start) >= (int32_t)(8u * FS_BIT_CYCLES);
}

#if !defined(SWITCH2_PROBE_HUB) || !SWITCH2_PROBE_HUB
static __force_inline uint32_t raw_line(const raw_packet* packet, uint32_t bit) {
    return (packet->words[bit >> 4] >> ((bit & 15u) * 2u)) & 3u;
}

static void __not_in_flash_func(decode_packet)(const raw_packet* packet, const routing_table* table) {
    // With LSB-first two-bit line samples, K J K J K J K K is 0xa666.
    if (packet->count < 8u || (packet->words[0] & 0xffffu) != 0xa666u) {
        return;
    }
    count_one(&counters.sync_ok);
    uint32_t previous = LINE_K;
    uint32_t ones = 1u; // Final decoded SYNC bit is one.
    uint32_t decoded = 0u;
    uint32_t value = 0u;
    uint32_t pid = 0u;
    bool token = false;
    for (uint32_t wire_bit = 8u; wire_bit < packet->count; ++wire_bit) {
        const uint32_t line = raw_line(packet, wire_bit);
        if (line != LINE_J && line != LINE_K) {
            return;
        }
        const uint32_t bit = line == previous;
        previous = line;
        if (ones == 6u) {
            if (bit != 0u) {
                return;
            }
            ones = 0u;
            continue;
        }
        ones = bit != 0u ? ones + 1u : 0u;
        if (decoded < 8u) {
            pid |= bit << decoded;
            ++decoded;
            if (decoded == 8u) {
                if ((((pid >> 4) ^ pid) & 15u) != 15u) {
                    return;
                }
                atomic_write(&counters.last_pid, pid);
                token = pid == PID_IN || pid == PID_OUT || pid == PID_SETUP;
                if (!token)
                    return; // Do not parse device data, SOFs, or handshakes.
            }
        } else {
            if (decoded == 24u) {
                return;
            }
            value |= bit << (decoded - 8u);
            ++decoded;
        }
    }
    if (!token || decoded != 24u || ones == 6u || !packet->eop || packet->late) {
        return;
    }
    uint32_t crc = 0x1fu;
    for (uint32_t bit = 0u; bit < 11u; ++bit) {
        const uint32_t feedback = (crc ^ (value >> bit)) & 1u;
        crc >>= 1;
        if (feedback != 0u)
            crc ^= 0x14u; // Reflected x^5 + x^2 + 1.
    }
    if (((crc ^ 0x1fu) & 0x1fu) != (value >> 11)) {
        count_one(&counters.crc_errors);
        return;
    }
    const uint32_t address = value & 0x7fu;
    const uint8_t owner = table->owner[address];
    atomic_write(&counters.last_address, address);
    count_one(&counters.valid_tokens);
    if (owner < PROBE_ROUTER_SLOTS)
        count_one(&counters.address_hits[owner]);
    if (pid == PID_SETUP) {
        count_one(&counters.valid_setups);
    }
}

#endif

static const routing_table* __not_in_flash_func(acquire_table)(uint32_t* generation) {
    for (;;) {
        const uint32_t selected = __atomic_load_n(&published_generation, __ATOMIC_SEQ_CST);
        __atomic_store_n(&reader_index, selected & 1u, __ATOMIC_SEQ_CST);
        if (__atomic_load_n(&published_generation, __ATOMIC_SEQ_CST) == selected) {
            *generation = selected;
            return &tables[selected & 1u];
        }
    }
}

static void __not_in_flash_func(observer_failed)(void) {
    atomic_write(&enabled, 0u);
    atomic_write(&counters.ready, 0u);
    invalidate_setup();
    __atomic_store_n(&reader_index, NO_READER, __ATOMIC_SEQ_CST);
    for (;;)
        __wfe();
}

void __not_in_flash_func(probe_router_core1)(void) {
    (void)save_and_disable_interrupts();
    if (!valid_clock || atomic_read(&fatal_fault) != 0u)
        observer_failed();

    sio_hw->mtime_ctrl = 0u;
    sio_hw->mtimecmp = UINT32_MAX;
    sio_hw->mtimecmph = UINT32_MAX;
    sio_hw->mtime = 0u;
    sio_hw->mtimeh = 0u;
    sio_hw->mtime_ctrl = SIO_MTIME_CTRL_EN_BITS | SIO_MTIME_CTRL_FULLSPEED_BITS;
    __dsb();
    __isb();
    bool timer_running = false;
    uint32_t previous_timer = cycles_now();
    for (uint32_t attempt = 0u; attempt < 256u; ++attempt) {
        const uint32_t now = cycles_now();
        const int32_t elapsed = cycles_after(now, previous_timer);
        if (elapsed > 0 && elapsed < 1024) {
            timer_running = true;
            break;
        }
        previous_timer = now;
    }
    if (!timer_running)
        observer_failed();
    atomic_write(&counters.ready, 1u);

    uint32_t generation;
    const routing_table* table = acquire_table(&generation);
    // Resynchronize at qualified EOP or a long idle J, never an arbitrary
    // data transition. An idle gap must not cost the next control's SETUP.
    bool draining = true;
    #if !defined(SWITCH2_PROBE_HUB) || !SWITCH2_PROBE_HUB
    bool saw_se0 = false;
    uint32_t se0_since = 0u;
    #endif
    for (;;) {
        if (atomic_read(&fatal_fault) != 0u)
            observer_failed();
        const uint32_t phase = atomic_read(&phase_cycles);
        for (;;) {
            if (atomic_read(&published_generation) != generation)
                table = acquire_table(&generation);
            #if !defined(SWITCH2_PROBE_HUB) || !SWITCH2_PROBE_HUB
            if (draining) {
                const uint32_t line = receive_line();
                const uint32_t now = cycles_now();
                if (line == LINE_SE0) {
                    if (!saw_se0) se0_since = now;
                    saw_se0 = true;
                } else {
                    // Reject momentary pad skew as EOP. A real SE0 persists
                    // across at least one complete bit before returning to J.
                    if (line == LINE_J && saw_se0 &&
                        cycles_after(now, se0_since) >= (int32_t)FS_BIT_CYCLES)
                        draining = false;
                    else if (line == LINE_J && observe_idle_j())
                        draining = false;
                    saw_se0 = false;
                }
                if (draining) continue;
                break;
            }
            #endif
            break;
        }
        // Phase is relative to the observed J->K edge, not a promised physical
        // edge timestamp. The host sweeps 0..19 cycles and correlates sampled
        // headers with the native DCD's CRC-accepted SETUP interrupts. A successful
        // passive phase still does NOT prove when the SIE latches its address.
        // Calibrate the real routing instruction path, not a lighter sampler
        // whose phase/register allocation changes when routing is enabled.
        // The independent enabled flag still forbids every dry-run USB write.
        #if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB
        const raw_packet packet = capture_packet(phase, table, draining);
        #else
        const raw_packet packet = capture_packet(phase, table);
        #endif
        if (!packet.sop) {
            if (packet.resync) {
                draining = true;
                #if !defined(SWITCH2_PROBE_HUB) || !SWITCH2_PROBE_HUB
                saw_se0 = false;
                #endif
            }
            continue;
        }
        #if !defined(SWITCH2_PROBE_HUB) || !SWITCH2_PROBE_HUB
        for (uint32_t i = 0; i < 3; ++i)
            atomic_write(&counters.last_raw[i], packet.words[i]);
        atomic_write(&counters.last_raw_count, packet.count);
        atomic_write(&counters.last_raw_eop, packet.eop);
        atomic_write(&counters.last_raw_late, packet.late);
        count_one(&counters.sops);
        atomic_write(&counters.retargets, atomic_read(&counters.retargets) + packet.retargets);
        if (packet.late)
            count_one(&counters.late_samples);
        decode_packet(&packet, table);
        // Decoding can outlast the minimum interpacket gap. Qualify another
        // EOP or a long idle J before accepting a new SOP; an arbitrary J->K
        // inside a packet is not a start. Missing traffic is preferable to
        // manufacturing a SETUP owner from a payload transition.
        draining = true;
        saw_se0 = false;
        #else
        // A response may start during the return/preparation path even if the
        // preceding EOP was sampled. Requalify from the prepared capture frame.
        draining = true;
        #endif
    }
}
