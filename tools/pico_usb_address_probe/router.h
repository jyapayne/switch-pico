#pragma once

#include <stdbool.h>
#include <stdint.h>

// The standalone RAM probe remains at 240 MHz. Only the integrated hub
// consumes the firmware clock; every receiver deadline is compiled for it.
#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB && defined(SWITCH_PICO_SYS_CLOCK_MHZ)
#define FS_CLOCK_MHZ SWITCH_PICO_SYS_CLOCK_MHZ
#else
#define FS_CLOCK_MHZ 240u
#endif
#if FS_CLOCK_MHZ != 240 && FS_CLOCK_MHZ != 300
#error "Native SIO receiver requires a compiled 240 or 300 MHz clock"
#endif
#define FS_CLOCK_HZ (FS_CLOCK_MHZ * 1000000u)
#define FS_BIT_CYCLES (FS_CLOCK_HZ / 12000000u)
#define FS_HALF_BIT_CYCLES ((FS_BIT_CYCLES + 1u) / 2u)
#define FS_IDLE_CYCLES (8u * FS_BIT_CYCLES)
// Preserve the baseline tight-poll duration: 64 polls at 240, 80 at 300 MHz.
#define FS_IDLE_POLLS (FS_CLOCK_MHZ * 64u / 240u)
// Relative to the software SOP timestamp, after edge detection—not a fraction
// of the USB bit period. Fixed instruction latency needs a separate offset at
// 300 MHz. Phase 12 passed loaded child-control tests where scaled phase 5 failed.
// Preserve the established 240 MHz setting; Switch recognition is a separate check.
#define PROBE_ROUTER_DEFAULT_PHASE (FS_CLOCK_MHZ == 300u ? 12u : 4u)

#if defined(SWITCH2_PROBE_HUB) && SWITCH2_PROBE_HUB && defined(PROBE_CONTROLLER_COUNT)
#define PROBE_ROUTER_SLOTS (PROBE_CONTROLLER_COUNT + 1u)
#else
#define PROBE_ROUTER_SLOTS 3u
#endif
#define PROBE_ROUTER_UNASSIGNED 0xffu

typedef struct {
    uint32_t ready;
    uint32_t sops;
    uint32_t sync_ok;
    uint32_t valid_tokens;
    uint32_t valid_setups;
    uint32_t crc_errors;
    uint32_t late_samples;
    uint32_t retargets;
    uint32_t address_hits[PROBE_ROUTER_SLOTS];
    uint32_t cycles_per_bit;
    uint32_t last_pid;
    uint32_t last_address;
    uint32_t last_setup_sequence;
    uint32_t last_setup_slot;
    uint32_t last_raw[3];
    uint32_t last_raw_count;
    uint32_t last_raw_eop;
    uint32_t last_raw_late;
    // Trace-enabled hub diagnostics, updated outside token sampling deadlines.
    uint32_t capture_returns;
    uint32_t discarded_headers;
    uint32_t last_discarded_header;
    uint32_t enabled, fatal_fault, published_generation, reader_index;
    // First non-token header following a selected root-IN candidate. These are
    // sampler observations, not endpoint decoding or full packet validation.
    // The pre-SETUP copy survives the recovery control's own IN/status response.
    uint32_t root_in_count, root_in_cutoff, root_header, root_header_cycle, root_eop_cycle;
    uint32_t before_setup_in_count, before_setup_in_cutoff;
    uint32_t before_setup_header, before_setup_header_cycle, before_setup_eop_cycle;
} probe_router_stats;

// Core 0 initializes before launching Core 1. The native SIE drives USB;
// Core 1 observes the existing socket and selects known device addresses.
void probe_router_init(uint32_t system_clock_hz);
void probe_router_core1(void);

// Core 0 publishes assigned addresses (0xff means unassigned). Address zero
// belongs only to default_slot, or nobody when default_slot is 0xff.

void probe_router_publish(const uint8_t addresses[PROBE_ROUTER_SLOTS], uint8_t default_slot);
void probe_router_enable(bool enabled);
// Sampling phase within one USB bit, for passive timing calibration only.
// Reject out-of-range values and changes after address retargeting is enabled.
bool probe_router_set_phase(uint32_t cycles);

// Diagnostic snapshots. Counters are individually atomic, not a transaction.
void probe_router_snapshot(probe_router_stats* out);
// Last sampled SETUP-header candidate, used for calibration correlation only.
// Runtime control ownership comes from the SIE address at its SETUP interrupt.
uint8_t probe_router_setup_slot(uint32_t* sequence);
