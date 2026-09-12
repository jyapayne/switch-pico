#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PROBE_ROUTER_SLOTS 3u
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
