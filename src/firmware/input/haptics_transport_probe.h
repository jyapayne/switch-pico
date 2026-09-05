#pragma once

#include <stdint.h>

// Operation 0x41, schema 2: 32 little-endian u32 fields in declaration order.
// All durations are host-side microseconds, not radio/actuator latency.
struct HapticsTransportProbe {
    uint32_t run_id = 0;
    uint32_t connection_generation = 0;
    uint32_t connection_handle = 0xffff;
    uint32_t timer_wakes = 0;
    uint32_t max_timer_lateness_us = 0;
    uint32_t total_timer_lateness_us = 0;
    uint32_t send_calls = 0;
    uint32_t max_send_us = 0;
    uint32_t total_send_us = 0;
    uint32_t write_calls = 0;
    uint32_t max_write_us = 0;
    uint32_t total_write_us = 0;
    uint32_t read_calls = 0;
    uint32_t read_packets = 0;
    uint32_t max_read_us = 0;
    uint32_t total_read_us = 0;
    uint32_t poll_calls = 0;
    uint32_t max_poll_us = 0;
    uint32_t total_poll_us = 0;
    uint32_t completion_events = 0;
    uint32_t completed_packets = 0;
    uint32_t max_completion_gap_us = 0;
    uint32_t max_outstanding_acl = 0;
    uint32_t min_free_acl = 0;
    uint32_t first_tone_send_return_us = 0;
    uint32_t active = 0;
    uint32_t max_permission_wait_us = 0;
    uint32_t total_permission_wait_us = 0;
    uint32_t permission_callbacks = 0;
    uint32_t max_poll_gap_us = 0;
    uint32_t controller_acl_packet_bytes = 0;
    uint32_t controller_acl_packet_count = 0;
};

// Prepare before core 1 starts. Snapshot alone is called from USB/core 0.
void haptics_transport_probe_prepare();
void haptics_transport_probe_snapshot(HapticsTransportProbe* output);
// Remaining API belongs to core 1. No probe lock may span a stack call.
void haptics_transport_probe_begin(uint32_t run_id, uint32_t generation,
                                  uint16_t handle);
void haptics_transport_probe_end();
void haptics_transport_probe_timer(uint32_t lateness_us);
void haptics_transport_probe_permission(uint32_t wait_us);
void haptics_transport_probe_send(uint32_t duration_us,
                                 uint32_t return_us, bool first_tone_success);
