#pragma once

#include <stdint.h>
#include "configuration/adapter_configuration.h"
#include "core/controller_identity.h"
#include "usb/switch/switch_haptics.h"

struct uni_hid_device_s;

// Submission timing is a transport measurement, not actuator onset. Percentiles
// are upper bounds from 250-us buckets, with the tail bounded by observed max.
struct SwitchNativeOutputDiagnostics {
    uint8_t slot = 0;
    uint8_t type = 0;
    uint8_t firmware_hi = 0;
    uint8_t firmware_lo = 0;
    uint32_t flags = 0;
    uint32_t generation = 0;
    uint32_t received_commands = 0;
    uint32_t submitted_reports = 0;
    uint32_t dropped_commands = 0;
    uint32_t resynchronizations = 0;
    uint32_t raw_commands = 0;
    uint32_t quantized_commands = 0;
    uint32_t congested_attempts = 0;
    uint32_t completed_commands = 0;
    uint32_t p50_upper_us = 0;
    uint32_t p95_upper_us = 0;
    uint32_t p99_upper_us = 0;
    uint32_t max_latency_us = 0;
    uint32_t queue_depth = 0;
    uint8_t last_wire[8]{};
    uint32_t max_encode_us = 0;
    uint32_t coalesced_commands = 0;
};

// prepare/submit/snapshot are Core-0 safe. All other calls belong to BTstack.
void switch_native_output_prepare();
void switch_native_output_attach(uint8_t slot, uint32_t generation,
                                uni_hid_device_s* device,
                                const ControllerIdentity& identity);
void switch_native_output_detach(uni_hid_device_s* device);
void switch_native_output_configure(const AdapterConfiguration& configuration,
                                   uint32_t generation);
bool switch_native_output_submit(uint8_t slot, uint32_t generation,
                                 uint64_t received_us,
                                 const ControllerRumbleOutput& rumble,
                                 bool stateful);
bool switch_native_output_owns(const uni_hid_device_s* device);
bool switch_native_output_on_can_send_now(uni_hid_device_s* device, uint16_t cid);
bool switch_native_output_feedback(uni_hid_device_s* device, uint8_t low,
                                   uint8_t high, uint16_t duration_ms);
void switch_native_output_snapshot(uint8_t slot,
                                  SwitchNativeOutputDiagnostics* output);
