// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uni_hid_device_s;
typedef struct {
    bool report_tracked;
    bool report_valid;
    uint32_t report_sequence;
    bool accel_valid;
    bool gyro_valid;
    uint32_t accel_sequence;
    uint32_t gyro_sequence;
    int32_t accel_q13[3];
    int32_t gyro_q10[3];
} uni_native_motion_snapshot_t;

#if SWITCH2_BRIDGE_FULL_INPUT
// Bluetooth owner only. Reads never advance sequences or rejuvenate samples.
// False/zero means no motion provider, not an unsupported controls device.
bool uni_hid_parser_native_motion_snapshot(struct uni_hid_device_s* d,
                                           uni_native_motion_snapshot_t* out);

// Parser ingress/lifecycle hooks. Only setup allocates a bounded slot; late
// reports cannot resurrect a retired provider. NULL sensor data invalidates
// that sensor without changing its last sequence. Samples use SDL Q13/Q10 axes.
void uni_hid_parser_native_motion_reset(struct uni_hid_device_s* d);
void uni_hid_parser_native_motion_forget(struct uni_hid_device_s* d);
void uni_hid_parser_native_motion_begin(struct uni_hid_device_s* d);
void uni_hid_parser_native_motion_accept(struct uni_hid_device_s* d);
void uni_hid_parser_native_motion_accel(struct uni_hid_device_s* d, const int32_t* value);
void uni_hid_parser_native_motion_gyro(struct uni_hid_device_s* d, const int32_t* value);
// Test a complete report's wrapping hardware clock, before publishing sensors.
// mask is UINT8_MAX/UINT16_MAX/UINT32_MAX; backwards/duplicate ticks are rejected.
bool uni_hid_parser_native_motion_fresh(struct uni_hid_device_s* d, uint32_t timestamp, uint32_t mask);
// Parser ingress only: process-wide nonzero IDs do not alias across reconnects.
uint32_t uni_hid_parser_native_motion_next_sequence(void);
#endif

#ifdef __cplusplus
}
#endif
