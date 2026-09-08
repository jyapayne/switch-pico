#pragma once

#include <stdint.h>

#ifndef SWITCH_PICO_HD_PACKET_FRAMES
#define SWITCH_PICO_HD_PACKET_FRAMES 32
#endif

struct uni_hid_device_s;
typedef struct uni_hid_device_s uni_hid_device_t;
struct SwitchHapticsFrame;

enum class HapticsExperimentState : uint8_t {
    kIdle = 0,
    kPending = 1,
    kRunning = 2,
    kCompleted = 3,
    kStopped = 4,
    kDisconnected = 5,
    kUnsupported = 6,
    kError = 7,
};

struct HapticsExperimentDiagnostics {
    uint32_t run_id = 0;
    uint32_t connection_generation = 0;
    uint32_t start_us = 0;
    uint32_t generated_packets = 0;
    uint32_t sent_packets = 0;
    uint32_t skipped_packets = 0;
    uint32_t send_failures = 0;
    uint32_t can_send_requests = 0;
    uint32_t synchronous_callbacks = 0;
    uint32_t max_generate_us = 0;
    uint32_t max_send_gap_us = 0;
    uint32_t max_lateness_us = 0;
    uint32_t max_request_wait_us = 0;
    uint32_t first_tone_due_us = 0;
    uint32_t first_tone_sent_us = 0;
    uint32_t last_sent_us = 0;
    uint32_t elapsed_us = 0;
    HapticsExperimentState state = HapticsExperimentState::kIdle;
    uint8_t slot = 0xff;
    uint8_t last_error = 0;
    uint8_t mode = 0;
    uint32_t host_updates = 0;
    uint32_t dropped_updates = 0;
    uint8_t packet_frames = SWITCH_PICO_HD_PACKET_FRAMES;
    bool last_packet_nonzero = false;
};

// Core 0 before launching BTstack; request/snapshot are cross-core safe.
void haptics_experiment_prepare();
bool haptics_experiment_request(uint8_t action, uint8_t slot);
void haptics_experiment_snapshot(HapticsExperimentDiagnostics* output);
// Core 0 USB delivery. Rejects non-selected/stale connections; never buffers PCM.
bool haptics_experiment_submit(uint8_t slot, uint32_t generation,
                               uint64_t received_us,
                               const SwitchHapticsFrame& frame);
// Stateful, already profile-scaled XInput strengths, including explicit zero.
bool haptics_experiment_submit_rumble(uint8_t slot, uint32_t generation,
                                      uint64_t received_us,
                                      uint8_t low, uint8_t high);

// Core 1 / BTstack only. Poll consumes management requests, not PCM cadence.
void haptics_experiment_attach(uint8_t slot, uint32_t generation,
                               uni_hid_device_t* device);
void haptics_experiment_detach(uni_hid_device_t* device);
void haptics_experiment_poll();
bool haptics_experiment_owns(const uni_hid_device_t* device);
bool haptics_experiment_gameplay_owns(const uni_hid_device_t* device);
bool haptics_experiment_feedback(uni_hid_device_t* device,
                                 uint8_t low, uint8_t high, uint16_t duration_ms);
// Preserve fixture/drain exclusivity on unsolicited or control-CID events.
bool haptics_experiment_blocks_generic(const uni_hid_device_t* device);
