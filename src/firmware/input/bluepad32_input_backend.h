#pragma once

#include <stdint.h>

#include "core/controller_color.h"
#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "core/controller_state.h"
#include "usb/switch/switch_haptics.h"

constexpr uint8_t BLUEPAD32_INPUT_BACKEND_SLOT_COUNT = 4;
constexpr uint8_t BLUEPAD32_PAIRING_RECORD_CAPACITY = 16;

enum class Bluepad32PairingTransport : uint8_t {
    kClassic = 1,
    kBle = 2,
};

enum class Bluepad32PairingSnapshotStatus : uint8_t {
    kReady = 0,
    kPending = 1,
};

struct Bluepad32PairingRecord {
    Bluepad32PairingTransport transport;
    uint8_t address_type;
    uint8_t address[6];
};

struct Bluepad32PairingSnapshot {
    uint32_t generation;
    // Unchanged by ordinary refreshes; published only after all clear work.
    uint32_t completed_clear_pairings_token;
    Bluepad32PairingSnapshotStatus status;
    uint8_t record_count;
    bool overflow;
    Bluepad32PairingRecord records[BLUEPAD32_PAIRING_RECORD_CAPACITY];
};
// Clear tokens form a bounded serial number space over every nonzero uint32_t.
// A completion at most half that space ahead of a request also acknowledges
// the request, including across the UINT32_MAX-to-1 wrap.
constexpr bool bluepad32_input_backend_clear_pairings_completed(
    const Bluepad32PairingSnapshot& snapshot, uint32_t request_token) {
    const uint32_t completed_token =
        snapshot.completed_clear_pairings_token;
    if (request_token == 0 || completed_token == 0) {
        return false;
    }
    const uint32_t forward_distance =
        completed_token >= request_token
            ? completed_token - request_token
            : (UINT32_MAX - request_token) + completed_token;
    return forward_distance <= UINT32_MAX / 2u;
}
struct Bluepad32SlotSnapshot {
    bool active;
    uint32_t connection_generation;
    ControllerIdentity identity;
    // Physical logical-button state before backend hotkey consumption.
    // Valid only for this snapshot's connection generation.
    uint16_t pre_hotkey_button_mask;
    ControllerState state;
};

struct Bluepad32BackendDiagnostics {
    uint32_t initialization_stage;
    uint32_t rumble_timer_ticks;
    uint32_t configuration_timer_ticks;
    uint32_t controller_reports;
    uint32_t host_rumble_requests;
    uint32_t local_feedback_requests;
    uint32_t rumble_dispatches;
    uint8_t active_slots;
    uint8_t rumble_capable_slots;
    uint8_t feedback_pending_slots;
    uint8_t rumble_pending_slots;
};



void bluepad32_input_backend_init();
void bluepad32_input_backend_start();
void bluepad32_input_backend_open_pairing_window();
// Repeated calls coalesce until Core 1 completes the operation and return the
// same nonzero token.
uint32_t bluepad32_input_backend_clear_pairings();
void bluepad32_input_backend_snapshot(uint8_t slot,
                                      Bluepad32SlotSnapshot* out);
void bluepad32_input_backend_request_pairing_snapshot();
void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out);
void bluepad32_input_backend_diagnostics(
    Bluepad32BackendDiagnostics* out);
void bluepad32_input_backend_report_sent(uint8_t slot);
void bluepad32_input_backend_queue_rumble(
    uint8_t slot, const ControllerRumbleOutput& rumble);
// Enqueue bounded local profile confirmation for the matching live connection
// generation. The two-entry per-slot FIFO preserves initial-then-switch
// ordering. Profile lighting is transient and restored to the steady slot
// indication after the final gap. Profile numbers are one-based (1..4).
void bluepad32_input_backend_queue_profile_feedback(
    uint8_t slot, uint32_t connection_generation,
    uint8_t active_profile_number,
    ControllerProfileConfirmationPolicy policy);
