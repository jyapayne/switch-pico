#pragma once

#include <stdint.h>

#include "core/controller_color.h"
#include "core/controller_identity.h"
#include "core/native_haptics.h"
#include "profile/controller_profile.h"
#include "core/controller_state.h"
#include "input/controller_macro_capture.h"
#include "input/wii_swing.h"
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
    kFailed = 2,
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
    WiiAccelerometerSample accelerometer{};
    WiiAccelerometerSample nunchuk_accelerometer{};
};

enum class Bluepad32ControllerLayout : uint8_t {
    kUnspecified = 0,
    kJoyCon2LeftSolo = 1,
    kJoyCon2RightSolo = 2,
    kJoyCon2MergedPair = 3,
    kWiiRemote = 4,
    kWiiNunchuk = 5,
    kWiiHorizontal = 6,
    kWiiVertical = 7,
};

#ifdef SWITCH2_BRIDGE_WII_INPUT
// Calibrated SDL axes, before legacy Switch int16 conversion and motion gating.
// Both sensor sequences and receipt times are independent of controller reports.
struct Bluepad32WiiBridgeSnapshot {
    uint8_t slot = 0xff;
    Bluepad32SlotSnapshot controller{};
    Bluepad32ControllerLayout layout = Bluepad32ControllerLayout::kUnspecified;
    uint32_t state_generation = 0;
    uint32_t received_us = 0;
    uint8_t battery = 0;
    bool accel_valid = false;
    bool gyro_valid = false;
    uint32_t accel_sequence = 0;
    uint32_t gyro_sequence = 0;
    uint32_t accel_received_us = 0;
    uint32_t gyro_received_us = 0;
    int32_t accel_q13[3]{};
    int32_t gyro_q10[3]{};
};

// Core 0, after init and before start. Reselection retires source-local work.
void bluepad32_input_backend_select_wii_source(const uint8_t address[6]);
void bluepad32_input_backend_wii_snapshot(Bluepad32WiiBridgeSnapshot* output);
// Bounded ERM approximations, not HD haptics. Completion means Core 1 driver
// dispatch, never a Wii application ACK. Tokens are unique for this boot.
bool bluepad32_input_backend_wii_sample_request(uint8_t sample_id, uint64_t* token);
int bluepad32_input_backend_wii_sample_result(uint64_t token);
void bluepad32_input_backend_wii_sample_cancel();
#endif

#if SWITCH2_BRIDGE_FULL_INPUT
#ifdef PROBE_CONTROLLER_COUNT
static_assert(PROBE_CONTROLLER_COUNT == 2 || PROBE_CONTROLLER_COUNT == 4);
constexpr uint8_t BLUEPAD32_NATIVE_PAIR_COUNT = PROBE_CONTROLLER_COUNT / 2;
#else
constexpr uint8_t BLUEPAD32_NATIVE_PAIR_COUNT = 1;
#endif
static_assert(BLUEPAD32_NATIVE_PAIR_COUNT == 1 || BLUEPAD32_NATIVE_PAIR_COUNT == 2);

// One logical gamepad, calibrated SDL axes before legacy int16 conversion.
// Sensor receipt times advance independently, only on actual parser ingress.
struct Bluepad32NativeGamepadSnapshot {
    uint8_t slot = 0xff;
    Bluepad32SlotSnapshot controller{};
    uint32_t state_generation = 0;
    uint32_t received_us = 0;
    uint8_t battery = 0;
    bool accel_valid = false;
    bool gyro_valid = false;
    bool track_stationary_bias = false;
    uint32_t accel_sequence = 0;
    uint32_t gyro_sequence = 0;
    uint32_t accel_received_us = 0;
    uint32_t gyro_received_us = 0;
    int32_t accel_q13[3]{};
    int32_t gyro_q10[3]{};
};

// nullptr selects automatic assignment: one pair requires a uniquely eligible
// gamepad; two pairs reserve stable logical identities in first-free order for
// this boot. Explicit member addresses reserve the whole logical controller.
// Conflicts fail closed. Reselection retires only affected input/cue epochs,
// without modifying pairings or saved profiles.
void bluepad32_input_backend_select_native_source(
    uint8_t pair_index, const uint8_t address[6]);
void bluepad32_input_backend_native_snapshot(
    uint8_t pair_index, Bluepad32NativeGamepadSnapshot* output);
// Instances are A_R, A_L, then B_R, B_L. Samples 0..7 are bounded compatibility cues, not HD
// haptics. A side stop removes only that side's contribution. Mono drivers combine
// both contributions on their one actuator; this does not promise stereo output.
// Result: 0 pending, 1 source-driver dispatch, -1 retired/failed/consumed.
// Dispatch is NOT an application ACK or proof of physical actuator onset.
bool bluepad32_input_backend_native_sample_request(
    uint8_t instance, uint8_t sample_id, uint64_t* token);
int bluepad32_input_backend_native_sample_result(uint8_t instance, uint64_t token);
void bluepad32_input_backend_native_sample_cancel(uint8_t instance);
// Gameplay blocks replace older gameplay/cues only on the addressed side.
// Conventional samples divide 12 ms; PCM retains native ~5.333 ms/sample.
// The last host sample holds only to its original 50 ms receipt watchdog.
// HD preserves both frequency bands and applies their strong/low and weak/high
// profile gains. Conventional motors retain the per-child peak approximation.
// Submission copies 1..3 samples; false means invalid/unbound/unsupported,
// higher-priority feedback, a retired source, or a selected HD stream not accepting.
// Cancellation retires gameplay only; callers resetting a child also cancel its cue.
bool bluepad32_input_backend_native_rumble_submit(
    uint8_t instance, const NativeHapticsActuatorFrame* frame);
void bluepad32_input_backend_native_rumble_cancel(uint8_t instance);
#endif

// Side-effect-free raw input snapshot for management telemetry. Unlike the
// report-path snapshot, reading this does not consume motion samples.
struct Bluepad32PlaytestSnapshot {
    bool active = false;
    uint32_t connection_generation = 0;
    uint32_t state_generation = 0;
    ControllerIdentity identity{};
    uint16_t physical_button_mask = 0;
    uint8_t battery = 0;
    uint8_t capabilities = 0;
    Bluepad32ControllerLayout controller_layout =
        Bluepad32ControllerLayout::kUnspecified;
    ControllerState state{};
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
    uint32_t switch2_ingress_drops;
    uint32_t switch2_output_drops;
};



// Core 0 startup. Normally start launches a dedicated Core 1 BTstack/storage
// owner; SWITCH2_PROBE_HUB keeps that owner on Core 0 and leaves Core 1 to USB.
void bluepad32_input_backend_init();
void bluepad32_input_backend_start();
// Hub only: call on Core 0 outside IRQs, without any application state lock
// held, once per main-loop iteration. Services the existing SDK CYW43 async
// context without waiting. Safe before start; a no-op in dedicated-Core 1 modes.
void bluepad32_input_backend_poll();
void bluepad32_input_backend_open_pairing_window();
// BTstack parser admission gate for fresh proprietary Switch 2 pairing; this
// never opens a pairing window or changes the bounded connection policy.
extern "C" bool switch_pico_switch2_pairing_allowed(void);
// Repeated calls coalesce until BTstack completes the operation and return the
// same nonzero token.
uint32_t bluepad32_input_backend_clear_pairings();
void bluepad32_input_backend_snapshot(uint8_t slot,
                                      Bluepad32SlotSnapshot* out);
void bluepad32_input_backend_playtest_snapshot(
    uint8_t slot, Bluepad32PlaytestSnapshot* out);
void bluepad32_input_backend_request_pairing_snapshot();
void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out);
void bluepad32_input_backend_diagnostics(
    Bluepad32BackendDiagnostics* out);
void bluepad32_input_backend_report_sent(uint8_t slot);
// Toggle motion and queue state feedback only for the matching live
// connection generation.
bool bluepad32_input_backend_toggle_motion(
    uint8_t slot, uint32_t connection_generation);
void bluepad32_input_backend_queue_rumble(
    uint8_t slot, const ControllerRumbleOutput& rumble);
bool bluepad32_input_backend_identify(
    const ControllerIdentity& identity);
// Queue a connection-local standalone Wii mapping change for Core 1's 5 ms
// timer. Acceptance is not application; dispatch advances the logical
// connection generation once, including reselection. Nunchuk is ineligible.
bool bluepad32_input_backend_set_wii_orientation(
    const ControllerIdentity& identity, uint32_t connection_generation,
    bool vertical);
// Enqueue bounded local profile confirmation for the matching live connection
// generation. The two-entry per-slot FIFO preserves initial-then-switch
// ordering. Profile lighting is transient and restored to the steady slot
// indication after the final gap. Profile numbers are one-based (1..8).
void bluepad32_input_backend_queue_profile_feedback(
    uint8_t slot, uint32_t connection_generation,
    uint8_t active_profile_number,
    ControllerProfileConfirmationPolicy policy);

constexpr uint8_t BLUEPAD32_CAPTURE_PAGE_EVENTS = 32;
struct Bluepad32CaptureSnapshot {
    uint32_t run_id = 0;
    uint32_t connection_generation = 0;
    uint32_t elapsed_us = 0;
    uint8_t slot = 0xff;
    CaptureState state = CaptureState::kIdle;
    CaptureOptions options{};
    uint16_t total_events = 0;
    uint16_t first_index = 0;
    uint8_t event_count = 0;
    CaptureEvent events[BLUEPAD32_CAPTURE_PAGE_EVENTS]{};
};

// Core 0 management operations; recording observes BTstack input before profile
// transforms. All recorder access uses the existing slot-state lock.
bool bluepad32_input_backend_capture_start(
    uint8_t slot, uint32_t connection_generation, const CaptureOptions& options);
bool bluepad32_input_backend_capture_stop(uint32_t run_id);
bool bluepad32_input_backend_capture_page(
    uint32_t run_id, uint16_t first_index, Bluepad32CaptureSnapshot* output);
