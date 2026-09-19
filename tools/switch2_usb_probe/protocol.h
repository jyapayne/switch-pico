#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "model.h"
#include "core/native_haptics.h"

#define PROBE_COMMAND_MAX_SIZE 263u
#define PROBE_REPLY_MAX_SIZE 96u
#define PROBE_HOST_MAX_ADDRESSES ((255u - 2u) / 6u)
#define PROBE_PAIRING_BLOB_SIZE (6u + 1u + PROBE_HOST_MAX_ADDRESSES * 6u + 16u)
#define PROBE_INPUT_SIZE 63u

typedef struct {
    bool is_left;
    void* context; // Caller-owned context shared by this state's callbacks.
    bool initialized;
    uint8_t report_id;
    bool test_rail_buttons;
    bool runtime03_0c; // Observed USB toggle; full semantics remain unknown.
    uint8_t stick_center[3];
    bool controller_active;
    uint8_t controller_buttons[2]; // Selected model's native Joy-Con button ordering.
    uint8_t controller_stick[3]; // Raw packed 12-bit axes from the selected donor.
    uint8_t player_leds; // Virtual four-LED mask, exposed through UART diagnostics.
    bool player_leds_flashing;
    // Negotiated virtual features; relative mouse events belong to the USB sender.
    uint8_t feature_mask;
    uint8_t enabled_features;
    bool vibration_parameters_set;
    uint8_t vibration_parameters[20]; // Captured 0A/08 format; no motor output.
    uint8_t host_address[6];
    uint8_t controller_address[6]; // Own virtual identity, wire byte order (LE).
    const uint8_t* firmware_version; // Twelve captured bytes; caller retains their lifetime.
    // Every payload-bearing address exchange starts a fresh pending context.
    uint8_t pending_host_count;
    uint8_t pending_host_addresses[PROBE_HOST_MAX_ADDRESSES][6];
    bool pending_key_valid;
    uint8_t pending_key[16]; // Standard AES byte order, not wire byte order.
    bool challenge_confirmed;
    // A zero count means no committed pairing record.
    uint8_t committed_host_count;
    uint8_t committed_host_addresses[PROBE_HOST_MAX_ADDRESSES][6];
    uint8_t committed_key[16]; // Standard AES byte order.
    // Synchronous durable save; NULL disables successful finalization.
    bool (*save_pairing)(void* context, const uint8_t* blob, size_t length);
    bool (*read_memory)(void* context, uint32_t address, uint8_t* output, size_t length);
    // Queue a physical sample, returning true only with a nonzero completion token.
    // Acceptance is not a Bluetooth application ACK.
    bool (*play_sample)(void* context, uint8_t sample_id, uint64_t* token);
    uint32_t report_counter;
} probe_protocol_state;

// Stateless Output 01 waveform decoding: report_id 0 includes the leading
// wire ID (17..64 bytes); report_id 1 omits it (16..63 bytes).
// Count zero means HOLD/no update, not stop or watchdog refresh. A nonempty
// zero-amplitude sample is stop. Malformed input leaves output unchanged.
// Both 10-bit frequency and amplitude fields survive decoding; each physical
// output backend chooses its own supported frequency range and rendering.
bool probe_protocol_decode_rumble(uint8_t report_id, const uint8_t* data,
                                  size_t length, NativeHapticsActuatorFrame* output);

void probe_protocol_reset(probe_protocol_state* state, bool is_left);
// Blob: own address[6], count[1], zero-padded host addresses[42][6], AES key[16].
// Rejects other identities, invalid counts/padding/lengths without mutation.
// A successful restore replaces the committed record and clears pending state.
bool probe_protocol_restore_pairing(probe_protocol_state* state,
                                    const uint8_t* blob, size_t length);
// Complete command frames only. Unsupported/malformed commands return zero
// and do not mutate state. Pairing finalization requires a successful save.
// When non-NULL, deferred_token is cleared before validation. Synchronous replies
// leave it zero. Sample 0A/02 requires this output and play_sample; acceptance
// prepares an 8-byte reply and returns its nonzero source completion token.
// Those bytes MUST NOT be transmitted until that token has a genuine positive
// Bluetooth application ACK. Failure, cancellation or expiry must discard them.
size_t probe_protocol_command(probe_protocol_state* state, const uint8_t* command,
                              size_t length, uint8_t* reply, size_t capacity,
                              uint64_t* deferred_token);
// Button/stick snapshot without relative mouse events; safe for GET_REPORT.
size_t probe_protocol_report(const probe_protocol_state* state, uint8_t report_id,
                             uint8_t* output, size_t capacity);
// Apply virtual feature gates to one complete native payload in place.
// Enabled mouse/motion and all opaque bytes remain unchanged.
void probe_protocol_gate_native_report(const probe_protocol_state* state,
                                       uint8_t input[PROBE_INPUT_SIZE]);
