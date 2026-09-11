#pragma once

#include <stddef.h>
#include <stdint.h>
#include "core/controller_state.h"

struct WiiIrMouseReport {
    int16_t dx;
    int16_t dy;
    uint8_t buttons;
    uint32_t generation;
    bool tracking;
    uint8_t owner;
    uint32_t connection_generation;
#if SWITCH2_BRIDGE_WII_INPUT
    // Full-pair upstream heading, excluding raw geometry still held as a glitch.
    bool optical_valid;
    float optical_yaw_radians;
    uint32_t optical_sequence;
    uint32_t optical_received_us;
#endif
};

// Core 0 initializes before starting Bluetooth. The producer runs on Core 1;
// USB peeks/commits on Core 0, preserving movement until a successful send.
void wii_ir_pointer_init();
void wii_ir_pointer_reset();
// Metadata is populated even without pending output. Nonpositive limits
// suppress movement; the default retains the legacy signed 8-bit HID range.
bool wii_ir_mouse_peek(WiiIrMouseReport* report, int16_t maximum_delta = 127);
void wii_ir_mouse_commit(const WiiIrMouseReport& report);
// Enabled by default. Transitions discard pending motion and its baseline,
// retaining owner and diagnostics. Native Wii keeps upstream camera history;
// legacy modes reset association. Repeated calls preserve live tracking.
// Reset/disconnect preserve this gate.
void wii_ir_mouse_set_output_enabled(bool enabled);
constexpr size_t WII_IR_MOUSE_DIAGNOSTIC_SIZE = 48;
size_t wii_ir_pointer_diagnostics(uint8_t* buffer, size_t capacity);

#if SWITCH2_BRIDGE_WII_INPUT
// Configure before starting Core 1. The viewport is in camera pixels; spans
// are native mouse counts across it, not a claim about console cursor pixels.
// Upstream sx/sy remain unclipped; this rectangle controls relative scale and
// diagnostic viewport membership, not absolute host cursor synchronization.
bool wii_ir_pointer_configure_screen(float width, float height, float offset_x,
                                     float offset_y, float span_x, float span_y);
#endif


// The first Wii source owns the experimental pointer until disconnected.
void wii_ir_pointer_disconnect(uint8_t slot);
// nunchuk_c is the unmapped physical C state, false without a Nunchuk.
// Native Wii: one call per actual camera sequence, raw unmirrored dot X/Y.
// Gravity roll is the raw bar angle in radians; invalid gravity retains the
// last valid upstream roll. Controller profile buttons never become clicks.
void wii_ir_pointer_observe(uint8_t slot, uint32_t connection_generation,
                          uint32_t sequence, uint16_t buttons,
                          const uint16_t x[4], const uint16_t y[4],
                          uint8_t valid_mask, bool nunchuk_c,
                          float gravity_roll_radians = 0.0f, bool gravity_valid = false);

#ifdef SWITCH_PICO_WII_IR_GYRO
struct WiiIrGyroReport {
    uint32_t generation;
    int32_t consumed_x_q8;
    int32_t consumed_y_q8;
    uint32_t submitted_us;
};

// Real gyro is the default. Selection is connection-local, never persisted.
// A different live Wii owner rejects selection; changing sources rebaselines.
bool wii_ir_gyro_select(uint8_t slot, uint32_t connection_generation,
                        bool infrared);
void wii_ir_gyro_update_motion(uint8_t slot, uint32_t connection_generation,
                              bool enabled, const ControllerMotionSample& sample);
// USB reconnect/reenumeration discards pending motion without changing source.
void wii_ir_gyro_reset_output();

// Call only for a due regular 0x30 report. False leaves real motion untouched.
// True writes the motion count and all three 5 ms samples without reading state,
// retaining the latest real accelerometer (or no motion when disabled).
// Uses a nominal 33 x 23 degree pinhole model, not a measured unit calibration.
// Gyro pending positions are Q8 at 50 units/degree (12800 units/degree).
// Consumed fields carry represented whole units; commit retains the fraction.
// Only a successful USB queue may commit. No successful send for 100 ms drops
// pending motion; backlog above one 720 degree/second report is discarded.
bool wii_ir_gyro_prepare(uint8_t slot, uint32_t now_us, ControllerState* state,
                         WiiIrGyroReport* report);
void wii_ir_gyro_commit(const WiiIrGyroReport& report);

constexpr size_t WII_IR_GYRO_DIAGNOSTIC_SIZE = 64;
// Bytes 0..47 retain the pointer diagnostic format. Bytes 48/49 are IR source
// and motion-enabled flags. Schema 2 LE fields: 50 gyro_x i16, 52 gyro_y i16,
// 54 gyro_z i16, 56 successful IR reports u32, 60 last send age_us u32.
size_t wii_ir_gyro_diagnostics(uint8_t* output, size_t capacity);
#endif