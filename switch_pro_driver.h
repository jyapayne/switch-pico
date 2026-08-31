/*
 * Minimal Switch Pro controller emulation glue derived from the GP2040-CE
 * SwitchProDriver. The driver keeps the same descriptors/handshake while
 * exposing a simple API for feeding inputs.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "switch_haptics.h"
#include "switch_pro_descriptors.h"


typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} SwitchImuSample;

typedef struct {
    bool dpad_up;
    bool dpad_down;
    bool dpad_left;
    bool dpad_right;

    bool button_a;
    bool button_b;
    bool button_x;
    bool button_y;
    bool button_l;
    bool button_r;
    bool button_zl;
    bool button_zr;
    bool button_plus;
    bool button_minus;
    bool button_home;
    bool button_capture;
    bool button_l3;
    bool button_r3;

    uint16_t lx; // 0-65535
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;

    uint8_t imu_sample_count;     // 0-3
    SwitchImuSample imu_samples[3];
} SwitchInputState;
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} SwitchRgbColor;
constexpr SwitchRgbColor switch_pro_calibrate_light_color(
    SwitchRgbColor grip) {
    const uint8_t minimum =
        grip.red < grip.green
            ? (grip.red < grip.blue ? grip.red : grip.blue)
            : (grip.green < grip.blue ? grip.green : grip.blue);
    const uint8_t maximum =
        grip.red > grip.green
            ? (grip.red > grip.blue ? grip.red : grip.blue)
            : (grip.green > grip.blue ? grip.green : grip.blue);
    const uint16_t chroma = static_cast<uint16_t>(maximum - minimum);
    const uint16_t peak =
        static_cast<uint16_t>((static_cast<uint16_t>(maximum) * 2u + 1u) /
                              3u);
    if (chroma == 0) {
        const uint8_t gray = static_cast<uint8_t>(peak);
        return {gray, gray, gray};
    }

    const auto calibrate = [minimum, chroma, peak](uint8_t component) {
        const uint32_t delta =
            static_cast<uint32_t>(component - minimum);
        return static_cast<uint8_t>(
            (static_cast<uint32_t>(peak) * delta * delta) /
            (static_cast<uint32_t>(chroma) * chroma));
    };
    return {calibrate(grip.red), calibrate(grip.green),
            calibrate(grip.blue)};
}


// Return the configured Switch grip color and its automatically calibrated
// physical LED color for one HID/controller slot.
SwitchRgbColor switch_pro_get_slot_color(uint8_t instance);
SwitchRgbColor switch_pro_get_slot_light_color(uint8_t instance);


// Initialize one HID instance before entering the main loop.
void switch_pro_init(uint8_t instance);

// Update the desired controller state for one HID instance.
void switch_pro_set_input(uint8_t instance, const SwitchInputState& state);

// Drive one Switch Pro USB state machine; returns true only when a regular
// 0x30 input report was successfully queued.
bool switch_pro_task(uint8_t instance);

// Convert a packed UART message into controller state (returns true if parsed).
bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length,
                                  SwitchInputState& out_state);

// Driver state helpers
bool switch_pro_is_ready(uint8_t instance);

// Optional callback fired with decoded rumble intensities from one host
// interface.
typedef void (*SwitchRumbleCallback)(uint8_t instance,
                                     const SwitchRumbleOutput& rumble);
void switch_pro_set_rumble_callback(uint8_t instance,
                                    SwitchRumbleCallback callback);
