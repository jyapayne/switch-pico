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
