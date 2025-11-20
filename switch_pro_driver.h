/*
 * Minimal Switch Pro controller emulation glue derived from the GP2040-CE
 * SwitchProDriver. The driver keeps the same descriptors/handshake while
 * exposing a simple API for feeding inputs.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "switch_pro_descriptors.h"

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
} SwitchInputState;

// Initialize USB state and calibration before entering the main loop.
void switch_pro_init();

// Update the desired controller state for the next USB report.
void switch_pro_set_input(const SwitchInputState& state);

// Drive the Switch Pro USB state machine; call this frequently in the main loop.
void switch_pro_task();

// Convert a packed UART message into controller state (returns true if parsed).
bool switch_pro_apply_uart_packet(const uint8_t* packet, uint8_t length);
