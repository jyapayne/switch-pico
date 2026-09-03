#pragma once

#include <stdint.h>

constexpr uint8_t CONTROLLER_MOTION_SAMPLE_CAPACITY = 3;
constexpr uint16_t CONTROLLER_AXIS_UNSIGNED_MIN = 0;
constexpr uint16_t CONTROLLER_AXIS_UNSIGNED_CENTER =
    static_cast<uint16_t>(UINT16_MAX / 2u + 1u);
constexpr uint16_t CONTROLLER_AXIS_UNSIGNED_MAX = UINT16_MAX;
constexpr uint16_t CONTROLLER_TRIGGER_MIN = 0;
constexpr uint16_t CONTROLLER_TRIGGER_MAX = UINT16_MAX;
constexpr uint8_t CONTROLLER_TRIGGER_TO_U8_SHIFT = 8;


// Motion uses the existing fixed-point units carried by the UART protocol and
// consumed by the Switch serializer. Axis names are controller-relative.
struct ControllerMotionSample {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

// Protocol-neutral controller state. Face buttons are positional, sticks are
// signed with zero at rest, and triggers retain their full analog range.
struct ControllerState {
    bool dpad_up;
    bool dpad_down;
    bool dpad_left;
    bool dpad_right;

    bool button_south;
    bool button_east;
    bool button_west;
    bool button_north;
    bool button_left_shoulder;
    bool button_right_shoulder;
    bool button_select;
    bool button_start;
    bool button_system;
    bool button_capture;
    bool button_left_stick;
    bool button_right_stick;

    uint16_t left_trigger;
    uint16_t right_trigger;

    int16_t left_stick_x;
    int16_t left_stick_y;
    int16_t right_stick_x;
    int16_t right_stick_y;

    uint8_t motion_sample_count;
    ControllerMotionSample
        motion_samples[CONTROLLER_MOTION_SAMPLE_CAPACITY];
};
constexpr uint16_t controller_axis_to_unsigned(int16_t value) {
    return static_cast<uint16_t>(
        static_cast<int32_t>(value) - INT16_MIN);
}

constexpr int16_t controller_axis_from_unsigned(uint16_t value) {
    return static_cast<int16_t>(
        static_cast<int32_t>(value) + INT16_MIN);
}

constexpr uint8_t controller_trigger_to_u8(uint16_t value) {
    return static_cast<uint8_t>(
        value >> CONTROLLER_TRIGGER_TO_U8_SHIFT);
}

constexpr ControllerState controller_neutral_state() {
    return {};
}

static_assert(controller_axis_to_unsigned(INT16_MIN) ==
              CONTROLLER_AXIS_UNSIGNED_MIN);
static_assert(controller_axis_to_unsigned(0) ==
              CONTROLLER_AXIS_UNSIGNED_CENTER);
static_assert(controller_axis_to_unsigned(INT16_MAX) ==
              CONTROLLER_AXIS_UNSIGNED_MAX);
static_assert(controller_axis_from_unsigned(
                  CONTROLLER_AXIS_UNSIGNED_MIN) == INT16_MIN);
static_assert(controller_axis_from_unsigned(
                  CONTROLLER_AXIS_UNSIGNED_CENTER) == 0);
static_assert(controller_axis_from_unsigned(
                  CONTROLLER_AXIS_UNSIGNED_MAX) == INT16_MAX);
static_assert(controller_trigger_to_u8(0x7fff) == 0x7f);
static_assert(controller_trigger_to_u8(0x8000) == 0x80);
static_assert(controller_trigger_to_u8(CONTROLLER_TRIGGER_MAX) == UINT8_MAX);
