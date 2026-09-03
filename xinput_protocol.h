#pragma once

#include <stdint.h>

#include "controller_state.h"
#include "switch_haptics.h"

namespace XInput {

constexpr uint16_t kDpadUp = 0x0001;
constexpr uint16_t kDpadDown = 0x0002;
constexpr uint16_t kDpadLeft = 0x0004;
constexpr uint16_t kDpadRight = 0x0008;
constexpr uint16_t kStart = 0x0010;
constexpr uint16_t kBack = 0x0020;
constexpr uint16_t kLeftThumb = 0x0040;
constexpr uint16_t kRightThumb = 0x0080;
constexpr uint16_t kLeftShoulder = 0x0100;
constexpr uint16_t kRightShoulder = 0x0200;
constexpr uint16_t kGuide = 0x0400;
constexpr uint16_t kShare = 0x0800;
constexpr uint16_t kButtonA = 0x1000;
constexpr uint16_t kButtonB = 0x2000;
constexpr uint16_t kButtonX = 0x4000;
constexpr uint16_t kButtonY = 0x8000;

#pragma pack(push, 1)
struct InputReport {
    uint8_t report_id;
    uint8_t report_size;
    uint16_t buttons;
    uint8_t left_trigger;
    uint8_t right_trigger;
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(InputReport) == 20);

constexpr int16_t invert_axis(int16_t value) {
    return value == INT16_MIN ? INT16_MAX
                              : static_cast<int16_t>(-value);
}

inline InputReport build_input_report(const ControllerState& state) {
    InputReport report{};
    report.report_size = sizeof(report);
    report.buttons =
        (state.dpad_up ? kDpadUp : 0) |
        (state.dpad_down ? kDpadDown : 0) |
        (state.dpad_left ? kDpadLeft : 0) |
        (state.dpad_right ? kDpadRight : 0) |
        (state.button_start ? kStart : 0) |
        (state.button_select ? kBack : 0) |
        (state.button_left_stick ? kLeftThumb : 0) |
        (state.button_right_stick ? kRightThumb : 0) |
        (state.button_left_shoulder ? kLeftShoulder : 0) |
        (state.button_right_shoulder ? kRightShoulder : 0) |
        (state.button_system ? kGuide : 0) |
        (state.button_capture ? kShare : 0) |
        (state.button_south ? kButtonA : 0) |
        (state.button_east ? kButtonB : 0) |
        (state.button_west ? kButtonX : 0) |
        (state.button_north ? kButtonY : 0);
    report.left_trigger = controller_trigger_to_u8(state.left_trigger);
    report.right_trigger = controller_trigger_to_u8(state.right_trigger);
    report.left_x = state.left_stick_x;
    report.left_y = invert_axis(state.left_stick_y);
    report.right_x = state.right_stick_x;
    report.right_y = invert_axis(state.right_stick_y);
    return report;
}

inline bool parse_rumble_report(const uint8_t *data, uint32_t size,
                                ControllerRumbleOutput *output) {
    if (data == nullptr || output == nullptr || size < 5 || data[0] != 0x00 ||
        data[1] != 0x08) {
        return false;
    }
    output->low_frequency_magnitude = data[3];
    output->high_frequency_magnitude = data[4];
    return true;
}

} // namespace XInput
