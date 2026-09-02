#pragma once

#include <stdint.h>

#include "switch_pro_driver.h"

namespace XInputFeasibility {

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

constexpr int16_t horizontal_axis(uint16_t value) {
    return static_cast<int16_t>(static_cast<int32_t>(value) - 32768);
}

constexpr int16_t vertical_axis(uint16_t value) {
    const int16_t horizontal = horizontal_axis(value);
    return horizontal == INT16_MIN ? INT16_MAX
                                   : static_cast<int16_t>(-horizontal);
}

inline InputReport build_input_report(const SwitchInputState &state) {
    InputReport report{};
    report.report_size = sizeof(report);
    report.buttons =
        (state.dpad_up ? kDpadUp : 0) | (state.dpad_down ? kDpadDown : 0) |
        (state.dpad_left ? kDpadLeft : 0) |
        (state.dpad_right ? kDpadRight : 0) | (state.button_plus ? kStart : 0) |
        (state.button_minus ? kBack : 0) | (state.button_l3 ? kLeftThumb : 0) |
        (state.button_r3 ? kRightThumb : 0) |
        (state.button_l ? kLeftShoulder : 0) |
        (state.button_r ? kRightShoulder : 0) |
        (state.button_home ? kGuide : 0) |
        // Switch labels are positional opposites of XInput labels.
        (state.button_b ? kButtonA : 0) | (state.button_a ? kButtonB : 0) |
        (state.button_y ? kButtonX : 0) | (state.button_x ? kButtonY : 0);
    report.left_trigger = state.button_zl ? 0xff : 0x00;
    report.right_trigger = state.button_zr ? 0xff : 0x00;
    report.left_x = horizontal_axis(state.lx);
    report.left_y = vertical_axis(state.ly);
    report.right_x = horizontal_axis(state.rx);
    report.right_y = vertical_axis(state.ry);
    return report;
}

inline bool parse_rumble_report(const uint8_t *data, uint32_t size,
                                SwitchRumbleOutput *output) {
    if (data == nullptr || output == nullptr || size < 5 || data[0] != 0x00 ||
        data[1] != 0x08) {
        return false;
    }
    output->low_frequency_magnitude = data[3];
    output->high_frequency_magnitude = data[4];
    return true;
}

} // namespace XInputFeasibility
