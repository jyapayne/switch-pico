#pragma once

#include <stddef.h>
#include <stdint.h>

#include "controller_state.h"

#ifndef SWITCH_PICO_HID_INSTANCE_COUNT
#define SWITCH_PICO_HID_INSTANCE_COUNT 1
#endif

#if SWITCH_PICO_HID_INSTANCE_COUNT < 1 || SWITCH_PICO_HID_INSTANCE_COUNT > 4
#error "SWITCH_PICO_HID_INSTANCE_COUNT must be between 1 and 4"
#endif

namespace GenericHid {

constexpr uint16_t kDevelopmentVendorId = 0xcafe;
constexpr uint16_t kDInputDevelopmentProductId = 0x4020;
constexpr uint16_t kMacDevelopmentProductId = 0x4021;
constexpr uint16_t kDevelopmentDeviceRevision = 0x0100;

constexpr uint8_t kReportSize = 15;
constexpr uint8_t kEndpointSize = kReportSize;
constexpr uint8_t kEndpointIntervalMs = 1;
constexpr uint8_t kInterfaceDescriptorSize = 25;
constexpr uint16_t kConfigurationDescriptorSize =
    9 + SWITCH_PICO_HID_INSTANCE_COUNT * kInterfaceDescriptorSize;

constexpr uint8_t kHatUp = 0;
constexpr uint8_t kHatUpRight = 1;
constexpr uint8_t kHatRight = 2;
constexpr uint8_t kHatDownRight = 3;
constexpr uint8_t kHatDown = 4;
constexpr uint8_t kHatDownLeft = 5;
constexpr uint8_t kHatLeft = 6;
constexpr uint8_t kHatUpLeft = 7;
constexpr uint8_t kHatCenter = 8;

constexpr uint16_t kButtonSouth = 1u << 0u;
constexpr uint16_t kButtonEast = 1u << 1u;
constexpr uint16_t kButtonWest = 1u << 2u;
constexpr uint16_t kButtonNorth = 1u << 3u;
constexpr uint16_t kButtonLeftShoulder = 1u << 4u;
constexpr uint16_t kButtonRightShoulder = 1u << 5u;
constexpr uint16_t kButtonSelect = 1u << 6u;
constexpr uint16_t kButtonStart = 1u << 7u;
constexpr uint16_t kButtonLeftStick = 1u << 8u;
constexpr uint16_t kButtonRightStick = 1u << 9u;
constexpr uint16_t kButtonSystem = 1u << 10u;
constexpr uint16_t kButtonCapture = 1u << 11u;
constexpr uint16_t kDefinedButtonMask = 0x0fffu;

struct InputReport {
    uint8_t data[kReportSize];
};

static_assert(sizeof(InputReport) == kReportSize);

constexpr void write_u16_le(uint8_t* destination, uint16_t value) {
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8u);
}

constexpr uint8_t build_hat(const ControllerState& state) {
    const bool up = state.dpad_up && !state.dpad_down;
    const bool down = state.dpad_down && !state.dpad_up;
    const bool left = state.dpad_left && !state.dpad_right;
    const bool right = state.dpad_right && !state.dpad_left;

    if (up) {
        if (right) {
            return kHatUpRight;
        }
        if (left) {
            return kHatUpLeft;
        }
        return kHatUp;
    }
    if (down) {
        if (right) {
            return kHatDownRight;
        }
        if (left) {
            return kHatDownLeft;
        }
        return kHatDown;
    }
    if (right) {
        return kHatRight;
    }
    if (left) {
        return kHatLeft;
    }
    return kHatCenter;
}

constexpr uint16_t build_buttons(const ControllerState& state) {
    return static_cast<uint16_t>(
        (state.button_south ? kButtonSouth : 0u) |
        (state.button_east ? kButtonEast : 0u) |
        (state.button_west ? kButtonWest : 0u) |
        (state.button_north ? kButtonNorth : 0u) |
        (state.button_left_shoulder ? kButtonLeftShoulder : 0u) |
        (state.button_right_shoulder ? kButtonRightShoulder : 0u) |
        (state.button_select ? kButtonSelect : 0u) |
        (state.button_start ? kButtonStart : 0u) |
        (state.button_left_stick ? kButtonLeftStick : 0u) |
        (state.button_right_stick ? kButtonRightStick : 0u) |
        (state.button_system ? kButtonSystem : 0u) |
        (state.button_capture ? kButtonCapture : 0u));
}

constexpr InputReport build_input_report(const ControllerState& state) {
    InputReport report{};
    write_u16_le(report.data + 0,
                 static_cast<uint16_t>(state.left_stick_x));
    write_u16_le(report.data + 2,
                 static_cast<uint16_t>(state.left_stick_y));
    write_u16_le(report.data + 4,
                 static_cast<uint16_t>(state.right_stick_x));
    write_u16_le(report.data + 6,
                 static_cast<uint16_t>(state.right_stick_y));
    write_u16_le(report.data + 8, state.left_trigger);
    write_u16_le(report.data + 10, state.right_trigger);
    report.data[12] = build_hat(state);
    write_u16_le(report.data + 13, build_buttons(state));
    return report;
}

inline constexpr char kManufacturerString[] = "Switch Pico";
inline constexpr char kDInputProductString[] = "DInput Development";
inline constexpr char kDInputSerialString[] = "DINPUT-DEV-4020";
inline constexpr char kMacProductString[] = "Mac HID Development";
inline constexpr char kMacSerialString[] = "MAC-HID-DEV-4021";

inline constexpr uint8_t kDInputDeviceDescriptor[] = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    static_cast<uint8_t>(kDevelopmentVendorId),
    static_cast<uint8_t>(kDevelopmentVendorId >> 8u),
    static_cast<uint8_t>(kDInputDevelopmentProductId),
    static_cast<uint8_t>(kDInputDevelopmentProductId >> 8u),
    static_cast<uint8_t>(kDevelopmentDeviceRevision),
    static_cast<uint8_t>(kDevelopmentDeviceRevision >> 8u),
    0x01, 0x02, 0x03, 0x01,
};

inline constexpr uint8_t kMacDeviceDescriptor[] = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    static_cast<uint8_t>(kDevelopmentVendorId),
    static_cast<uint8_t>(kDevelopmentVendorId >> 8u),
    static_cast<uint8_t>(kMacDevelopmentProductId),
    static_cast<uint8_t>(kMacDevelopmentProductId >> 8u),
    static_cast<uint8_t>(kDevelopmentDeviceRevision),
    static_cast<uint8_t>(kDevelopmentDeviceRevision >> 8u),
    0x01, 0x02, 0x03, 0x01,
};

// One report per interface, so no Report ID item is needed.
inline constexpr uint8_t kDInputReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xa1, 0x01,        // Collection (Application)

    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x16, 0x00, 0x80,  // Logical Minimum (-32768)
    0x26, 0xff, 0x7f,  // Logical Maximum (32767)
    0x75, 0x10,        // Report Size (16)
    0x95, 0x04,        // Report Count (4)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x09, 0x33,        // Usage (Rx)
    0x09, 0x34,        // Usage (Ry)
    0x81, 0x02,        // Input (Data, Variable, Absolute)

    0x15, 0x00,                    // Logical Minimum (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  // Logical Maximum (65535)
    0x95, 0x02,                    // Report Count (2)
    0x09, 0x32,                    // Usage (Z)
    0x09, 0x35,                    // Usage (Rz)
    0x81, 0x02,                    // Input (Data, Variable, Absolute)

    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x07,        // Logical Maximum (7)
    0x35, 0x00,        // Physical Minimum (0)
    0x46, 0x3b, 0x01,  // Physical Maximum (315)
    0x65, 0x14,        // Unit (English Rotation, Degrees)
    0x75, 0x04,        // Report Size (4)
    0x95, 0x01,        // Report Count (1)
    0x09, 0x39,        // Usage (Hat Switch)
    0x81, 0x42,        // Input (Data, Variable, Absolute, Null State)
    0x75, 0x04,        // Report Size (4)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x03,        // Input (Constant, Variable, Absolute)

    0x05, 0x09,  // Usage Page (Button)
    0x15, 0x00,  // Logical Minimum (0)
    0x25, 0x01,  // Logical Maximum (1)
    0x35, 0x00,  // Physical Minimum (0)
    0x45, 0x00,  // Physical Maximum (0)
    0x65, 0x00,  // Unit (None)
    0x19, 0x01,  // Usage Minimum (Button 1)
    0x29, 0x10,  // Usage Maximum (Button 16)
    0x75, 0x01,  // Report Size (1)
    0x95, 0x10,  // Report Count (16)
    0x81, 0x02,  // Input (Data, Variable, Absolute)
    0xc0,        // End Collection
};

inline constexpr uint8_t kMacReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xa1, 0x01,        // Collection (Application)

    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x16, 0x00, 0x80,  // Logical Minimum (-32768)
    0x26, 0xff, 0x7f,  // Logical Maximum (32767)
    0x75, 0x10,        // Report Size (16)
    0x95, 0x04,        // Report Count (4)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x09, 0x32,        // Usage (Z)
    0x09, 0x33,        // Usage (Rx)
    0x81, 0x02,        // Input (Data, Variable, Absolute)

    0x15, 0x00,                    // Logical Minimum (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  // Logical Maximum (65535)
    0x95, 0x02,                    // Report Count (2)
    0x05, 0x02,                    // Usage Page (Simulation Controls)
    0x09, 0xc5,                    // Usage (Brake)
    0x09, 0xc4,                    // Usage (Accelerator)
    0x81, 0x02,                    // Input (Data, Variable, Absolute)

    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x07,        // Logical Maximum (7)
    0x35, 0x00,        // Physical Minimum (0)
    0x46, 0x3b, 0x01,  // Physical Maximum (315)
    0x65, 0x14,        // Unit (English Rotation, Degrees)
    0x75, 0x04,        // Report Size (4)
    0x95, 0x01,        // Report Count (1)
    0x09, 0x39,        // Usage (Hat Switch)
    0x81, 0x42,        // Input (Data, Variable, Absolute, Null State)
    0x75, 0x04,        // Report Size (4)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x03,        // Input (Constant, Variable, Absolute)

    0x05, 0x09,  // Usage Page (Button)
    0x15, 0x00,  // Logical Minimum (0)
    0x25, 0x01,  // Logical Maximum (1)
    0x35, 0x00,  // Physical Minimum (0)
    0x45, 0x00,  // Physical Maximum (0)
    0x65, 0x00,  // Unit (None)
    0x19, 0x01,  // Usage Minimum (Button 1)
    0x29, 0x10,  // Usage Maximum (Button 16)
    0x75, 0x01,  // Report Size (1)
    0x95, 0x10,  // Report Count (16)
    0x81, 0x02,  // Input (Data, Variable, Absolute)
    0xc0,        // End Collection
};


#define GENERIC_HID_INTERFACE(number, endpoint, report_descriptor)           \
    0x09, 0x04, number, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,               \
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,                          \
        static_cast<uint8_t>(sizeof(report_descriptor)),                    \
        static_cast<uint8_t>(sizeof(report_descriptor) >> 8u),              \
        0x07, 0x05, static_cast<uint8_t>(0x80u | endpoint), 0x03,           \
        kEndpointSize, 0x00, kEndpointIntervalMs

inline constexpr uint8_t kDInputConfigurationDescriptor[] = {
    0x09, 0x02,
    static_cast<uint8_t>(kConfigurationDescriptorSize),
    static_cast<uint8_t>(kConfigurationDescriptorSize >> 8u),
    SWITCH_PICO_HID_INSTANCE_COUNT,
    0x01, 0x00, 0x80, 0xfa,
    GENERIC_HID_INTERFACE(0x00, 0x01, kDInputReportDescriptor),
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 2
    GENERIC_HID_INTERFACE(0x01, 0x02, kDInputReportDescriptor),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 3
    GENERIC_HID_INTERFACE(0x02, 0x03, kDInputReportDescriptor),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 4
    GENERIC_HID_INTERFACE(0x03, 0x04, kDInputReportDescriptor),
#endif
};

inline constexpr uint8_t kMacConfigurationDescriptor[] = {
    0x09, 0x02,
    static_cast<uint8_t>(kConfigurationDescriptorSize),
    static_cast<uint8_t>(kConfigurationDescriptorSize >> 8u),
    SWITCH_PICO_HID_INSTANCE_COUNT,
    0x01, 0x00, 0x80, 0xfa,
    GENERIC_HID_INTERFACE(0x00, 0x01, kMacReportDescriptor),
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 2
    GENERIC_HID_INTERFACE(0x01, 0x02, kMacReportDescriptor),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 3
    GENERIC_HID_INTERFACE(0x02, 0x03, kMacReportDescriptor),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 4
    GENERIC_HID_INTERFACE(0x03, 0x04, kMacReportDescriptor),
#endif
};

#undef GENERIC_HID_INTERFACE

static_assert(sizeof(kDInputDeviceDescriptor) == 18);
static_assert(sizeof(kMacDeviceDescriptor) == 18);
static_assert(sizeof(kDInputConfigurationDescriptor) ==
              kConfigurationDescriptorSize);
static_assert(sizeof(kMacConfigurationDescriptor) ==
              kConfigurationDescriptorSize);

}  // namespace GenericHid
