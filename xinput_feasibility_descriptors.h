#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef SWITCH_PICO_HID_INSTANCE_COUNT
#define SWITCH_PICO_HID_INSTANCE_COUNT 1
#endif

#if SWITCH_PICO_HID_INSTANCE_COUNT < 1 || SWITCH_PICO_HID_INSTANCE_COUNT > 4
#error "SWITCH_PICO_HID_INSTANCE_COUNT must be between 1 and 4"
#endif

namespace XInputFeasibility {

constexpr uint16_t kPrototypeVendorId = 0xcafe;
constexpr uint16_t kPrototypeProductId = 0x4010;
constexpr uint8_t kInterfaceDescriptorSize = 39;
constexpr uint16_t kConfigurationDescriptorSize =
    9 + SWITCH_PICO_HID_INSTANCE_COUNT * kInterfaceDescriptorSize;
constexpr uint16_t kMsCompatIdDescriptorSize =
    16 + SWITCH_PICO_HID_INSTANCE_COUNT * 24;
constexpr uint8_t kMsVendorRequest = 0x20;
constexpr uint16_t kMsCompatIdIndex = 0x0004;

static const uint8_t kDeviceDescriptor[] = {
    0x12,
    0x01, // Device descriptor
    0x00,
    0x02, // USB 2.0
    0xff,
    0xff,
    0xff, // Vendor-specific device
    0x40, // Endpoint zero packet size
    static_cast<uint8_t>(kPrototypeVendorId & 0xff),
    static_cast<uint8_t>(kPrototypeVendorId >> 8),
    static_cast<uint8_t>(kPrototypeProductId & 0xff),
    static_cast<uint8_t>(kPrototypeProductId >> 8),
    0x00,
    0x01, // Prototype revision 1.00
    0x01,
    0x02,
    0x03, // Manufacturer, product, serial strings
    0x01, // One configuration
};

#define XINPUT_FEASIBILITY_INTERFACE(number, endpoint)                         \
    0x09, 0x04, number, 0x00, 0x02, 0xff, 0x5d, 0x01, 0x00, 0x10, 0x21, 0x10,  \
        0x01, 0x01, 0x24, static_cast<uint8_t>(0x80 | endpoint), 0x14, 0x03,   \
        0x00, 0x03, 0x13, endpoint, 0x00, 0x03, 0x00, 0x07, 0x05,              \
        static_cast<uint8_t>(0x80 | endpoint), 0x03, 0x20, 0x00, 0x04, 0x07,   \
        0x05, endpoint, 0x03, 0x20, 0x00, 0x08

static const uint8_t kConfigurationDescriptor[] = {
    0x09,
    0x02,
    static_cast<uint8_t>(kConfigurationDescriptorSize & 0xff),
    static_cast<uint8_t>(kConfigurationDescriptorSize >> 8),
    SWITCH_PICO_HID_INSTANCE_COUNT,
    0x01,
    0x00,
    0x80,
    0xfa,
    XINPUT_FEASIBILITY_INTERFACE(0x00, 0x01),
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 2
    XINPUT_FEASIBILITY_INTERFACE(0x01, 0x02),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 3
    XINPUT_FEASIBILITY_INTERFACE(0x02, 0x03),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 4
    XINPUT_FEASIBILITY_INTERFACE(0x03, 0x04),
#endif
};

#undef XINPUT_FEASIBILITY_INTERFACE

#define XINPUT_FEASIBILITY_COMPAT_FUNCTION(number)                             \
    number, 0x01, 'X', 'U', 'S', 'B', '1', '0', 0x00, 0x00, 0x00, 0x00, 0x00,  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

static const uint8_t kMsCompatIdDescriptor[] = {
    static_cast<uint8_t>(kMsCompatIdDescriptorSize & 0xff),
    static_cast<uint8_t>(kMsCompatIdDescriptorSize >> 8),
    0x00,
    0x00,
    0x00,
    0x01, // Microsoft OS descriptor version 1.0
    0x04,
    0x00, // Extended compatible ID descriptor
    SWITCH_PICO_HID_INSTANCE_COUNT,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    XINPUT_FEASIBILITY_COMPAT_FUNCTION(0x00),
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 2
    XINPUT_FEASIBILITY_COMPAT_FUNCTION(0x01),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 3
    XINPUT_FEASIBILITY_COMPAT_FUNCTION(0x02),
#endif
#if SWITCH_PICO_HID_INSTANCE_COUNT >= 4
    XINPUT_FEASIBILITY_COMPAT_FUNCTION(0x03),
#endif
};

#undef XINPUT_FEASIBILITY_COMPAT_FUNCTION

static const uint8_t kProbeMsCompatIdDescriptor[] = {
    0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static_assert(sizeof(kDeviceDescriptor) == 18);
static_assert(sizeof(kConfigurationDescriptor) == kConfigurationDescriptorSize);
static_assert(sizeof(kMsCompatIdDescriptor) == kMsCompatIdDescriptorSize);
static_assert(sizeof(kProbeMsCompatIdDescriptor) == 16);

} // namespace XInputFeasibility
