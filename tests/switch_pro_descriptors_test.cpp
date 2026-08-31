#include "switch_pro_descriptors.h"
#include "tusb_config.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#ifndef EXPECTED_HID_INSTANCE_COUNT
#error "EXPECTED_HID_INSTANCE_COUNT must be defined by the test build"
#endif

static_assert(SWITCH_PICO_HID_INSTANCE_COUNT == EXPECTED_HID_INSTANCE_COUNT,
              "the requested HID instance count did not reach the descriptors");
static_assert(CFG_TUD_HID == EXPECTED_HID_INSTANCE_COUNT,
              "TinyUSB HID count differs from the descriptor count");
static_assert(sizeof(switch_pro_configuration_descriptor) ==
                  9u + 32u * EXPECTED_HID_INSTANCE_COUNT,
              "configuration descriptor has the wrong total size");

namespace {

constexpr uint8_t kConfigurationDescriptor = 0x02;
constexpr uint8_t kInterfaceDescriptor = 0x04;
constexpr uint8_t kEndpointDescriptor = 0x05;
constexpr uint8_t kHidDescriptor = 0x21;

#if EXPECTED_HID_INSTANCE_COUNT == 1
constexpr std::array<uint8_t, 41> kUartConfigurationDescriptor = {
    0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xA0, 0xFA,
    0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0xCB, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x08,
    0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x08,
};
#endif

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        ++failures;
    }
}

uint16_t read_u16(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8u);
}

struct InterfaceContract {
    bool present = false;
    bool in_endpoint = false;
    bool out_endpoint = false;
    uint8_t endpoint_count = 0;
    uint8_t hid_count = 0;
};

void inspect_configuration_descriptor() {
    const auto* descriptor = switch_pro_configuration_descriptor;
    constexpr size_t descriptor_size =
        sizeof(switch_pro_configuration_descriptor);
#if EXPECTED_HID_INSTANCE_COUNT == 1
    expect(std::memcmp(descriptor, kUartConfigurationDescriptor.data(),
                       descriptor_size) == 0,
           "UART configuration descriptor bytes changed");
#endif

    expect(descriptor[0] == 9 && descriptor[1] == kConfigurationDescriptor,
           "configuration header is malformed");
    expect(read_u16(descriptor + 2) == descriptor_size,
           "wTotalLength does not match the emitted descriptor");
    expect(descriptor[4] == EXPECTED_HID_INSTANCE_COUNT,
           "bNumInterfaces does not match the HID instance count");

    std::array<InterfaceContract, EXPECTED_HID_INSTANCE_COUNT> interfaces{};
    std::array<bool, 256> endpoint_addresses{};
    int current_interface = -1;
    size_t offset = descriptor[0];

    while (offset < descriptor_size) {
        const uint8_t length = descriptor[offset];
        expect(length >= 2, "descriptor block has an invalid length");
        if (length < 2) {
            break;
        }
        expect(offset + length <= descriptor_size,
               "descriptor block extends beyond wTotalLength");
        if (offset + length > descriptor_size) {
            break;
        }

        const uint8_t type = descriptor[offset + 1];
        if (type == kInterfaceDescriptor) {
            expect(length == 9, "interface descriptor has the wrong length");
            const uint8_t number = descriptor[offset + 2];
            expect(number < interfaces.size(),
                   "interface number is outside the configured range");
            if (number < interfaces.size()) {
                expect(!interfaces[number].present,
                       "interface number is duplicated");
                interfaces[number].present = true;
                current_interface = number;
            } else {
                current_interface = -1;
            }
            expect(descriptor[offset + 3] == 0,
                   "interface uses an unexpected alternate setting");
            expect(descriptor[offset + 4] == 2,
                   "interface does not declare two endpoints");
            expect(descriptor[offset + 5] == 0x03,
                   "interface is not HID class");
        } else if (type == kHidDescriptor) {
            expect(current_interface >= 0,
                   "HID descriptor appears before an interface");
            expect(length == sizeof(switch_pro_hid_descriptor),
                   "HID descriptor has the wrong length");
            expect(std::memcmp(descriptor + offset, switch_pro_hid_descriptor,
                               sizeof(switch_pro_hid_descriptor)) == 0,
                   "interfaces do not reuse the shared HID/report contract");
            expect(read_u16(descriptor + offset + 7) ==
                       sizeof(switch_pro_report_descriptor),
                   "HID descriptor advertises the wrong report descriptor size");
            if (current_interface >= 0) {
                ++interfaces[static_cast<size_t>(current_interface)].hid_count;
            }
        } else if (type == kEndpointDescriptor) {
            expect(current_interface >= 0,
                   "endpoint descriptor appears before an interface");
            expect(length == 7, "endpoint descriptor has the wrong length");
            const uint8_t address = descriptor[offset + 2];
            expect(!endpoint_addresses[address],
                   "endpoint address is duplicated across interfaces");
            endpoint_addresses[address] = true;
            expect(descriptor[offset + 3] == 0x03,
                   "endpoint is not interrupt type");
            expect(read_u16(descriptor + offset + 4) ==
                       SWITCH_PRO_ENDPOINT_SIZE,
                   "endpoint has the wrong maximum packet size");
            expect(descriptor[offset + 6] == 8,
                   "endpoint has the wrong polling interval");

            if (current_interface >= 0) {
                auto& interface =
                    interfaces[static_cast<size_t>(current_interface)];
                ++interface.endpoint_count;
                const uint8_t endpoint_number =
                    static_cast<uint8_t>(current_interface + 1);
                if ((address & 0x80u) != 0) {
                    expect(address == static_cast<uint8_t>(0x80u | endpoint_number),
                           "IN endpoint does not belong to its interface");
                    interface.in_endpoint = true;
                } else {
                    expect(address == endpoint_number,
                           "OUT endpoint does not belong to its interface");
                    interface.out_endpoint = true;
                }
            }
        }

        offset += length;
    }

    expect(offset == descriptor_size,
           "descriptor parser did not finish at wTotalLength");
    for (const auto& interface : interfaces) {
        expect(interface.present, "configured HID interface is missing");
        expect(interface.hid_count == 1,
               "interface does not contain exactly one HID descriptor");
        expect(interface.endpoint_count == 2,
               "interface does not contain exactly two endpoints");
        expect(interface.in_endpoint && interface.out_endpoint,
               "interface is missing an IN or OUT endpoint");
    }
}

}  // namespace

int main() {
    inspect_configuration_descriptor();
    return failures == 0 ? 0 : 1;
}
