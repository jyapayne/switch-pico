#include "usb/generic_hid/generic_hid_descriptors.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#ifndef EXPECTED_HID_INSTANCE_COUNT
#error "EXPECTED_HID_INSTANCE_COUNT must be defined by the test build"
#endif

static_assert(SWITCH_PICO_HID_INSTANCE_COUNT == EXPECTED_HID_INSTANCE_COUNT);
static_assert(sizeof(GenericHid::kDInputConfigurationDescriptor) ==
              9u + 25u * EXPECTED_HID_INSTANCE_COUNT);
static_assert(sizeof(GenericHid::kMacConfigurationDescriptor) ==
              9u + 25u * EXPECTED_HID_INSTANCE_COUNT);
static_assert(sizeof(GenericHid::InputReport) == 15);
static_assert(sizeof(GenericHid::kMacReportDescriptor) ==
              sizeof(GenericHid::kDInputReportDescriptor) + 4u);


namespace {

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        ++failures;
    }
}

uint16_t read_u16(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8u);
}

int16_t read_i16(const uint8_t* bytes) {
    return static_cast<int16_t>(read_u16(bytes));
}

struct ItemGolden {
    uint8_t type;
    uint8_t tag;
    uint8_t size;
    uint32_t value;
};

constexpr std::array<ItemGolden, 43> kDInputReportItemGolden{{
    {1, 0, 1, 0x01}, {2, 0, 1, 0x05}, {0, 10, 1, 0x01},
    {1, 0, 1, 0x01}, {1, 1, 2, 0x8000}, {1, 2, 2, 0x7fff},
    {1, 7, 1, 0x10}, {1, 9, 1, 0x04}, {2, 0, 1, 0x30},
    {2, 0, 1, 0x31}, {2, 0, 1, 0x33}, {2, 0, 1, 0x34},
    {0, 8, 1, 0x02}, {1, 1, 1, 0x00}, {1, 2, 4, 0xffff},
    {1, 9, 1, 0x02}, {2, 0, 1, 0x32}, {2, 0, 1, 0x35},
    {0, 8, 1, 0x02}, {1, 1, 1, 0x00}, {1, 2, 1, 0x07},
    {1, 3, 1, 0x00}, {1, 4, 2, 0x013b}, {1, 6, 1, 0x14},
    {1, 7, 1, 0x04}, {1, 9, 1, 0x01}, {2, 0, 1, 0x39},
    {0, 8, 1, 0x42}, {1, 7, 1, 0x04}, {1, 9, 1, 0x01},
    {0, 8, 1, 0x03}, {1, 0, 1, 0x09}, {1, 1, 1, 0x00},
    {1, 2, 1, 0x01}, {1, 3, 1, 0x00}, {1, 4, 1, 0x00},
    {1, 6, 1, 0x00}, {2, 1, 1, 0x01}, {2, 2, 1, 0x10},
    {1, 7, 1, 0x01}, {1, 9, 1, 0x10}, {0, 8, 1, 0x02},
    {0, 12, 0, 0x00},
}};

bool expected_report_item(size_t decoded, bool mac_variant,
                          ItemGolden* expected) {
    if (!mac_variant) {
        if (decoded >= kDInputReportItemGolden.size()) {
            return false;
        }
        *expected = kDInputReportItemGolden[decoded];
        return true;
    }

    if (decoded < 16) {
        if (decoded >= kDInputReportItemGolden.size()) {
            return false;
        }
        *expected = kDInputReportItemGolden[decoded];
        if (decoded == 10) {
            expected->value = 0x32;
        } else if (decoded == 11) {
            expected->value = 0x33;
        }
        return true;
    }
    if (decoded == 16) {
        *expected = {1, 0, 1, 0x02};
        return true;
    }
    if (decoded == 17) {
        *expected = {2, 0, 1, 0xc5};
        return true;
    }
    if (decoded == 18) {
        *expected = {2, 0, 1, 0xc4};
        return true;
    }
    if (decoded == 19) {
        *expected = kDInputReportItemGolden[18];
        return true;
    }
    if (decoded == 20) {
        *expected = {1, 0, 1, 0x01};
        return true;
    }
    if (decoded - 2u >= kDInputReportItemGolden.size()) {
        return false;
    }
    *expected = kDInputReportItemGolden[decoded - 2u];
    return true;
}

int32_t sign_extend(uint32_t value, uint8_t size) {
    if (size == 4 || size == 0) {
        return static_cast<int32_t>(value);
    }
    const uint8_t bits = static_cast<uint8_t>(size * 8u);
    const uint32_t sign = 1u << (bits - 1u);
    return static_cast<int32_t>((value ^ sign) - sign);
}

struct GlobalState {
    uint32_t usage_page = 0;
    int32_t logical_minimum = 0;
    int64_t logical_maximum = 0;
    int32_t physical_minimum = 0;
    int64_t physical_maximum = 0;
    uint32_t unit = 0;
    uint32_t report_size = 0;
    uint32_t report_count = 0;
};

struct LocalState {
    std::array<uint32_t, 4> usages{};
    uint8_t usage_count = 0;
    uint32_t usage_minimum = 0;
    uint32_t usage_maximum = 0;
    bool has_usage_range = false;

    void clear() {
        *this = {};
    }
};

struct InputField {
    uint16_t bit_offset;
    uint8_t size;
    uint8_t count;
    uint8_t flags;
    GlobalState globals;
    LocalState locals;
};

void inspect_report_descriptor(const uint8_t* descriptor,
                               size_t descriptor_size, bool mac_variant) {
    const std::array<uint32_t, 4> expected_stick_usages =
        mac_variant ? std::array<uint32_t, 4>{{0x30, 0x31, 0x32, 0x33}}
                    : std::array<uint32_t, 4>{{0x30, 0x31, 0x33, 0x34}};
    const std::array<uint32_t, 2> expected_trigger_usages =
        mac_variant ? std::array<uint32_t, 2>{{0xc5, 0xc4}}
                    : std::array<uint32_t, 2>{{0x32, 0x35}};
    size_t offset = 0;
    size_t decoded = 0;
    uint16_t report_bits = 0;
    uint8_t collection_depth = 0;
    bool gamepad_application = false;
    bool saw_report_id = false;
    bool saw_output = false;
    bool saw_feature = false;
    GlobalState globals{};
    LocalState locals{};
    std::vector<InputField> fields;

    while (offset < descriptor_size) {
        const uint8_t prefix = descriptor[offset++];
        expect(prefix != 0xfe, "long HID item is not part of the golden contract");
        if (prefix == 0xfe) {
            break;
        }
        uint8_t size = prefix & 0x03u;
        if (size == 3) {
            size = 4;
        }
        expect(offset + size <= descriptor_size,
               "HID item extends beyond the report descriptor");
        if (offset + size > descriptor_size) {
            break;
        }
        uint32_t value = 0;
        for (uint8_t byte = 0; byte < size; ++byte) {
            value |= static_cast<uint32_t>(descriptor[offset + byte]) <<
                     (8u * byte);
        }
        offset += size;
        const uint8_t type = static_cast<uint8_t>((prefix >> 2u) & 0x03u);
        const uint8_t tag = static_cast<uint8_t>(prefix >> 4u);

        ItemGolden golden{};
        const bool has_golden =
            expected_report_item(decoded, mac_variant, &golden);
        expect(has_golden, "report descriptor contains an extra HID item");
        if (has_golden) {
            expect(type == golden.type && tag == golden.tag &&
                       size == golden.size && value == golden.value,
                   "decoded HID item differs from its mode golden");
        }
        ++decoded;

        if (type == 1) {
            switch (tag) {
                case 0:
                    globals.usage_page = value;
                    break;
                case 1:
                    globals.logical_minimum = sign_extend(value, size);
                    break;
                case 2:
                    globals.logical_maximum = globals.logical_minimum < 0
                                                  ? sign_extend(value, size)
                                                  : value;
                    break;
                case 3:
                    globals.physical_minimum = sign_extend(value, size);
                    break;
                case 4:
                    globals.physical_maximum = globals.physical_minimum < 0
                                                   ? sign_extend(value, size)
                                                   : value;
                    break;
                case 6:
                    globals.unit = value;
                    break;
                case 7:
                    globals.report_size = value;
                    break;
                case 8:
                    saw_report_id = true;
                    break;
                case 9:
                    globals.report_count = value;
                    break;
                default:
                    expect(false, "unexpected global HID item");
                    break;
            }
            continue;
        }
        if (type == 2) {
            if (tag == 0) {
                expect(locals.usage_count < locals.usages.size(),
                       "too many local usages in the report descriptor");
                if (locals.usage_count < locals.usages.size()) {
                    locals.usages[locals.usage_count++] = value;
                }
            } else if (tag == 1) {
                locals.usage_minimum = value;
                locals.has_usage_range = true;
            } else if (tag == 2) {
                locals.usage_maximum = value;
                locals.has_usage_range = true;
            } else {
                expect(false, "unexpected local HID item");
            }
            continue;
        }

        expect(type == 0, "reserved HID item type is present");
        if (type != 0) {
            continue;
        }
        if (tag == 10) {
            gamepad_application = collection_depth == 0 && value == 1 &&
                                  globals.usage_page == 1 &&
                                  locals.usage_count == 1 &&
                                  locals.usages[0] == 5;
            ++collection_depth;
        } else if (tag == 12) {
            expect(collection_depth > 0, "unbalanced End Collection item");
            if (collection_depth > 0) {
                --collection_depth;
            }
        } else if (tag == 8) {
            fields.push_back(InputField{
                report_bits,
                static_cast<uint8_t>(globals.report_size),
                static_cast<uint8_t>(globals.report_count),
                static_cast<uint8_t>(value),
                globals,
                locals,
            });
            report_bits = static_cast<uint16_t>(
                report_bits + globals.report_size * globals.report_count);
        } else if (tag == 9) {
            saw_output = true;
        } else if (tag == 11) {
            saw_feature = true;
        } else {
            expect(false, "unexpected main HID item");
        }
        locals.clear();
    }

    const size_t expected_item_count =
        kDInputReportItemGolden.size() + (mac_variant ? 2u : 0u);
    expect(decoded == expected_item_count,
           "report descriptor is missing a golden HID item");
    expect(gamepad_application && collection_depth == 0,
           "report is not one balanced Game Pad application collection");
    expect(!saw_report_id, "single-interface report unexpectedly has a Report ID");
    expect(!saw_output && !saw_feature,
           "input-only generic HID descriptor declares output or feature data");
    expect(report_bits == GenericHid::kReportSize * 8u,
           "input report does not contain exactly 15 bytes");
    expect(fields.size() == 5, "input report has the wrong field count");
    if (fields.size() != 5) {
        return;
    }

    const InputField& sticks = fields[0];
    expect(sticks.bit_offset == 0 && sticks.size == 16 && sticks.count == 4 &&
               sticks.flags == 0x02 && sticks.globals.usage_page == 1 &&
               sticks.globals.logical_minimum == -32768 &&
               sticks.globals.logical_maximum == 32767 &&
               sticks.locals.usage_count == expected_stick_usages.size() &&
               sticks.locals.usages[0] == expected_stick_usages[0] &&
               sticks.locals.usages[1] == expected_stick_usages[1] &&
               sticks.locals.usages[2] == expected_stick_usages[2] &&
               sticks.locals.usages[3] == expected_stick_usages[3],
           mac_variant ? "signed Mac X/Y/Z/Rx stick field layout is wrong"
                       : "signed DInput X/Y/Rx/Ry stick field layout is wrong");

    const InputField& triggers = fields[1];
    expect(triggers.bit_offset == 64 && triggers.size == 16 &&
               triggers.count == 2 && triggers.flags == 0x02 &&
               triggers.globals.usage_page == (mac_variant ? 2u : 1u) &&
               triggers.globals.logical_minimum == 0 &&
               triggers.globals.logical_maximum == 65535 &&
               triggers.locals.usage_count ==
                   expected_trigger_usages.size() &&
               triggers.locals.usages[0] == expected_trigger_usages[0] &&
               triggers.locals.usages[1] == expected_trigger_usages[1],
           mac_variant
               ? "unsigned Mac Brake/Accelerator trigger field layout is wrong"
               : "unsigned DInput Z/Rz trigger field layout is wrong");
    if (mac_variant) {
        expect((sticks.bit_offset + 3u * sticks.size) / 8u == 6u &&
                   sticks.locals.usages[3] == 0x33u &&
                   triggers.bit_offset / 8u == 8u &&
                   (triggers.bit_offset + triggers.size) / 8u == 10u,
               "Mac X/Y/Z/Rx sticks and following trigger offsets changed");
    }

    const InputField& hat = fields[2];
    expect(hat.bit_offset == 96 && hat.size == 4 && hat.count == 1 &&
               hat.flags == 0x42 && hat.globals.usage_page == 1 &&
               hat.globals.logical_minimum == 0 &&
               hat.globals.logical_maximum == 7 &&
               hat.globals.physical_minimum == 0 &&
               hat.globals.physical_maximum == 315 &&
               hat.globals.unit == 0x14 && hat.locals.usage_count == 1 &&
               hat.locals.usages[0] == 0x39,
           "Hat Switch field or declared null-state semantics are wrong");

    const InputField& padding = fields[3];
    expect(padding.bit_offset == 100 && padding.size == 4 &&
               padding.count == 1 && padding.flags == 0x03 &&
               padding.locals.usage_count == 0 &&
               !padding.locals.has_usage_range,
           "hat padding is not four constant bits");

    const InputField& buttons = fields[4];
    expect(buttons.bit_offset == 104 && buttons.size == 1 &&
               buttons.count == 16 && buttons.flags == 0x02 &&
               buttons.globals.usage_page == 9 &&
               buttons.globals.logical_minimum == 0 &&
               buttons.globals.logical_maximum == 1 &&
               buttons.globals.unit == 0 && buttons.locals.has_usage_range &&
               buttons.locals.usage_minimum == 1 &&
               buttons.locals.usage_maximum == 16,
           "sequential Button 1..16 field layout is wrong");
}


void inspect_device_descriptors_and_strings() {
    constexpr std::array<uint8_t, 18> dinput_golden{{
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xfe,
        0xca, 0x20, 0x40, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
    }};
    constexpr std::array<uint8_t, 18> mac_golden{{
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xfe,
        0xca, 0x21, 0x40, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
    }};
    expect(std::memcmp(GenericHid::kDInputDeviceDescriptor,
                       dinput_golden.data(), dinput_golden.size()) == 0,
           "DInput development device descriptor differs from its golden");
    expect(std::memcmp(GenericHid::kMacDeviceDescriptor, mac_golden.data(),
                       mac_golden.size()) == 0,
           "Mac development device descriptor differs from its golden");
    expect(read_u16(GenericHid::kDInputDeviceDescriptor + 8) == 0xcafe &&
               read_u16(GenericHid::kDInputDeviceDescriptor + 10) == 0x4020 &&
               read_u16(GenericHid::kMacDeviceDescriptor + 8) == 0xcafe &&
               read_u16(GenericHid::kMacDeviceDescriptor + 10) == 0x4021,
           "development VID/PIDs are wrong");
    expect(std::strcmp(GenericHid::kManufacturerString, "Switch Pico") == 0 &&
               std::strcmp(GenericHid::kDInputProductString,
                           "DInput Development") == 0 &&
               std::strcmp(GenericHid::kDInputSerialString,
                           "DINPUT-DEV-4020") == 0 &&
               std::strcmp(GenericHid::kMacProductString,
                           "Mac HID Development") == 0 &&
               std::strcmp(GenericHid::kMacSerialString,
                           "MAC-HID-DEV-4021") == 0,
           "generic HID USB strings differ from their goldens");
    expect(std::strcmp(GenericHid::kDInputProductString,
                       GenericHid::kMacProductString) != 0 &&
               std::strcmp(GenericHid::kDInputSerialString,
                           GenericHid::kMacSerialString) != 0,
           "DInput and Mac identities do not have distinct strings");
}

void inspect_configuration_descriptor(
    const uint8_t* descriptor, size_t descriptor_size,
    size_t expected_report_descriptor_size) {
    constexpr uint8_t kConfiguration = 0x02;
    constexpr uint8_t kInterface = 0x04;
    constexpr uint8_t kEndpoint = 0x05;
    constexpr uint8_t kHid = 0x21;

    expect(descriptor[0] == 9 && descriptor[1] == kConfiguration &&
               read_u16(descriptor + 2) == descriptor_size &&
               descriptor[4] == EXPECTED_HID_INSTANCE_COUNT,
           "generic HID configuration header is malformed");

    std::array<bool, EXPECTED_HID_INSTANCE_COUNT> interfaces{};
    std::array<bool, 16> endpoints{};
    std::array<uint8_t, EXPECTED_HID_INSTANCE_COUNT> hid_counts{};
    std::array<uint8_t, EXPECTED_HID_INSTANCE_COUNT> endpoint_counts{};
    int current_interface = -1;
    size_t offset = descriptor[0];
    while (offset < descriptor_size) {
        const uint8_t length = descriptor[offset];
        expect(length >= 2 && offset + length <= descriptor_size,
               "configuration child descriptor has an invalid length");
        if (length < 2 || offset + length > descriptor_size) {
            break;
        }
        const uint8_t type = descriptor[offset + 1];
        if (type == kInterface) {
            expect(length == 9, "HID interface descriptor length is wrong");
            const uint8_t number = descriptor[offset + 2];
            expect(number < interfaces.size(),
                   "HID interface number is outside the configured range");
            if (number < interfaces.size()) {
                expect(!interfaces[number], "HID interface number is duplicated");
                interfaces[number] = true;
                current_interface = number;
            } else {
                current_interface = -1;
            }
            expect(descriptor[offset + 3] == 0 &&
                       descriptor[offset + 4] == 1 &&
                       descriptor[offset + 5] == 0x03 &&
                       descriptor[offset + 6] == 0 &&
                       descriptor[offset + 7] == 0,
                   "generic HID interface class or endpoint count is wrong");
        } else if (type == kHid) {
            expect(current_interface >= 0 && length == 9,
                   "HID descriptor is not attached to an interface");
            const uint16_t report_descriptor_length =
                read_u16(descriptor + offset + 7);
            expect(report_descriptor_length ==
                       expected_report_descriptor_size,
                   "HID descriptor advertises the wrong report length");
            if (current_interface >= 0) {
                ++hid_counts[static_cast<size_t>(current_interface)];
            }
        } else if (type == kEndpoint) {
            expect(current_interface >= 0 && length == 7,
                   "endpoint is not attached to an interface");
            const uint8_t address = descriptor[offset + 2];
            const uint8_t endpoint_number = address & 0x0fu;
            expect((address & 0x80u) != 0,
                   "generic HID exposes an OUT endpoint");
            expect(endpoint_number > 0 && endpoint_number < endpoints.size(),
                   "generic HID endpoint number is invalid");
            if (endpoint_number < endpoints.size()) {
                expect(!endpoints[endpoint_number],
                       "generic HID endpoint address is duplicated");
                endpoints[endpoint_number] = true;
            }
            expect(current_interface < 0 ||
                       address == static_cast<uint8_t>(
                                      0x81u + current_interface),
                   "IN endpoint does not belong to its HID interface");
            expect(descriptor[offset + 3] == 0x03 &&
                       read_u16(descriptor + offset + 4) ==
                           GenericHid::kEndpointSize &&
                       descriptor[offset + 6] ==
                           GenericHid::kEndpointIntervalMs,
                   "generic HID interrupt endpoint contract is wrong");
            if (current_interface >= 0) {
                ++endpoint_counts[static_cast<size_t>(current_interface)];
            }
        } else {
            expect(false, "unexpected configuration child descriptor type");
        }
        offset += length;
    }

    expect(offset == descriptor_size,
           "configuration descriptor was not fully decoded");
    for (size_t instance = 0; instance < interfaces.size(); ++instance) {
        expect(interfaces[instance] && hid_counts[instance] == 1 &&
                   endpoint_counts[instance] == 1 && endpoints[instance + 1],
               "configured HID interface is missing or not isolated");
    }
}

void inspect_configuration_descriptor_parity() {
    size_t difference_count = 0;
    for (size_t offset = 0;
         offset < sizeof(GenericHid::kDInputConfigurationDescriptor);
         ++offset) {
        const uint8_t dinput =
            GenericHid::kDInputConfigurationDescriptor[offset];
        const uint8_t mac = GenericHid::kMacConfigurationDescriptor[offset];
        if (dinput == mac) {
            continue;
        }
        const size_t expected_offset = 25u + 25u * difference_count;
        expect(offset == expected_offset &&
                   dinput == sizeof(GenericHid::kDInputReportDescriptor) &&
                   mac == sizeof(GenericHid::kMacReportDescriptor),
               "configuration descriptors differ outside report lengths");
        ++difference_count;
    }
    expect(difference_count == EXPECTED_HID_INSTANCE_COUNT,
           "configuration descriptors do not differ once per interface");
}

void inspect_report_encoding() {
    ControllerState state{};
    state.left_stick_x = INT16_MIN;
    state.left_stick_y = INT16_MAX;
    state.right_stick_x = static_cast<int16_t>(0x1234);
    state.right_stick_y = static_cast<int16_t>(-0x1234);
    state.left_trigger = 0;
    state.right_trigger = UINT16_MAX;
    GenericHid::InputReport report = GenericHid::build_input_report(state);
    expect(read_i16(report.data + 0) == INT16_MIN &&
               read_i16(report.data + 2) == INT16_MAX &&
               read_i16(report.data + 4) == static_cast<int16_t>(0x1234) &&
               read_i16(report.data + 6) == static_cast<int16_t>(-0x1234),
           "signed stick endpoints or little-endian encoding are wrong");
    expect(read_u16(report.data + 8) == 0 &&
               read_u16(report.data + 10) == UINT16_MAX,
           "unsigned trigger endpoints or little-endian encoding are wrong");
    expect(report.data[12] == GenericHid::kHatCenter &&
               (report.data[12] & 0xf0u) == 0,
           "neutral Hat Switch does not use null value 8 with zero padding");

    struct HatCase {
        bool up;
        bool down;
        bool left;
        bool right;
        uint8_t expected;
    };
    constexpr std::array<HatCase, 9> hats{{
        {false, false, false, false, GenericHid::kHatCenter},
        {true, false, false, false, GenericHid::kHatUp},
        {true, false, false, true, GenericHid::kHatUpRight},
        {false, false, false, true, GenericHid::kHatRight},
        {false, true, false, true, GenericHid::kHatDownRight},
        {false, true, false, false, GenericHid::kHatDown},
        {false, true, true, false, GenericHid::kHatDownLeft},
        {false, false, true, false, GenericHid::kHatLeft},
        {true, false, true, false, GenericHid::kHatUpLeft},
    }};
    for (const HatCase& hat : hats) {
        ControllerState direction{};
        direction.dpad_up = hat.up;
        direction.dpad_down = hat.down;
        direction.dpad_left = hat.left;
        direction.dpad_right = hat.right;
        const GenericHid::InputReport direction_report =
            GenericHid::build_input_report(direction);
        expect(direction_report.data[12] == hat.expected,
               "D-pad direction or diagonal maps to the wrong hat value");
    }
    ControllerState contradictory{};
    contradictory.dpad_up = true;
    contradictory.dpad_down = true;
    contradictory.dpad_left = true;
    contradictory.dpad_right = true;
    expect(GenericHid::build_input_report(contradictory).data[12] ==
               GenericHid::kHatCenter,
           "contradictory D-pad input does not resolve to center");

    using ButtonMember = bool ControllerState::*;
    constexpr std::array<ButtonMember, 12> button_members{{
        &ControllerState::button_south,
        &ControllerState::button_east,
        &ControllerState::button_west,
        &ControllerState::button_north,
        &ControllerState::button_left_shoulder,
        &ControllerState::button_right_shoulder,
        &ControllerState::button_select,
        &ControllerState::button_start,
        &ControllerState::button_left_stick,
        &ControllerState::button_right_stick,
        &ControllerState::button_system,
        &ControllerState::button_capture,
    }};
    for (size_t button = 0; button < button_members.size(); ++button) {
        ControllerState pressed{};
        pressed.*button_members[button] = true;
        const GenericHid::InputReport button_report =
            GenericHid::build_input_report(pressed);
        expect(read_u16(button_report.data + 13) == (1u << button),
               "positional button maps to the wrong sequential usage");
    }
    ControllerState all_buttons{};
    for (ButtonMember member : button_members) {
        all_buttons.*member = true;
    }
    expect(read_u16(GenericHid::build_input_report(all_buttons).data + 13) ==
               GenericHid::kDefinedButtonMask,
           "button bits 12..15 are not reserved zero");
}

}  // namespace

int main() {
    inspect_report_descriptor(GenericHid::kDInputReportDescriptor,
                              sizeof(GenericHid::kDInputReportDescriptor),
                              false);
    inspect_report_descriptor(GenericHid::kMacReportDescriptor,
                              sizeof(GenericHid::kMacReportDescriptor), true);
    inspect_device_descriptors_and_strings();
    inspect_configuration_descriptor(
        GenericHid::kDInputConfigurationDescriptor,
        sizeof(GenericHid::kDInputConfigurationDescriptor),
        sizeof(GenericHid::kDInputReportDescriptor));
    inspect_configuration_descriptor(
        GenericHid::kMacConfigurationDescriptor,
        sizeof(GenericHid::kMacConfigurationDescriptor),
        sizeof(GenericHid::kMacReportDescriptor));
    inspect_configuration_descriptor_parity();
    inspect_report_encoding();
    return failures == 0 ? 0 : 1;
}
