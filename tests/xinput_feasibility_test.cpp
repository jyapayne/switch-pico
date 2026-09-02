#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "adapter_host_probe_state.h"
#include "xinput_feasibility_descriptors.h"
#include "xinput_feasibility_protocol.h"

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

uint16_t read_le16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0] | (data[1] << 8));
}

uint32_t read_le32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

void test_device_and_configuration_descriptors() {
    using namespace XInputFeasibility;
    expect(read_le16(&kDeviceDescriptor[8]) == kPrototypeVendorId,
           "prototype VID mismatch");
    expect(read_le16(&kDeviceDescriptor[10]) == kPrototypeProductId,
           "prototype PID mismatch");
    expect(kPrototypeVendorId != 0x045e,
           "prototype must not impersonate Microsoft's VID");
    expect(read_le16(&kConfigurationDescriptor[2]) ==
               sizeof(kConfigurationDescriptor),
           "configuration total length mismatch");
    expect(kConfigurationDescriptor[4] == SWITCH_PICO_HID_INSTANCE_COUNT,
           "configuration interface count mismatch");

    std::array<bool, 16> endpoints{};
    for (uint8_t instance = 0; instance < SWITCH_PICO_HID_INSTANCE_COUNT;
         ++instance) {
        const size_t offset = 9 + instance * kInterfaceDescriptorSize;
        const uint8_t *interface = &kConfigurationDescriptor[offset];
        expect(interface[0] == 9 && interface[1] == 4,
               "missing interface descriptor");
        expect(interface[2] == instance, "interface number mismatch");
        expect(interface[5] == 0xff && interface[6] == 0x5d &&
                   interface[7] == 0x01,
               "XInput interface class tuple mismatch");
        expect(interface[9] == 0x10 && interface[10] == 0x21,
               "XInput capability descriptor missing");
        const uint8_t in_endpoint = interface[27];
        const uint8_t out_endpoint = interface[34];
        expect(in_endpoint == static_cast<uint8_t>(0x81 + instance),
               "input endpoint mismatch");
        expect(out_endpoint == static_cast<uint8_t>(0x01 + instance),
               "output endpoint mismatch");
        expect(interface[15] == in_endpoint && interface[21] == out_endpoint,
               "capability descriptor endpoint mismatch");
        expect(!endpoints[in_endpoint & 0x0f] &&
                   !endpoints[out_endpoint & 0x0f],
               "endpoint number reused");
        endpoints[in_endpoint & 0x0f] = true;
    }
}

void test_microsoft_compatible_id_descriptor() {
    using namespace XInputFeasibility;
    expect(read_le32(kMsCompatIdDescriptor) == sizeof(kMsCompatIdDescriptor),
           "Microsoft descriptor total length mismatch");
    expect(read_le16(&kMsCompatIdDescriptor[4]) == 0x0100,
           "Microsoft descriptor version mismatch");
    expect(read_le16(&kMsCompatIdDescriptor[6]) == kMsCompatIdIndex,
           "Microsoft descriptor index mismatch");
    expect(kMsCompatIdDescriptor[8] == SWITCH_PICO_HID_INSTANCE_COUNT,
           "Microsoft function count mismatch");
    for (uint8_t instance = 0; instance < SWITCH_PICO_HID_INSTANCE_COUNT;
         ++instance) {
        const uint8_t *function = &kMsCompatIdDescriptor[16 + instance * 24];
        expect(function[0] == instance,
               "Microsoft descriptor interface mismatch");
        expect(std::memcmp(&function[2], "XUSB10", 6) == 0,
               "XUSB10 compatible ID missing");
    }
    expect(read_le32(kProbeMsCompatIdDescriptor) == 16 &&
               kProbeMsCompatIdDescriptor[8] == 0,
           "probe descriptor must expose no compatible functions");
}

void test_input_report_mapping() {
    SwitchInputState state{};
    state.lx = state.ly = state.rx = state.ry = 32768;
    auto report = XInputFeasibility::build_input_report(state);
    expect(report.report_id == 0 && report.report_size == 20,
           "neutral report header mismatch");
    expect(report.buttons == 0 && report.left_trigger == 0 &&
               report.right_trigger == 0,
           "neutral report controls mismatch");
    expect(report.left_x == 0 && report.left_y == 0 && report.right_x == 0 &&
               report.right_y == 0,
           "neutral axes mismatch");

    state.dpad_up = true;
    state.button_b = true;
    state.button_a = true;
    state.button_y = true;
    state.button_x = true;
    state.button_plus = true;
    state.button_minus = true;
    state.button_home = true;
    state.button_zl = true;
    state.button_zr = true;
    state.lx = 0;
    state.ly = 0;
    state.rx = UINT16_MAX;
    state.ry = UINT16_MAX;
    report = XInputFeasibility::build_input_report(state);
    expect((report.buttons & XInputFeasibility::kDpadUp) != 0,
           "D-pad mapping missing");
    expect((report.buttons & XInputFeasibility::kButtonA) != 0 &&
               (report.buttons & XInputFeasibility::kButtonB) != 0 &&
               (report.buttons & XInputFeasibility::kButtonX) != 0 &&
               (report.buttons & XInputFeasibility::kButtonY) != 0,
           "positional face-button mapping mismatch");
    expect(report.left_trigger == 0xff && report.right_trigger == 0xff,
           "digital trigger mapping mismatch");
    expect(report.left_x == INT16_MIN && report.left_y == INT16_MAX &&
               report.right_x == INT16_MAX && report.right_y == -INT16_MAX,
           "axis endpoint mapping mismatch");
}

void test_rumble_report() {
    const uint8_t packet[8] = {0x00, 0x08, 0x00, 0xa5, 0x5a, 0x00, 0x00, 0x00};
    SwitchRumbleOutput output{};
    expect(
        XInputFeasibility::parse_rumble_report(packet, sizeof(packet), &output),
        "valid rumble report rejected");
    expect(output.low_frequency_magnitude == 0xa5 &&
               output.high_frequency_magnitude == 0x5a,
           "rumble magnitudes mapped incorrectly");
    expect(!XInputFeasibility::parse_rumble_report(packet, 4, &output),
           "truncated rumble report accepted");
    uint8_t wrong_type[8]{};
    expect(!XInputFeasibility::parse_rumble_report(wrong_type,
                                                   sizeof(wrong_type), &output),
           "wrong rumble report type accepted");
}

void test_host_probe_sequence() {
    AdapterHostProbeState state;
    state.note_ms_compat_id_request(10);
    expect(!state.windows_confirmed(),
           "compatible-ID request without signature confirmed Windows");
    state.note_ms_os_string();
    state.note_ms_compat_id_request(20);
    expect(state.windows_confirmed(),
           "two-stage Windows signature not confirmed");
    expect(!state.should_reboot(119), "probe rebooted before delay");
    expect(state.should_reboot(120), "probe did not reboot at deadline");

    AdapterHostProbeState wrapped;
    wrapped.note_ms_os_string();
    wrapped.note_ms_compat_id_request(UINT32_MAX - 50);
    expect(!wrapped.should_reboot(48),
           "wrapped timer rebooted before deadline");
    expect(wrapped.should_reboot(49), "wrapped timer missed deadline");
}

} // namespace

int main() {
    test_device_and_configuration_descriptors();
    test_microsoft_compatible_id_descriptor();
    test_input_report_mapping();
    test_rumble_report();
    test_host_probe_sequence();
    return failures == 0 ? 0 : 1;
}
