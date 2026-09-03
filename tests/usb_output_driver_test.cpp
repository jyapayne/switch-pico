#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "adapter_host_probe_state.h"
#include "xinput_descriptors.h"
#include "xinput_protocol.h"
#include "device/usbd_pvt.h"
#include "pico/time.h"
#include "switch_pro_driver.h"
#include "tusb.h"
#include "usb_output_driver.h"
#include "xinput_driver.h"

namespace {

int failures = 0;

constexpr uint8_t kInstanceCount = SWITCH_PICO_HID_INSTANCE_COUNT;
constexpr uint8_t kInvalidInstance = kInstanceCount;

struct EndpointHarness {
    bool opened = false;
    bool busy = false;
    bool claimed = false;
    uint8_t* armed_buffer = nullptr;
    uint16_t armed_length = 0;
    std::array<uint8_t, 64> last_transfer{};
    uint16_t last_transfer_length = 0;
    unsigned transfer_count = 0;
};

struct RumbleEvent {
    unsigned count = 0;
    ControllerRumbleOutput output{};
};

std::array<EndpointHarness, 256> endpoint_harness{};
std::array<RumbleEvent, kInstanceCount> xinput_rumble_events{};
uint64_t now_ms = 0;
uint32_t random_value = 1;
bool usb_ready = true;
unsigned hid_report_count = 0;
unsigned switch_inactive_rumble_count = 0;
unsigned observed_string_count = 0;
uint8_t last_observed_string = 0;

#ifdef SWITCH_PICO_BLUEPAD32
bool management_vendor_control_result = false;
unsigned management_vendor_control_count = 0;
uint8_t management_vendor_control_rhport = 0;
uint8_t management_vendor_control_stage = 0;
const tusb_control_request_t* management_vendor_control_request = nullptr;
#endif

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
    using namespace XInput;
    expect(read_le16(&kSwitchProbeDeviceDescriptor[8]) ==
               kSwitchProbeVendorId,
           "Switch probe VID mismatch");
    expect(read_le16(&kSwitchProbeDeviceDescriptor[10]) ==
               kSwitchProbeProductId,
           "Switch probe PID mismatch");
    expect(read_le16(&kSwitchProbeDeviceDescriptor[12]) ==
               kSwitchProbeDeviceRevision,
           "Switch probe revision mismatch");
    expect(kSwitchProbeDeviceRevision != 0x0210,
           "Switch probe reuses the genuine controller cache identity");
    expect(read_le16(&kDeviceDescriptor[8]) == kDevelopmentVendorId,
           "development VID mismatch");
    expect(read_le16(&kDeviceDescriptor[10]) == kDevelopmentProductId,
           "development PID mismatch");
    expect(read_le16(&kDeviceDescriptor[12]) ==
               kDevelopmentDeviceRevision,
           "development revision mismatch");
    expect(kDeviceDescriptor[4] == 0 && kDeviceDescriptor[5] == 0 &&
               kDeviceDescriptor[6] == 0,
           "multi-interface development device is not composite");
    expect(kDevelopmentVendorId != 0x045e,
           "development device must not impersonate Microsoft's VID");
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
    using namespace XInput;
    expect(read_le32(kMsCompatIdDescriptor) == sizeof(kMsCompatIdDescriptor),
           "Microsoft descriptor total length mismatch");
    expect(read_le16(&kMsCompatIdDescriptor[4]) == 0x0100,
           "Microsoft descriptor version mismatch");
    expect(read_le16(&kMsCompatIdDescriptor[6]) == kMsCompatIdIndex,
           "Microsoft descriptor index mismatch");
    expect(kMsCompatIdDescriptor[8] == SWITCH_PICO_HID_INSTANCE_COUNT,
           "Microsoft function count mismatch");
    for (uint8_t instance = 0;
         instance < SWITCH_PICO_HID_INSTANCE_COUNT; ++instance) {
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
    ControllerState state{};
    auto report = XInput::build_input_report(state);
    expect(report.report_id == 0 && report.report_size == 20,
           "neutral report header mismatch");
    expect(report.buttons == 0 && report.left_trigger == 0 &&
               report.right_trigger == 0,
           "neutral report controls mismatch");
    expect(report.left_x == 0 && report.left_y == 0 && report.right_x == 0 &&
               report.right_y == 0,
           "neutral axes mismatch");

    state.dpad_up = true;
    state.button_south = true;
    state.button_east = true;
    state.button_west = true;
    state.button_north = true;
    state.button_start = true;
    state.button_select = true;
    state.button_system = true;
    state.left_trigger = UINT16_MAX;
    state.right_trigger = UINT16_MAX;
    state.left_stick_x = INT16_MIN;
    state.left_stick_y = INT16_MIN;
    state.right_stick_x = INT16_MAX;
    state.right_stick_y = INT16_MAX;
    report = XInput::build_input_report(state);
    expect((report.buttons & XInput::kDpadUp) != 0,
           "D-pad mapping missing");
    expect((report.buttons & XInput::kButtonA) != 0 &&
               (report.buttons & XInput::kButtonB) != 0 &&
               (report.buttons & XInput::kButtonX) != 0 &&
               (report.buttons & XInput::kButtonY) != 0,
           "positional face-button mapping mismatch");
    expect(report.left_trigger == 0xff && report.right_trigger == 0xff,
           "full analog trigger mapping mismatch");
    expect(report.left_x == INT16_MIN && report.left_y == INT16_MAX &&
               report.right_x == INT16_MAX &&
               report.right_y == -INT16_MAX,
           "axis endpoint mapping mismatch");

    state.left_trigger = 0x8000;
    state.right_trigger = 0x7fff;
    report = XInput::build_input_report(state);
    expect(report.left_trigger == 0x80 && report.right_trigger == 0x7f,
           "analog trigger precision was discarded");
}

void test_rumble_report() {
    const uint8_t packet[8] = {0x00, 0x08, 0x00, 0xa5, 0x5a, 0x00, 0x00, 0x00};
    ControllerRumbleOutput output{};
    expect(XInput::parse_rumble_report(packet, sizeof(packet), &output),
           "valid rumble report rejected");
    expect(output.low_frequency_magnitude == 0xa5 &&
               output.high_frequency_magnitude == 0x5a,
           "rumble magnitudes mapped incorrectly");
    expect(!XInput::parse_rumble_report(packet, 4, &output),
           "truncated rumble report accepted");
    uint8_t wrong_type[8]{};
    expect(!XInput::parse_rumble_report(wrong_type, sizeof(wrong_type),
                                        &output),
           "wrong rumble report type accepted");
}

void test_host_probe_sequence() {
    AdapterHostProbeState state;
    state.note_ms_compat_id_request(10);
    expect(!state.windows_confirmed(),
           "compatible-ID request without signature confirmed Windows");
    state.note_ms_os_string();
    expect(state.saw_ms_os_string(),
           "Microsoft OS string observation was not retained");
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

void reset_usb_harness() {
    endpoint_harness = {};
    xinput_rumble_events = {};
    now_ms = 0;
    usb_ready = true;
    hid_report_count = 0;
    switch_inactive_rumble_count = 0;
    observed_string_count = 0;
    last_observed_string = 0;
}

void xinput_rumble_callback(uint8_t instance,
                            const ControllerRumbleOutput& output) {
    expect(instance < xinput_rumble_events.size(),
           "XInput rumble used an invalid instance");
    if (instance < xinput_rumble_events.size()) {
        ++xinput_rumble_events[instance].count;
        xinput_rumble_events[instance].output = output;
    }
}

void inactive_switch_rumble_callback(
    uint8_t instance, const ControllerRumbleOutput& output) {
    (void)instance;
    (void)output;
    ++switch_inactive_rumble_count;
}

void expect_usb_string(uint8_t index, const char* expected,
                       const char* message) {
    const uint16_t* descriptor = tud_descriptor_string_cb(index, 0x0409);
    const size_t length = std::strlen(expected);
    bool matches = descriptor != nullptr &&
                   (descriptor[0] & 0xffu) == 2u * length + 2u &&
                   (descriptor[0] >> 8u) == TUSB_DESC_STRING;
    if (matches) {
        for (size_t i = 0; i < length; ++i) {
            if (descriptor[i + 1] !=
                static_cast<uint8_t>(expected[i])) {
                matches = false;
                break;
            }
        }
    }
    expect(matches, message);
}

void test_switch_boundary_dispatch() {

    reset_usb_harness();
    usb_output_driver_init(AdapterUsbMode::kSwitchProbe);
    expect(usb_output_driver_mode() == AdapterUsbMode::kSwitchProbe &&
               std::strcmp(usb_output_driver_name(), "SWITCH") == 0 &&
               std::strcmp(usb_output_driver_mode_name(),
                           "Switch probe") == 0,
           "Switch boundary mode was not frozen");
    expect(std::memcmp(tud_descriptor_device_cb(),
                       XInput::kSwitchProbeDeviceDescriptor,
                       sizeof(XInput::kSwitchProbeDeviceDescriptor)) == 0,
           "Switch probe device descriptor changed at the boundary");
    expect(std::memcmp(tud_descriptor_configuration_cb(0),
                       switch_pro_configuration_descriptor,
                       sizeof(switch_pro_configuration_descriptor)) == 0,
           "Switch configuration descriptor changed at the boundary");
    const uint8_t* hid_descriptor = tud_hid_descriptor_report_cb(0);
    expect(hid_descriptor != nullptr &&
               std::memcmp(hid_descriptor, switch_pro_report_descriptor,
                           sizeof(switch_pro_report_descriptor)) == 0,
           "Switch HID report descriptor changed at the boundary");
    expect(tud_hid_descriptor_report_cb(kInvalidInstance) == nullptr,
           "Switch HID callback accepted an invalid instance");

    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> report{};
    expect(tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT,
                                 report.data(), report.size()) ==
               sizeof(SwitchProReport),
           "Switch GET_REPORT was not dispatched");
    expect(tud_hid_get_report_cb(kInvalidInstance, 0,
                                 HID_REPORT_TYPE_INPUT, report.data(),
                                 report.size()) == 0,
           "Switch GET_REPORT accepted an invalid instance");

    uint8_t driver_count = 0xff;
    expect(usbd_app_driver_get_cb(&driver_count) == nullptr &&
               driver_count == 0,
           "Switch mode registered the XInput custom class");
    expect(usbd_app_driver_get_cb(nullptr) == nullptr,
           "custom class callback accepted a null count");

    ControllerState switch_state{};
    switch_state.button_south = true;
    usb_output_driver_set_input(0, switch_state,
                                SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                                SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 15;
    expect(usb_output_driver_task(0) && hid_report_count == 1,
           "Switch input task was not dispatched");
    expect(tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT,
                                 report.data(), report.size()) ==
               sizeof(SwitchProReport),
           "dispatched Switch report was unavailable");
    SwitchProReport switch_report{};
    std::memcpy(&switch_report, report.data(), sizeof(switch_report));
    expect(switch_report.inputs.buttonB,
           "Switch boundary did not apply input state");
    hid_report_count = 0;

    expect(usb_output_driver_is_ready(0),
           "Switch context was not ready after initialization");
    tud_mount_cb();
    expect(!usb_output_driver_is_ready(0),
           "Switch mount did not reset handshake readiness");
    expect(!usb_output_driver_task(0) && hid_report_count == 1,
           "Switch startup identify was not dispatched");
    tud_umount_cb();
    expect(!usb_output_driver_is_ready(0),
           "Switch unmount did not reset handshake readiness");
    expect(!usb_output_driver_task(kInvalidInstance),
           "Switch task accepted an invalid instance");

    const uint16_t* os_string = tud_descriptor_string_cb(0xee, 0x0409);
    expect(os_string != nullptr && os_string[0] == 0x0312 &&
               os_string[1] == 'M' && os_string[2] == 'S' &&
               os_string[3] == 'F' && os_string[4] == 'T' &&
               os_string[5] == '1' && os_string[6] == '0' &&
               os_string[7] == '0' &&
               os_string[8] == XInput::kMsVendorRequest,
           "Microsoft OS string changed at the boundary");
    expect(observed_string_count == 1 && last_observed_string == 0xee,
           "host probe did not observe the Microsoft OS string");
    expect(!tud_control_request_cb(0, nullptr),
           "generic control routing claimed an unhandled request");
}
void test_manual_switch_selection() {
    reset_usb_harness();
    usb_output_driver_init(AdapterUsbMode::kSwitch);
    expect(usb_output_driver_mode() == AdapterUsbMode::kSwitch &&
               std::strcmp(usb_output_driver_mode_name(), "Switch") == 0,
           "manual Switch mode was not frozen separately from probe mode");
    expect(std::memcmp(tud_descriptor_device_cb(),
                       switch_pro_device_descriptor,
                       sizeof(switch_pro_device_descriptor)) == 0,
           "manual Switch did not use the production Switch descriptor");
    expect(tud_descriptor_string_cb(0xee, 0x0409) == nullptr,
           "manual Switch exposed the automatic Windows probe string");
}

void open_xinput_interfaces(usbd_class_driver_t const* driver) {
    endpoint_harness = {};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        auto const* interface_descriptor =
            reinterpret_cast<tusb_desc_interface_t const*>(
                &XInput::kConfigurationDescriptor[
                    9 + instance * XInput::kInterfaceDescriptorSize]);
        expect(driver->open(0, interface_descriptor,
                            XInput::kInterfaceDescriptorSize) ==
                   XInput::kInterfaceDescriptorSize,
               "XInput interface did not open");
        expect(endpoint_harness[static_cast<uint8_t>(0x81 + instance)]
                   .opened &&
                   endpoint_harness[static_cast<uint8_t>(0x01 + instance)]
                       .opened,
               "XInput interface endpoints were not opened");
    }
}

void test_xinput_boundary_dispatch() {
    reset_usb_harness();
    usb_output_driver_init(AdapterUsbMode::kXInput);

    expect(usb_output_driver_mode() == AdapterUsbMode::kXInput &&
               std::strcmp(usb_output_driver_name(), "XINPUT") == 0 &&
               std::strcmp(usb_output_driver_mode_name(), "XInput") == 0,
           "XInput boundary mode was not frozen");
    expect(std::memcmp(tud_descriptor_device_cb(),
                       XInput::kDeviceDescriptor,
                       sizeof(XInput::kDeviceDescriptor)) == 0,
           "XInput device descriptor changed at the boundary");
    expect(std::memcmp(tud_descriptor_configuration_cb(0),
                       XInput::kConfigurationDescriptor,
                       sizeof(XInput::kConfigurationDescriptor)) == 0,
           "XInput configuration descriptor changed at the boundary");
    expect(tud_hid_descriptor_report_cb(0) == nullptr,
           "inactive HID class claimed an XInput report descriptor");
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> hid_report{};
    expect(tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT,
                                 hid_report.data(), hid_report.size()) == 0,
           "inactive HID GET_REPORT handled XInput mode");

    uint8_t driver_count = 0;
    usbd_class_driver_t const* driver =
        usbd_app_driver_get_cb(&driver_count);
    expect(driver == xinput_class_driver() && driver_count == 1,
           "XInput custom class was not selected");
    expect(driver != nullptr && std::strcmp(driver->name, "XINPUT") == 0,
           "XInput custom class retained a feasibility name");
    if (driver == nullptr) {
        return;
    }
    driver->init();
    open_xinput_interfaces(driver);

    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        usb_output_driver_set_rumble_callback(
            instance, xinput_rumble_callback);
        ControllerState state{};
        state.button_south = (instance & 1u) == 0;
        state.button_north = (instance & 1u) != 0;
        state.left_trigger =
            static_cast<uint16_t>(0x1000u * (instance + 1u));
        state.right_trigger =
            static_cast<uint16_t>(0x0800u * (instance + 1u));
        state.left_stick_x = static_cast<int16_t>(100 + instance);
        state.right_stick_y = static_cast<int16_t>(-200 - instance);
        usb_output_driver_set_input(instance, state, 1, 2);
        expect(usb_output_driver_is_ready(instance),
               "configured XInput instance was not ready");
        expect(usb_output_driver_task(instance),
               "XInput input report was not dispatched");

        const EndpointHarness& input_endpoint =
            endpoint_harness[static_cast<uint8_t>(0x81 + instance)];
        const XInput::InputReport expected =
            XInput::build_input_report(state);
        expect(input_endpoint.last_transfer_length == sizeof(expected) &&
                   std::memcmp(input_endpoint.last_transfer.data(),
                               &expected, sizeof(expected)) == 0,
               "XInput boundary changed an input report");
    }
    expect(!usb_output_driver_task(kInvalidInstance),
           "XInput task accepted an invalid instance");

    switch_pro_set_rumble_callback(0,
                                   inactive_switch_rumble_callback);
    std::array<uint8_t, 10> switch_output{};
    switch_output[0] = REPORT_OUTPUT_10;
    tud_hid_report_received_cb(0, 0, switch_output.data(),
                               switch_output.size());
    tud_hid_set_report_cb(0, REPORT_OUTPUT_10, HID_REPORT_TYPE_OUTPUT,
                          switch_output.data() + 1,
                          switch_output.size() - 1);
    expect(switch_inactive_rumble_count == 0,
           "inactive Switch HID callbacks handled XInput output");

    tud_mount_cb();
    tud_umount_cb();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(usb_output_driver_is_ready(instance),
               "generic mount callback reset the active XInput class");
    }

    driver->reset(0);
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(!usb_output_driver_is_ready(instance),
               "XInput bus reset retained configured state");
    }
    open_xinput_interfaces(driver);

    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        EndpointHarness& output_endpoint =
            endpoint_harness[static_cast<uint8_t>(0x01 + instance)];
        expect(output_endpoint.armed_buffer != nullptr &&
                   output_endpoint.armed_length == 32,
               "XInput output endpoint was not armed");
        if (output_endpoint.armed_buffer == nullptr) {
            continue;
        }
        std::memset(output_endpoint.armed_buffer, 0,
                    output_endpoint.armed_length);
        output_endpoint.armed_buffer[0] = 0x00;
        output_endpoint.armed_buffer[1] = 0x08;
        output_endpoint.armed_buffer[3] =
            static_cast<uint8_t>(0x20 + instance);
        output_endpoint.armed_buffer[4] =
            static_cast<uint8_t>(0x40 + instance);
        expect(driver->xfer_cb(
                   0, static_cast<uint8_t>(0x01 + instance),
                   XFER_RESULT_SUCCESS, 8),
               "XInput output transfer was not rearmed");
    }
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(xinput_rumble_events[instance].count == 1 &&
                   xinput_rumble_events[instance]
                           .output.low_frequency_magnitude ==
                       static_cast<uint8_t>(0x20 + instance) &&
                   xinput_rumble_events[instance]
                           .output.high_frequency_magnitude ==
                       static_cast<uint8_t>(0x40 + instance),
               "XInput rumble crossed instance boundaries");
    }

    expect_usb_string(1, "Switch Pico",
                      "XInput manufacturer string changed");
    expect_usb_string(2, "XInput Feasibility",
                      "XInput development product string changed");
    expect_usb_string(3, "XINPUT-PROTOTYPE",
                      "XInput development serial string changed");
}

void test_vendor_control_boundary() {
    constexpr uint8_t kSetupStage = 0;
    constexpr uint8_t kRhport = 2;
    tusb_control_request_t request{};
    request.bRequest = 0x42;

#ifdef SWITCH_PICO_BLUEPAD32
    management_vendor_control_result = true;
    management_vendor_control_count = 0;
    expect(tud_vendor_control_xfer_cb(kRhport, kSetupStage, &request),
           "BLUEPAD32 vendor control was not forwarded");
    expect(management_vendor_control_count == 1 &&
               management_vendor_control_rhport == kRhport &&
               management_vendor_control_stage == kSetupStage &&
               management_vendor_control_request == &request,
           "BLUEPAD32 vendor control forwarding changed its arguments");

    management_vendor_control_result = false;
    expect(!tud_vendor_control_xfer_cb(kRhport, kSetupStage, &request) &&
               management_vendor_control_count == 2,
           "inactive BLUEPAD32 vendor control was claimed");
#else
    expect(!tud_vendor_control_xfer_cb(kRhport, kSetupStage, &request),
           "UART vendor control was claimed");
#endif
}

} // namespace

extern "C" absolute_time_t get_absolute_time(void) {
    return {now_ms};
}

extern "C" uint32_t to_ms_since_boot(absolute_time_t time) {
    return static_cast<uint32_t>(time.milliseconds);
}

extern "C" uint32_t get_rand_32(void) {
    return random_value++;
}

extern "C" bool tud_hid_n_ready(uint8_t instance) {
    return instance < kInstanceCount;
}

extern "C" bool tud_hid_n_report(uint8_t instance, uint8_t report_id,
                                  const void* report, uint16_t length) {
    (void)report_id;
    if (instance >= kInstanceCount || report == nullptr || length == 0) {
        return false;
    }
    ++hid_report_count;
    return true;
}

extern "C" bool tud_suspended(void) {
    return false;
}

extern "C" bool tud_remote_wakeup(void) {
    return true;
}

extern "C" bool tud_ready(void) {
    return usb_ready;
}

extern "C" bool usbd_edpt_open(
    uint8_t rhport, tusb_desc_endpoint_t const* endpoint_descriptor) {
    (void)rhport;
    if (endpoint_descriptor == nullptr) {
        return false;
    }
    endpoint_harness[endpoint_descriptor->bEndpointAddress].opened = true;
    return true;
}

extern "C" bool usbd_edpt_xfer(uint8_t rhport, uint8_t endpoint,
                                uint8_t* buffer, uint16_t total_bytes) {
    (void)rhport;
    if (buffer == nullptr) {
        return false;
    }
    EndpointHarness& harness = endpoint_harness[endpoint];
    ++harness.transfer_count;
    harness.claimed = false;
    if (tu_edpt_dir(endpoint) == TUSB_DIR_IN) {
        harness.last_transfer_length =
            total_bytes < harness.last_transfer.size()
                ? total_bytes
                : static_cast<uint16_t>(harness.last_transfer.size());
        std::memcpy(harness.last_transfer.data(), buffer,
                    harness.last_transfer_length);
    } else {
        harness.armed_buffer = buffer;
        harness.armed_length = total_bytes;
    }
    return true;
}

extern "C" bool usbd_edpt_busy(uint8_t rhport, uint8_t endpoint) {
    (void)rhport;
    return endpoint_harness[endpoint].busy;
}

extern "C" bool usbd_edpt_claim(uint8_t rhport, uint8_t endpoint) {
    (void)rhport;
    EndpointHarness& harness = endpoint_harness[endpoint];
    if (harness.claimed) {
        return false;
    }
    harness.claimed = true;
    return true;
}

extern "C" bool usbd_edpt_release(uint8_t rhport, uint8_t endpoint) {
    (void)rhport;
    endpoint_harness[endpoint].claimed = false;
    return true;
}

void adapter_host_probe_note_string_descriptor(uint8_t index) {
    ++observed_string_count;
    last_observed_string = index;
}

#ifdef SWITCH_PICO_BLUEPAD32
bool usb_configuration_management_vendor_control(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
    ++management_vendor_control_count;
    management_vendor_control_rhport = rhport;
    management_vendor_control_stage = stage;
    management_vendor_control_request = request;
    return management_vendor_control_result;
}
#endif

int main() {
    test_device_and_configuration_descriptors();
    test_microsoft_compatible_id_descriptor();
    test_input_report_mapping();
    test_rumble_report();
    test_host_probe_sequence();
    test_switch_boundary_dispatch();
    test_manual_switch_selection();
    test_xinput_boundary_dispatch();
    test_vendor_control_boundary();
    return failures == 0 ? 0 : 1;
}
