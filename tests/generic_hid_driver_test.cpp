#include "usb/generic_hid/generic_hid_driver.h"
#include "usb/generic_hid/generic_hid_descriptors.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

constexpr uint8_t kInstanceCount = SWITCH_PICO_HID_INSTANCE_COUNT;
constexpr uint8_t kInvalidInstance = kInstanceCount;
static_assert(kInstanceCount == 4,
              "the generic HID driver harness must exercise four interfaces");

struct SentReport {
    uint8_t instance = 0xff;
    uint8_t report_id = 0xff;
    uint16_t length = 0;
    GenericHid::InputReport report{};
};

std::array<bool, kInstanceCount> hid_ready{};
std::array<bool, kInstanceCount> send_succeeds{};
std::array<unsigned, kInstanceCount> send_attempts{};
std::array<SentReport, 32> sent_reports{};
size_t sent_report_count = 0;
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

void reset_harness() {
    hid_ready.fill(true);
    send_succeeds.fill(true);
    send_attempts = {};
    sent_reports = {};
    sent_report_count = 0;
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        generic_hid_init(instance);
    }
}

GenericHid::InputReport get_report(uint8_t instance) {
    GenericHid::InputReport report{};
    expect(generic_hid_get_report(instance, 0, HID_REPORT_TYPE_INPUT,
                                  report.data, sizeof(report)) ==
               sizeof(report),
           "valid input GetReport did not return 15 bytes");
    return report;
}

void test_initial_report_and_descriptor_routing() {
    reset_harness();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        const GenericHid::InputReport report = get_report(instance);
        expect(report.data[12] == GenericHid::kHatCenter,
               "initialized generic HID report is not centered");
        for (size_t byte = 0; byte < sizeof(report); ++byte) {
            if (byte != 12) {
                expect(report.data[byte] == 0,
                       "initialized generic HID report is not neutral");
            }
        }
        const uint8_t* dinput_descriptor = generic_hid_report_descriptor(
            instance, GenericHid::ReportDescriptorVariant::kDInput);
        const uint8_t* mac_descriptor = generic_hid_report_descriptor(
            instance, GenericHid::ReportDescriptorVariant::kMac);
        expect(dinput_descriptor != nullptr &&
                   std::memcmp(dinput_descriptor,
                               GenericHid::kDInputReportDescriptor,
                               sizeof(GenericHid::kDInputReportDescriptor)) ==
                       0,
               "valid HID instance did not receive the DInput descriptor");
        expect(mac_descriptor != nullptr &&
                   std::memcmp(mac_descriptor,
                               GenericHid::kMacReportDescriptor,
                               sizeof(GenericHid::kMacReportDescriptor)) == 0,
               "valid HID instance did not receive the Mac descriptor");
    }
    expect(generic_hid_report_descriptor(
               kInvalidInstance,
               GenericHid::ReportDescriptorVariant::kDInput) == nullptr &&
               generic_hid_report_descriptor(
                   kInvalidInstance,
                   GenericHid::ReportDescriptorVariant::kMac) == nullptr,
           "invalid HID instance received a report descriptor");
}

void test_latest_get_report_and_instance_isolation() {
    reset_harness();
    std::array<ControllerState, kInstanceCount> states{};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        ControllerState& state = states[instance];
        state.left_stick_x = static_cast<int16_t>(-30000 + instance * 1000);
        state.left_stick_y = static_cast<int16_t>(1000 + instance * 2000);
        state.right_stick_x = static_cast<int16_t>(3000 + instance * 3000);
        state.right_stick_y = static_cast<int16_t>(-4000 - instance * 4000);
        state.left_trigger = static_cast<uint16_t>(0x1111u * (instance + 1u));
        state.right_trigger = static_cast<uint16_t>(0x8888u + instance * 0x1111u);
        if (instance == 0) {
            state.button_south = true;
            state.dpad_up = true;
        } else if (instance == 1) {
            state.button_east = true;
            state.dpad_right = true;
        } else if (instance == 2) {
            state.button_system = true;
            state.dpad_down = true;
        } else {
            state.button_capture = true;
            state.dpad_left = true;
        }
        generic_hid_set_input(instance, state);
    }

    std::array<GenericHid::InputReport, kInstanceCount> reports{};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        reports[instance] = get_report(instance);
        const GenericHid::InputReport expected =
            GenericHid::build_input_report(states[instance]);
        expect(std::memcmp(reports[instance].data, expected.data,
                           sizeof(expected)) == 0,
               "GetReport did not expose the latest addressed input");
    }
    for (uint8_t left = 0; left < kInstanceCount; ++left) {
        for (uint8_t right = static_cast<uint8_t>(left + 1u);
             right < kInstanceCount; ++right) {
            expect(std::memcmp(reports[left].data, reports[right].data,
                               sizeof(GenericHid::InputReport)) != 0,
                   "latest reports crossed HID instances");
        }
    }

    ControllerState changed = states[2];
    changed.button_system = false;
    changed.button_north = true;
    changed.left_trigger = UINT16_MAX;
    generic_hid_set_input(2, changed);
    const GenericHid::InputReport latest = get_report(2);
    const GenericHid::InputReport unchanged = get_report(1);
    expect(read_u16(latest.data + 13) == GenericHid::kButtonNorth &&
               read_u16(latest.data + 8) == UINT16_MAX,
           "changed input was not materialized before the next task call");
    expect(std::memcmp(unchanged.data, reports[1].data, sizeof(unchanged)) == 0,
           "changing one input mutated another HID context");
}

void test_ready_and_periodic_send_routing() {
    reset_harness();
    ControllerState state{};
    state.left_stick_x = INT16_MIN;
    state.right_stick_y = INT16_MAX;
    state.left_trigger = 0x1234;
    state.right_trigger = 0xabcd;
    state.button_left_stick = true;
    state.dpad_down = true;
    state.dpad_right = true;
    generic_hid_set_input(3, state);

    hid_ready[3] = false;
    expect(!generic_hid_is_ready(3) && !generic_hid_task(3) &&
               send_attempts[3] == 0 && sent_report_count == 0,
           "not-ready HID instance attempted an interrupt send");
    hid_ready[3] = true;
    expect(generic_hid_is_ready(3) && generic_hid_task(3) &&
               send_attempts[3] == 1 && sent_report_count == 1,
           "ready HID instance did not send its periodic input report");
    expect(sent_reports[0].instance == 3 &&
               sent_reports[0].report_id == 0 &&
               sent_reports[0].length == GenericHid::kReportSize,
           "generic input used the wrong TinyUSB interface, ID, or length");
    const GenericHid::InputReport expected =
        GenericHid::build_input_report(state);
    expect(std::memcmp(sent_reports[0].report.data, expected.data,
                       sizeof(expected)) == 0,
           "interrupt send did not use the latest generic input report");

    expect(generic_hid_task(3) && send_attempts[3] == 2 &&
               sent_report_count == 2,
           "ready polling did not permit the next report interval send");

    send_succeeds[3] = false;
    expect(!generic_hid_task(3) && send_attempts[3] == 3 &&
               sent_report_count == 2,
           "failed TinyUSB send was reported as successful");
    expect(!generic_hid_is_ready(kInvalidInstance) &&
               !generic_hid_task(kInvalidInstance),
           "invalid HID instance was ready or attempted a send");
    for (uint8_t instance = 0; instance < 3; ++instance) {
        expect(send_attempts[instance] == 0,
               "task send leaked to a different HID interface");
    }
}

void test_reset_and_invalid_inputs() {
    reset_harness();
    ControllerState zero{};
    zero.button_south = true;
    zero.left_stick_x = 1234;
    ControllerState one{};
    one.button_east = true;
    one.right_trigger = 4321;
    generic_hid_set_input(0, zero);
    generic_hid_set_input(1, one);

    generic_hid_init(0);
    const GenericHid::InputReport reset = get_report(0);
    const GenericHid::InputReport preserved = get_report(1);
    expect(reset.data[12] == GenericHid::kHatCenter &&
               read_u16(reset.data + 13) == 0 &&
               read_u16(reset.data) == 0,
           "init did not reset the addressed HID context");
    expect(read_u16(preserved.data + 13) == GenericHid::kButtonEast &&
               read_u16(preserved.data + 10) == 4321,
           "reset crossed into another HID context");

    ControllerState invalid{};
    invalid.button_capture = true;
    generic_hid_set_input(kInvalidInstance, invalid);
    generic_hid_init(kInvalidInstance);
    expect(std::memcmp(get_report(1).data, preserved.data,
                       sizeof(preserved)) == 0,
           "invalid instance input/reset mutated a valid context");
}

void test_get_report_rejections_and_truncation() {
    reset_harness();
    ControllerState state{};
    state.left_stick_x = static_cast<int16_t>(0x1234);
    state.left_stick_y = static_cast<int16_t>(0x5678);
    generic_hid_set_input(0, state);

    std::array<uint8_t, GenericHid::kReportSize> buffer{};
    buffer.fill(0xa5);
    expect(generic_hid_get_report(kInvalidInstance, 0,
                                  HID_REPORT_TYPE_INPUT, buffer.data(),
                                  buffer.size()) == 0,
           "GetReport accepted an invalid HID instance");
    expect(generic_hid_get_report(0, 1, HID_REPORT_TYPE_INPUT, buffer.data(),
                                  buffer.size()) == 0,
           "GetReport accepted a nonzero Report ID");
    expect(generic_hid_get_report(0, 0, HID_REPORT_TYPE_OUTPUT, buffer.data(),
                                  buffer.size()) == 0 &&
               generic_hid_get_report(0, 0, HID_REPORT_TYPE_FEATURE,
                                      buffer.data(), buffer.size()) == 0,
           "input-only driver accepted output or feature GetReport");
    expect(generic_hid_get_report(0, 0, HID_REPORT_TYPE_INPUT, nullptr,
                                  buffer.size()) == 0 &&
               generic_hid_get_report(0, 0, HID_REPORT_TYPE_INPUT,
                                      buffer.data(), 0) == 0,
           "GetReport accepted an invalid destination");
    for (uint8_t byte : buffer) {
        expect(byte == 0xa5, "rejected GetReport modified its destination");
    }

    std::array<uint8_t, 3> truncated{};
    expect(generic_hid_get_report(0, 0, HID_REPORT_TYPE_INPUT,
                                  truncated.data(), truncated.size()) ==
               truncated.size() &&
               truncated[0] == 0x34 && truncated[1] == 0x12 &&
               truncated[2] == 0x78,
           "GetReport did not safely truncate the latest input report");
}

}  // namespace

extern "C" bool tud_hid_n_ready(uint8_t instance) {
    return instance < kInstanceCount && hid_ready[instance];
}

extern "C" bool tud_hid_n_report(uint8_t instance, uint8_t report_id,
                                  const void* report, uint16_t length) {
    if (instance >= kInstanceCount || report == nullptr ||
        length != sizeof(GenericHid::InputReport)) {
        return false;
    }
    ++send_attempts[instance];
    if (!send_succeeds[instance] || sent_report_count >= sent_reports.size()) {
        return false;
    }
    SentReport& sent = sent_reports[sent_report_count++];
    sent.instance = instance;
    sent.report_id = report_id;
    sent.length = length;
    std::memcpy(&sent.report, report, length);
    return true;
}

int main() {
    test_initial_report_and_descriptor_routing();
    test_latest_get_report_and_instance_isolation();
    test_ready_and_periodic_send_routing();
    test_reset_and_invalid_inputs();
    test_get_report_rejections_and_truncation();
    if (failures != 0) {
        std::cerr << failures << " generic HID driver test(s) failed\n";
        return 1;
    }
    return 0;
}
