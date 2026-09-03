#include "usb/switch/switch_pro_driver.h"
#include "usb/usb_output_driver.h"
#include "platform/pico/controller_color_config.h"
#include "tusb.h"
#include "pico/time.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {
constexpr uint8_t kInstanceCount = SWITCH_PICO_HID_INSTANCE_COUNT;
constexpr uint8_t kInvalidInstance = kInstanceCount;
static_assert(kInstanceCount == 4,
              "the native driver harness must exercise four HID instances");


struct SentReport {
    uint8_t instance = 0;
    uint8_t report_id = 0;
    uint16_t length = 0;
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> data{};
};

struct RumbleEvent {
    unsigned count = 0;
    uint8_t instance = 0xff;
    ControllerRumbleOutput output{};
};

uint64_t now_ms = 0;
uint32_t random_value = 1;
std::array<bool, kInstanceCount> hid_ready{};
std::array<bool, kInstanceCount> hid_report_succeeds{};
std::array<unsigned, kInstanceCount> hid_report_attempts{};
std::array<SentReport, 32> sent_reports{};
unsigned sent_report_count = 0;
std::array<RumbleEvent, kInstanceCount> rumble_events{};
int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        ++failures;
    }
}

void clear_sent_reports() {
    sent_reports = {};
    sent_report_count = 0;
}

void initialize_contexts() {
    now_ms = 0;
    hid_report_attempts = {};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        hid_ready[instance] = true;
        hid_report_succeeds[instance] = true;
    }
    usb_output_driver_init(AdapterUsbMode::kSwitchProbe);
    clear_sent_reports();
}

const SentReport* latest_regular_report(uint8_t instance) {
    for (unsigned i = sent_report_count; i > 0; --i) {
        const SentReport& report = sent_reports[i - 1];
        if (report.instance == instance &&
            report.length == sizeof(SwitchProReport) &&
            report.data[0] == 0x30) {
            return &report;
        }
    }
    return nullptr;
}

SwitchProReport copy_switch_report(const SentReport* sent) {
    SwitchProReport report{};
    if (sent != nullptr) {
        std::memcpy(&report, sent->data.data(), sizeof(report));
    }
    return report;
}
SwitchProReport get_current_report(uint8_t instance,
                                   const char* length_failure) {
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> data{};
    expect(tud_hid_get_report_cb(instance, 0, HID_REPORT_TYPE_INPUT,
                                 data.data(), data.size()) ==
               sizeof(SwitchProReport),
           length_failure);
    SwitchProReport report{};
    std::memcpy(&report, data.data(), sizeof(report));
    return report;
}

void expect_neutral_sticks(SwitchProReport& report,
                           const char* state_failure) {
    constexpr uint16_t packed_mid =
        CONTROLLER_AXIS_UNSIGNED_CENTER >> 4u;
    constexpr uint16_t packed_inverted_mid =
        static_cast<uint16_t>(-static_cast<int32_t>(packed_mid)) & 0x0fffu;
    expect(report.inputs.leftStick.getX() == packed_mid &&
               report.inputs.leftStick.getY() == packed_inverted_mid &&
               report.inputs.rightStick.getX() == packed_mid &&
               report.inputs.rightStick.getY() == packed_inverted_mid,
           state_failure);
}


unsigned reports_for_instance(uint8_t instance) {
    unsigned count = 0;
    for (unsigned i = 0; i < sent_report_count; ++i) {
        if (sent_reports[i].instance == instance) {
            ++count;
        }
    }
    return count;
}

uint32_t read_bits_le(const uint8_t* bytes, uint16_t bit_offset,
                      uint8_t width) {
    uint32_t value = 0;
    for (uint8_t bit = 0; bit < width; ++bit) {
        uint16_t source_bit = static_cast<uint16_t>(bit_offset + bit);
        if ((bytes[source_bit >> 3] & (1u << (source_bit & 7u))) != 0) {
            value |= 1u << bit;
        }
    }
    return value;
}

int16_t read_int16_le(const uint8_t* bytes) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(bytes[0]) |
        (static_cast<uint16_t>(bytes[1]) << 8u));
}

void send_feature(uint8_t instance, uint8_t command, uint8_t value) {
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> report{};
    report[0] = REPORT_FEATURE;
    report[10] = command;
    report[11] = value;
    tud_hid_report_received_cb(instance, 0, report.data(), report.size());
}
void send_spi_read(uint8_t instance, uint32_t address, uint8_t size) {
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> report{};
    report[0] = REPORT_FEATURE;
    report[10] = SPI_READ;
    report[11] = static_cast<uint8_t>(address);
    report[12] = static_cast<uint8_t>(address >> 8u);
    report[13] = static_cast<uint8_t>(address >> 16u);
    report[14] = static_cast<uint8_t>(address >> 24u);
    report[15] = size;
    tud_hid_report_received_cb(instance, 0, report.data(), report.size());
}


void send_config(uint8_t instance, uint8_t subtype) {
    const uint8_t report[] = {REPORT_CONFIGURATION, subtype};
    tud_hid_report_received_cb(instance, 0, report, sizeof(report));
}

uint32_t type_2(uint8_t high_frequency, uint8_t high_amplitude,
                uint8_t low_frequency, uint8_t low_amplitude) {
    return (1u << 30u) |
           ((static_cast<uint32_t>(low_amplitude) & 0x7fu) << 23u) |
           ((static_cast<uint32_t>(low_frequency) & 0x7fu) << 16u) |
           ((static_cast<uint32_t>(high_amplitude) & 0x7fu) << 9u) |
           ((static_cast<uint32_t>(high_frequency) & 0x7fu) << 2u);
}

uint32_t type_1_one_sample(uint8_t high_command, uint8_t low_command) {
    return (1u << 30u) |
           ((static_cast<uint32_t>(low_command) & 0x1fu) << 25u) |
           ((static_cast<uint32_t>(high_command) & 0x1fu) << 20u);
}

std::array<uint8_t, 8> rumble_payload(uint32_t left, uint32_t right) {
    std::array<uint8_t, 8> payload{};
    const uint32_t words[] = {left, right};
    for (unsigned actuator = 0; actuator < 2; ++actuator) {
        unsigned offset = actuator * 4u;
        payload[offset] = static_cast<uint8_t>(words[actuator]);
        payload[offset + 1] = static_cast<uint8_t>(words[actuator] >> 8u);
        payload[offset + 2] = static_cast<uint8_t>(words[actuator] >> 16u);
        payload[offset + 3] = static_cast<uint8_t>(words[actuator] >> 24u);
    }
    return payload;
}

std::array<uint8_t, 10> complete_rumble_report(
    const std::array<uint8_t, 8>& payload) {
    std::array<uint8_t, 10> report{};
    report[0] = REPORT_OUTPUT_10;
    std::memcpy(report.data() + 2, payload.data(), payload.size());
    return report;
}

void rumble_callback(uint8_t instance, const ControllerRumbleOutput& output) {
    expect(instance < rumble_events.size(),
           "rumble callback received an invalid instance");
    if (instance >= rumble_events.size()) {
        return;
    }
    RumbleEvent& event = rumble_events[instance];
    ++event.count;
    event.instance = instance;
    event.output = output;
}

void test_reset_materializes_neutral_sticks() {
    initialize_contexts();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        SwitchProReport initialized = get_current_report(
            instance, "GET_REPORT failed immediately after init");
        expect_neutral_sticks(
            initialized, "instance sticks were not neutral after init");
    }

    tud_mount_cb();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        SwitchProReport mounted = get_current_report(
            instance, "GET_REPORT failed immediately after mount");
        expect_neutral_sticks(
            mounted, "instance sticks were not neutral after mount");
    }
}

void test_startup_identify_preserves_first_reply_counter() {
    initialize_contexts();
    tud_mount_cb();

    expect(!switch_pro_task(0), "startup identify counted as regular input");
    expect(sent_report_count == 1 && sent_reports[0].instance == 0 &&
               sent_reports[0].data[0] == REPORT_USB_INPUT_81 &&
               sent_reports[0].data[1] == IDENTIFY,
           "startup identify did not use the addressed raw HID route");

    send_feature(0, GET_CONTROLLER_STATE, 0);
    now_ms = 6;
    expect(!switch_pro_task(0), "first subcommand reply counted as regular input");
    expect(sent_report_count == 2 &&
               sent_reports[1].data[0] == REPORT_OUTPUT_21 &&
               sent_reports[1].data[1] == 0,
           "startup identify consumed the first subcommand reply counter");
}

void test_failed_startup_identify_retries_preserve_counter() {
    initialize_contexts();
    tud_mount_cb();
    hid_report_succeeds[0] = false;

    switch_pro_task(0);
    switch_pro_task(0);
    expect(hid_report_attempts[0] == 2 && reports_for_instance(0) == 0,
           "failed startup identify was not retried");

    hid_report_succeeds[0] = true;
    expect(!switch_pro_task(0), "retried startup identify counted as regular input");
    expect(hid_report_attempts[0] == 3 && reports_for_instance(0) == 1,
           "startup identify did not recover after failed sends");

    send_feature(0, GET_CONTROLLER_STATE, 0);
    now_ms = 6;
    switch_pro_task(0);
    expect(sent_report_count == 2 &&
               sent_reports[1].data[0] == REPORT_OUTPUT_21 &&
               sent_reports[1].data[1] == 0,
           "failed startup identify retries consumed the reply counter");
}

void test_input_reports_and_timers_are_isolated() {
    initialize_contexts();
    std::array<ControllerState, kInstanceCount> states{};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        ControllerState& state = states[instance];
        state.left_stick_x = controller_axis_from_unsigned(
            static_cast<uint16_t>(0x1111u * (instance + 1u)));
        state.left_stick_y = controller_axis_from_unsigned(
            static_cast<uint16_t>(0x2222u + 0x1111u * instance));
        state.right_stick_x = controller_axis_from_unsigned(
            static_cast<uint16_t>(0x5555u + 0x1111u * instance));
        state.right_stick_y = controller_axis_from_unsigned(
            static_cast<uint16_t>(0x8888u + 0x1111u * instance));
    }
    states[0].button_east = true;
    states[1].button_south = true;
    states[2].button_north = true;
    states[3].button_west = true;

    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        switch_pro_set_input(instance, states[instance],
                             SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                             SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    }

    now_ms = 15;
    std::array<SwitchProReport, kInstanceCount> sent{};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(switch_pro_task(instance),
               "configured instance did not send its timed report");
        const SentReport* routed = latest_regular_report(instance);
        expect(routed != nullptr, "input report used the wrong HID route");
        sent[instance] = copy_switch_report(routed);
        expect(sent[instance].inputs.buttonA == (instance == 0) &&
                   sent[instance].inputs.buttonB == (instance == 1) &&
                   sent[instance].inputs.buttonX == (instance == 2) &&
                   sent[instance].inputs.buttonY == (instance == 3),
               "button state crossed HID instances");
    }

    std::array<std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE>, kInstanceCount>
        current{};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(tud_hid_get_report_cb(instance, 0, HID_REPORT_TYPE_INPUT,
                                     current[instance].data(),
                                     current[instance].size()) ==
                   sizeof(SwitchProReport),
               "GET_REPORT rejected a configured instance");
    }
    for (uint8_t left = 0; left < kInstanceCount; ++left) {
        for (uint8_t right = static_cast<uint8_t>(left + 1u);
             right < kInstanceCount; ++right) {
            expect(std::memcmp(current[left].data(), current[right].data(),
                               current[left].size()) != 0,
                   "GET_REPORT returned shared state across HID instances");
        }
    }

    ControllerState changed_zero = states[0];
    changed_zero.button_east = false;
    changed_zero.button_system = true;
    switch_pro_set_input(0, changed_zero,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 30;
    expect(switch_pro_task(0),
           "instance 0 did not apply its changed input state");
    SwitchProReport unchanged_three = get_current_report(
        3, "GET_REPORT failed for instance 3 after instance 0 changed");
    expect(unchanged_three.inputs.buttonY &&
               !unchanged_three.inputs.buttonHome,
           "instance 0 input change leaked into instance 3");

    ControllerState changed_three = states[3];
    changed_three.button_west = false;
    changed_three.button_capture = true;
    switch_pro_set_input(3, changed_three,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 45;
    expect(switch_pro_task(3),
           "instance 3 did not apply its changed input state");
    SwitchProReport unchanged_zero = get_current_report(
        0, "GET_REPORT failed for instance 0 after instance 3 changed");
    expect(unchanged_zero.inputs.buttonHome &&
               !unchanged_zero.inputs.buttonCapture,
           "instance 3 input change leaked into instance 0");
}

void test_callback_send_and_imu_modes_are_isolated() {
    initialize_contexts();
    send_feature(0, TOGGLE_IMU, 1);
    now_ms = 6;
    expect(!switch_pro_task(0), "feature reply was reported as regular input");
    expect(reports_for_instance(0) == 1,
           "feature callback reply did not use instance 0");
    expect(reports_for_instance(1) == 0,
           "feature callback queued a reply on instance 1");

    ControllerState zero{};
    zero.left_stick_x = zero.left_stick_y =
        zero.right_stick_x = zero.right_stick_y = 0;
    zero.motion_sample_count = 1;
    zero.motion_samples[0] = {101, 202, 303, 404, 505, 606};
    ControllerState one = zero;
    one.button_north = true;
    one.motion_samples[0] = {1001, 2002, 3003, 4004, 5005, 6006};
    switch_pro_set_input(0, zero, SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    switch_pro_set_input(1, one, SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 21;
    expect(switch_pro_task(0), "raw-IMU instance did not send input");
    expect(switch_pro_task(1), "off-IMU instance timer did not send input");
    SwitchProReport raw = copy_switch_report(latest_regular_report(0));
    SwitchProReport off = copy_switch_report(latest_regular_report(1));
    expect(read_int16_le(raw.imuData) == 101 &&
               read_int16_le(raw.imuData + 6) == 404,
           "instance 0 raw IMU sample was not preserved");
    std::array<uint8_t, 36> zero_imu{};
    expect(std::memcmp(off.imuData, zero_imu.data(), zero_imu.size()) == 0,
           "instance 0 IMU mode leaked into instance 1");

    initialize_contexts();
    send_feature(0, TOGGLE_IMU, 2);
    send_feature(1, TOGGLE_IMU, 2);
    now_ms = 6;
    switch_pro_task(0);
    switch_pro_task(1);
    ControllerState moving{};
    moving.left_stick_x = moving.left_stick_y =
        moving.right_stick_x = moving.right_stick_y = 0;
    moving.motion_sample_count = 1;
    moving.motion_samples[0] = {100, 200, 300, 20000, 0, 0};
    ControllerState stationary{};
    stationary.left_stick_x = stationary.left_stick_y =
        stationary.right_stick_x = stationary.right_stick_y = 0;
    stationary.motion_sample_count = 1;
    stationary.motion_samples[0] = {1000, 2000, 3000, 0, 0, 0};
    switch_pro_set_input(0, moving, SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    switch_pro_set_input(1, stationary,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 21;
    expect(switch_pro_task(0), "moving quaternion instance did not report");
    expect(switch_pro_task(1), "stationary quaternion timer crossed instances");
    SwitchProReport moving_report =
        copy_switch_report(latest_regular_report(0));
    SwitchProReport stationary_report =
        copy_switch_report(latest_regular_report(1));
    bool moving_component =
        read_bits_le(moving_report.imuData, 52, 21) != 0 ||
        read_bits_le(moving_report.imuData, 73, 21) != 0 ||
        read_bits_le(moving_report.imuData, 94, 2) != 0 ||
        read_bits_le(moving_report.imuData, 144, 19) != 0;
    bool stationary_component =
        read_bits_le(stationary_report.imuData, 52, 21) != 0 ||
        read_bits_le(stationary_report.imuData, 73, 21) != 0 ||
        read_bits_le(stationary_report.imuData, 94, 2) != 0 ||
        read_bits_le(stationary_report.imuData, 144, 19) != 0;
    expect(moving_component, "moving quaternion did not integrate");
    expect(!stationary_component,
           "instance 0 quaternion state leaked into instance 1");
    expect(read_int16_le(stationary_report.imuData) == 2000 &&
               read_int16_le(stationary_report.imuData + 2) == 1000,
           "instance 1 quaternion accelerometer state was overwritten");
}

void test_grip_colors_are_isolated() {
    initialize_contexts();
    constexpr uint32_t grip_address =
        0x6000u + offsetof(SwitchFactoryConfig, leftGripColor);
    constexpr uint8_t grip_bytes =
        sizeof(SwitchColorDefinition) * 2u;
    constexpr SwitchRgbColor calibrated_blue =
        switch_pro_calibrate_light_color({0x00, 0x89, 0xEB});
    constexpr SwitchRgbColor calibrated_gray =
        switch_pro_calibrate_light_color({0x96, 0x96, 0x96});
    static_assert(calibrated_blue.red == 0x00 &&
                  calibrated_blue.green == 0x35 &&
                  calibrated_blue.blue == 0x9D);
    static_assert(calibrated_gray.red == 0x64 &&
                  calibrated_gray.green == 0x64 &&
                  calibrated_gray.blue == 0x64);


    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        send_spi_read(instance, grip_address, grip_bytes);
        now_ms += 6;
        expect(!switch_pro_task(instance),
               "grip color SPI reply counted as regular input");
        expect(sent_report_count == static_cast<unsigned>(instance + 1u),
               "grip color SPI reply was not sent");
        const SentReport& response = sent_reports[sent_report_count - 1u];
        const SwitchRgbColor expected =
            switch_pro_get_slot_color(instance);
        const uint8_t expected_bytes[] = {
            expected.red, expected.green, expected.blue,
            expected.red, expected.green, expected.blue,
        };
        expect(response.instance == instance &&
                   response.data[13] == 0x90 &&
                   response.data[14] == SPI_READ &&
                   std::memcmp(response.data.data() + 20, expected_bytes,
                               sizeof(expected_bytes)) == 0,
               "Switch grip color did not match its HID slot");
        const SwitchRgbColor light =
            switch_pro_get_slot_light_color(instance);
        const SwitchRgbColor calibrated =
            switch_pro_calibrate_light_color(expected);
        expect(light.red == calibrated.red &&
                   light.green == calibrated.green &&
                   light.blue == calibrated.blue,
               "physical controller light was not derived from its grip");
    }

    const SwitchRgbColor invalid_grip =
        switch_pro_get_slot_color(kInvalidInstance);
    const SwitchRgbColor invalid_light =
        switch_pro_get_slot_light_color(kInvalidInstance);
    expect(invalid_grip.red == 0 && invalid_grip.green == 0 &&
               invalid_grip.blue == 0 && invalid_light.red == 0 &&
               invalid_light.green == 0 && invalid_light.blue == 0,
           "invalid HID slot returned a configured color");
}

void test_rumble_callbacks_and_decoders_are_isolated() {
    initialize_contexts();
    rumble_events = {};
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        usb_output_driver_set_rumble_callback(instance, rumble_callback);
    }
    constexpr uint32_t neutral = 0x40400100u;
    auto full_payload = rumble_payload(type_2(64, 16, 64, 16), neutral);
    auto full_report = complete_rumble_report(full_payload);
    tud_hid_report_received_cb(kInvalidInstance, 0, full_report.data(),
                               full_report.size());
    for (const auto& event : rumble_events) {
        expect(event.count == 0,
               "invalid output instance reached a rumble callback");
    }

    std::array<uint8_t, 9> stripped{};
    std::memcpy(stripped.data() + 1, full_payload.data(), full_payload.size());
    tud_hid_set_report_cb(0, REPORT_OUTPUT_10, HID_REPORT_TYPE_OUTPUT,
                          stripped.data(), stripped.size());
    expect(rumble_events[0].count == 1 && rumble_events[0].instance == 0,
           "control output did not route to instance 0 callback");
    expect(rumble_events[0].output.low_frequency_magnitude == 16 &&
               rumble_events[0].output.high_frequency_magnitude == 16,
           "instance 0 full rumble state decoded incorrectly");
    for (uint8_t instance = 1; instance < kInstanceCount; ++instance) {
        expect(rumble_events[instance].count == 0,
               "instance 0 rumble invoked another instance callback");
    }

    auto delta_payload = rumble_payload(type_1_one_sample(17, 20), neutral);
    auto delta_report = complete_rumble_report(delta_payload);
    tud_hid_report_received_cb(1, 0, delta_report.data(), delta_report.size());
    expect(rumble_events[1].count == 1 && rumble_events[1].instance == 1,
           "interrupt output did not route to instance 1 callback");
    expect(rumble_events[1].output.low_frequency_magnitude == 0 &&
               rumble_events[1].output.high_frequency_magnitude == 1,
           "instance 1 decoder inherited instance 0 rumble state");
    tud_hid_report_received_cb(0, 0, delta_report.data(), delta_report.size());
    expect(rumble_events[0].count == 2 &&
               rumble_events[0].output.low_frequency_magnitude == 17 &&
               rumble_events[0].output.high_frequency_magnitude == 18,
           "instance 0 decoder lost its own prior rumble state");

    for (uint8_t instance = 2; instance < kInstanceCount; ++instance) {
        const uint8_t magnitude = instance == 2 ? 16 : 32;
        auto payload =
            rumble_payload(type_2(64, magnitude, 64, magnitude), neutral);
        auto report = complete_rumble_report(payload);
        tud_hid_report_received_cb(instance, 0, report.data(), report.size());
        expect(rumble_events[instance].count == 1 &&
                   rumble_events[instance].instance == instance,
               "rumble output did not route to its configured instance");
        expect(rumble_events[instance].output.low_frequency_magnitude ==
                       magnitude &&
                   rumble_events[instance].output.high_frequency_magnitude ==
                       magnitude,
               "configured instance decoded another rumble context");
    }
    expect(rumble_events[1].count == 1,
           "another instance's rumble reached instance 1 callback");
}

void test_lifecycle_and_invalid_instances() {
    initialize_contexts();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(switch_pro_is_ready(instance),
               "initialized context was not ready");
    }

    tud_mount_cb();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(!switch_pro_is_ready(instance),
               "mount did not reset every configured context");
    }

    for (uint8_t addressed = 0; addressed < kInstanceCount; ++addressed) {
        send_config(addressed, DISABLE_USB_TIMEOUT);
        for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
            expect(switch_pro_is_ready(instance) == (instance <= addressed),
                   "handshake readiness crossed configured contexts");
        }
    }

    tud_umount_cb();
    for (uint8_t instance = 0; instance < kInstanceCount; ++instance) {
        expect(!switch_pro_is_ready(instance),
               "unmount did not reset every configured context");
    }

    ControllerState ignored{};
    ignored.button_system = true;
    switch_pro_init(kInvalidInstance);
    switch_pro_set_input(kInvalidInstance, ignored,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    usb_output_driver_set_rumble_callback(kInvalidInstance, rumble_callback);
    expect(!switch_pro_task(kInvalidInstance),
           "invalid instance ran a driver task");
    expect(!switch_pro_is_ready(kInvalidInstance),
           "invalid instance reported ready");
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> buffer{};
    expect(tud_hid_get_report_cb(kInvalidInstance, 0, HID_REPORT_TYPE_INPUT,

                                 buffer.data(), buffer.size()) == 0,
           "invalid instance served GET_REPORT data");
    expect(tud_hid_descriptor_report_cb(kInvalidInstance) == nullptr,
           "invalid instance served a report descriptor");
}
void test_protocol_neutral_trigger_threshold() {
    initialize_contexts();
    ControllerState state{};
    state.left_trigger =
        static_cast<uint16_t>(SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD - 1u);
    state.right_trigger = SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD;
    switch_pro_set_input(0, state, SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 15;
    expect(switch_pro_task(0), "trigger threshold report was not sent");
    SwitchProReport report = copy_switch_report(latest_regular_report(0));
    expect(!report.inputs.buttonZL && report.inputs.buttonZR,
           "Switch trigger threshold changed at the lower boundary");

    state.left_trigger = SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD;
    state.right_trigger = CONTROLLER_TRIGGER_MIN;
    switch_pro_set_input(0, state, SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 30;
    expect(switch_pro_task(0), "second trigger threshold report was not sent");
    report = copy_switch_report(latest_regular_report(0));
    expect(report.inputs.buttonZL && !report.inputs.buttonZR,
           "Switch trigger threshold changed at the upper boundary");
}

void test_custom_trigger_thresholds_are_isolated() {
    initialize_contexts();
    ControllerState state{};
    state.left_trigger = 300;
    state.right_trigger = 300;
    switch_pro_set_input(0, state, 300, 301);
    switch_pro_set_input(1, state, 301, 300);

    now_ms = 15;
    expect(switch_pro_task(0) && switch_pro_task(1),
           "custom trigger threshold reports were not sent");
    const SwitchProReport first =
        copy_switch_report(latest_regular_report(0));
    const SwitchProReport second =
        copy_switch_report(latest_regular_report(1));
    expect(first.inputs.buttonZL && !first.inputs.buttonZR,
           "instance 0 did not use its exact left/right trigger thresholds");
    expect(!second.inputs.buttonZL && second.inputs.buttonZR,
           "instance 1 trigger thresholds crossed HID contexts");
}

void test_uart_parser_is_pure() {
    initialize_contexts();
    ControllerState driver_state{};
    driver_state.left_stick_x = driver_state.left_stick_y =
        driver_state.right_stick_x = driver_state.right_stick_y = 0;
    driver_state.button_north = true;
    switch_pro_set_input(0, driver_state,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                         SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    now_ms = 15;
    switch_pro_task(0);

    std::array<uint8_t, 12> packet{};
    packet[0] = 0xaa;
    packet[1] = 0x02;
    packet[2] = 8;
    uint16_t buttons = SWITCH_PRO_MASK_A | SWITCH_PRO_MASK_L;
    packet[3] = static_cast<uint8_t>(buttons);
    packet[4] = static_cast<uint8_t>(buttons >> 8u);
    packet[5] = SWITCH_PRO_HAT_DOWNLEFT;
    packet[6] = 0x12;
    packet[7] = 0x34;
    packet[8] = 0x56;
    packet[9] = 0x78;
    for (unsigned i = 0; i < packet.size() - 1; ++i) {
        packet.back() = static_cast<uint8_t>(packet.back() + packet[i]);
    }
    ControllerState parsed{};
    expect(switch_pro_apply_uart_packet(packet.data(), packet.size(), parsed),
           "valid UART packet was rejected");
    expect(parsed.button_east && parsed.button_left_shoulder && parsed.dpad_down &&
               parsed.dpad_left,
           "UART buttons or hat were parsed incorrectly");
    expect(parsed.left_stick_x ==
               controller_axis_from_unsigned(0x1212) &&
               parsed.left_stick_y ==
               controller_axis_from_unsigned(0x3434) &&
               parsed.right_stick_x ==
               controller_axis_from_unsigned(0x5656) &&
               parsed.right_stick_y ==
               controller_axis_from_unsigned(0x7878),
           "UART axes were parsed incorrectly");

    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> current{};
    tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT, current.data(),
                          current.size());
    SwitchProReport current_report{};
    std::memcpy(&current_report, current.data(), sizeof(current_report));
    expect(current_report.inputs.buttonX && !current_report.inputs.buttonA,
           "UART parsing mutated driver context state");

    ControllerState unchanged{};
    unchanged.button_system = true;
    unchanged.left_stick_x = 123;
    packet.back() ^= 0xffu;
    expect(!switch_pro_apply_uart_packet(packet.data(), packet.size(),
                                         unchanged),
           "invalid UART checksum was accepted");
    expect(unchanged.button_system && unchanged.left_stick_x == 123,
           "failed UART parse modified its output reference");
}

}  // namespace

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
    return instance < SWITCH_PICO_HID_INSTANCE_COUNT && hid_ready[instance];
}
extern "C" bool tud_hid_n_report(uint8_t instance, uint8_t report_id,
                                  const void* report, uint16_t length) {
    if (instance >= SWITCH_PICO_HID_INSTANCE_COUNT || report == nullptr ||
        length > SWITCH_PRO_ENDPOINT_SIZE) {
        return false;
    }
    ++hid_report_attempts[instance];
    if (!hid_report_succeeds[instance] ||
        sent_report_count >= sent_reports.size()) {
        return false;
    }
    SentReport& sent = sent_reports[sent_report_count++];
    sent.instance = instance;
    sent.report_id = report_id;
    sent.length = length;
    std::memcpy(sent.data.data(), report, length);
    return true;
}
extern "C" bool tud_suspended(void) {
    return false;
}
extern "C" bool tud_remote_wakeup(void) {
    return true;
}

int main() {
    test_reset_materializes_neutral_sticks();
    test_startup_identify_preserves_first_reply_counter();
    test_failed_startup_identify_retries_preserve_counter();
    test_input_reports_and_timers_are_isolated();
    test_callback_send_and_imu_modes_are_isolated();
    test_rumble_callbacks_and_decoders_are_isolated();
    test_grip_colors_are_isolated();
    test_lifecycle_and_invalid_instances();
    test_protocol_neutral_trigger_threshold();
    test_custom_trigger_thresholds_are_isolated();
    test_uart_parser_is_pure();
    if (failures != 0) {
        std::cerr << failures << " driver context test(s) failed\n";
        return 1;
    }
    return 0;
}
