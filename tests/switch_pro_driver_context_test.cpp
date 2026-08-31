#include "switch_pro_driver.h"
#include "tusb.h"
#include "pico/time.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

struct SentReport {
    uint8_t instance = 0;
    uint8_t report_id = 0;
    uint16_t length = 0;
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> data{};
};

struct RumbleEvent {
    unsigned count = 0;
    uint8_t instance = 0xff;
    SwitchRumbleOutput output{};
};

uint64_t now_ms = 0;
uint32_t random_value = 1;
bool hid_ready[SWITCH_PICO_HID_INSTANCE_COUNT] = {true, true};
bool hid_report_succeeds[SWITCH_PICO_HID_INSTANCE_COUNT] = {true, true};
std::array<unsigned, SWITCH_PICO_HID_INSTANCE_COUNT> hid_report_attempts{};
std::array<SentReport, 32> sent_reports{};
unsigned sent_report_count = 0;
RumbleEvent rumble_zero{};
RumbleEvent rumble_one{};
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
    hid_ready[0] = true;
    hid_ready[1] = true;
    hid_report_succeeds[0] = true;
    hid_report_succeeds[1] = true;
    hid_report_attempts = {};
    switch_pro_init(0);
    switch_pro_init(1);
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
    constexpr uint16_t packed_mid = SWITCH_PRO_JOYSTICK_MID >> 4u;
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

void rumble_callback_zero(uint8_t instance,
                          const SwitchRumbleOutput& output) {
    ++rumble_zero.count;
    rumble_zero.instance = instance;
    rumble_zero.output = output;
}

void rumble_callback_one(uint8_t instance,
                         const SwitchRumbleOutput& output) {
    ++rumble_one.count;
    rumble_one.instance = instance;
    rumble_one.output = output;
}

void test_reset_materializes_neutral_sticks() {
    initialize_contexts();
    SwitchProReport init_zero =
        get_current_report(0, "GET_REPORT failed immediately after init");
    SwitchProReport init_one =
        get_current_report(1, "GET_REPORT failed for second initialized context");
    expect_neutral_sticks(
        init_zero, "instance 0 sticks were not neutral immediately after init");
    expect_neutral_sticks(
        init_one, "instance 1 sticks were not neutral immediately after init");

    tud_mount_cb();
    SwitchProReport mount_zero =
        get_current_report(0, "GET_REPORT failed immediately after mount");
    SwitchProReport mount_one =
        get_current_report(1, "GET_REPORT failed for second mounted context");
    expect_neutral_sticks(
        mount_zero, "instance 0 sticks were not neutral immediately after mount");
    expect_neutral_sticks(
        mount_one, "instance 1 sticks were not neutral immediately after mount");
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
    SwitchInputState zero{};
    zero.lx = 0x1111;
    zero.ly = 0x2222;
    zero.rx = 0x3333;
    zero.ry = 0x4444;
    zero.button_a = true;
    SwitchInputState one{};
    one.lx = 0xaaaa;
    one.ly = 0xbbbb;
    one.rx = 0xcccc;
    one.ry = 0xdddd;
    one.button_b = true;
    one.dpad_right = true;
    switch_pro_set_input(0, zero);
    switch_pro_set_input(1, one);

    now_ms = 15;
    expect(switch_pro_task(0), "instance 0 did not send its timed report");
    expect(switch_pro_task(1), "instance 1 timer was changed by instance 0");
    const SentReport* sent_zero = latest_regular_report(0);
    const SentReport* sent_one = latest_regular_report(1);
    expect(sent_zero != nullptr, "instance 0 report used the wrong HID route");
    expect(sent_one != nullptr, "instance 1 report used the wrong HID route");
    SwitchProReport report_zero = copy_switch_report(sent_zero);
    SwitchProReport report_one = copy_switch_report(sent_one);
    expect(report_zero.inputs.buttonA && !report_zero.inputs.buttonB,
           "instance 0 button state crossed with instance 1");
    expect(report_one.inputs.buttonB && !report_one.inputs.buttonA,
           "instance 1 button state crossed with instance 0");
    expect(report_zero.inputs.leftStick.getX() !=
               report_one.inputs.leftStick.getX(),
           "instance stick reports were merged");

    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> get_zero{};
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> get_one{};
    expect(tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT,
                                 get_zero.data(), get_zero.size()) ==
               sizeof(SwitchProReport),
           "GET_REPORT rejected valid instance 0");
    expect(tud_hid_get_report_cb(1, 0, HID_REPORT_TYPE_INPUT,
                                 get_one.data(), get_one.size()) ==
               sizeof(SwitchProReport),
           "GET_REPORT rejected valid instance 1");
    expect(std::memcmp(get_zero.data(), get_one.data(), get_zero.size()) != 0,
           "GET_REPORT returned a shared report for both instances");
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

    SwitchInputState zero{};
    zero.lx = zero.ly = zero.rx = zero.ry = SWITCH_PRO_JOYSTICK_MID;
    zero.imu_sample_count = 1;
    zero.imu_samples[0] = {101, 202, 303, 404, 505, 606};
    SwitchInputState one = zero;
    one.button_x = true;
    one.imu_samples[0] = {1001, 2002, 3003, 4004, 5005, 6006};
    switch_pro_set_input(0, zero);
    switch_pro_set_input(1, one);
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
    SwitchInputState moving{};
    moving.lx = moving.ly = moving.rx = moving.ry = SWITCH_PRO_JOYSTICK_MID;
    moving.imu_sample_count = 1;
    moving.imu_samples[0] = {100, 200, 300, 20000, 0, 0};
    SwitchInputState stationary{};
    stationary.lx = stationary.ly = stationary.rx = stationary.ry =
        SWITCH_PRO_JOYSTICK_MID;
    stationary.imu_sample_count = 1;
    stationary.imu_samples[0] = {1000, 2000, 3000, 0, 0, 0};
    switch_pro_set_input(0, moving);
    switch_pro_set_input(1, stationary);
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

void test_rumble_callbacks_and_decoders_are_isolated() {
    initialize_contexts();
    rumble_zero = {};
    rumble_one = {};
    switch_pro_set_rumble_callback(0, rumble_callback_zero);
    switch_pro_set_rumble_callback(1, rumble_callback_one);
    constexpr uint32_t neutral = 0x40400100u;
    auto full_payload = rumble_payload(type_2(64, 16, 64, 16), neutral);
    auto full_report = complete_rumble_report(full_payload);
    tud_hid_report_received_cb(2, 0, full_report.data(), full_report.size());
    expect(rumble_zero.count == 0 && rumble_one.count == 0,
           "invalid output instance reached a rumble callback");

    std::array<uint8_t, 9> stripped{};
    std::memcpy(stripped.data() + 1, full_payload.data(), full_payload.size());
    tud_hid_set_report_cb(0, REPORT_OUTPUT_10, HID_REPORT_TYPE_OUTPUT,
                          stripped.data(), stripped.size());
    expect(rumble_zero.count == 1 && rumble_zero.instance == 0,
           "control output did not route to instance 0 callback");
    expect(rumble_zero.output.low_frequency_magnitude == 16 &&
               rumble_zero.output.high_frequency_magnitude == 16,
           "instance 0 full rumble state decoded incorrectly");
    expect(rumble_one.count == 0,
           "instance 0 rumble invoked instance 1 callback");

    auto delta_payload = rumble_payload(type_1_one_sample(17, 20), neutral);
    auto delta_report = complete_rumble_report(delta_payload);
    tud_hid_report_received_cb(1, 0, delta_report.data(), delta_report.size());
    expect(rumble_one.count == 1 && rumble_one.instance == 1,
           "interrupt output did not route to instance 1 callback");
    expect(rumble_one.output.low_frequency_magnitude == 0 &&
               rumble_one.output.high_frequency_magnitude == 1,
           "instance 1 decoder inherited instance 0 rumble state");
    tud_hid_report_received_cb(0, 0, delta_report.data(), delta_report.size());
    expect(rumble_zero.count == 2 &&
               rumble_zero.output.low_frequency_magnitude == 17 &&
               rumble_zero.output.high_frequency_magnitude == 18,
           "instance 0 decoder lost its own prior rumble state");
    expect(rumble_one.count == 1,
           "instance 0 delta reached instance 1 callback");
}

void test_lifecycle_and_invalid_instances() {
    initialize_contexts();
    expect(switch_pro_is_ready(0) && switch_pro_is_ready(1),
           "initialized contexts were not ready");
    tud_mount_cb();
    expect(!switch_pro_is_ready(0) && !switch_pro_is_ready(1),
           "mount did not reset every configured context");
    send_config(0, DISABLE_USB_TIMEOUT);
    expect(switch_pro_is_ready(0) && !switch_pro_is_ready(1),
           "instance 0 handshake changed instance 1 readiness");
    send_config(1, DISABLE_USB_TIMEOUT);
    expect(switch_pro_is_ready(0) && switch_pro_is_ready(1),
           "instance 1 handshake did not address its context");
    tud_umount_cb();
    expect(!switch_pro_is_ready(0) && !switch_pro_is_ready(1),
           "unmount did not reset every configured context");

    SwitchInputState ignored{};
    ignored.button_home = true;
    switch_pro_init(2);
    switch_pro_set_input(2, ignored);
    switch_pro_set_rumble_callback(2, rumble_callback_zero);
    expect(!switch_pro_task(2), "invalid instance ran a driver task");
    expect(!switch_pro_is_ready(2), "invalid instance reported ready");
    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> buffer{};
    expect(tud_hid_get_report_cb(2, 0, HID_REPORT_TYPE_INPUT,
                                 buffer.data(), buffer.size()) == 0,
           "invalid instance served GET_REPORT data");
    expect(tud_hid_descriptor_report_cb(2) == nullptr,
           "invalid instance served a report descriptor");
}

void test_uart_parser_is_pure() {
    initialize_contexts();
    SwitchInputState driver_state{};
    driver_state.lx = driver_state.ly = driver_state.rx = driver_state.ry =
        SWITCH_PRO_JOYSTICK_MID;
    driver_state.button_x = true;
    switch_pro_set_input(0, driver_state);
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
    SwitchInputState parsed{};
    expect(switch_pro_apply_uart_packet(packet.data(), packet.size(), parsed),
           "valid UART packet was rejected");
    expect(parsed.button_a && parsed.button_l && parsed.dpad_down &&
               parsed.dpad_left,
           "UART buttons or hat were parsed incorrectly");
    expect(parsed.lx == 0x1212 && parsed.ly == 0x3434 &&
               parsed.rx == 0x5656 && parsed.ry == 0x7878,
           "UART axes were parsed incorrectly");

    std::array<uint8_t, SWITCH_PRO_ENDPOINT_SIZE> current{};
    tud_hid_get_report_cb(0, 0, HID_REPORT_TYPE_INPUT, current.data(),
                          current.size());
    SwitchProReport current_report{};
    std::memcpy(&current_report, current.data(), sizeof(current_report));
    expect(current_report.inputs.buttonX && !current_report.inputs.buttonA,
           "UART parsing mutated driver context state");

    SwitchInputState unchanged{};
    unchanged.button_home = true;
    unchanged.lx = 123;
    packet.back() ^= 0xffu;
    expect(!switch_pro_apply_uart_packet(packet.data(), packet.size(),
                                         unchanged),
           "invalid UART checksum was accepted");
    expect(unchanged.button_home && unchanged.lx == 123,
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
    test_lifecycle_and_invalid_instances();
    test_uart_parser_is_pure();
    if (failures != 0) {
        std::cerr << failures << " driver context test(s) failed\n";
        return 1;
    }
    return 0;
}
