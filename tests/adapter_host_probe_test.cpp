#include "adapter/adapter_host_probe.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "hardware/structs/watchdog.h"
#include "pico/time.h"
#include "usb/xinput/xinput_descriptors.h"

namespace {

constexpr uint32_t kXInputBootMagic = 0x58494e50u;
watchdog_hw_t watchdog_registers{};
uint64_t now_ms = 0;
alarm_callback_t pending_alarm = nullptr;
void* pending_alarm_user_data = nullptr;
int64_t pending_alarm_delay_ms = 0;
int alarm_count = 0;
int reset_count = 0;
int reboot_count = 0;
int call_sequence = 0;
int reset_sequence = 0;
int reboot_sequence = 0;
bool control_result = true;
int control_count = 0;
const void* control_buffer = nullptr;
uint16_t control_length = 0;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void reset_harness(uint32_t scratch) {
    watchdog_registers = {};
    watchdog_registers.scratch[0] = scratch;
    now_ms = 0;
    pending_alarm = nullptr;
    pending_alarm_user_data = nullptr;
    pending_alarm_delay_ms = 0;
    alarm_count = 0;
    reset_count = 0;
    reboot_count = 0;
    call_sequence = 0;
    reset_sequence = 0;
    reboot_sequence = 0;
    control_result = true;
    control_count = 0;
    control_buffer = nullptr;
    control_length = 0;
}

tusb_control_request_t microsoft_request() {
    tusb_control_request_t request{};
    request.bmRequestType_bit.direction = TUSB_DIR_IN;
    request.bmRequestType_bit.type = TUSB_REQ_TYPE_VENDOR;
    request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_DEVICE;
    request.bRequest = XInput::kMsVendorRequest;
    request.wIndex = XInput::kMsCompatIdIndex;
    return request;
}

void test_auto_scratch_lifecycle() {
    reset_harness(0);
    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    require(adapter_host_probe_mode() == AdapterUsbMode::kSwitchProbe &&
                watchdog_registers.scratch[0] == 0,
            "auto cold boot did not select Switch probe");

    reset_harness(kXInputBootMagic);
    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    require(adapter_host_probe_mode() == AdapterUsbMode::kXInput &&
                watchdog_registers.scratch[0] == 0,
            "auto transition token did not select one XInput boot");

    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    require(adapter_host_probe_mode() == AdapterUsbMode::kSwitchProbe,
            "consumed scratch token caused a reboot loop");

    reset_harness(0xa5a55a5au);
    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    require(adapter_host_probe_mode() == AdapterUsbMode::kSwitchProbe &&
                watchdog_registers.scratch[0] == 0,
            "stale non-token scratch was not consumed safely");
}

void test_manual_modes_bypass_and_consume_probe_state() {
    tusb_control_request_t request = microsoft_request();

    reset_harness(kXInputBootMagic);
    adapter_host_probe_init(AdapterRequestedMode::kSwitch);
    require(adapter_host_probe_mode() == AdapterUsbMode::kSwitch &&
                watchdog_registers.scratch[0] == 0,
            "manual Switch honored stale auto scratch");
    adapter_host_probe_note_string_descriptor(0xee);
    require(!adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_count == 0 && alarm_count == 0,
            "manual Switch entered the host probe path");

    reset_harness(0xdeadbeefu);
    adapter_host_probe_init(AdapterRequestedMode::kXInput);
    require(adapter_host_probe_mode() == AdapterUsbMode::kXInput &&
                watchdog_registers.scratch[0] == 0,
            "manual XInput honored stale scratch");
    require(adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_count == 1 &&
                control_length == sizeof(XInput::kMsCompatIdDescriptor) &&
                std::memcmp(control_buffer, XInput::kMsCompatIdDescriptor,
                            control_length) == 0 &&
                alarm_count == 0,
            "manual XInput did not serve XInput descriptors without probing");
    require(adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_DATA, &request) &&
                adapter_host_probe_vendor_control(
                    0, CONTROL_STAGE_ACK, &request) &&
                control_count == 1,
            "manual XInput did not retain the compatible-ID transfer");

    reset_harness(kXInputBootMagic);
    adapter_host_probe_init(AdapterRequestedMode::kDInput);
    require(adapter_host_probe_mode() == AdapterUsbMode::kDInput &&
                watchdog_registers.scratch[0] == 0 && alarm_count == 0,
            "manual DInput did not bypass and consume stale auto scratch");
    adapter_host_probe_note_string_descriptor(0xee);
    require(!adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_count == 0 && alarm_count == 0,
            "manual DInput entered the XInput probe path");

    reset_harness(kXInputBootMagic);
    adapter_host_probe_init(AdapterRequestedMode::kMac);
    require(adapter_host_probe_mode() == AdapterUsbMode::kMac &&
                watchdog_registers.scratch[0] == 0 && alarm_count == 0,
            "manual Mac did not bypass and consume stale auto scratch");
    adapter_host_probe_note_string_descriptor(0xee);
    require(!adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_count == 0 && alarm_count == 0,
            "manual Mac entered the XInput probe path");
}

void test_probe_transition_resets_before_watchdog() {
    reset_harness(0);
    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    adapter_host_probe_note_string_descriptor(0xee);
    tusb_control_request_t request = microsoft_request();
    require(adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_length ==
                    sizeof(XInput::kProbeMsCompatIdDescriptor) &&
                std::memcmp(control_buffer,
                            XInput::kProbeMsCompatIdDescriptor,
                            control_length) == 0 &&
                alarm_count == 1 && pending_alarm_delay_ms == 100 &&
                reboot_count == 0,
            "auto probe did not queue the exact delayed transition");
    require(adapter_host_probe_vendor_control(
                0, CONTROL_STAGE_DATA, &request) &&
                adapter_host_probe_vendor_control(
                    0, CONTROL_STAGE_ACK, &request) &&
                control_count == 1 && alarm_count == 1,
            "auto probe did not retain the compatible-ID transfer");

    pending_alarm(1, pending_alarm_user_data);
    require(reset_count == 1 && reboot_count == 1 &&
                reset_sequence < reboot_sequence &&
                watchdog_registers.scratch[0] == kXInputBootMagic,
            "probe reboot did not reset synthetic state before scratch reset");

    adapter_host_probe_init(AdapterRequestedMode::kAuto);
    require(adapter_host_probe_mode() == AdapterUsbMode::kXInput &&
                watchdog_registers.scratch[0] == 0,
            "probe transition was not consumed on the next boot");
}

}  // namespace

watchdog_hw_t* watchdog_hw = &watchdog_registers;

absolute_time_t get_absolute_time() { return now_ms; }
uint64_t to_ms_since_boot(absolute_time_t time) { return time; }

alarm_id_t add_alarm_in_ms(int64_t delay_ms, alarm_callback_t callback,
                           void* user_data, bool) {
    ++alarm_count;
    pending_alarm_delay_ms = delay_ms;
    pending_alarm = callback;
    pending_alarm_user_data = user_data;
    return alarm_count;
}

bool tud_control_xfer(uint8_t, const tusb_control_request_t*, void* buffer,
                      uint16_t length) {
    ++control_count;
    control_buffer = buffer;
    control_length = length;
    return control_result;
}

void controller_profile_runtime_reset() {
    ++reset_count;
    reset_sequence = ++call_sequence;
}

void watchdog_reboot(uint32_t, uint32_t, uint32_t) {
    ++reboot_count;
    reboot_sequence = ++call_sequence;
}

int main() {
    test_auto_scratch_lifecycle();
    test_manual_modes_bypass_and_consume_probe_state();
    test_probe_transition_resets_before_watchdog();
    return 0;
}
