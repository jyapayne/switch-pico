#include "adapter_host_probe.h"

#include <stddef.h>
#ifdef SWITCH_PICO_LOG
#include <stdio.h>
#define PROBE_LOG(...) printf(__VA_ARGS__)
#else
#define PROBE_LOG(...) ((void)0)
#endif

#include "adapter_host_probe_state.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "pico/time.h"
#include "xinput_feasibility_descriptors.h"

namespace {

constexpr uint32_t kXInputBootMagic = 0x58494e50; // "XINP"
constexpr uint8_t kModeScratchRegister = 0;
constexpr uint8_t kStatusRequest = 0x21;
constexpr uint16_t kStatusIndex = 0x0005;
uint8_t g_status_response[4]{};

AdapterUsbMode g_mode = AdapterUsbMode::kSwitchProbe;
AdapterHostProbeState g_probe;
alarm_id_t g_reboot_alarm = 0;

uint32_t now_ms() {
    return static_cast<uint32_t>(to_ms_since_boot(get_absolute_time()));
}
int64_t reboot_to_xinput(alarm_id_t alarm_id, void *user_data) {
    (void)alarm_id;
    (void)user_data;
    watchdog_hw->scratch[kModeScratchRegister] = kXInputBootMagic;
    watchdog_reboot(0, 0, 0);
    return 0;
}

} // namespace

void adapter_host_probe_init() {
    if (watchdog_hw->scratch[kModeScratchRegister] == kXInputBootMagic) {
        watchdog_hw->scratch[kModeScratchRegister] = 0;
        g_mode = AdapterUsbMode::kXInput;
    } else {
        g_mode = AdapterUsbMode::kSwitchProbe;
    }
    g_probe = {};
    PROBE_LOG("[HOST PROBE] boot mode=%s\n",
              g_mode == AdapterUsbMode::kXInput ? "XInput" : "Switch probe");
}

AdapterUsbMode adapter_host_probe_mode() { return g_mode; }

void adapter_host_probe_note_string_descriptor(uint8_t index) {
    if (g_mode == AdapterUsbMode::kSwitchProbe && index == 0xee) {
        g_probe.note_ms_os_string();
        PROBE_LOG("[HOST PROBE] Microsoft OS string requested\n");
    }
}

bool adapter_host_probe_vendor_control(uint8_t rhport, uint8_t stage,
                                       tusb_control_request_t const *request) {
    if (stage != CONTROL_STAGE_SETUP || request == nullptr ||
        request->bmRequestType_bit.direction != TUSB_DIR_IN ||
        request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE) {
        return false;
    }

    if (request->bRequest == kStatusRequest &&
        request->wIndex == kStatusIndex) {
        const uint32_t current_time = now_ms();
        g_status_response[0] =
            g_mode == AdapterUsbMode::kXInput ? 1 : 0;
        g_status_response[1] = g_probe.saw_ms_os_string() ? 1 : 0;
        g_status_response[2] = g_probe.windows_confirmed() ? 1 : 0;
        g_status_response[3] =
            g_probe.should_reboot(current_time) ? 1 : 0;
        return tud_control_xfer(rhport, request, g_status_response,
                                sizeof(g_status_response));
    }

    if (request->bRequest != XInputFeasibility::kMsVendorRequest ||
        request->wIndex != XInputFeasibility::kMsCompatIdIndex) {
        return false;
    }

    if (g_mode == AdapterUsbMode::kSwitchProbe) {
        g_probe.note_ms_compat_id_request(now_ms());
        PROBE_LOG("[HOST PROBE] Microsoft compatible-ID request confirmed\n");
        const bool queued = tud_control_xfer(
            rhport, request,
            const_cast<uint8_t *>(
                XInputFeasibility::kProbeMsCompatIdDescriptor),
            sizeof(XInputFeasibility::kProbeMsCompatIdDescriptor));
        if (queued && g_reboot_alarm == 0) {
            g_reboot_alarm = add_alarm_in_ms(
                AdapterHostProbeState::kRebootDelayMs,
                reboot_to_xinput, nullptr, true);
            PROBE_LOG("[HOST PROBE] XInput reboot alarm=%d\n",
                      static_cast<int>(g_reboot_alarm));
        }
        return queued;
    }

    return tud_control_xfer(
        rhport, request,
        const_cast<uint8_t *>(XInputFeasibility::kMsCompatIdDescriptor),
        sizeof(XInputFeasibility::kMsCompatIdDescriptor));
}
