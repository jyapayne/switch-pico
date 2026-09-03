#include "adapter/adapter_host_probe.h"
#include <stddef.h>
#ifdef SWITCH_PICO_LOG
#include <stdio.h>
#define PROBE_LOG(...) printf(__VA_ARGS__)
#else
#define PROBE_LOG(...) ((void)0)
#endif

#include "adapter/adapter_host_probe_state.h"
#include "profile/controller_profile_runtime.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "pico/time.h"
#include "usb/xinput/xinput_descriptors.h"

namespace {

constexpr uint32_t kXInputBootMagic = 0x58494e50; // "XINP"
constexpr uint8_t kModeScratchRegister = 0;
constexpr uint8_t kStatusRequest = 0x21;
constexpr uint16_t kStatusIndex = 0x0005;
uint8_t g_status_response[4]{};

AdapterUsbMode g_mode = AdapterUsbMode::kSwitch;
AdapterHostProbeState g_probe;
alarm_id_t g_reboot_alarm = 0;

uint32_t now_ms() {
    return static_cast<uint32_t>(to_ms_since_boot(get_absolute_time()));
}
int64_t reboot_to_xinput(alarm_id_t alarm_id, void *user_data) {
    (void)alarm_id;
    (void)user_data;
    controller_profile_runtime_reset();
    watchdog_hw->scratch[kModeScratchRegister] = kXInputBootMagic;
    watchdog_reboot(0, 0, 0);
    return 0;
}

} // namespace

void adapter_host_probe_init(AdapterRequestedMode requested_mode) {
    const uint32_t scratch = watchdog_hw->scratch[kModeScratchRegister];
    // Scratch is a one-boot transition token. Always consume it, including
    // stale tokens left behind when a persistent manual mode bypasses auto.
    watchdog_hw->scratch[kModeScratchRegister] = 0;

    switch (requested_mode) {
        case AdapterRequestedMode::kSwitch:
            g_mode = AdapterUsbMode::kSwitch;
            break;
        case AdapterRequestedMode::kXInput:
            g_mode = AdapterUsbMode::kXInput;
            break;
        case AdapterRequestedMode::kDInput:
            g_mode = AdapterUsbMode::kDInput;
            break;
        case AdapterRequestedMode::kMac:
            g_mode = AdapterUsbMode::kMac;
            break;
        case AdapterRequestedMode::kAuto:
            g_mode = scratch == kXInputBootMagic
                         ? AdapterUsbMode::kXInput
                         : AdapterUsbMode::kSwitchProbe;
            break;
    }
    g_probe = {};
    g_reboot_alarm = 0;
#ifdef SWITCH_PICO_LOG
    const char* mode_name = "Switch probe";
    switch (g_mode) {
        case AdapterUsbMode::kSwitch:
            mode_name = "Switch";
            break;
        case AdapterUsbMode::kSwitchProbe:
            break;
        case AdapterUsbMode::kXInput:
            mode_name = "XInput";
            break;
        case AdapterUsbMode::kDInput:
            mode_name = "DInput";
            break;
        case AdapterUsbMode::kMac:
            mode_name = "Mac";
            break;
    }
    PROBE_LOG("[HOST PROBE] boot mode=%s\n", mode_name);
#endif
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
    if (request == nullptr ||
        request->bmRequestType_bit.direction != TUSB_DIR_IN ||
        request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE) {
        return false;
    }
    const bool status_request =
        request->bRequest == kStatusRequest &&
        request->wIndex == kStatusIndex;
    const bool compatible_id_request =
        request->bRequest == XInput::kMsVendorRequest &&
        request->wIndex == XInput::kMsCompatIdIndex &&
        (g_mode == AdapterUsbMode::kSwitchProbe ||
         g_mode == AdapterUsbMode::kXInput);
    if (!status_request && !compatible_id_request) {
        return false;
    }
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if (status_request) {
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


    if (g_mode == AdapterUsbMode::kSwitchProbe) {
        g_probe.note_ms_compat_id_request(now_ms());
        PROBE_LOG("[HOST PROBE] Microsoft compatible-ID request confirmed\n");
        const bool queued = tud_control_xfer(
            rhport, request,
            const_cast<uint8_t *>(
                XInput::kProbeMsCompatIdDescriptor),
            sizeof(XInput::kProbeMsCompatIdDescriptor));
        if (queued && g_reboot_alarm == 0) {
            g_reboot_alarm = add_alarm_in_ms(
                AdapterHostProbeState::kRebootDelayMs,
                reboot_to_xinput, nullptr, true);
            PROBE_LOG("[HOST PROBE] XInput reboot alarm=%d\n",
                      static_cast<int>(g_reboot_alarm));
        }
        return queued;
    }

    if (g_mode == AdapterUsbMode::kXInput) {
        return tud_control_xfer(
            rhport, request,
            const_cast<uint8_t *>(XInput::kMsCompatIdDescriptor),
            sizeof(XInput::kMsCompatIdDescriptor));
    }
    return false;
}
