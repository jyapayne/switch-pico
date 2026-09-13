#include "bootsel.h"

#include "model.h"
#if SWITCH2_PROBE_HUB
#include <string.h>
#include "pico/bootrom.h"
#include "usb/native_hub/native_hub.h"
#else
#include "adapter/adapter_mode_controller.h"
#endif
#include "usb/usb_configuration_management.h"

namespace {
bool bootsel_accepted;
#if SWITCH2_PROBE_HUB
constexpr uint32_t kBootselRebootDelayMs = 50;
struct BootselTransfer {
    uint8_t envelope[UsbConfigurationManagement::kRequestHeaderSize];
    bool pending;
    bool validated;
};
// Control state is independent even when the two children enumerate together.
BootselTransfer bootsel_transfers[PROBE_CONTROLLER_COUNT + 1];
bool bootsel_delay_started;
uint32_t bootsel_deadline_ms;
#endif
}

bool probe_management_vendor_control(uint8_t rhport, uint8_t stage,
                                     const tusb_control_request_t* request) {
    using namespace UsbConfigurationManagement;
#if SWITCH2_PROBE_HUB
    if (rhport > PROBE_CONTROLLER_COUNT) return false;
    BootselTransfer& transfer = bootsel_transfers[rhport];
    if (stage == CONTROL_STAGE_SETUP) {
        transfer.pending = false;
        transfer.validated = false;
    }
#endif
    if (request == nullptr || request->bmRequestType != 0x40 ||
        request->bRequest != static_cast<uint8_t>(Operation::kBootselReboot) ||
        request->wValue != kRequestValue || request->wIndex != kRequestIndex ||
        request->wLength != kRequestHeaderSize) {
#if SWITCH2_PROBE_HUB
        return rhport == 0 &&
            usb_configuration_management_vendor_control(rhport, stage, request);
#else
        return false;
#endif
    }
#if SWITCH2_PROBE_HUB
    if (stage == CONTROL_STAGE_SETUP) {
        // Any short OUT leaves nonzero reserved/CRC bytes and fails decoding.
        memset(transfer.envelope, 0xff, sizeof(transfer.envelope));
        transfer.pending = native_hub_control_xfer(
            rhport, request, transfer.envelope, sizeof(transfer.envelope));
        return transfer.pending;
    }
    if (stage == CONTROL_STAGE_DATA) {
        DecodedRequest decoded{};
        transfer.validated = transfer.pending &&
            decode_request(Operation::kBootselReboot, transfer.envelope,
                           sizeof(transfer.envelope), &decoded) &&
            decoded.payload_size == 0;
        return transfer.validated;
    }
    if (stage == CONTROL_STAGE_ACK && transfer.pending && transfer.validated) {
        transfer.pending = false;
        bootsel_accepted = true;
        return true;
    }
    return false;
#else
    // The shared handler receives the envelope at SETUP and validates and
    // dispatches it only at ACK, after the host's control transfer completes.
    const bool accepted =
        usb_configuration_management_vendor_control(rhport, stage, request);
    if (accepted && stage == CONTROL_STAGE_ACK) {
        bootsel_accepted = true;
    }
    return accepted;
#endif
}

void probe_bootsel_task(uint32_t now_ms) {
#if SWITCH2_PROBE_HUB
    if (!bootsel_accepted) return;
    if (!bootsel_delay_started) {
        bootsel_delay_started = true;
        bootsel_deadline_ms = now_ms + kBootselRebootDelayMs;
    } else if (static_cast<int32_t>(now_ms - bootsel_deadline_ms) >= 0) {
        bootsel_accepted = false;
        reset_usb_boot(0, 0);
    }
#else
    // The native bridge does not initialize ordinary adapter-mode selection.
    // A successful BOOTSEL dispatch guarantees the task takes its reboot path.
    if (bootsel_accepted) {
        adapter_mode_controller_task(now_ms);
    }
#endif
}
