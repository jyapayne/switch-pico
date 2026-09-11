#include "bootsel.h"

#include "adapter/adapter_mode_controller.h"
#include "usb/usb_configuration_management.h"

namespace {
bool bootsel_accepted;
}

bool probe_bootsel_vendor_control(uint8_t rhport, uint8_t stage,
                                  const tusb_control_request_t* request) {
    using namespace UsbConfigurationManagement;
    if (request == nullptr || request->bmRequestType != 0x40 ||
        request->bRequest != static_cast<uint8_t>(Operation::kBootselReboot) ||
        request->wValue != kRequestValue || request->wIndex != kRequestIndex ||
        request->wLength != kRequestHeaderSize) {
        return false;
    }
    // The shared handler receives the envelope at SETUP and validates and
    // dispatches it only at ACK, after the host's control transfer completes.
    const bool accepted =
        usb_configuration_management_vendor_control(rhport, stage, request);
    if (accepted && stage == CONTROL_STAGE_ACK) {
        bootsel_accepted = true;
    }
    return accepted;
}

void probe_bootsel_task(uint32_t now_ms) {
    // The native bridge does not initialize ordinary adapter-mode selection.
    // A successful BOOTSEL dispatch guarantees the task takes its reboot path.
    if (bootsel_accepted) {
        adapter_mode_controller_task(now_ms);
    }
}
