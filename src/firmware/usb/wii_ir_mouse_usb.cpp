#include "usb/wii_ir_mouse_usb.h"

#ifdef SWITCH_PICO_WII_IR_MOUSE

#include <cstring>

#include "input/wii_ir_pointer.h"

namespace {

constexpr uint8_t kMouseInstance = SWITCH_PICO_HID_INSTANCE_COUNT;
uint8_t g_buttons = 0;

static_assert(WII_IR_MOUSE_DIAGNOSTIC_SIZE == 48,
              "Mouse Feature report descriptor must match diagnostics size");

}  // namespace

void usb_wii_ir_mouse_reset() {
    g_buttons = 0;
    wii_ir_pointer_reset();
}

void usb_wii_ir_mouse_task() {
    if (!tud_mounted()) {
        return;
    }

    WiiIrMouseReport pending{};
    if (!wii_ir_mouse_peek(&pending) || !tud_hid_n_ready(kMouseInstance)) {
        return;
    }

    // Boot and report protocols share these exact three bytes. TinyUSB's
    // mouse helper also sends wheels, which are not part of this descriptor.
    const uint8_t report[] = {
        static_cast<uint8_t>(pending.buttons & 0x03),
        static_cast<uint8_t>(pending.dx),
        static_cast<uint8_t>(pending.dy),
    };
    if (tud_hid_n_report(kMouseInstance, 0, report, sizeof(report))) {
        g_buttons = report[0];
        wii_ir_mouse_commit(pending);
    }
}

uint16_t usb_wii_ir_mouse_get_report(uint8_t report_id,
                                     hid_report_type_t report_type,
                                     uint8_t* buffer,
                                     uint16_t requested_length) {
    if (report_id != 0 || buffer == nullptr || requested_length == 0) {
        return 0;
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        const size_t capacity = requested_length < WII_IR_MOUSE_DIAGNOSTIC_SIZE
                                    ? requested_length
                                    : WII_IR_MOUSE_DIAGNOSTIC_SIZE;
        return static_cast<uint16_t>(wii_ir_pointer_diagnostics(buffer, capacity));
    }
    if (report_type == HID_REPORT_TYPE_INPUT) {
        // Control requests must not consume or replay relative movement.
        const uint8_t report[] = {g_buttons, 0, 0};
        const uint16_t length = requested_length < sizeof(report)
                                    ? requested_length
                                    : sizeof(report);
        memcpy(buffer, report, length);
        return length;
    }
    return 0;
}

#endif
