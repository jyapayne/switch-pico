// Host harness for the UART firmware's EP0 management handler. Drives the
// TinyUSB control stages the way usbd does and reports what reached the
// (stubbed) ROM. Request/response bytes cross stdin/stdout as hex so the
// Python test can build them with config_manager itself.
//
//   uart_usb_management_test info            -> prints INFO response hex
//   uart_usb_management_test bootsel <hex>   -> prints "accepted=<0|1> reboot=<0|1>"
//   uart_usb_management_test other <op>      -> prints "accepted=<0|1>" for an IN request
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "tusb.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "usb/uart_usb_management.h"
#include "usb/usb_output_driver.h"

namespace {

void* g_xfer_buffer = nullptr;
uint16_t g_xfer_length = 0;
int g_reboots = 0;
uint64_t g_now_ms = 1000;

std::vector<uint8_t> parse_hex(const char* text) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; text[i] != '\0' && text[i + 1] != '\0'; i += 2) {
        bytes.push_back(static_cast<uint8_t>(strtoul(std::string(text + i, 2).c_str(), nullptr, 16)));
    }
    return bytes;
}

tusb_control_request_t make_request(uint8_t direction, uint8_t operation, uint16_t length) {
    tusb_control_request_t request{};
    request.bmRequestType = static_cast<uint8_t>((direction << 7) | (TUSB_REQ_TYPE_VENDOR << 5));
    request.bRequest = operation;
    request.wValue = 0x5350;
    request.wIndex = 0x0001;
    request.wLength = length;
    return request;
}

}  // namespace

extern "C" bool tud_control_xfer(uint8_t, tusb_control_request_t const*, void* buffer, uint16_t length) {
    g_xfer_buffer = buffer;
    g_xfer_length = length;
    return true;
}

extern "C" void reset_usb_boot(uint32_t, uint32_t) { ++g_reboots; }

extern "C" absolute_time_t make_timeout_time_ms(uint32_t milliseconds) {
    return absolute_time_t{g_now_ms + milliseconds};
}

extern "C" bool time_reached(absolute_time_t time) { return g_now_ms >= time.milliseconds; }

AdapterUsbMode usb_output_driver_mode() { return AdapterUsbMode::kSwitch; }
uint8_t usb_output_driver_capabilities() {
    return USB_OUTPUT_CAPABILITY_INPUT | USB_OUTPUT_CAPABILITY_RUMBLE | USB_OUTPUT_CAPABILITY_MOTION;
}

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const std::string mode = argv[1];
    if (mode == "info" || mode == "other") {
        const uint8_t operation = mode == "info" ? 0x01 : static_cast<uint8_t>(strtoul(argv[2], nullptr, 0));
        const tusb_control_request_t request = make_request(TUSB_DIR_IN, operation, 837);
        const bool accepted = uart_usb_management_vendor_control(0, CONTROL_STAGE_SETUP, &request);
        if (mode == "other") {
            printf("accepted=%d\n", accepted ? 1 : 0);
            return 0;
        }
        if (!accepted) return 1;
        uart_usb_management_vendor_control(0, CONTROL_STAGE_ACK, &request);
        const uint8_t* bytes = static_cast<const uint8_t*>(g_xfer_buffer);
        for (uint16_t i = 0; i < g_xfer_length; ++i) printf("%02x", bytes[i]);
        printf("\n");
        return 0;
    }
    if (mode == "bootsel" && argc >= 3) {
        const std::vector<uint8_t> payload = parse_hex(argv[2]);
        const tusb_control_request_t request =
            make_request(TUSB_DIR_OUT, 0x04, static_cast<uint16_t>(payload.size()));
        bool accepted = uart_usb_management_vendor_control(0, CONTROL_STAGE_SETUP, &request);
        if (accepted) {
            if (g_xfer_length < payload.size()) return 3;
            memcpy(g_xfer_buffer, payload.data(), payload.size());
            accepted = uart_usb_management_vendor_control(0, CONTROL_STAGE_DATA, &request) &&
                       uart_usb_management_vendor_control(0, CONTROL_STAGE_ACK, &request);
        }
        // The reboot must wait for the status stage, then fire exactly once.
        uart_usb_management_task();
        const int early = g_reboots;
        g_now_ms += 49;
        uart_usb_management_task();
        const int before_deadline = g_reboots;
        g_now_ms += 1;
        uart_usb_management_task();
        printf("accepted=%d early=%d reboot=%d\n", accepted ? 1 : 0, early + before_deadline, g_reboots);
        return 0;
    }
    return 2;
}
