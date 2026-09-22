#include "usb/uart_usb_management.h"

#include <string.h>

#include "pico/bootrom.h"
#include "pico/time.h"

#include "configuration/configuration_storage.h"
#include "usb/usb_management_protocol.h"
#include "usb/usb_output_driver.h"

namespace {

using namespace UsbConfigurationManagement;

// Let the status ACK reach the host before the USB controller disappears.
constexpr uint32_t kBootselRebootDelayMs = 50;
constexpr uint8_t kFirmwareVersion[3] = {0, 2, 0};

uint8_t g_request[kRequestHeaderSize]{};
uint8_t g_response[kResponseHeaderSize + kInfoPayloadSize]{};
tusb_control_request_t g_pending_setup{};
bool g_out_pending = false;
bool g_reboot_requested = false;
absolute_time_t g_reboot_deadline{};

void write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t* output, uint32_t value) {
    write_u16(output, static_cast<uint16_t>(value));
    write_u16(output + 2, static_cast<uint16_t>(value >> 16));
}

uint16_t read_u16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0] | (input[1] << 8));
}

uint32_t read_u32(const uint8_t* input) {
    return static_cast<uint32_t>(read_u16(input)) |
           (static_cast<uint32_t>(read_u16(input + 2)) << 16);
}

// Same envelope as usb_configuration_management.cpp encode_response.
size_t encode_info() {
    uint8_t payload[kInfoPayloadSize] = {
        kFirmwareVersion[0], kFirmwareVersion[1], kFirmwareVersion[2],
        kBoardPico,
        static_cast<uint8_t>(usb_output_driver_mode()),
        usb_output_driver_capabilities(),
        0, 0,  // No configuration storage on this firmware.
    };
    memcpy(g_response, "SPMG", 4);
    g_response[4] = kProtocolVersion;
    g_response[5] = static_cast<uint8_t>(Operation::kInfo);
    g_response[6] = static_cast<uint8_t>(Status::kOk);
    g_response[7] = 0;
    write_u16(&g_response[8], kInfoPayloadSize);
    write_u16(&g_response[10], 0);
    write_u32(&g_response[12], 0);
    write_u32(&g_response[16], configuration_crc32(payload, sizeof(payload)));
    memcpy(&g_response[kResponseHeaderSize], payload, sizeof(payload));
    return sizeof(g_response);
}

// Same header validation as usb_configuration_management.cpp decode_request,
// restricted to the empty-payload BOOTSEL request.
bool valid_bootsel_request() {
    return memcmp(g_request, "SPMG", 4) == 0 &&
           g_request[4] == kProtocolVersion &&
           g_request[5] == static_cast<uint8_t>(Operation::kBootselReboot) &&
           g_request[6] == 0 && g_request[7] == 0 &&
           read_u16(&g_request[8]) == 0 && read_u16(&g_request[10]) == 0 &&
           read_u32(&g_request[12]) == configuration_crc32(nullptr, 0);
}

}  // namespace

bool uart_usb_management_vendor_control(uint8_t rhport, uint8_t stage,
                                        const tusb_control_request_t* request) {
    if (request == nullptr ||
        request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE ||
        request->wValue != kRequestValue ||
        request->wIndex != kRequestIndex) {
        return false;
    }
    const Operation operation = static_cast<Operation>(request->bRequest);
    const bool input = request->bmRequestType_bit.direction == TUSB_DIR_IN;

    if (stage == CONTROL_STAGE_SETUP) {
        g_out_pending = false;
        if (input) {
            if (operation != Operation::kInfo) return false;
            return tud_control_xfer(rhport, request, g_response,
                                    static_cast<uint16_t>(encode_info()));
        }
        if (operation != Operation::kBootselReboot ||
            request->wLength != kRequestHeaderSize) {
            return false;
        }
        g_pending_setup = *request;
        g_out_pending = tud_control_xfer(rhport, request, g_request,
                                         request->wLength);
        return g_out_pending;
    }
    if (stage == CONTROL_STAGE_DATA) {
        return true;
    }
    if (stage != CONTROL_STAGE_ACK) {
        return false;
    }
    if (input) {
        return true;
    }
    if (!g_out_pending ||
        memcmp(request, &g_pending_setup, sizeof(*request)) != 0 ||
        !valid_bootsel_request()) {
        g_out_pending = false;
        return false;
    }
    g_out_pending = false;
    if (!g_reboot_requested) {
        g_reboot_requested = true;
        g_reboot_deadline = make_timeout_time_ms(kBootselRebootDelayMs);
    }
    return true;
}

void uart_usb_management_task() {
    if (g_reboot_requested && time_reached(g_reboot_deadline)) {
        reset_usb_boot(0, 0);
    }
}
