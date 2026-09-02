#include "usb_pairing_management.h"

#include <string.h>

#include "tusb.h"
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
#include "adapter_host_probe.h"
#endif

namespace UsbPairingManagement {

size_t encode_snapshot(const Bluepad32PairingSnapshot& snapshot,
                       uint8_t* output, size_t output_size) {
    const size_t required =
        kResponseHeaderSize + snapshot.record_count * kRecordSize;
    if (output == nullptr || output_size < required ||
        snapshot.record_count > BLUEPAD32_PAIRING_RECORD_CAPACITY) {
        return 0;
    }

    output[0] = 'S';
    output[1] = 'P';
    output[2] = 'P';
    output[3] = 'M';
    output[4] = kProtocolVersion;
    output[5] = static_cast<uint8_t>(snapshot.status);
    output[6] = snapshot.record_count;
    output[7] = snapshot.overflow ? 1 : 0;
    output[8] = static_cast<uint8_t>(snapshot.generation);
    output[9] = static_cast<uint8_t>(snapshot.generation >> 8);
    output[10] = static_cast<uint8_t>(snapshot.generation >> 16);
    output[11] = static_cast<uint8_t>(snapshot.generation >> 24);

    size_t offset = kResponseHeaderSize;
    for (uint8_t index = 0; index < snapshot.record_count; ++index) {
        const Bluepad32PairingRecord& record = snapshot.records[index];
        output[offset] = static_cast<uint8_t>(record.transport);
        output[offset + 1] = record.address_type;
        memcpy(&output[offset + 2], record.address,
               sizeof(record.address));
        offset += kRecordSize;
    }
    return required;
}

}  // namespace UsbPairingManagement

extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
    if (adapter_host_probe_vendor_control(rhport, stage, request)) {
        return true;
    }
#endif
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request == nullptr ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE ||
        request->wValue != UsbPairingManagement::kRequestValue ||
        request->wIndex != UsbPairingManagement::kRequestIndex) {
        return false;
    }

    switch (request->bRequest) {
        case UsbPairingManagement::kRequestGet: {
            if (request->bmRequestType_bit.direction != TUSB_DIR_IN) {
                return false;
            }
            static uint8_t response[
                UsbPairingManagement::kMaximumResponseSize];
            Bluepad32PairingSnapshot snapshot{};
            bluepad32_input_backend_pairing_snapshot(&snapshot);
            const size_t response_size =
                UsbPairingManagement::encode_snapshot(
                    snapshot, response, sizeof(response));
            return response_size != 0 &&
                   tud_control_xfer(
                       rhport, request, response,
                       static_cast<uint16_t>(response_size));
        }
        case UsbPairingManagement::kRequestRefresh:
            if (request->bmRequestType_bit.direction != TUSB_DIR_OUT ||
                request->wLength != 0) {
                return false;
            }
            bluepad32_input_backend_request_pairing_snapshot();
            return tud_control_status(rhport, request);
        case UsbPairingManagement::kRequestClear:
            if (request->bmRequestType_bit.direction != TUSB_DIR_OUT ||
                request->wLength != 0) {
                return false;
            }
            bluepad32_input_backend_clear_pairings();
            return tud_control_status(rhport, request);
        default:
            return false;
    }
}
