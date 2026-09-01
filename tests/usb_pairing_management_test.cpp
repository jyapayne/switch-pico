#include "usb_pairing_management.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include <tusb.h>

namespace {

Bluepad32PairingSnapshot current_snapshot{};
bool refresh_requested = false;
bool clear_requested = false;
bool control_status_sent = false;
std::vector<uint8_t> control_payload;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void test_encoding() {
    Bluepad32PairingSnapshot snapshot{};
    snapshot.generation = 0x78563412;
    snapshot.status = Bluepad32PairingSnapshotStatus::kReady;
    snapshot.record_count = 2;
    snapshot.overflow = true;
    snapshot.records[0].transport =
        Bluepad32PairingTransport::kClassic;
    snapshot.records[0].address_type = 0xfe;
    const uint8_t classic_address[6] = {1, 2, 3, 4, 5, 6};
    memcpy(snapshot.records[0].address, classic_address, 6);
    snapshot.records[1].transport = Bluepad32PairingTransport::kBle;
    snapshot.records[1].address_type = 2;
    const uint8_t ble_address[6] = {6, 5, 4, 3, 2, 1};
    memcpy(snapshot.records[1].address, ble_address, 6);

    uint8_t payload[UsbPairingManagement::kMaximumResponseSize]{};
    const size_t size = UsbPairingManagement::encode_snapshot(
        snapshot, payload, sizeof(payload));
    require(size == UsbPairingManagement::kResponseHeaderSize +
                        2 * UsbPairingManagement::kRecordSize,
            "snapshot encoded with the wrong size");
    require(memcmp(payload, "SPPM", 4) == 0 &&
                payload[4] == UsbPairingManagement::kProtocolVersion &&
                payload[5] == 0 && payload[6] == 2 && payload[7] == 1,
            "snapshot header encoding is invalid");
    require(payload[8] == 0x12 && payload[9] == 0x34 &&
                payload[10] == 0x56 && payload[11] == 0x78,
            "snapshot generation is not little endian");
    require(payload[12] == 1 && payload[13] == 0xfe &&
                memcmp(&payload[14], classic_address, 6) == 0 &&
                payload[20] == 2 && payload[21] == 2 &&
                memcmp(&payload[22], ble_address, 6) == 0,
            "pairing records are encoded incorrectly");
    require(UsbPairingManagement::encode_snapshot(
                snapshot, payload, size - 1) == 0,
            "encoder accepted a short destination buffer");
}

void test_vendor_requests() {
    current_snapshot = {};
    current_snapshot.generation = 7;
    current_snapshot.status = Bluepad32PairingSnapshotStatus::kReady;
    current_snapshot.record_count = 1;
    current_snapshot.records[0].transport =
        Bluepad32PairingTransport::kClassic;

    tusb_control_request_t request{};
    request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_DEVICE;
    request.bmRequestType_bit.direction = TUSB_DIR_IN;
    request.bRequest = UsbPairingManagement::kRequestGet;
    request.wValue = UsbPairingManagement::kRequestValue;
    request.wIndex = UsbPairingManagement::kRequestIndex;
    request.wLength = UsbPairingManagement::kMaximumResponseSize;
    require(tud_vendor_control_xfer_cb(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() ==
                    UsbPairingManagement::kResponseHeaderSize +
                        UsbPairingManagement::kRecordSize &&
                control_payload[8] == 7,
            "GET request did not return the current pairing snapshot");

    request.bmRequestType_bit.direction = TUSB_DIR_OUT;
    request.wLength = 0;
    request.bRequest = UsbPairingManagement::kRequestRefresh;
    require(tud_vendor_control_xfer_cb(
                0, CONTROL_STAGE_SETUP, &request) &&
                refresh_requested && control_status_sent,
            "REFRESH request was not acknowledged and queued");

    control_status_sent = false;
    request.bRequest = UsbPairingManagement::kRequestClear;
    require(tud_vendor_control_xfer_cb(
                0, CONTROL_STAGE_SETUP, &request) &&
                clear_requested && control_status_sent,
            "CLEAR request was not acknowledged and queued");

    request.wValue = 0;
    require(!tud_vendor_control_xfer_cb(
                0, CONTROL_STAGE_SETUP, &request),
            "request with invalid magic was accepted");
    require(tud_vendor_control_xfer_cb(
                0, CONTROL_STAGE_ACK, &request),
            "non-setup control stage was rejected");
}

}  // namespace

void bluepad32_input_backend_request_pairing_snapshot() {
    refresh_requested = true;
}

void bluepad32_input_backend_clear_pairings() {
    clear_requested = true;
}

void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out) {
    *out = current_snapshot;
}

bool tud_control_xfer(uint8_t, const tusb_control_request_t*,
                      void* buffer, uint16_t length) {
    const auto* bytes = static_cast<const uint8_t*>(buffer);
    control_payload.assign(bytes, bytes + length);
    return true;
}

bool tud_control_status(uint8_t, const tusb_control_request_t*) {
    control_status_sent = true;
    return true;
}

#include "../usb_pairing_management.cpp"

int main() {
    test_encoding();
    test_vendor_requests();
    return 0;
}
