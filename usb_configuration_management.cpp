#include "usb_configuration_management.h"

#include <string.h>

#include "adapter_configuration.h"
#include "tusb.h"
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
#include "adapter_host_probe.h"
#endif

namespace UsbConfigurationManagement {
namespace {

uint16_t read_u16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
           static_cast<uint16_t>(input[1] << 8);
}

uint32_t read_u32(const uint8_t* input) {
    return static_cast<uint32_t>(input[0]) |
           (static_cast<uint32_t>(input[1]) << 8) |
           (static_cast<uint32_t>(input[2]) << 16) |
           (static_cast<uint32_t>(input[3]) << 24);
}

void write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t* output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

Status transaction_status(ConfigurationTransactionStatus status) {
    switch (status) {
        case ConfigurationTransactionStatus::kIdle:
        case ConfigurationTransactionStatus::kCommitted:
        case ConfigurationTransactionStatus::kUnchanged:
            return Status::kOk;
        case ConfigurationTransactionStatus::kReceiving:
        case ConfigurationTransactionStatus::kPending:
            return Status::kPending;
        case ConfigurationTransactionStatus::kMalformed:
            return Status::kMalformed;
        case ConfigurationTransactionStatus::kUnsupportedSchema:
            return Status::kUnsupportedSchema;
        case ConfigurationTransactionStatus::kTooLarge:
            return Status::kTooLarge;
        case ConfigurationTransactionStatus::kOutOfOrder:
            return Status::kOutOfOrder;
        case ConfigurationTransactionStatus::kBadCrc:
            return Status::kBadCrc;
        case ConfigurationTransactionStatus::kBusy:
            return Status::kBusy;
        case ConfigurationTransactionStatus::kStorageError:
            return Status::kStorageError;
    }
    return Status::kStorageError;
}

bool valid_out_size(Operation operation, size_t size) {
    switch (operation) {
        case Operation::kConfigurationBegin:
            return size == kRequestHeaderSize + 12;
        case Operation::kConfigurationChunk:
            return size > kRequestHeaderSize + 8 &&
                   size <= kMaximumRequestSize;
        case Operation::kConfigurationCommit:
        case Operation::kConfigurationReset:
            return size == kRequestHeaderSize + 4;
        case Operation::kPairingRefresh:
        case Operation::kPairingClear:
            return size == kRequestHeaderSize;
        default:
            return false;
    }
}

size_t encode_configuration(uint8_t* output, size_t output_size) {
    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    const bool encoded = adapter_configuration_encode(
        snapshot.configuration, payload, sizeof(payload));
    Status status = Status::kOk;
    if (snapshot.state == ConfigurationServiceState::kLoading) {
        status = Status::kPending;
    } else if (snapshot.state ==
               ConfigurationServiceState::kStorageError) {
        status = Status::kStorageError;
    }
    return encoded
               ? encode_response(
                     Operation::kConfigurationRead, status, 0,
                     ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                     snapshot.generation, payload, sizeof(payload),
                     output, output_size)
               : 0;
}

size_t encode_transaction(uint8_t* output, size_t output_size) {
    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    const ConfigurationTransactionSnapshot& transaction =
        snapshot.transaction;
    uint8_t payload[20]{};
    write_u32(&payload[0], transaction.transaction_id);
    write_u16(&payload[4], transaction.received_size);
    write_u16(&payload[6], transaction.expected_size);
    write_u32(&payload[8], transaction.expected_crc);
    write_u32(&payload[12], transaction.stored_generation);
    write_u32(&payload[16], transaction.stored_crc);
    return encode_response(
        Operation::kTransactionStatus,
        transaction_status(transaction.status), 0,
        ADAPTER_CONFIGURATION_SCHEMA_VERSION,
        transaction.stored_generation, payload, sizeof(payload),
        output, output_size);
}

size_t encode_info(uint8_t* output, size_t output_size) {
    uint8_t payload[8] = {
        0, 2, 0, 2,
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
        adapter_host_probe_mode() == AdapterUsbMode::kXInput ? 1u : 0u,
#else
        0,
#endif
        0,
        static_cast<uint8_t>(CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE),
        static_cast<uint8_t>(
            CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE >> 8),
    };
    return encode_response(Operation::kInfo, Status::kOk, 0, 0, 0,
                           payload, sizeof(payload), output, output_size);
}

}  // namespace

bool decode_request(Operation setup_operation, const uint8_t* input,
                    size_t input_size, DecodedRequest* output) {
    if (input == nullptr || output == nullptr ||
        input_size < kRequestHeaderSize ||
        memcmp(input, "SPMG", 4) != 0 ||
        input[4] != kProtocolVersion ||
        input[5] != static_cast<uint8_t>(setup_operation) ||
        input[6] != 0 || input[7] != 0 ||
        read_u16(&input[10]) != 0) {
        return false;
    }
    const uint16_t payload_size = read_u16(&input[8]);
    if (input_size != kRequestHeaderSize + payload_size ||
        configuration_crc32(&input[kRequestHeaderSize], payload_size) !=
            read_u32(&input[12])) {
        return false;
    }
    output->operation = setup_operation;
    output->payload = &input[kRequestHeaderSize];
    output->payload_size = payload_size;
    return true;
}

size_t encode_response(Operation operation, Status status, uint8_t flags,
                       uint16_t schema_version, uint32_t generation,
                       const uint8_t* payload, size_t payload_size,
                       uint8_t* output, size_t output_size) {
    const size_t required = kResponseHeaderSize + payload_size;
    if (output == nullptr || output_size < required ||
        payload_size > UINT16_MAX ||
        (payload_size != 0 && payload == nullptr)) {
        return 0;
    }
    memcpy(output, "SPMG", 4);
    output[4] = kProtocolVersion;
    output[5] = static_cast<uint8_t>(operation);
    output[6] = static_cast<uint8_t>(status);
    output[7] = flags;
    write_u16(&output[8], static_cast<uint16_t>(payload_size));
    write_u16(&output[10], schema_version);
    write_u32(&output[12], generation);
    write_u32(&output[16],
              configuration_crc32(payload, payload_size));
    if (payload_size != 0) {
        memcpy(&output[kResponseHeaderSize], payload, payload_size);
    }
    return required;
}

size_t encode_pairing_snapshot(const Bluepad32PairingSnapshot& snapshot,
                               uint8_t* output, size_t output_size) {
    if (snapshot.record_count > BLUEPAD32_PAIRING_RECORD_CAPACITY) {
        return 0;
    }
    uint8_t payload[kPairingPayloadHeaderSize +
                    BLUEPAD32_PAIRING_RECORD_CAPACITY *
                        kPairingRecordSize]{};
    payload[0] = snapshot.record_count;
    payload[1] = snapshot.overflow ? 1 : 0;
    size_t offset = kPairingPayloadHeaderSize;
    for (uint8_t index = 0; index < snapshot.record_count; ++index) {
        const Bluepad32PairingRecord& record = snapshot.records[index];
        payload[offset] = static_cast<uint8_t>(record.transport);
        payload[offset + 1] = record.address_type;
        memcpy(&payload[offset + 2], record.address,
               sizeof(record.address));
        offset += kPairingRecordSize;
    }
    return encode_response(
        Operation::kPairingRead,
        snapshot.status == Bluepad32PairingSnapshotStatus::kReady
            ? Status::kOk
            : Status::kPending,
        snapshot.overflow ? 1 : 0, 0, snapshot.generation,
        payload, offset, output, output_size);
}

}  // namespace UsbConfigurationManagement

namespace {

uint8_t g_request_buffer[
    UsbConfigurationManagement::kMaximumRequestSize]{};
UsbConfigurationManagement::Operation g_pending_operation =
    UsbConfigurationManagement::Operation::kInfo;
bool g_out_pending = false;
size_t g_pending_request_size = 0;

bool process_out_request() {
    using namespace UsbConfigurationManagement;
    DecodedRequest request{};
    if (!decode_request(g_pending_operation, g_request_buffer,
                        g_pending_request_size, &request)) {
        return false;
    }

    const uint8_t* payload = request.payload;
    switch (request.operation) {
        case Operation::kConfigurationBegin:
            configuration_service_begin(
                static_cast<uint32_t>(payload[0]) |
                    (static_cast<uint32_t>(payload[1]) << 8) |
                    (static_cast<uint32_t>(payload[2]) << 16) |
                    (static_cast<uint32_t>(payload[3]) << 24),
                static_cast<uint16_t>(payload[4] |
                                      (payload[5] << 8)),
                static_cast<uint16_t>(payload[6] |
                                      (payload[7] << 8)),
                static_cast<uint32_t>(payload[8]) |
                    (static_cast<uint32_t>(payload[9]) << 8) |
                    (static_cast<uint32_t>(payload[10]) << 16) |
                    (static_cast<uint32_t>(payload[11]) << 24));
            return true;
        case Operation::kConfigurationChunk: {
            const uint16_t chunk_size =
                static_cast<uint16_t>(payload[6] |
                                      (payload[7] << 8));
            if (request.payload_size != 8 + chunk_size ||
                chunk_size > kMaximumChunkSize) {
                return false;
            }
            configuration_service_append(
                static_cast<uint32_t>(payload[0]) |
                    (static_cast<uint32_t>(payload[1]) << 8) |
                    (static_cast<uint32_t>(payload[2]) << 16) |
                    (static_cast<uint32_t>(payload[3]) << 24),
                static_cast<uint16_t>(payload[4] |
                                      (payload[5] << 8)),
                &payload[8], chunk_size);
            return true;
        }
        case Operation::kConfigurationCommit:
            configuration_service_commit(
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24));
            return true;
        case Operation::kConfigurationReset:
            configuration_service_reset(
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24));
            return true;
        case Operation::kPairingRefresh:
            bluepad32_input_backend_request_pairing_snapshot();
            return true;
        case Operation::kPairingClear:
            bluepad32_input_backend_clear_pairings();
            return true;
        default:
            return false;
    }
}

}  // namespace

extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
#ifdef SWITCH_PICO_ADAPTER_FEASIBILITY
    if (adapter_host_probe_vendor_control(rhport, stage, request)) {
        return true;
    }
#endif
    using namespace UsbConfigurationManagement;
    if (request == nullptr ||
        request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_DEVICE ||
        request->wValue != kRequestValue ||
        request->wIndex != kRequestIndex) {
        return false;
    }

    const Operation operation =
        static_cast<Operation>(request->bRequest);
    if (stage == CONTROL_STAGE_ACK) {
        if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            return true;
        }
        if (!g_out_pending || operation != g_pending_operation) {
            return false;
        }
        g_out_pending = false;
        return process_out_request();
    }
    if (stage == CONTROL_STAGE_DATA) {
        return true;
    }
    if (stage != CONTROL_STAGE_SETUP) {
        return false;
    }

    if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        if (!valid_out_size(operation, request->wLength)) {
            return false;
        }
        g_pending_operation = operation;
        g_pending_request_size = request->wLength;
        g_out_pending = true;
        return tud_control_xfer(rhport, request, g_request_buffer,
                                request->wLength);
    }

    static uint8_t response[kMaximumResponseSize]{};
    size_t response_size = 0;
    switch (operation) {
        case Operation::kInfo:
            response_size = encode_info(response, sizeof(response));
            break;
        case Operation::kConfigurationRead:
            response_size =
                encode_configuration(response, sizeof(response));
            break;
        case Operation::kTransactionStatus:
            response_size =
                encode_transaction(response, sizeof(response));
            break;
        case Operation::kPairingRead: {
            Bluepad32PairingSnapshot snapshot{};
            bluepad32_input_backend_pairing_snapshot(&snapshot);
            response_size = encode_pairing_snapshot(
                snapshot, response, sizeof(response));
            break;
        }
        default:
            return false;
    }
    return response_size != 0 &&
           tud_control_xfer(
               rhport, request, response,
               static_cast<uint16_t>(response_size));
}
