#include "usb_configuration_management.h"

#include <string.h>

#include "adapter_configuration.h"
#include "adapter_host_probe.h"
#include "adapter_reboot.h"
#include "adapter_usb_mode.h"
#include "tusb.h"
#include "usb_output_driver.h"

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
Status profile_service_status(const ProfileServiceMetadata& metadata) {
    if (metadata.state == ProfileServiceState::kLoading) {
        return Status::kPending;
    }
    if (metadata.state == ProfileServiceState::kStorageError) {
        return Status::kStorageError;
    }
    return Status::kOk;
}


bool valid_out_size(Operation operation, size_t size) {
    switch (operation) {
        case Operation::kModeSet:
            return size == kRequestHeaderSize + 5;
        case Operation::kReboot:
            return size == kRequestHeaderSize + 4;
        case Operation::kConfigurationBegin:
            return size == kRequestHeaderSize + 12;
        case Operation::kProfileBegin:
            return size == kRequestHeaderSize + 28;
        case Operation::kConfigurationChunk:
            return size > kRequestHeaderSize + 8 &&
                   size <= kMaximumRequestSize;
        case Operation::kProfileChunk:
            return size > kRequestHeaderSize + 8 &&
                   size <= kMaximumRequestSize;
        case Operation::kConfigurationCommit:
        case Operation::kConfigurationReset:
            return size == kRequestHeaderSize + 4;
        case Operation::kProfileCommit:
            return size == kRequestHeaderSize + 4;
        case Operation::kProfileSelect:
            return size == kRequestHeaderSize + 15;
        case Operation::kProfileReset:
        case Operation::kProfileActivate:
            return size == kRequestHeaderSize + 19;
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
        static_cast<uint8_t>(usb_output_driver_mode()),
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

size_t encode_profile_list(const ProfileServiceListSnapshot& snapshot,
                           uint8_t* output, size_t output_size) {
    if (snapshot.count > PROFILE_SERVICE_LIST_CAPACITY) {
        return 0;
    }
    uint8_t payload[kProfileListPayloadSize]{};
    payload[0] = snapshot.count;
    size_t offset = 1;
    for (uint8_t index = 0; index < snapshot.count; ++index) {
        if (!controller_identity_encode(snapshot.rows[index].identity,
                                        &payload[offset],
                                        CONTROLLER_IDENTITY_ENCODED_SIZE) ||
            snapshot.rows[index].active_profile >=
                CONTROLLER_PROFILE_COUNT) {
            return 0;
        }
        payload[offset + 14] = snapshot.rows[index].active_profile;
        offset += 16;
    }
    return encode_response(
        Operation::kProfileList, profile_service_status(snapshot.metadata),
        0, CONTROLLER_PROFILE_SCHEMA_VERSION, snapshot.metadata.generation,
        payload, offset, output, output_size);
}

size_t encode_profile_read(const ProfileServiceSelectedSnapshot& snapshot,
                           uint8_t* output, size_t output_size) {
    Status status = profile_service_status(snapshot.metadata);
    if (status == Status::kOk) {
        status = transaction_status(snapshot.status);
    }
    uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    size_t payload_size = 0;
    if (snapshot.valid) {
        if (!controller_profile_encode(snapshot.profile, payload,
                                       sizeof(payload))) {
            return 0;
        }
        payload_size = sizeof(payload);
    }
    return encode_response(
        Operation::kProfileRead, status, 0,
        CONTROLLER_PROFILE_SCHEMA_VERSION, snapshot.metadata.generation,
        payload, payload_size, output, output_size);
}

size_t encode_profile_transaction(
    const ProfileServiceTransactionSnapshot& snapshot,
    uint8_t* output, size_t output_size) {
    const ConfigurationTransactionSnapshot& transaction =
        snapshot.transaction;
    uint8_t payload[20]{};
    write_u32(&payload[0], transaction.transaction_id);
    write_u16(&payload[4], transaction.received_size);
    write_u16(&payload[6], transaction.expected_size);
    write_u32(&payload[8], transaction.expected_crc);
    write_u32(&payload[12], transaction.stored_generation);
    write_u32(&payload[16], transaction.stored_crc);
    Status status = profile_service_status(snapshot.metadata);
    if (status == Status::kOk) {
        status = transaction_status(transaction.status);
    }
    return encode_response(
        Operation::kProfileTransactionStatus, status, 0,
        CONTROLLER_PROFILE_SCHEMA_VERSION, snapshot.metadata.generation,
        payload, sizeof(payload), output, output_size);
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
        case Operation::kModeSet: {
            const uint32_t transaction_id =
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24);
            const AdapterRequestedMode requested_mode =
                static_cast<AdapterRequestedMode>(payload[4]);
            if (transaction_id == 0 ||
                (transaction_id &
                 CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0 ||
                !adapter_requested_mode_valid(requested_mode)) {
                return false;
            }
            const ConfigurationTransactionStatus status =
                configuration_service_set_mode(
                    transaction_id, requested_mode,
                    adapter_usb_mode_availability());
            return status == ConfigurationTransactionStatus::kPending ||
                   status == ConfigurationTransactionStatus::kCommitted ||
                   status == ConfigurationTransactionStatus::kUnchanged;
        }
        case Operation::kReboot: {
            const uint32_t transaction_id =
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24);
            if (transaction_id == 0 ||
                (transaction_id &
                 CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
                return false;
            }
            return adapter_reboot_for_mode_transaction(transaction_id);
        }
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
        case Operation::kProfileSelect: {
            ControllerIdentity identity{};
            if (payload[14] >= CONTROLLER_PROFILE_COUNT ||
                !controller_identity_decode(
                    payload, CONTROLLER_IDENTITY_ENCODED_SIZE, &identity)) {
                return false;
            }
            const ConfigurationTransactionStatus status =
                profile_service_select(identity, payload[14]);
            return status == ConfigurationTransactionStatus::kCommitted ||
                   status == ConfigurationTransactionStatus::kPending;
        }
        case Operation::kProfileBegin: {
            ControllerIdentity identity{};
            if (payload[19] != 0 ||
                !controller_identity_decode(
                    &payload[4], CONTROLLER_IDENTITY_ENCODED_SIZE,
                    &identity)) {
                return false;
            }
            profile_service_begin(
                static_cast<uint32_t>(payload[0]) |
                    (static_cast<uint32_t>(payload[1]) << 8) |
                    (static_cast<uint32_t>(payload[2]) << 16) |
                    (static_cast<uint32_t>(payload[3]) << 24),
                identity, payload[18],
                static_cast<uint16_t>(payload[20] |
                                      (payload[21] << 8)),
                static_cast<uint16_t>(payload[22] |
                                      (payload[23] << 8)),
                static_cast<uint32_t>(payload[24]) |
                    (static_cast<uint32_t>(payload[25]) << 8) |
                    (static_cast<uint32_t>(payload[26]) << 16) |
                    (static_cast<uint32_t>(payload[27]) << 24));
            return true;
        }
        case Operation::kProfileChunk: {
            const uint16_t chunk_size =
                static_cast<uint16_t>(payload[6] |
                                      (payload[7] << 8));
            if (request.payload_size != 8 + chunk_size ||
                chunk_size > kMaximumChunkSize) {
                return false;
            }
            profile_service_append(
                static_cast<uint32_t>(payload[0]) |
                    (static_cast<uint32_t>(payload[1]) << 8) |
                    (static_cast<uint32_t>(payload[2]) << 16) |
                    (static_cast<uint32_t>(payload[3]) << 24),
                static_cast<uint16_t>(payload[4] |
                                      (payload[5] << 8)),
                &payload[8], chunk_size);
            return true;
        }
        case Operation::kProfileCommit:
            profile_service_commit(
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24));
            return true;
        case Operation::kProfileReset:
        case Operation::kProfileActivate: {
            const uint32_t transaction_id =
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24);
            ControllerIdentity identity{};
            if (transaction_id == 0 ||
                (request.operation == Operation::kProfileActivate &&
                 payload[18] >= CONTROLLER_PROFILE_COUNT) ||
                (request.operation == Operation::kProfileReset &&
                 payload[18] != CONTROLLER_PROFILE_ALL &&
                 payload[18] >= CONTROLLER_PROFILE_COUNT) ||
                !controller_identity_decode(
                    &payload[4], CONTROLLER_IDENTITY_ENCODED_SIZE,
                    &identity)) {
                return false;
            }
            const ConfigurationTransactionStatus status =
                request.operation == Operation::kProfileReset
                    ? profile_service_reset(
                          transaction_id, identity, payload[18])
                    : profile_service_activate(
                          transaction_id, identity, payload[18]);
            return status == ConfigurationTransactionStatus::kPending ||
                   status == ConfigurationTransactionStatus::kUnchanged ||
                   status == ConfigurationTransactionStatus::kCommitted;
        }
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

bool usb_configuration_management_vendor_control(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
    if (adapter_host_probe_vendor_control(rhport, stage, request)) {
        return true;
    }
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
        case Operation::kProfileList: {
            ProfileServiceListSnapshot snapshot{};
            profile_service_list_snapshot(&snapshot);
            response_size = encode_profile_list(
                snapshot, response, sizeof(response));
            break;
        }
        case Operation::kProfileRead: {
            ProfileServiceSelectedSnapshot snapshot{};
            profile_service_selected_snapshot(&snapshot);
            response_size = encode_profile_read(
                snapshot, response, sizeof(response));
            break;
        }
        case Operation::kProfileTransactionStatus: {
            ProfileServiceTransactionSnapshot snapshot{};
            profile_service_transaction_snapshot(&snapshot);
            response_size = encode_profile_transaction(
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
