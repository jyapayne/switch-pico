#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tusb.h"

#include "bluepad32_input_backend.h"
#include "configuration_service.h"
#include "profile_service.h"

namespace UsbConfigurationManagement {

constexpr uint16_t kRequestValue = 0x5350;
constexpr uint16_t kRequestIndex = 0x0001;
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kRequestHeaderSize = 16;
constexpr size_t kResponseHeaderSize = 20;
constexpr size_t kPairingRecordSize = 8;
constexpr size_t kPairingPayloadHeaderSize = 4;
constexpr size_t kMaximumRequestSize = 64;
constexpr size_t kProfileListPayloadSize =
    1 + PROFILE_SERVICE_LIST_CAPACITY * 16;
constexpr size_t kMaximumResponseSize =
    kResponseHeaderSize + kProfileListPayloadSize;
constexpr size_t kMaximumChunkSize =
    kMaximumRequestSize - kRequestHeaderSize - 8;
static_assert(kMaximumResponseSize == 293,
              "profile list no longer fits the EP0 response buffer");

enum class Operation : uint8_t {
    kInfo = 0x01,
    kConfigurationRead = 0x10,
    kConfigurationBegin = 0x11,
    kConfigurationChunk = 0x12,
    kConfigurationCommit = 0x13,
    kConfigurationReset = 0x14,
    kTransactionStatus = 0x15,
    kPairingRead = 0x20,
    kPairingRefresh = 0x21,
    kPairingClear = 0x22,
    kProfileList = 0x30,
    kProfileSelect = 0x31,
    kProfileRead = 0x32,
    kProfileBegin = 0x33,
    kProfileChunk = 0x34,
    kProfileCommit = 0x35,
    kProfileReset = 0x36,
    kProfileActivate = 0x37,
    kProfileTransactionStatus = 0x38,
};

enum class Status : uint8_t {
    kOk = 0,
    kPending = 1,
    kMalformed = 2,
    kUnsupportedSchema = 3,
    kTooLarge = 4,
    kOutOfOrder = 5,
    kBadCrc = 6,
    kBusy = 7,
    kStorageError = 8,
};

struct DecodedRequest {
    Operation operation = Operation::kInfo;
    const uint8_t* payload = nullptr;
    uint16_t payload_size = 0;
};

bool decode_request(Operation setup_operation, const uint8_t* input,
                    size_t input_size, DecodedRequest* output);
size_t encode_response(Operation operation, Status status, uint8_t flags,
                       uint16_t schema_version, uint32_t generation,
                       const uint8_t* payload, size_t payload_size,
                       uint8_t* output, size_t output_size);
size_t encode_pairing_snapshot(const Bluepad32PairingSnapshot& snapshot,
                               uint8_t* output, size_t output_size);
size_t encode_profile_list(const ProfileServiceListSnapshot& snapshot,
                           uint8_t* output, size_t output_size);
size_t encode_profile_read(const ProfileServiceSelectedSnapshot& snapshot,
                           uint8_t* output, size_t output_size);
size_t encode_profile_transaction(
    const ProfileServiceTransactionSnapshot& snapshot,
    uint8_t* output, size_t output_size);

}  // namespace UsbConfigurationManagement

bool usb_configuration_management_vendor_control(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request);
