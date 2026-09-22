#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tusb.h"

#include "input/bluepad32_input_backend.h"
#include "configuration/configuration_service.h"
#include "profile/profile_service.h"
#include "usb/usb_management_protocol.h"

namespace UsbConfigurationManagement {

// INFO-only active mode; never persisted as an AdapterRequestedMode.
constexpr uint8_t kNativeHubActiveMode = 5;
constexpr uint16_t kSwitch2WakeSchemaVersion = 1;
constexpr size_t kSwitch2WakePayloadSize = 20;
constexpr size_t kPairingRecordSize = 8;
constexpr size_t kPairingPayloadHeaderSize = 4;
constexpr size_t kMaximumRequestSize = 80;
constexpr size_t kProfileListRowSize =
    16 + PROFILE_SERVICE_METADATA_MAX_BYTES + 1;
constexpr size_t kProfileListPayloadSize =
    1 + PROFILE_SERVICE_LIST_CAPACITY * kProfileListRowSize;
constexpr uint16_t kProfilePlaytestSchemaVersion = 5;
constexpr size_t kProfilePlaytestPayloadSize = 56;
constexpr size_t kProfileMetadataPayloadSize =
    (CONTROLLER_PROFILE_COUNT + 1) *
    (PROFILE_SERVICE_METADATA_MAX_BYTES + 1);
constexpr uint16_t kProfileMetadataSchemaVersion = 1;
constexpr uint16_t kHapticsExperimentSchemaVersion = 5;
constexpr size_t kHapticsExperimentPayloadSize = 84;
constexpr uint16_t kHapticsTransportProbeSchemaVersion = 3;
constexpr size_t kHapticsTransportProbePayloadSize = 176;
constexpr uint16_t kMacroCaptureSchemaVersion = 1;
constexpr size_t kMacroCaptureHeaderSize = 32;
constexpr size_t kMacroCaptureEventSize = 20;
constexpr uint16_t kNativeSwitchRumbleSchemaVersion = 2;
constexpr size_t kNativeSwitchRumbleRowSize = 80;
constexpr size_t kNativeSwitchRumblePayloadSize = 4 * kNativeSwitchRumbleRowSize;
constexpr uint16_t kSwitch2MouseCaptureSchemaVersion = 1;
constexpr size_t kMaximumResponseSize =
    kResponseHeaderSize + kProfileListPayloadSize;
constexpr size_t kMaximumChunkSize =
    kMaximumRequestSize - kRequestHeaderSize - 8;
static_assert(kMaximumResponseSize == 837,
              "profile list no longer fits the EP0 response buffer");

struct DecodedRequest {
    Operation operation = Operation::kInfo;
    const uint8_t* payload = nullptr;
    uint16_t payload_size = 0;
};

bool decode_request(Operation setup_operation, const uint8_t* input,
                    size_t input_size, DecodedRequest* output);
// Payload may already occupy output + kResponseHeaderSize for in-place encoding.
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
size_t encode_profile_playtest(
    uint8_t slot, const Bluepad32PlaytestSnapshot& snapshot,
    uint8_t* output, size_t output_size);
size_t encode_profile_transaction(
    const ProfileServiceTransactionSnapshot& snapshot,
    uint8_t* output, size_t output_size);
size_t encode_profile_metadata(
    const ProfileServiceMetadataSnapshot& snapshot,
    uint8_t* output, size_t output_size);

}  // namespace UsbConfigurationManagement

bool usb_configuration_management_vendor_control(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request);

#if SWITCH2_PROBE_HUB && !SWITCH2_PROBE_NEUTRAL_INPUT
// Native child Interface 1 only: read-only INFO and volatile WAKE. Each child
// owns its EP0 state independently from the root's full management service.
bool usb_configuration_management_child_vendor_control(
    uint8_t rhport, uint8_t stage,
    const tusb_control_request_t* request);
#endif
