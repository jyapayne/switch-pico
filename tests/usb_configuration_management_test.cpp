#include "usb/usb_configuration_management.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include <tusb.h>
#include "usb/usb_output_driver.h"
#include "input/haptics_experiment.h"
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
#include "input/haptics_transport_probe.h"
#endif

namespace {

Bluepad32PairingSnapshot current_pairings{};
ConfigurationServiceSnapshot current_configuration{};
ProfileServiceListSnapshot current_profile_list{};
ProfileServiceSelectedSnapshot current_profile_selected{};
ProfileServiceTransactionSnapshot current_profile_transaction{};
ProfileServiceMetadataSnapshot current_profile_metadata{};
Bluepad32PlaytestSnapshot current_playtest[
    BLUEPAD32_INPUT_BACKEND_SLOT_COUNT]{};
AdapterUsbMode current_active_mode = AdapterUsbMode::kSwitchProbe;
uint8_t current_capabilities =
    USB_OUTPUT_CAPABILITY_INPUT | USB_OUTPUT_CAPABILITY_RUMBLE |
    USB_OUTPUT_CAPABILITY_MOTION;
ConfigurationTransactionStatus mode_set_result =
    ConfigurationTransactionStatus::kPending;
uint32_t mode_set_transaction_id = 0;
AdapterRequestedMode mode_set_requested_mode = AdapterRequestedMode::kAuto;
AdapterModeAvailability mode_set_availability{};
AdapterModeAvailability runtime_mode_availability{true, true, true, true};
uint32_t mode_availability_query_count = 0;
uint32_t mode_set_call_count = 0;
uint32_t correlated_reboot_transaction_id = 0;
uint32_t reboot_transaction_id = 0;
uint32_t reboot_call_count = 0;
bool bootsel_reboot_requested = false;
bool refresh_requested = false;
bool clear_requested = false;
Bluepad32BackendDiagnostics current_diagnostics{};
std::vector<uint8_t> control_payload;
std::vector<uint8_t> next_out_payload;
uint32_t begin_transaction_id = 0;
uint32_t append_transaction_id = 0;
uint32_t commit_transaction_id = 0;
size_t append_offset = 0;
std::vector<uint8_t> appended_bytes;
ControllerIdentity profile_identity{};
uint8_t profile_index = 0;
uint16_t profile_schema = 0;
size_t profile_size = 0;
uint32_t profile_crc = 0;
bool profile_reset_requested = false;
uint32_t profile_reset_transaction_id = 0;
uint32_t profile_commit_transaction_id = 0;
bool profile_activate_requested = false;
uint32_t profile_activate_transaction_id = 0;
bool profile_metadata_requested = false;
uint32_t profile_metadata_transaction_id = 0;
uint8_t profile_metadata_index = 0;
std::string profile_metadata_value;
bool identify_requested = false;
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
HapticsExperimentDiagnostics current_haptics{};
uint32_t haptics_request_count = 0;
HapticsTransportProbe current_transport{};
#endif

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void write_u16(std::vector<uint8_t>* output, size_t offset,
               uint16_t value) {
    (*output)[offset] = static_cast<uint8_t>(value);
    (*output)[offset + 1] = static_cast<uint8_t>(value >> 8);
}

uint16_t read_u16(const std::vector<uint8_t>& input, size_t offset) {
    return static_cast<uint16_t>(input[offset]) |
           static_cast<uint16_t>(input[offset + 1] << 8);
}

void write_u32(std::vector<uint8_t>* output, size_t offset,
               uint32_t value) {
    (*output)[offset] = static_cast<uint8_t>(value);
    (*output)[offset + 1] = static_cast<uint8_t>(value >> 8);
    (*output)[offset + 2] = static_cast<uint8_t>(value >> 16);
    (*output)[offset + 3] = static_cast<uint8_t>(value >> 24);
}

uint32_t read_u32(const std::vector<uint8_t>& input, size_t offset) {
    return static_cast<uint32_t>(input[offset]) |
           (static_cast<uint32_t>(input[offset + 1]) << 8) |
           (static_cast<uint32_t>(input[offset + 2]) << 16) |
           (static_cast<uint32_t>(input[offset + 3]) << 24);
}

std::vector<uint8_t> make_request(
    UsbConfigurationManagement::Operation operation,
    const std::vector<uint8_t>& payload) {
    using namespace UsbConfigurationManagement;
    std::vector<uint8_t> request(kRequestHeaderSize + payload.size());
    memcpy(request.data(), "SPMG", 4);
    request[4] = kProtocolVersion;
    request[5] = static_cast<uint8_t>(operation);
    write_u16(&request, 8, static_cast<uint16_t>(payload.size()));
    write_u32(&request, 12,
              configuration_crc32(payload.data(), payload.size()));
    memcpy(request.data() + kRequestHeaderSize, payload.data(),
           payload.size());
    return request;
}

tusb_control_request_t setup_request(
    UsbConfigurationManagement::Operation operation, uint8_t direction,
    uint16_t length) {
    tusb_control_request_t request{};
    request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_DEVICE;
    request.bmRequestType_bit.type = TUSB_REQ_TYPE_VENDOR;
    request.bmRequestType_bit.direction = direction;
    request.bRequest = static_cast<uint8_t>(operation);
    request.wValue = UsbConfigurationManagement::kRequestValue;
    request.wIndex = UsbConfigurationManagement::kRequestIndex;
    request.wLength = length;
    return request;
}

void test_envelope_encoding() {
    using namespace UsbConfigurationManagement;
    const uint8_t payload[] = {1, 2, 3};
    uint8_t encoded[32]{};
    const size_t size = encode_response(
        Operation::kConfigurationRead, Status::kOk, 5, 1,
        0x78563412, payload, sizeof(payload), encoded, sizeof(encoded));
    require(size == kResponseHeaderSize + sizeof(payload) &&
                memcmp(encoded, "SPMG", 4) == 0 &&
                encoded[4] == kProtocolVersion &&
                encoded[5] ==
                    static_cast<uint8_t>(Operation::kConfigurationRead) &&
                encoded[6] == static_cast<uint8_t>(Status::kOk) &&
                encoded[7] == 5 && encoded[8] == 3 &&
                encoded[10] == 1 && encoded[12] == 0x12 &&
                encoded[15] == 0x78 &&
                memcmp(&encoded[kResponseHeaderSize], payload,
                       sizeof(payload)) == 0,
            "versioned response envelope encoded incorrectly");
    require(encode_response(
                Operation::kConfigurationRead, Status::kOk, 0, 1, 0,
                payload, sizeof(payload), encoded, size - 1) == 0,
            "response encoder accepted a short destination");
}

void test_pairing_encoding() {
    using namespace UsbConfigurationManagement;
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

    uint8_t payload[kMaximumResponseSize]{};
    const size_t size =
        encode_pairing_snapshot(snapshot, payload, sizeof(payload));
    require(size == kResponseHeaderSize + kPairingPayloadHeaderSize +
                        2 * kPairingRecordSize &&
                payload[5] ==
                    static_cast<uint8_t>(Operation::kPairingRead) &&
                payload[7] == 1 && payload[12] == 0x12 &&
                payload[kResponseHeaderSize] == 2 &&
                payload[kResponseHeaderSize + 1] == 1 &&
                payload[kResponseHeaderSize + 4] == 1 &&
                payload[kResponseHeaderSize + 5] == 0xfe &&
                memcmp(&payload[kResponseHeaderSize + 6],
                       classic_address, 6) == 0,
            "pairings were not migrated into the versioned envelope");
    snapshot.status = Bluepad32PairingSnapshotStatus::kFailed;
    require(encode_pairing_snapshot(snapshot, payload, sizeof(payload)) == size &&
                payload[6] == static_cast<uint8_t>(Status::kStorageError),
            "failed persistent pairing clear must report storage failure, not success or pending");
}

void perform_out(UsbConfigurationManagement::Operation operation,
                 const std::vector<uint8_t>& payload,
                 bool expected_ack = true) {
    next_out_payload = make_request(operation, payload);
    tusb_control_request_t request = setup_request(
        operation, TUSB_DIR_OUT,
        static_cast<uint16_t>(next_out_payload.size()));
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "valid OUT setup was rejected");
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_ACK, &request) == expected_ack,
            "OUT acknowledgement result was incorrect");
}

void test_vendor_requests() {
    using namespace UsbConfigurationManagement;
    current_pairings = {};
    current_pairings.generation = 7;
    current_pairings.status = Bluepad32PairingSnapshotStatus::kReady;
    current_pairings.record_count = 1;
    current_pairings.records[0].transport =
        Bluepad32PairingTransport::kClassic;

    tusb_control_request_t request = setup_request(
        Operation::kPairingRead, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload[5] ==
                    static_cast<uint8_t>(Operation::kPairingRead) &&
                control_payload[12] == 7,
            "pairing read did not use the versioned envelope");

    current_diagnostics = {
        6, 1200, 120, 5000, 8, 2, 10, 2, 2, 1, 1, 3, UINT32_MAX,
    };
    request = setup_request(
        Operation::kRuntimeDiagnostics, TUSB_DIR_IN,
        kMaximumResponseSize);
    require(
        usb_configuration_management_vendor_control(
            0, CONTROL_STAGE_SETUP, &request) &&
            control_payload[5] ==
                static_cast<uint8_t>(Operation::kRuntimeDiagnostics) &&
            read_u32(control_payload, kResponseHeaderSize) == 6 &&
            read_u32(control_payload, kResponseHeaderSize + 4) == 1200 &&
            control_payload[kResponseHeaderSize + 28] == 2 &&
            control_payload[kResponseHeaderSize + 31] == 1 &&
            read_u32(control_payload, kResponseHeaderSize + 32) == 3 &&
            read_u32(control_payload, kResponseHeaderSize + 36) == UINT32_MAX,
        "runtime diagnostics did not expose backend counters");

    perform_out(Operation::kPairingRefresh, {});
    require(refresh_requested,
            "pairing refresh was not dispatched");
    perform_out(Operation::kBootselReboot, {});
    require(bootsel_reboot_requested,
            "BOOTSEL reboot request was not dispatched");
    perform_out(Operation::kPairingClear, {});
    require(clear_requested, "pairing clear was not dispatched");

    std::vector<uint8_t> begin(12);
    write_u32(&begin, 0, 0x11223344);
    write_u16(&begin, 4, ADAPTER_CONFIGURATION_SCHEMA_VERSION);
    write_u16(&begin, 6, ADAPTER_CONFIGURATION_ENCODED_SIZE);
    write_u32(&begin, 8, 0xaabbccdd);
    perform_out(Operation::kConfigurationBegin, begin);
    require(begin_transaction_id == 0x11223344,
            "configuration begin was not dispatched");

    std::vector<uint8_t> chunk(12);
    write_u32(&chunk, 0, 0x11223344);
    write_u16(&chunk, 4, 0);
    write_u16(&chunk, 6, 4);
    chunk[8] = 60;
    perform_out(Operation::kConfigurationChunk, chunk);
    require(append_transaction_id == 0x11223344 &&
                append_offset == 0 && appended_bytes.size() == 4,
            "configuration chunk was not dispatched");

    std::vector<uint8_t> commit(4);
    write_u32(&commit, 0, 0x11223344);
    perform_out(Operation::kConfigurationCommit, commit);
    require(commit_transaction_id == 0x11223344,
            "configuration commit was not dispatched");

    next_out_payload =
        make_request(Operation::kPairingRefresh, {});
    next_out_payload[12] ^= 1;
    request = setup_request(
        Operation::kPairingRefresh, TUSB_DIR_OUT,
        static_cast<uint16_t>(next_out_payload.size()));
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                !usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_ACK, &request),
            "bad request CRC was accepted");

    request.wValue = 0;
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "request with invalid magic was accepted");
}

void test_mode_vendor_requests() {
    using namespace UsbConfigurationManagement;
    current_configuration.configuration.requested_mode =
        AdapterRequestedMode::kXInput;
    current_configuration.configuration.joycon_mode = JoyConMode::kIndividual;
    current_active_mode = AdapterUsbMode::kSwitchProbe;

    tusb_control_request_t request =
        setup_request(Operation::kInfo, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload[kResponseHeaderSize + 4] ==
                    static_cast<uint8_t>(AdapterUsbMode::kSwitchProbe) &&
                control_payload[kResponseHeaderSize + 5] ==
                    current_capabilities,
            "info response did not report active mode capabilities");

    current_active_mode = AdapterUsbMode::kDInput;
    current_capabilities = USB_OUTPUT_CAPABILITY_INPUT;
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload[kResponseHeaderSize + 4] ==
                    static_cast<uint8_t>(AdapterUsbMode::kDInput) &&
                control_payload[kResponseHeaderSize + 5] ==
                    USB_OUTPUT_CAPABILITY_INPUT,
            "generic info response promised unsupported output capabilities");
    current_active_mode = AdapterUsbMode::kSwitchProbe;
    current_capabilities =
        USB_OUTPUT_CAPABILITY_INPUT | USB_OUTPUT_CAPABILITY_RUMBLE |
        USB_OUTPUT_CAPABILITY_MOTION;

    request = setup_request(
        Operation::kConfigurationRead, TUSB_DIR_IN,
        kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() ==
                    kResponseHeaderSize +
                        ADAPTER_CONFIGURATION_ENCODED_SIZE &&
                control_payload[10] ==
                    ADAPTER_CONFIGURATION_SCHEMA_VERSION &&
                control_payload[kResponseHeaderSize + 2] ==
                    static_cast<uint8_t>(AdapterRequestedMode::kXInput) &&
                control_payload[kResponseHeaderSize + 4] ==
                    static_cast<uint8_t>(JoyConMode::kIndividual),
            "configuration response did not report the saved USB and player modes");

    std::vector<uint8_t> mode_set(5);
    write_u32(&mode_set, 0, 0x12345678);
    mode_set[4] =
        static_cast<uint8_t>(AdapterRequestedMode::kXInput);
    const std::vector<uint8_t> encoded =
        make_request(Operation::kModeSet, mode_set);
    require(encoded.size() == kRequestHeaderSize + 5 &&
                encoded[5] ==
                    static_cast<uint8_t>(Operation::kModeSet) &&
                encoded[8] == 5 &&
                read_u32(encoded, kRequestHeaderSize) == 0x12345678 &&
                encoded[kRequestHeaderSize + 4] ==
                    static_cast<uint8_t>(
                        AdapterRequestedMode::kXInput),
            "mode-set request envelope does not match the protocol");

    mode_set_result = ConfigurationTransactionStatus::kPending;
    perform_out(Operation::kModeSet, mode_set);
    require(mode_set_transaction_id == 0x12345678 &&
                mode_set_requested_mode ==
                    AdapterRequestedMode::kXInput &&
                mode_set_availability.switch_mode &&
                mode_set_availability.xinput_mode &&
                mode_set_availability.dinput_mode &&
                mode_set_availability.mac_mode &&
                mode_availability_query_count == 1,
            "XInput mode set did not use runtime availability");
    write_u32(&mode_set, 0, 0x12345679);
    mode_set[4] = static_cast<uint8_t>(AdapterRequestedMode::kSwitch);
    perform_out(Operation::kModeSet, mode_set);
    require(mode_set_requested_mode == AdapterRequestedMode::kSwitch &&
                mode_availability_query_count == 2,
            "Switch mode set did not use runtime availability");

    mode_set_result = ConfigurationTransactionStatus::kBusy;
    write_u32(&mode_set, 0, 0x1234567a);
    perform_out(Operation::kModeSet, mode_set, false);
    mode_set_result = ConfigurationTransactionStatus::kStorageError;
    write_u32(&mode_set, 0, 0x1234567b);
    perform_out(Operation::kModeSet, mode_set, false);

    const uint32_t calls_before_invalid = mode_set_call_count;
    const uint32_t availability_queries_before_invalid =
        mode_availability_query_count;
    for (const uint32_t transaction_id : {0u, 0x80000000u}) {
        write_u32(&mode_set, 0, transaction_id);
        perform_out(Operation::kModeSet, mode_set, false);
    }
    write_u32(&mode_set, 0, 0x1234567c);
    mode_set[4] = 0xff;
    perform_out(Operation::kModeSet, mode_set, false);
    require(mode_set_call_count == calls_before_invalid &&
                mode_availability_query_count ==
                    availability_queries_before_invalid,
            "malformed mode request reached runtime mode selection");

    mode_set_result = ConfigurationTransactionStatus::kPending;
    for (const AdapterRequestedMode generic_mode : {
             AdapterRequestedMode::kDInput,
             AdapterRequestedMode::kMac,
         }) {
        write_u32(&mode_set, 0,
                  0x12345680u +
                      static_cast<uint8_t>(generic_mode));
        mode_set[4] = static_cast<uint8_t>(generic_mode);
        perform_out(Operation::kModeSet, mode_set);
        require(mode_set_requested_mode == generic_mode,
                "generic mode request was not dispatched");
    }
    require(mode_set_call_count == calls_before_invalid + 2 &&
                mode_availability_query_count ==
                    availability_queries_before_invalid + 2,
            "generic modes did not use runtime availability");

    request = setup_request(
        Operation::kModeSet, TUSB_DIR_OUT, kRequestHeaderSize + 4);
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "short mode-set request was accepted");

    std::vector<uint8_t> reboot(4);
    write_u32(&reboot, 0, 0x12345678);
    const std::vector<uint8_t> encoded_reboot =
        make_request(Operation::kReboot, reboot);
    require(encoded_reboot.size() == kRequestHeaderSize + 4 &&
                encoded_reboot[5] ==
                    static_cast<uint8_t>(Operation::kReboot) &&
                encoded_reboot[8] == 4 &&
                read_u32(encoded_reboot, kRequestHeaderSize) ==
                    0x12345678,
            "reboot request envelope does not match the protocol");

    correlated_reboot_transaction_id = 0x12345678;
    perform_out(Operation::kReboot, reboot);
    require(reboot_transaction_id == correlated_reboot_transaction_id,
            "correlated reboot request was not dispatched");
    write_u32(&reboot, 0, 0x12345679);
    perform_out(Operation::kReboot, reboot, false);
    const uint32_t reboot_calls_before_invalid = reboot_call_count;
    for (const uint32_t transaction_id : {0u, 0x80000000u}) {
        write_u32(&reboot, 0, transaction_id);
        perform_out(Operation::kReboot, reboot, false);
    }
    request = setup_request(
        Operation::kReboot, TUSB_DIR_OUT, kRequestHeaderSize + 5);
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "oversized reboot request was accepted");
    require(reboot_call_count == reboot_calls_before_invalid,
            "malformed reboot transaction reached the helper");
}


void test_profile_vendor_requests() {
    using namespace UsbConfigurationManagement;
    ControllerIdentity expected_identity{};
    expected_identity.stable = true;
    expected_identity.transport = ControllerTransport::kClassic;
    expected_identity.address[5] = 7;
    expected_identity.vendor_id = 0x057e;
    expected_identity.product_id = 0x2009;

    current_profile_list = {};
    current_profile_list.metadata.state = ProfileServiceState::kReady;
    current_profile_list.metadata.generation = 9;
    current_profile_list.count = 2;
    current_profile_list.rows[0].identity = controller_identity_global();
    current_profile_list.rows[1].identity = expected_identity;
    current_profile_list.rows[1].active_profile = 2;
    tusb_control_request_t request = setup_request(
        Operation::kProfileList, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() == kResponseHeaderSize + 97 &&
                control_payload[5] ==
                    static_cast<uint8_t>(Operation::kProfileList) &&
                control_payload[10] == CONTROLLER_PROFILE_SCHEMA_VERSION &&
                control_payload[kResponseHeaderSize] == 2 &&
                control_payload[kResponseHeaderSize + 63] == 2,
            "profile list response was not encoded");

    current_profile_selected = {};
    current_profile_selected.metadata.state =
        ProfileServiceState::kReady;
    current_profile_selected.metadata.generation = 9;
    current_profile_selected.valid = true;
    current_profile_selected.status =
        ConfigurationTransactionStatus::kCommitted;
    current_profile_selected.identity = expected_identity;
    current_profile_selected.profile_index = 2;
    current_profile_selected.profile =
        controller_profile_default(expected_identity, 2);
    request = setup_request(
        Operation::kProfileRead, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() ==
                    kResponseHeaderSize +
                        CONTROLLER_PROFILE_ENCODED_SIZE &&
                control_payload[10] ==
                    CONTROLLER_PROFILE_SCHEMA_VERSION &&
                control_payload[kResponseHeaderSize] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_SCHEMA_VERSION) &&
                control_payload[kResponseHeaderSize + 1] == 0 &&
                control_payload[kResponseHeaderSize + 2] == 0x80 &&
                control_payload[kResponseHeaderSize + 3] == 1,
            "selected profile response was not encoded");

    current_playtest[2] = {};
    current_playtest[2].active = true;
    current_playtest[2].connection_generation = 0x11223344;
    current_playtest[2].state_generation = 0x55667788;
    current_playtest[2].identity = expected_identity;
    current_playtest[2].physical_button_mask = 0x8001;
    current_playtest[2].state.extra_buttons = 0x7f;
    current_playtest[2].controller_layout =
        Bluepad32ControllerLayout::kJoyCon2MergedPair;
    current_playtest[2].state.left_stick_x = -1234;
    current_playtest[2].state.left_stick_y = 2345;
    current_playtest[2].state.right_stick_x = INT16_MIN;
    current_playtest[2].state.right_stick_y = INT16_MAX;
    current_playtest[2].state.left_trigger = 123;
    current_playtest[2].state.right_trigger = 65000;
    current_playtest[2].battery = 201;
    current_playtest[2].capabilities = 0x0f;
    current_playtest[2].state.motion_sample_count = 1;
    current_playtest[2].state.motion_samples[0] =
        {1, -2, 3, -4, 5, -6};
    request = setup_request(
        Operation::kProfilePlaytest, TUSB_DIR_IN,
        kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() ==
                    kResponseHeaderSize + kProfilePlaytestPayloadSize &&
                control_payload[5] ==
                    static_cast<uint8_t>(Operation::kProfilePlaytest) &&
                control_payload[7] == 3 &&
                control_payload[10] ==
                    kProfilePlaytestSchemaVersion &&
                control_payload[kResponseHeaderSize] == 3 &&
                control_payload[kResponseHeaderSize + 1] == 2 &&
                read_u16(control_payload, kResponseHeaderSize + 2) ==
                    0x8001 &&
                read_u32(control_payload, kResponseHeaderSize + 4) ==
                    0x11223344 &&
                read_u32(control_payload, kResponseHeaderSize + 8) ==
                    0x55667788 &&
                static_cast<int16_t>(read_u16(
                    control_payload, kResponseHeaderSize + 26)) ==
                    -1234 &&
                read_u16(control_payload, kResponseHeaderSize + 36) ==
                    65000 &&
                static_cast<int16_t>(read_u16(
                    control_payload, kResponseHeaderSize + 52)) == -6 &&
                control_payload[kResponseHeaderSize + 39] == 201 &&
                control_payload[kResponseHeaderSize + 40] == 0x0f &&
                control_payload[kResponseHeaderSize + 54] == 0x7f &&
                control_payload[kResponseHeaderSize + 55] == 3,
            "profile playtest response lost live controller state");
    current_playtest[2].controller_layout =
        static_cast<Bluepad32ControllerLayout>(6);
    uint8_t invalid_playtest[kMaximumResponseSize]{};
    require(encode_profile_playtest(
                2, current_playtest[2], invalid_playtest, sizeof(invalid_playtest)) == 0,
            "unknown playtest layout metadata must be rejected");
    current_playtest[2].controller_layout =
        Bluepad32ControllerLayout::kJoyCon2MergedPair;
    current_playtest[2].active = false;
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload[kResponseHeaderSize] == 0 &&
                control_payload[kResponseHeaderSize + 1] == 0xff &&
                control_payload[kResponseHeaderSize + 54] == 0 &&
                control_payload[kResponseHeaderSize + 55] == 0,
            "disconnected profile playtest was not encoded");
    current_profile_metadata = {};
    current_profile_metadata.metadata.state = ProfileServiceState::kReady;
    current_profile_metadata.metadata.generation = 10;
    current_profile_metadata.status =
        ConfigurationTransactionStatus::kCommitted;
    current_profile_metadata.valid = true;
    memcpy(current_profile_metadata.alias, "Desk pad", 9);
    memcpy(current_profile_metadata.profile_names[7], "Desktop", 8);
    request = setup_request(
        Operation::kProfileMetadataRead, TUSB_DIR_IN,
        kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() ==
                    kResponseHeaderSize + kProfileMetadataPayloadSize &&
                control_payload[kResponseHeaderSize] == 8 &&
                memcmp(&control_payload[kResponseHeaderSize + 1],
                       "Desk pad", 8) == 0 &&
                control_payload[
                    kResponseHeaderSize +
                    8 * (PROFILE_SERVICE_METADATA_MAX_BYTES + 1)] == 7,
            "profile metadata response was not encoded");

    std::vector<uint8_t> metadata(27);
    write_u32(&metadata, 0, 0x12345678);
    require(controller_identity_encode(
                expected_identity, &metadata[4],
                CONTROLLER_IDENTITY_ENCODED_SIZE),
            "metadata identity did not encode");
    metadata[18] = 7;
    metadata[19] = 7;
    memcpy(&metadata[20], "Desktop", 7);
    perform_out(Operation::kProfileMetadataSet, metadata);
    require(profile_metadata_requested &&
                profile_metadata_transaction_id == 0x12345678 &&
                profile_metadata_index == 7 &&
                profile_metadata_value == "Desktop",
            "profile metadata mutation was not dispatched");

    std::vector<uint8_t> identify(CONTROLLER_IDENTITY_ENCODED_SIZE);
    require(controller_identity_encode(
                expected_identity, identify.data(), identify.size()),
            "identify identity did not encode");
    perform_out(Operation::kProfileIdentify, identify);
    require(identify_requested,
            "controller Identify request was not dispatched");

    current_profile_transaction = {};
    current_profile_transaction.metadata.state =
        ProfileServiceState::kReady;
    current_profile_transaction.metadata.generation = 9;
    current_profile_transaction.transaction.transaction_id = 0x01020304;
    current_profile_transaction.transaction.status =
        ConfigurationTransactionStatus::kPending;
    request = setup_request(
        Operation::kProfileTransactionStatus, TUSB_DIR_IN,
        kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload.size() == kResponseHeaderSize + 20 &&
                control_payload[6] ==
                    static_cast<uint8_t>(Status::kPending) &&
                read_u32(control_payload, kResponseHeaderSize) ==
                    0x01020304,
            "pending profile transaction status lost its transaction ID");

    current_profile_transaction.transaction.status =
        ConfigurationTransactionStatus::kCommitted;
    current_profile_transaction.transaction.stored_generation =
        0x11223344;
    current_profile_transaction.transaction.stored_crc = 0xaabbccdd;
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                control_payload[6] == static_cast<uint8_t>(Status::kOk) &&
                read_u32(control_payload, kResponseHeaderSize) ==
                    0x01020304 &&
                read_u32(control_payload, kResponseHeaderSize + 12) ==
                    0x11223344 &&
                read_u32(control_payload, kResponseHeaderSize + 16) ==
                    0xaabbccdd,
            "final profile transaction status lost its commit result");

    std::vector<uint8_t> identity_payload(15);
    require(controller_identity_encode(
                expected_identity, identity_payload.data(),
                CONTROLLER_IDENTITY_ENCODED_SIZE),
            "profile test identity did not encode");
    identity_payload[14] = 2;
    perform_out(Operation::kProfileSelect, identity_payload);
    require(controller_identity_equal(expected_identity,
                                      profile_identity) &&
                profile_index == 2,
            "profile selection was not dispatched");

    std::vector<uint8_t> begin(28);
    write_u32(&begin, 0, 0x55667788);
    require(controller_identity_encode(
                expected_identity, &begin[4],
                CONTROLLER_IDENTITY_ENCODED_SIZE),
            "profile begin identity did not encode");
    begin[18] = 1;
    write_u16(&begin, 20, CONTROLLER_PROFILE_SCHEMA_VERSION);
    write_u16(&begin, 22, CONTROLLER_PROFILE_ENCODED_SIZE);
    write_u32(&begin, 24, 0xaabbccdd);
    perform_out(Operation::kProfileBegin, begin);
    require(begin_transaction_id == 0x55667788 &&
                profile_index == 1 &&
                profile_schema == CONTROLLER_PROFILE_SCHEMA_VERSION &&
                profile_size == CONTROLLER_PROFILE_ENCODED_SIZE &&
                profile_crc == 0xaabbccdd,
            "profile begin was not dispatched");

    std::vector<uint8_t> chunk(48);
    write_u32(&chunk, 0, 0x55667788);
    write_u16(&chunk, 4, 0);
    write_u16(&chunk, 6, 40);
    perform_out(Operation::kProfileChunk, chunk);
    require(append_transaction_id == 0x55667788 &&
                append_offset == 0 && appended_bytes.size() == 40,
            "profile chunk was not dispatched");

    std::vector<uint8_t> commit(4);
    write_u32(&commit, 0, 0x55667788);
    perform_out(Operation::kProfileCommit, commit);
    require(profile_commit_transaction_id == 0x55667788,
            "profile commit was not dispatched");

    std::vector<uint8_t> mutation(19);
    write_u32(&mutation, 0, 0x10203040);
    require(controller_identity_encode(
                expected_identity, &mutation[4],
                CONTROLLER_IDENTITY_ENCODED_SIZE),
            "profile mutation identity did not encode");
    mutation[18] = CONTROLLER_PROFILE_ALL;
    perform_out(Operation::kProfileReset, mutation);
    require(profile_reset_requested &&
                profile_reset_transaction_id == 0x10203040,
            "profile reset transaction was not dispatched");
    write_u32(&mutation, 0, 0x50607080);
    mutation[18] = 3;
    perform_out(Operation::kProfileActivate, mutation);
    require(profile_activate_requested && profile_index == 3 &&
                profile_activate_transaction_id == 0x50607080,
            "profile activation transaction was not dispatched");

    write_u32(&mutation, 0, 0);
    perform_out(Operation::kProfileActivate, mutation, false);
    request = setup_request(
        Operation::kProfileReset, TUSB_DIR_OUT, kRequestHeaderSize + 15);
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "legacy profile reset payload was accepted");

    begin[19] = 1;
    perform_out(Operation::kProfileBegin, begin, false);
    request = setup_request(
        Operation::kProfileSelect, TUSB_DIR_OUT,
        kRequestHeaderSize + 14);
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "short profile selection request was accepted");
}

std::vector<uint8_t> read_haptics_payload() {
    using namespace UsbConfigurationManagement;
    tusb_control_request_t request = setup_request(
        Operation::kHapticsExperiment, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "experiment diagnostics IN was rejected");
    require(control_payload.size() == kResponseHeaderSize + 84 &&
                control_payload[5] == 0x40 &&
                control_payload[6] == static_cast<uint8_t>(Status::kOk) &&
                control_payload[7] == 0 &&
                read_u16(control_payload, 8) == 84 &&
                read_u16(control_payload, 10) == 5,
            "experiment schema-5 envelope is invalid");
    std::vector<uint8_t> payload(
        control_payload.begin() + kResponseHeaderSize, control_payload.end());
    require(read_u32(control_payload, 16) ==
                configuration_crc32(payload.data(), payload.size()),
            "experiment response CRC is invalid");
    require(payload[71] == 0 && (payload[73] == 32 || payload[73] == 64) &&
                payload[74] <= 1 && payload[75] == 0,
            "experiment reserved payload bytes must remain zero");
    return payload;
}

#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
void perform_haptics_out(uint8_t action, uint8_t slot, bool accepted = true) {
    using namespace UsbConfigurationManagement;
    next_out_payload = make_request(Operation::kHapticsExperiment, {action, slot});
    tusb_control_request_t request = setup_request(
        Operation::kHapticsExperiment, TUSB_DIR_OUT,
        static_cast<uint16_t>(next_out_payload.size()));
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "experiment OUT setup was rejected");
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_DATA, &request) == accepted,
            "experiment must reject invalid or busy requests before status ACK");
    const uint32_t requests_after_data = haptics_request_count;
    if (accepted) {
        require(!usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_DATA, &request) &&
                    haptics_request_count == requests_after_data,
                "duplicate DATA replayed experiment control");
    }
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_ACK, &request) == accepted,
            "experiment ACK did not preserve DATA-stage result");
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_ACK, &request) &&
                haptics_request_count == requests_after_data,
            "ACK replayed experiment control");
}
#endif

void test_haptics_experiment_requests() {
    using namespace UsbConfigurationManagement;
    for (uint16_t payload_size : {0, 1, 3}) {
        tusb_control_request_t request = setup_request(
            Operation::kHapticsExperiment, TUSB_DIR_OUT,
            kRequestHeaderSize + payload_size);
        require(!usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_SETUP, &request),
                "malformed experiment control size was accepted");
    }
    std::vector<uint8_t> expected(84, 0);
    expected[69] = 0xff;
    expected[73] = 64;
#ifndef SWITCH_PICO_HAPTICS_EXPERIMENT
    expected[68] = 6;
    require(read_haptics_payload() == expected,
            "disabled firmware must expose only the unsupported snapshot");
    for (uint8_t action : {0, 1, 2}) {
        next_out_payload = make_request(Operation::kHapticsExperiment, {action, 0});
        tusb_control_request_t request = setup_request(
            Operation::kHapticsExperiment, TUSB_DIR_OUT,
            static_cast<uint16_t>(next_out_payload.size()));
        require(!usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_SETUP, &request),
                "disabled firmware accepted experiment control");
    }
#else
    perform_haptics_out(3, 0, false);
    perform_haptics_out(1, 4, false);
    perform_haptics_out(0, 0xff, false);
    require(haptics_request_count == 0,
            "malformed control reached the experiment service");

    next_out_payload = make_request(Operation::kHapticsExperiment, {1, 2});
    next_out_payload.back() ^= 1;
    tusb_control_request_t request = setup_request(
        Operation::kHapticsExperiment, TUSB_DIR_OUT,
        static_cast<uint16_t>(next_out_payload.size()));
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request) &&
                !usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_DATA, &request),
            "bad experiment request CRC was accepted");
    require(haptics_request_count == 0,
            "bad CRC control reached the experiment service");

    perform_haptics_out(2, 2);
    auto payload = read_haptics_payload();
    require(payload[68] == 1 && payload[69] == 2 && payload[72] == 1 &&
                read_u32(payload, 0) == 1 && read_u32(payload, 16) == 0,
            "USB acceptance must remain pending until the service starts");
    perform_haptics_out(1, 1, false);
    perform_haptics_out(2, 1, false);
    payload = read_haptics_payload();
    require(payload[68] == 1 && payload[69] == 2 && payload[72] == 1 &&
                read_u32(payload, 0) == 1,
            "busy start overwrote the accepted run");

    // Model the independently progressing Core 1 service, not a USB echo.
    current_haptics = {
        1, 0x11223344, 0xffff0000, 103, 101, 2, 3, 106, 4,
        123, 22000, 11001, 9876, 0xfffffff0, 0x30, 0x76543210,
        1100000, HapticsExperimentState::kRunning, 2, 0, 1, 0x89abcdef, 0x12345678,
    };
    const uint32_t fields[] = {
        1, 0x11223344, 0xffff0000, 103, 101, 2, 3, 106, 4,
        123, 22000, 11001, 9876, 0xfffffff0, 0x30, 0x76543210, 1100000,
    };
    for (size_t index = 0; index < 17; ++index) {
        write_u32(&expected, index * 4, fields[index]);
    }
    expected[68] = 2;
    expected[69] = 2;
    expected[72] = 1;
    write_u32(&expected, 76, 0x89abcdef);
    write_u32(&expected, 80, 0x12345678);
    require(read_haptics_payload() == expected,
            "schema-3 timing and gameplay mode fields are not in wire order");

    perform_haptics_out(0, 2);
    payload = read_haptics_payload();
    require(payload[68] == 2 && payload[72] == 1 && read_u32(payload, 0) == 1,
            "USB stop ACK must not fabricate terminal completion");
    current_haptics.state = HapticsExperimentState::kStopped;
    payload = read_haptics_payload();
    require(payload[68] == 4 && payload[72] == 1,
            "service stop transition was not observable");

    next_out_payload = make_request(Operation::kHapticsExperiment, {1, 3});
    request = setup_request(
        Operation::kHapticsExperiment, TUSB_DIR_OUT,
        static_cast<uint16_t>(next_out_payload.size()));
    const uint32_t requests_before_cancel = haptics_request_count;
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "canceled experiment setup was rejected");
    read_haptics_payload();  // A new SETUP cancels the previous OUT transfer.
    require(!usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_DATA, &request) &&
                !usb_configuration_management_vendor_control(
                    0, CONTROL_STAGE_ACK, &request) &&
                haptics_request_count == requests_before_cancel,
            "canceled experiment request reused stale payload");

    perform_haptics_out(1, 3);
    current_haptics.state = HapticsExperimentState::kDisconnected;
    current_haptics.last_error = 3;
    payload = read_haptics_payload();
    require(payload[68] == 5 && payload[69] == 3 && payload[70] == 3 &&
                read_u32(payload, 0) == 2 && payload[72] == 0,
            "asynchronous connection failure lost request correlation");
#endif
}

void test_haptics_transport_probe_requests() {
    using namespace UsbConfigurationManagement;
    for (uint16_t length : {0, 16, 18, 120}) {
        tusb_control_request_t request = setup_request(
            Operation::kHapticsTransportProbe, TUSB_DIR_OUT, length);
        for (uint8_t stage : {
                 CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA, CONTROL_STAGE_ACK}) {
            require(!usb_configuration_management_vendor_control(
                        0, stage, &request),
                    "IN-only transport probe accepted an OUT transfer");
        }
    }
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    current_transport.run_id = 0x10203040;
    current_transport.connection_generation = 0x50607080;
    current_transport.connection_handle = 0xffff;
    current_transport.timer_wakes = 4;
    current_transport.max_timer_lateness_us = 5;
    current_transport.total_timer_lateness_us = 6;
    current_transport.send_calls = 7;
    current_transport.max_send_us = 8;
    current_transport.total_send_us = 9;
    current_transport.write_calls = 10;
    current_transport.max_write_us = 11;
    current_transport.total_write_us = 12;
    current_transport.read_calls = 13;
    current_transport.read_packets = 14;
    current_transport.max_read_us = 15;
    current_transport.total_read_us = 16;
    current_transport.poll_calls = 17;
    current_transport.max_poll_us = 18;
    current_transport.total_poll_us = 19;
    current_transport.completion_events = 20;
    current_transport.completed_packets = 21;
    current_transport.max_completion_gap_us = 22;
    current_transport.max_outstanding_acl = 23;
    current_transport.min_free_acl = 24;
    current_transport.first_tone_send_return_us = 0xfffffff0;
    current_transport.active = 1;
    current_transport.max_permission_wait_us = 27;
    current_transport.total_permission_wait_us = 0xffffffff;
    current_transport.permission_callbacks = 29;
    current_transport.max_poll_gap_us = 30;
    current_transport.controller_acl_packet_bytes = 1021;
    current_transport.controller_acl_packet_count = 10;
#endif
    tusb_control_request_t request = setup_request(
        Operation::kHapticsTransportProbe, TUSB_DIR_IN, kMaximumResponseSize);
    require(usb_configuration_management_vendor_control(
                0, CONTROL_STAGE_SETUP, &request),
            "transport probe IN was rejected");
    require(control_payload.size() >= kResponseHeaderSize,
            "transport probe response header is truncated");
    require(control_payload[5] == 0x41 && control_payload[7] == 0 &&
                read_u16(control_payload, 10) == 3,
            "transport probe operation, flags, or schema are invalid");
    const std::vector<uint8_t> payload(
        control_payload.begin() + kResponseHeaderSize, control_payload.end());
    require(read_u32(control_payload, 16) ==
                configuration_crc32(payload.data(), payload.size()),
            "transport probe response CRC is invalid");
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    require(control_payload.size() == kResponseHeaderSize + 176 &&
                read_u16(control_payload, 8) == 176 &&
                control_payload[6] == static_cast<uint8_t>(Status::kOk) &&
                read_u32(control_payload, 12) == 0x10203040,
            "transport probe schema-3 envelope is invalid");
    const uint32_t fields[] = {
        0x10203040, 0x50607080, 0xffff, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
        0xfffffff0, 1, 27, 0xffffffff, 29, 30,
        1021, 10,
    };
    std::vector<uint8_t> expected(176);
    for (size_t index = 0; index < 32; ++index) {
        write_u32(&expected, index * 4, fields[index]);
    }
    require(payload == expected,
            "transport probe fields are not in explicit little-endian wire order");
#else
    require(control_payload.size() == kResponseHeaderSize &&
                read_u16(control_payload, 8) == 0 &&
                read_u32(control_payload, 12) == 0 &&
                control_payload[6] ==
                    static_cast<uint8_t>(Status::kUnsupportedSchema),
            "disabled firmware must report the transport probe as unsupported");
#endif
}

}  // namespace

uint32_t configuration_crc32(const uint8_t* data, size_t size) {
    uint32_t crc = 0xffffffffu;
    for (size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return ~crc;
}

void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    *output = current_configuration;
}

ConfigurationTransactionStatus configuration_service_begin(
    uint32_t transaction_id, uint16_t, size_t, uint32_t) {
    begin_transaction_id = transaction_id;
    return ConfigurationTransactionStatus::kReceiving;
}

ConfigurationTransactionStatus configuration_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
    append_transaction_id = transaction_id;
    append_offset = offset;
    appended_bytes.assign(data, data + size);
    return ConfigurationTransactionStatus::kReceiving;
}

ConfigurationTransactionStatus configuration_service_commit(
    uint32_t transaction_id) {
    commit_transaction_id = transaction_id;
    return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus configuration_service_reset(uint32_t) {
    return ConfigurationTransactionStatus::kPending;
}

const AdapterModeAvailability& adapter_usb_mode_availability() {
    ++mode_availability_query_count;
    return runtime_mode_availability;
}

ConfigurationTransactionStatus configuration_service_set_mode(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    mode_set_transaction_id = transaction_id;
    mode_set_requested_mode = requested_mode;
    mode_set_availability = availability;
    ++mode_set_call_count;
    if (!adapter_requested_mode_available(requested_mode, availability)) {
        return ConfigurationTransactionStatus::kUnsupportedSchema;
    }
    return mode_set_result;
}

AdapterUsbMode usb_output_driver_mode() { return current_active_mode; }
uint8_t usb_output_driver_capabilities() {
    return current_capabilities;
}

bool adapter_reboot_for_mode_transaction(uint32_t transaction_id) {
    reboot_transaction_id = transaction_id;
    ++reboot_call_count;
    return transaction_id == correlated_reboot_transaction_id;
}

ConfigurationTransactionStatus profile_service_select(
    const ControllerIdentity& identity, uint8_t selected_profile) {
    profile_identity = identity;
    profile_index = selected_profile;
    return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus profile_service_set_metadata(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t selected_profile, const char* value, size_t value_size) {
    profile_metadata_requested = true;
    profile_metadata_transaction_id = transaction_id;
    profile_identity = identity;
    profile_metadata_index = selected_profile;
    profile_metadata_value.assign(value, value + value_size);
    return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus profile_service_begin(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t selected_profile, uint16_t schema_version,
    size_t payload_size, uint32_t payload_crc) {
    begin_transaction_id = transaction_id;
    profile_identity = identity;
    profile_index = selected_profile;
    profile_schema = schema_version;
    profile_size = payload_size;
    profile_crc = payload_crc;
    return ConfigurationTransactionStatus::kReceiving;
}

ConfigurationTransactionStatus profile_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
    append_transaction_id = transaction_id;
    append_offset = offset;
    appended_bytes.assign(data, data + size);
    return ConfigurationTransactionStatus::kReceiving;
}

ConfigurationTransactionStatus profile_service_commit(
    uint32_t transaction_id) {
    profile_commit_transaction_id = transaction_id;
    return ConfigurationTransactionStatus::kPending;
}

void profile_service_metadata_snapshot(
    ProfileServiceMetadataSnapshot* output) {
    *output = current_profile_metadata;
}

bool bluepad32_input_backend_identify(
    const ControllerIdentity&) {
    identify_requested = true;
    return true;
}

ConfigurationTransactionStatus profile_service_reset(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t selected_profile) {
    profile_reset_transaction_id = transaction_id;
    profile_identity = identity;
    profile_index = selected_profile;
    profile_reset_requested = true;
    return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus profile_service_activate(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t selected_profile) {
    profile_activate_transaction_id = transaction_id;
    profile_identity = identity;
    profile_index = selected_profile;
    profile_activate_requested = true;
    return ConfigurationTransactionStatus::kPending;
}

void profile_service_list_snapshot(ProfileServiceListSnapshot* output) {
    *output = current_profile_list;
}

void profile_service_selected_snapshot(
    ProfileServiceSelectedSnapshot* output) {
    *output = current_profile_selected;
}

void profile_service_transaction_snapshot(
    ProfileServiceTransactionSnapshot* output) {
    *output = current_profile_transaction;
}

namespace {
ControllerMacroCapture capture_fixture;
}

bool bluepad32_input_backend_capture_start(
    uint8_t slot, uint32_t generation, const CaptureOptions& options) {
    return capture_fixture.start(slot, generation, options, 0,
                                 controller_neutral_state());
}

bool bluepad32_input_backend_capture_stop(uint32_t run_id) {
    if (run_id == 0 || run_id != capture_fixture.run_id()) return false;
    capture_fixture.stop(100000);
    return true;
}

bool bluepad32_input_backend_capture_page(
    uint32_t run_id, uint16_t first, Bluepad32CaptureSnapshot* output) {
    if (output == nullptr || (run_id && run_id != capture_fixture.run_id()) ||
        first > capture_fixture.event_count()) return false;
    *output = {};
    output->run_id = capture_fixture.run_id();
    output->connection_generation = capture_fixture.generation();
    output->elapsed_us = capture_fixture.elapsed_us(100000);
    output->slot = capture_fixture.slot();
    output->state = capture_fixture.state();
    output->options = capture_fixture.options();
    output->total_events = capture_fixture.event_count();
    output->first_index = first;
    while (output->event_count < BLUEPAD32_CAPTURE_PAGE_EVENTS &&
           capture_fixture.event(first + output->event_count,
                                 &output->events[output->event_count])) {
        ++output->event_count;
    }
    return true;
}

void bluepad32_input_backend_request_pairing_snapshot() {
    refresh_requested = true;
}

uint32_t bluepad32_input_backend_clear_pairings() {
    clear_requested = true;
    return 1;
}

void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out) {
    *out = current_pairings;
}

void bluepad32_input_backend_playtest_snapshot(
    uint8_t slot, Bluepad32PlaytestSnapshot* out) {
    *out = current_playtest[slot];
}

void bluepad32_input_backend_diagnostics(
    Bluepad32BackendDiagnostics* out) {
    *out = current_diagnostics;
}

#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
bool haptics_experiment_request(uint8_t action, uint8_t slot) {
    ++haptics_request_count;
    if ((action == 1 || action == 2) &&
        (current_haptics.state == HapticsExperimentState::kPending ||
         current_haptics.state == HapticsExperimentState::kRunning)) {
        return false;
    }
    if (action == 1 || action == 2) {
        const uint32_t run_id = current_haptics.run_id + 1;
        current_haptics = {};
        current_haptics.run_id = run_id;
        current_haptics.slot = slot;
        current_haptics.state = HapticsExperimentState::kPending;
        current_haptics.mode = action == 2 ? 1 : 0;
    }
    return true;
}

void haptics_experiment_snapshot(HapticsExperimentDiagnostics* output) {
    *output = current_haptics;
}

void haptics_transport_probe_snapshot(HapticsTransportProbe* output) {
    *output = current_transport;
}
#endif

bool adapter_reboot_to_bootsel() {
    bootsel_reboot_requested = true;
    return true;
}

bool adapter_host_probe_vendor_control(
    uint8_t, uint8_t, const tusb_control_request_t*) {
    return false;
}

bool tud_control_xfer(uint8_t, const tusb_control_request_t* request,
                      void* buffer, uint16_t length) {
    if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        if (next_out_payload.size() != length) {
            return false;
        }
        memcpy(buffer, next_out_payload.data(), length);
    } else {
        const auto* bytes = static_cast<const uint8_t*>(buffer);
        control_payload.assign(bytes, bytes + length);
    }
    return true;
}

bool tud_control_status(uint8_t, const tusb_control_request_t*) {
    return true;
}

#include "configuration/adapter_configuration.cpp"
#include "core/controller_identity.cpp"
#include "profile/controller_profile.cpp"
#include "usb/usb_configuration_management.cpp"

int main() {
    current_configuration.state = ConfigurationServiceState::kReady;
    current_configuration.configuration =
        adapter_configuration_default();
    test_envelope_encoding();
    test_pairing_encoding();
    test_vendor_requests();
    test_mode_vendor_requests();
    test_profile_vendor_requests();
    test_haptics_experiment_requests();
    test_haptics_transport_probe_requests();
    return 0;
}
