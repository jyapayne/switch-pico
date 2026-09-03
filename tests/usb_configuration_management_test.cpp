#include "usb/usb_configuration_management.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include <tusb.h>
#include "usb/usb_output_driver.h"

namespace {

Bluepad32PairingSnapshot current_pairings{};
ConfigurationServiceSnapshot current_configuration{};
ProfileServiceListSnapshot current_profile_list{};
ProfileServiceSelectedSnapshot current_profile_selected{};
ProfileServiceTransactionSnapshot current_profile_transaction{};
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
        6, 1200, 120, 5000, 8, 2, 10, 2, 2, 1, 1,
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
            control_payload[kResponseHeaderSize + 31] == 1,
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
                    static_cast<uint8_t>(AdapterRequestedMode::kXInput),
            "configuration response did not keep requested mode separate");

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
                control_payload.size() == kResponseHeaderSize + 33 &&
                control_payload[5] ==
                    static_cast<uint8_t>(Operation::kProfileList) &&
                control_payload[10] == CONTROLLER_PROFILE_SCHEMA_VERSION &&
                control_payload[kResponseHeaderSize] == 2 &&
                control_payload[kResponseHeaderSize + 31] == 2,
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
                control_payload[kResponseHeaderSize + 2] == 0 &&
                control_payload[kResponseHeaderSize + 3] == 1,
            "selected profile response was not encoded");

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

void bluepad32_input_backend_diagnostics(
    Bluepad32BackendDiagnostics* out) {
    *out = current_diagnostics;
}

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
    return 0;
}
