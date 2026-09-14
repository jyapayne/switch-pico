#include "usb/usb_configuration_management.h"
#include <string.h>

#include "configuration/adapter_configuration.h"
#include "adapter/adapter_host_probe.h"
#include "adapter/adapter_reboot.h"
#include "adapter/adapter_usb_mode.h"
#include "input/haptics_experiment.h"
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
#include "input/switch_native_output.h"
#endif
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
#include "input/haptics_transport_probe.h"
#endif
#ifdef SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
#include "input/switch2_mouse_capture.h"
#endif
#ifdef SWITCH_PICO_WII_IR_GYRO
#include "input/wii_ir_pointer.h"
#endif
#include "tusb.h"
#include "usb/usb_output_driver.h"
#if SWITCH2_PROBE_HUB
#include "usb/native_hub/native_hub.h"
#endif

namespace UsbConfigurationManagement {
namespace {

uint32_t g_capture_read_run = 0;
uint16_t g_capture_read_index = 0;

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
        case Operation::kMacroCapture:
            return size == kRequestHeaderSize + 5 ||
                   size == kRequestHeaderSize + 7 ||
                   size == kRequestHeaderSize + 16;
        case Operation::kModeSet:
            return size == kRequestHeaderSize + 5;
        case Operation::kReboot:
            return size == kRequestHeaderSize + 4;
        case Operation::kBootselReboot:
            return size == kRequestHeaderSize;
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
        case Operation::kProfileMetadataSet:
            return size >= kRequestHeaderSize + 20 &&
                   size <= kRequestHeaderSize + 20 +
                               PROFILE_SERVICE_METADATA_MAX_BYTES;
        case Operation::kProfileIdentify:
            return size == kRequestHeaderSize +
                               CONTROLLER_IDENTITY_ENCODED_SIZE;
        case Operation::kWiiOrientation:
            return size == kRequestHeaderSize +
                               CONTROLLER_IDENTITY_ENCODED_SIZE + 5;
        case Operation::kPairingRefresh:
        case Operation::kPairingClear:
            return size == kRequestHeaderSize;
        case Operation::kHapticsExperiment:
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
            return size == kRequestHeaderSize + 2;
#else
            return false;
#endif
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
#if SWITCH2_PROBE_HUB
        0, 78, 0, 2,
        kNativeHubActiveMode,
        USB_OUTPUT_CAPABILITY_INPUT | USB_OUTPUT_CAPABILITY_RUMBLE |
            USB_OUTPUT_CAPABILITY_MOTION,
#else
        0, 2, 0, 2,
        static_cast<uint8_t>(usb_output_driver_mode()),
        usb_output_driver_capabilities(),
#endif
        static_cast<uint8_t>(CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE),
        static_cast<uint8_t>(
            CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE >> 8),
    };
    return encode_response(Operation::kInfo, Status::kOk, 0, 0, 0,
                           payload, sizeof(payload), output, output_size);
}

size_t encode_runtime_diagnostics(uint8_t* output, size_t output_size) {
    Bluepad32BackendDiagnostics diagnostics{};
    bluepad32_input_backend_diagnostics(&diagnostics);
    uint8_t payload[40]{};
    write_u32(&payload[0], diagnostics.initialization_stage);
    write_u32(&payload[4], diagnostics.rumble_timer_ticks);
    write_u32(&payload[8], diagnostics.configuration_timer_ticks);
    write_u32(&payload[12], diagnostics.controller_reports);
    write_u32(&payload[16], diagnostics.host_rumble_requests);
    write_u32(&payload[20], diagnostics.local_feedback_requests);
    write_u32(&payload[24], diagnostics.rumble_dispatches);
    payload[28] = diagnostics.active_slots;
    payload[29] = diagnostics.rumble_capable_slots;
    payload[30] = diagnostics.feedback_pending_slots;
    payload[31] = diagnostics.rumble_pending_slots;
    write_u32(&payload[32], diagnostics.switch2_ingress_drops);
    write_u32(&payload[36], diagnostics.switch2_output_drops);
    return encode_response(Operation::kRuntimeDiagnostics, Status::kOk,
                           0, 0, 0, payload, sizeof(payload), output,
                           output_size);
}

size_t encode_haptics_experiment(uint8_t* output, size_t output_size) {
    HapticsExperimentDiagnostics diagnostics{};
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    haptics_experiment_snapshot(&diagnostics);
#else
    diagnostics.state = HapticsExperimentState::kUnsupported;
#endif
    uint8_t payload[kHapticsExperimentPayloadSize]{};
    write_u32(&payload[0], diagnostics.run_id);
    write_u32(&payload[4], diagnostics.connection_generation);
    write_u32(&payload[8], diagnostics.start_us);
    write_u32(&payload[12], diagnostics.generated_packets);
    write_u32(&payload[16], diagnostics.sent_packets);
    write_u32(&payload[20], diagnostics.skipped_packets);
    write_u32(&payload[24], diagnostics.send_failures);
    write_u32(&payload[28], diagnostics.can_send_requests);
    write_u32(&payload[32], diagnostics.synchronous_callbacks);
    write_u32(&payload[36], diagnostics.max_generate_us);
    write_u32(&payload[40], diagnostics.max_send_gap_us);
    write_u32(&payload[44], diagnostics.max_lateness_us);
    write_u32(&payload[48], diagnostics.max_request_wait_us);
    write_u32(&payload[52], diagnostics.first_tone_due_us);
    write_u32(&payload[56], diagnostics.first_tone_sent_us);
    write_u32(&payload[60], diagnostics.last_sent_us);
    write_u32(&payload[64], diagnostics.elapsed_us);
    payload[68] = static_cast<uint8_t>(diagnostics.state);
    payload[69] = diagnostics.slot;
    payload[70] = diagnostics.last_error;
    payload[72] = diagnostics.mode;
    write_u32(&payload[76], diagnostics.host_updates);
    write_u32(&payload[80], diagnostics.dropped_updates);
    payload[73] = diagnostics.packet_frames;
    payload[74] = diagnostics.last_packet_nonzero ? 1 : 0;
    return encode_response(
        Operation::kHapticsExperiment, Status::kOk, 0,
        kHapticsExperimentSchemaVersion, diagnostics.run_id,
        payload, sizeof(payload), output, output_size);
}

size_t encode_macro_capture(uint8_t* output, size_t output_size) {
    Bluepad32CaptureSnapshot capture{};
    if (!bluepad32_input_backend_capture_page(
            g_capture_read_run, g_capture_read_index, &capture)) {
        return encode_response(Operation::kMacroCapture, Status::kMalformed, 0,
                               kMacroCaptureSchemaVersion, 0,
                               nullptr, 0, output, output_size);
    }
    uint8_t payload[kMacroCaptureHeaderSize +
                    BLUEPAD32_CAPTURE_PAGE_EVENTS * kMacroCaptureEventSize]{};
    write_u32(payload, capture.run_id);
    write_u32(payload + 4, capture.connection_generation);
    write_u32(payload + 8, capture.elapsed_us);
    payload[12] = capture.slot;
    payload[13] = static_cast<uint8_t>(capture.state);
    payload[14] = capture.options.channels;
    payload[15] = capture.event_count;
    write_u16(payload + 16, capture.total_events);
    write_u16(payload + 18, capture.first_index);
    write_u16(payload + 20, capture.options.axis_quantum);
    write_u16(payload + 22, capture.options.trigger_quantum);
    write_u32(payload + 24, capture.options.max_duration_ms);
    payload[28] = capture.options.max_events;
    for (uint8_t index = 0; index < capture.event_count; ++index) {
        const CaptureEvent& event = capture.events[index];
        uint8_t* encoded = payload + kMacroCaptureHeaderSize +
                           index * kMacroCaptureEventSize;
        write_u32(encoded, event.at_us);
        write_u16(encoded + 4, event.buttons);
        write_u16(encoded + 6, static_cast<uint16_t>(event.left_x));
        write_u16(encoded + 8, static_cast<uint16_t>(event.left_y));
        write_u16(encoded + 10, static_cast<uint16_t>(event.right_x));
        write_u16(encoded + 12, static_cast<uint16_t>(event.right_y));
        write_u16(encoded + 14, event.left_trigger);
        write_u16(encoded + 16, event.right_trigger);
    }
    return encode_response(
        Operation::kMacroCapture, Status::kOk, 0, kMacroCaptureSchemaVersion,
        capture.run_id, payload,
        kMacroCaptureHeaderSize + capture.event_count * kMacroCaptureEventSize,
        output, output_size);
}

size_t encode_haptics_transport_probe(uint8_t* output, size_t output_size) {
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
    HapticsTransportProbe probe{};
    haptics_transport_probe_snapshot(&probe);
    uint8_t payload[kHapticsTransportProbePayloadSize]{};
    write_u32(&payload[0], probe.run_id);
    write_u32(&payload[4], probe.connection_generation);
    write_u32(&payload[8], probe.connection_handle);
    write_u32(&payload[12], probe.timer_wakes);
    write_u32(&payload[16], probe.max_timer_lateness_us);
    write_u32(&payload[20], probe.total_timer_lateness_us);
    write_u32(&payload[24], probe.send_calls);
    write_u32(&payload[28], probe.max_send_us);
    write_u32(&payload[32], probe.total_send_us);
    write_u32(&payload[36], probe.write_calls);
    write_u32(&payload[40], probe.max_write_us);
    write_u32(&payload[44], probe.total_write_us);
    write_u32(&payload[48], probe.read_calls);
    write_u32(&payload[52], probe.read_packets);
    write_u32(&payload[56], probe.max_read_us);
    write_u32(&payload[60], probe.total_read_us);
    write_u32(&payload[64], probe.poll_calls);
    write_u32(&payload[68], probe.max_poll_us);
    write_u32(&payload[72], probe.total_poll_us);
    write_u32(&payload[76], probe.completion_events);
    write_u32(&payload[80], probe.completed_packets);
    write_u32(&payload[84], probe.max_completion_gap_us);
    write_u32(&payload[88], probe.max_outstanding_acl);
    write_u32(&payload[92], probe.min_free_acl);
    write_u32(&payload[96], probe.first_tone_send_return_us);
    write_u32(&payload[100], probe.active);
    write_u32(&payload[104], probe.max_permission_wait_us);
    write_u32(&payload[108], probe.total_permission_wait_us);
    write_u32(&payload[112], probe.permission_callbacks);
    write_u32(&payload[116], probe.max_poll_gap_us);
    write_u32(&payload[120], probe.controller_acl_packet_bytes);
    write_u32(&payload[124], probe.controller_acl_packet_count);
    write_u32(&payload[128], probe.requested_sys_khz);
    write_u32(&payload[132], probe.measured_sys_khz);
    write_u32(&payload[136], probe.measured_usb_khz);
    write_u32(&payload[140], probe.core_voltage_mv);
    write_u32(&payload[144], probe.flash_clock_divider);
    write_u32(&payload[148], probe.cyw43_pio_divider256);
    write_u32(&payload[152], static_cast<uint32_t>(probe.temperature_millicelsius));
    write_u32(&payload[156], probe.host_completed_writes);
    write_u32(&payload[160], probe.acl_writes);
    write_u32(&payload[164], probe.other_writes);
    write_u32(&payload[168], probe.write_failures);
    write_u32(&payload[172], probe.packet_read_optimized);
    return encode_response(
        Operation::kHapticsTransportProbe, Status::kOk, 0,
        kHapticsTransportProbeSchemaVersion, probe.run_id,
        payload, sizeof(payload), output, output_size);
#else
    return encode_response(
        Operation::kHapticsTransportProbe, Status::kUnsupportedSchema, 0,
        kHapticsTransportProbeSchemaVersion, 0,
        nullptr, 0, output, output_size);
#endif
}

size_t encode_native_switch_rumble(uint8_t* output, size_t output_size) {
#ifdef SWITCH_PICO_NATIVE_SWITCH_RUMBLE
    uint8_t payload[kNativeSwitchRumblePayloadSize]{};
    for (uint8_t slot = 0; slot < 4; ++slot) {
        SwitchNativeOutputDiagnostics snapshot{};
        switch_native_output_snapshot(slot, &snapshot);
        uint8_t* row = payload + slot * kNativeSwitchRumbleRowSize;
        row[0] = slot;
        row[1] = snapshot.type;
        row[2] = snapshot.firmware_hi;
        row[3] = snapshot.firmware_lo;
        write_u32(row + 4, snapshot.flags);
        write_u32(row + 8, snapshot.generation);
        write_u32(row + 12, snapshot.received_commands);
        write_u32(row + 16, snapshot.submitted_reports);
        write_u32(row + 20, snapshot.dropped_commands);
        write_u32(row + 24, snapshot.resynchronizations);
        write_u32(row + 28, snapshot.raw_commands);
        write_u32(row + 32, snapshot.quantized_commands);
        write_u32(row + 36, snapshot.congested_attempts);
        write_u32(row + 40, snapshot.completed_commands);
        write_u32(row + 44, snapshot.p50_upper_us);
        write_u32(row + 48, snapshot.p95_upper_us);
        write_u32(row + 52, snapshot.p99_upper_us);
        write_u32(row + 56, snapshot.max_latency_us);
        write_u32(row + 60, snapshot.queue_depth);
        memcpy(row + 64, snapshot.last_wire, 8);
        write_u32(row + 72, snapshot.max_encode_us);
        write_u32(row + 76, snapshot.coalesced_commands);
    }
    return encode_response(Operation::kNativeSwitchRumble, Status::kOk, 0,
        kNativeSwitchRumbleSchemaVersion, 0, payload, sizeof(payload), output, output_size);
#else
    return encode_response(Operation::kNativeSwitchRumble, Status::kUnsupportedSchema, 0,
        kNativeSwitchRumbleSchemaVersion, 0, nullptr, 0, output, output_size);
#endif
}

size_t encode_switch2_mouse_capture(uint8_t* output, size_t output_size) {
#ifdef SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    static_assert(kResponseHeaderSize + SWITCH2_MOUSE_CAPTURE_MAXIMUM_PAYLOAD_SIZE <=
                  kMaximumResponseSize,
                  "Switch 2 mouse capture no longer fits the EP0 response buffer");
    if (output == nullptr || output_size < kResponseHeaderSize) return 0;
    uint32_t total_records = 0;
    uint8_t* payload = output + kResponseHeaderSize;
    const size_t payload_size = switch2_mouse_capture_snapshot(
        payload, output_size - kResponseHeaderSize, &total_records);
    if (payload_size == 0) return 0;
    return encode_response(
        Operation::kSwitch2MouseCapture, Status::kOk, 0,
        kSwitch2MouseCaptureSchemaVersion, total_records,
        payload, payload_size, output, output_size);
#else
    return encode_response(
        Operation::kSwitch2MouseCapture, Status::kUnsupportedSchema, 0,
        kSwitch2MouseCaptureSchemaVersion, 0, nullptr, 0, output, output_size);
#endif
}

size_t encode_wii_ir_gyro(uint8_t* output, size_t output_size) {
#ifdef SWITCH_PICO_WII_IR_GYRO
    if (output == nullptr || output_size < kResponseHeaderSize) return 0;
    uint8_t* payload = output + kResponseHeaderSize;
    const size_t size = wii_ir_gyro_diagnostics(
        payload, output_size - kResponseHeaderSize);
    if (size == 0) return 0;
    return encode_response(Operation::kWiiIrGyro, Status::kOk, 0, 2, 0,
                           payload, size, output, output_size);
#else
    return encode_response(Operation::kWiiIrGyro, Status::kUnsupportedSchema,
                           0, 2, 0, nullptr, 0, output, output_size);
#endif
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
    if (payload_size != 0 && payload != &output[kResponseHeaderSize]) {
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
            : snapshot.status == Bluepad32PairingSnapshotStatus::kFailed
                  ? Status::kStorageError
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
        const size_t alias_size = strlen(snapshot.rows[index].alias);
        if (!controller_identity_encode(snapshot.rows[index].identity,
                                        &payload[offset],
                                        CONTROLLER_IDENTITY_ENCODED_SIZE) ||
            snapshot.rows[index].active_profile >=
                CONTROLLER_PROFILE_COUNT ||
            alias_size > PROFILE_SERVICE_METADATA_MAX_BYTES) {
            return 0;
        }
        payload[offset + 14] = snapshot.rows[index].active_profile;
        payload[offset + 16] = static_cast<uint8_t>(alias_size);
        memcpy(&payload[offset + 17], snapshot.rows[index].alias,
               alias_size);
        offset += kProfileListRowSize;
    }
    return encode_response(
        Operation::kProfileList, profile_service_status(snapshot.metadata),
        0, CONTROLLER_PROFILE_SCHEMA_VERSION, snapshot.metadata.generation,
        payload, offset, output, output_size);
}

size_t encode_profile_playtest(
    uint8_t slot, const Bluepad32PlaytestSnapshot& snapshot,
    uint8_t* output, size_t output_size) {
    uint8_t payload[kProfilePlaytestPayloadSize]{};
    payload[1] = 0xff;
    if (snapshot.active) {
        if (slot >= BLUEPAD32_INPUT_BACKEND_SLOT_COUNT ||
            !controller_identity_encode(
                snapshot.identity, &payload[12],
                CONTROLLER_IDENTITY_ENCODED_SIZE) ||
            snapshot.state.motion_sample_count >
                CONTROLLER_MOTION_SAMPLE_CAPACITY ||
            (snapshot.state.extra_buttons & 0x80u) != 0 ||
            static_cast<uint8_t>(snapshot.controller_layout) >
                static_cast<uint8_t>(Bluepad32ControllerLayout::kWiiVertical)) {
            return 0;
        }
        payload[0] = 1;
        payload[1] = slot;
        write_u16(&payload[2], snapshot.physical_button_mask);
        write_u32(&payload[4], snapshot.connection_generation);
        write_u32(&payload[8], snapshot.state_generation);
        write_u16(&payload[26],
                  static_cast<uint16_t>(snapshot.state.left_stick_x));
        write_u16(&payload[28],
                  static_cast<uint16_t>(snapshot.state.left_stick_y));
        write_u16(&payload[30],
                  static_cast<uint16_t>(snapshot.state.right_stick_x));
        write_u16(&payload[32],
                  static_cast<uint16_t>(snapshot.state.right_stick_y));
        write_u16(&payload[34], snapshot.state.left_trigger);
        write_u16(&payload[36], snapshot.state.right_trigger);
        payload[38] = snapshot.state.motion_sample_count;
        payload[39] = snapshot.battery;
        payload[40] = snapshot.capabilities;
        payload[54] = snapshot.state.extra_buttons;
        payload[55] = static_cast<uint8_t>(snapshot.controller_layout);
        if (snapshot.state.motion_sample_count != 0) {
            payload[0] |= 2;
            const ControllerMotionSample& motion =
                snapshot.state.motion_samples[
                    snapshot.state.motion_sample_count - 1u];
            write_u16(&payload[42], static_cast<uint16_t>(motion.accel_x));
            write_u16(&payload[44], static_cast<uint16_t>(motion.accel_y));
            write_u16(&payload[46], static_cast<uint16_t>(motion.accel_z));
            write_u16(&payload[48], static_cast<uint16_t>(motion.gyro_x));
            write_u16(&payload[50], static_cast<uint16_t>(motion.gyro_y));
            write_u16(&payload[52], static_cast<uint16_t>(motion.gyro_z));
        }
    }
    return encode_response(
        Operation::kProfilePlaytest, Status::kOk, payload[0],
        kProfilePlaytestSchemaVersion,
        snapshot.active ? snapshot.state_generation : 0,
        payload, sizeof(payload), output, output_size);
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

size_t encode_profile_metadata(
    const ProfileServiceMetadataSnapshot& snapshot,
    uint8_t* output, size_t output_size) {
    Status status = profile_service_status(snapshot.metadata);
    if (status == Status::kOk) {
        status = transaction_status(snapshot.status);
    }
    uint8_t payload[kProfileMetadataPayloadSize]{};
    if (snapshot.valid) {
        const char* values[CONTROLLER_PROFILE_COUNT + 1] = {
            snapshot.alias,
            snapshot.profile_names[0], snapshot.profile_names[1],
            snapshot.profile_names[2], snapshot.profile_names[3],
            snapshot.profile_names[4], snapshot.profile_names[5],
            snapshot.profile_names[6], snapshot.profile_names[7],
        };
        for (size_t index = 0;
             index < CONTROLLER_PROFILE_COUNT + 1; ++index) {
            const size_t size = strlen(values[index]);
            if (size > PROFILE_SERVICE_METADATA_MAX_BYTES) {
                return 0;
            }
            const size_t offset =
                index * (PROFILE_SERVICE_METADATA_MAX_BYTES + 1);
            payload[offset] = static_cast<uint8_t>(size);
            memcpy(&payload[offset + 1], values[index], size);
        }
    }
    return encode_response(
        Operation::kProfileMetadataRead, status, 0,
        kProfileMetadataSchemaVersion, snapshot.metadata.generation,
        payload, snapshot.valid ? sizeof(payload) : 0,
        output, output_size);
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
bool g_out_processed = false;
size_t g_pending_request_size = 0;
uint8_t g_pending_rhport = 0;
tusb_control_request_t g_pending_setup{};
#if SWITCH2_PROBE_HUB
bool g_out_validated = false;
#endif

bool management_control_xfer(uint8_t rhport,
                             const tusb_control_request_t* request,
                             void* buffer, uint16_t length) {
#if SWITCH2_PROBE_HUB
    return native_hub_control_xfer(rhport, request, buffer, length);
#else
    return tud_control_xfer(rhport, request, buffer, length);
#endif
}

bool process_out_request() {
    using namespace UsbConfigurationManagement;
    DecodedRequest request{};
    if (!decode_request(g_pending_operation, g_request_buffer,
                        g_pending_request_size, &request)) {
        return false;
    }

    const uint8_t* payload = request.payload;
    switch (request.operation) {
        case Operation::kMacroCapture: {
            if (request.payload_size == 16 && payload[0] == 1) {
                CaptureOptions options{};
                options.channels = payload[6];
                options.max_events = payload[7];
                options.axis_quantum = UsbConfigurationManagement::read_u16(payload + 8);
                options.trigger_quantum = UsbConfigurationManagement::read_u16(payload + 10);
                options.max_duration_ms = UsbConfigurationManagement::read_u32(payload + 12);
                if (!bluepad32_input_backend_capture_start(
                        payload[1], UsbConfigurationManagement::read_u32(payload + 2), options)) return false;
                g_capture_read_run = 0;
                g_capture_read_index = 0;
                return true;
            }
            if (request.payload_size == 5 && payload[0] == 0) {
                return bluepad32_input_backend_capture_stop(UsbConfigurationManagement::read_u32(payload + 1));
            }
            if (request.payload_size == 7 && payload[0] == 2) {
                const uint32_t run = UsbConfigurationManagement::read_u32(payload + 1);
                const uint16_t index = UsbConfigurationManagement::read_u16(payload + 5);
                Bluepad32CaptureSnapshot snapshot{};
                if (!bluepad32_input_backend_capture_page(run, index, &snapshot))
                    return false;
                g_capture_read_run = run;
                g_capture_read_index = index;
                return true;
            }
            return false;
        }
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
        case Operation::kBootselReboot:
            return adapter_reboot_to_bootsel();
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
        case Operation::kProfileMetadataSet: {
            const uint32_t transaction_id =
                static_cast<uint32_t>(payload[0]) |
                (static_cast<uint32_t>(payload[1]) << 8) |
                (static_cast<uint32_t>(payload[2]) << 16) |
                (static_cast<uint32_t>(payload[3]) << 24);
            ControllerIdentity identity{};
            const uint8_t profile_index = payload[18];
            const size_t value_size = payload[19];
            if (transaction_id == 0 ||
                (profile_index != CONTROLLER_PROFILE_ALL &&
                 profile_index >= CONTROLLER_PROFILE_COUNT) ||
                value_size > PROFILE_SERVICE_METADATA_MAX_BYTES ||
                request.payload_size != 20 + value_size ||
                !controller_identity_decode(
                    &payload[4], CONTROLLER_IDENTITY_ENCODED_SIZE,
                    &identity)) {
                return false;
            }
            const ConfigurationTransactionStatus status =
                profile_service_set_metadata(
                    transaction_id, identity, profile_index,
                    reinterpret_cast<const char*>(&payload[20]),
                    value_size);
            return status == ConfigurationTransactionStatus::kPending ||
                   status == ConfigurationTransactionStatus::kUnchanged ||
                   status == ConfigurationTransactionStatus::kCommitted;
        }
        case Operation::kProfileIdentify: {
            ControllerIdentity identity{};
            return controller_identity_decode(
                       payload, CONTROLLER_IDENTITY_ENCODED_SIZE,
                       &identity) &&
                   bluepad32_input_backend_identify(identity);
        }
        case Operation::kWiiOrientation: {
            ControllerIdentity identity{};
            constexpr size_t offset = CONTROLLER_IDENTITY_ENCODED_SIZE;
            return payload[offset + 4] <= 1 &&
                   controller_identity_decode(payload, offset, &identity) &&
                   bluepad32_input_backend_set_wii_orientation(
                       identity, UsbConfigurationManagement::read_u32(payload + offset),
                       payload[offset + 4] != 0);
        }
        case Operation::kPairingRefresh:
            bluepad32_input_backend_request_pairing_snapshot();
            return true;
        case Operation::kPairingClear:
            bluepad32_input_backend_clear_pairings();
            return true;
        case Operation::kHapticsExperiment:
            if (request.payload_size != 2 ||
                payload[0] > 2 ||
                payload[1] >= BLUEPAD32_INPUT_BACKEND_SLOT_COUNT) {
                return false;
            }
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
            return haptics_experiment_request(payload[0], payload[1]);
#else
            return false;
#endif
        default:
            return false;
    }
}

}  // namespace

bool usb_configuration_management_vendor_control(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
#if SWITCH2_PROBE_HUB
    // Native children have independent EP0 protocols and must never touch the
    // root management buffers, even to abort a pending root request.
    if (rhport != 0) return false;
#endif
    if (stage == CONTROL_STAGE_SETUP) {
        g_out_pending = false;
        g_out_processed = false;
        g_pending_request_size = 0;
#if SWITCH2_PROBE_HUB
        g_out_validated = false;
#endif
    }
#if !SWITCH2_PROBE_HUB
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
#if SWITCH2_PROBE_HUB
    // This image has a fixed native output. BOOTSEL uses the probe's private,
    // independently validated path so its status ACK retains the reboot delay.
    if (operation == Operation::kModeSet || operation == Operation::kReboot ||
        operation == Operation::kBootselReboot) return false;
#endif
    if ((operation == Operation::kHapticsTransportProbe ||
         operation == Operation::kSwitch2MouseCapture ||
         operation == Operation::kWiiIrGyro) &&
        request->bmRequestType_bit.direction != TUSB_DIR_IN) {
        return false;
    }
    if (stage == CONTROL_STAGE_ACK) {
        if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            return true;
        }
        if (!g_out_pending || rhport != g_pending_rhport ||
            memcmp(request, &g_pending_setup, sizeof(*request)) != 0 ||
            operation != g_pending_operation) {
            return false;
        }
        g_out_pending = false;
#if SWITCH2_PROBE_HUB
        if (!g_out_validated) return false;
        g_out_validated = false;
#endif
        if (operation == Operation::kHapticsExperiment) {
            return g_out_processed;
        }
        return process_out_request();
    }
    if (stage == CONTROL_STAGE_DATA) {
#if SWITCH2_PROBE_HUB
        if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            DecodedRequest decoded{};
            g_out_validated = g_out_pending && rhport == g_pending_rhport &&
                memcmp(request, &g_pending_setup, sizeof(*request)) == 0 &&
                decode_request(operation, g_request_buffer,
                               g_pending_request_size, &decoded);
            if (!g_out_validated) return false;
        }
#endif
        if (operation == Operation::kHapticsExperiment &&
            request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            if (!g_out_pending || operation != g_pending_operation ||
                g_out_processed) {
                return false;
            }
            // Reject before the USB status ACK, and never enqueue twice.
            g_out_processed = process_out_request();
            return g_out_processed;
        }
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
        g_pending_rhport = rhport;
        g_pending_setup = *request;
        g_out_pending = management_control_xfer(
            rhport, request, g_request_buffer, request->wLength);
        g_out_processed = false;
        return g_out_pending;
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
        case Operation::kRuntimeDiagnostics:
            response_size =
                encode_runtime_diagnostics(response, sizeof(response));
            break;
        case Operation::kHapticsExperiment:
            response_size =
                encode_haptics_experiment(response, sizeof(response));
            break;
        case Operation::kHapticsTransportProbe:
            response_size =
                encode_haptics_transport_probe(response, sizeof(response));
            break;
        case Operation::kNativeSwitchRumble:
            response_size = encode_native_switch_rumble(response, sizeof(response));
            break;
        case Operation::kSwitch2MouseCapture:
            response_size = encode_switch2_mouse_capture(response, sizeof(response));
            break;
        case Operation::kWiiIrGyro:
            response_size = encode_wii_ir_gyro(response, sizeof(response));
            break;
        case Operation::kMacroCapture:
            response_size = encode_macro_capture(response, sizeof(response));
            break;
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
        case Operation::kProfilePlaytest: {
            ProfileServiceSelectedSnapshot selected{};
            profile_service_selected_snapshot(&selected);
            Bluepad32PlaytestSnapshot playtest{};
            uint8_t selected_slot = 0xff;
            for (uint8_t slot = 0;
                 slot < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++slot) {
                Bluepad32PlaytestSnapshot candidate{};
                bluepad32_input_backend_playtest_snapshot(
                    slot, &candidate);
                if (!candidate.active ||
                    (!controller_identity_is_global(selected.identity) &&
                     !controller_identity_equal(
                         selected.identity, candidate.identity))) {
                    continue;
                }
                playtest = candidate;
                selected_slot = slot;
                break;
            }
            response_size = encode_profile_playtest(
                selected_slot, playtest, response, sizeof(response));
            break;
        }
        case Operation::kProfileMetadataRead: {
            ProfileServiceMetadataSnapshot snapshot{};
            profile_service_metadata_snapshot(&snapshot);
            response_size = encode_profile_metadata(
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
           management_control_xfer(
               rhport, request, response,
               static_cast<uint16_t>(response_size));
}
