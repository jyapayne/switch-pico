#include "configuration/adapter_configuration.h"
#include "configuration/configuration_storage.h"
#include "configuration/configuration_transaction.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

constexpr size_t kSectorSize = 4096;
constexpr size_t kPageSize = 256;

struct FakeFlash {
    uint8_t bytes[CONFIGURATION_STORAGE_COPY_COUNT][kSectorSize];
    int successful_programs = 0;
    int fail_after_programs = -1;

    FakeFlash() { memset(bytes, 0xff, sizeof(bytes)); }
};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

bool fake_read(void* context, uint8_t copy, size_t offset,
               uint8_t* output, size_t size) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT ||
        offset + size > kSectorSize) {
        return false;
    }
    memcpy(output, &flash->bytes[copy][offset], size);
    return true;
}

bool fake_erase(void* context, uint8_t copy) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT) {
        return false;
    }
    memset(flash->bytes[copy], 0xff, kSectorSize);
    return true;
}

bool fake_program(void* context, uint8_t copy, size_t offset,
                  const uint8_t* data, size_t size) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT || size != kPageSize ||
        offset + size > kSectorSize) {
        return false;
    }
    if (flash->fail_after_programs >= 0 &&
        flash->successful_programs >= flash->fail_after_programs) {
        return false;
    }
    for (size_t index = 0; index < size; ++index) {
        flash->bytes[copy][offset + index] &= data[index];
    }
    ++flash->successful_programs;
    return true;
}

ConfigurationStorageIo fake_io(FakeFlash* flash) {
    return {
        flash,
        kSectorSize,
        kPageSize,
        fake_read,
        fake_erase,
        fake_program,
    };
}

void test_schema_encoding() {
    AdapterConfiguration configuration{};
    configuration.pairing_window_seconds = 90;
    configuration.requested_mode = AdapterRequestedMode::kXInput;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload,
                                         sizeof(payload)),
            "valid v2 configuration did not encode");
    const uint8_t expected[] = {
        90,
        0,
        static_cast<uint8_t>(AdapterRequestedMode::kXInput),
        0,
        0,
        0,
        0,
        0,
    };
    require(memcmp(payload, expected, sizeof(expected)) == 0,
            "v2 configuration bytes are not canonical");

    AdapterConfiguration decoded{};
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                sizeof(payload), &decoded) &&
                decoded.pairing_window_seconds == 90 &&
                decoded.requested_mode == AdapterRequestedMode::kXInput,
            "v2 configuration did not round trip");

    const AdapterRequestedMode valid_modes[] = {
        AdapterRequestedMode::kAuto,
        AdapterRequestedMode::kSwitch,
        AdapterRequestedMode::kXInput,
        AdapterRequestedMode::kDInput,
        AdapterRequestedMode::kMac,
    };
    for (AdapterRequestedMode mode : valid_modes) {
        configuration.requested_mode = mode;
        require(adapter_configuration_encode(configuration, payload,
                                             sizeof(payload)) &&
                    adapter_configuration_decode(
                        ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                        sizeof(payload), &decoded) &&
                    decoded.requested_mode == mode,
                "valid requested mode did not round trip");
    }

    AdapterModeAvailability availability{};
    require(!adapter_requested_mode_available(
                AdapterRequestedMode::kDInput, availability) &&
                !adapter_requested_mode_available(
                    AdapterRequestedMode::kMac, availability),
            "future modes were available by default");
    availability.dinput_mode = true;
    availability.mac_mode = true;
    require(adapter_requested_mode_available(
                AdapterRequestedMode::kDInput, availability) &&
                adapter_requested_mode_available(
                    AdapterRequestedMode::kMac, availability),
            "availability API could not enable future modes");

    const uint8_t legacy[] = {120, 0, 0, 0};
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION, legacy,
                sizeof(legacy), &decoded) &&
                decoded.pairing_window_seconds == 120 &&
                decoded.requested_mode == AdapterRequestedMode::kAuto,
            "v1 configuration did not migrate to auto");
    uint8_t malformed_legacy[sizeof(legacy)];
    memcpy(malformed_legacy, legacy, sizeof(legacy));
    malformed_legacy[3] = 1;
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION,
                malformed_legacy, sizeof(malformed_legacy), &decoded),
            "v1 nonzero reserved byte was accepted");
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION, legacy,
                sizeof(legacy) - 1, &decoded),
            "v1 record with wrong size was accepted");

    for (size_t index = 3; index < sizeof(payload); ++index) {
        uint8_t malformed[sizeof(payload)];
        memcpy(malformed, payload, sizeof(payload));
        malformed[index] = 1;
        require(!adapter_configuration_decode(
                    ADAPTER_CONFIGURATION_SCHEMA_VERSION, malformed,
                    sizeof(malformed), &decoded),
                "v2 nonzero reserved byte was accepted");
    }
    uint8_t invalid_mode[sizeof(payload)];
    memcpy(invalid_mode, payload, sizeof(payload));
    invalid_mode[2] = 5;
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, invalid_mode,
                sizeof(invalid_mode), &decoded),
            "out-of-range requested mode was accepted");
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                sizeof(payload) - 1, &decoded),
            "short v2 record was accepted");
    uint8_t oversized[ADAPTER_CONFIGURATION_ENCODED_SIZE + 1]{};
    memcpy(oversized, payload, sizeof(payload));
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, oversized,
                sizeof(oversized), &decoded),
            "oversized v2 record was accepted");

    configuration.pairing_window_seconds = 9;
    require(!adapter_configuration_encode(configuration, payload,
                                          sizeof(payload)),
            "out-of-range pairing window was accepted");
    configuration.pairing_window_seconds = 90;
    configuration.requested_mode =
        static_cast<AdapterRequestedMode>(5);
    require(!adapter_configuration_encode(configuration, payload,
                                          sizeof(payload)),
            "out-of-range requested mode encoded");
    configuration.requested_mode = AdapterRequestedMode::kAuto;
    require(!adapter_configuration_encode(configuration, oversized,
                                          sizeof(oversized)),
            "v2 encoder accepted a noncanonical output size");
}

void test_two_copy_recovery() {
    FakeFlash flash;
    ConfigurationStorage store;
    require(store.initialize(fake_io(&flash)),
            "storage did not initialize");
    require(!store.snapshot().valid,
            "erased storage appeared valid");

    const uint8_t first[] = {1, 2, 3, 4};
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kOk &&
                store.snapshot().generation == 1,
            "first generation did not commit");
    const int programs_after_first = flash.successful_programs;
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kUnchanged &&
                flash.successful_programs == programs_after_first,
            "identical configuration consumed a flash write");

    const uint8_t second[] = {5, 6, 7, 8};
    require(store.commit(1, second, sizeof(second)) ==
                ConfigurationStorageResult::kOk &&
                store.snapshot().generation == 2,
            "second generation did not commit");

    ConfigurationStorage after_power_cycle;
    require(after_power_cycle.initialize(fake_io(&flash)) &&
                after_power_cycle.snapshot().generation == 2 &&
                memcmp(after_power_cycle.snapshot().payload, second,
                       sizeof(second)) == 0,
            "latest generation did not survive reinitialization");

    flash.bytes[1][CONFIGURATION_STORAGE_RECORD_HEADER_SIZE] ^= 0x01;
    ConfigurationStorage after_corruption;
    require(after_corruption.initialize(fake_io(&flash)) &&
                after_corruption.snapshot().generation == 1 &&
                memcmp(after_corruption.snapshot().payload, first,
                       sizeof(first)) == 0,
            "corrupt newest generation did not roll back");
}

void test_interrupted_write_retains_previous_generation() {
    FakeFlash flash;
    ConfigurationStorage store;
    require(store.initialize(fake_io(&flash)),
            "storage did not initialize for interruption test");
    const uint8_t first[] = {9, 8, 7, 6};
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kOk,
            "baseline generation did not commit");

    uint8_t large[300];
    memset(large, 0x5a, sizeof(large));
    flash.fail_after_programs = flash.successful_programs + 1;
    require(store.commit(1, large, sizeof(large)) ==
                ConfigurationStorageResult::kIoError,
            "interrupted multi-page write reported success");

    flash.fail_after_programs = -1;
    ConfigurationStorage recovered;
    require(recovered.initialize(fake_io(&flash)) &&
                recovered.snapshot().generation == 1 &&
                recovered.snapshot().payload_size == sizeof(first) &&
                memcmp(recovered.snapshot().payload, first,
                       sizeof(first)) == 0,
            "interrupted write replaced the previous generation");
}

void test_transaction_validation() {
    AdapterConfiguration configuration{};
    configuration.pairing_window_seconds = 120;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload,
                                         sizeof(payload)),
            "transaction payload did not encode");
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));

    ConfigurationTransaction transaction;
    require(transaction.begin(7, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kReceiving,
            "valid transaction did not begin");
    require(transaction.append(7, 1, payload, 1) ==
                ConfigurationTransactionStatus::kOutOfOrder,
            "out-of-order chunk was accepted");

    require(transaction.begin(8, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc ^ 1u) ==
                ConfigurationTransactionStatus::kReceiving,
            "replacement transaction did not begin");
    require(transaction.append(8, 0, payload, 2) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(8, 2, &payload[2],
                                   sizeof(payload) - 2) ==
                    ConfigurationTransactionStatus::kReceiving,
            "ordered chunks were rejected");
    require(transaction.finish(8) ==
                ConfigurationTransactionStatus::kBadCrc,
            "bad transaction CRC was accepted");

    require(transaction.begin(9, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(9, 0, payload, sizeof(payload)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                transaction.finish(9) ==
                    ConfigurationTransactionStatus::kPending,
            "valid transaction did not reach pending commit");
    require(transaction.begin(10, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kBusy,
            "pending transaction was replaced");

    transaction.set_result(ConfigurationTransactionStatus::kCommitted,
                           4, crc);
    require(transaction.begin(11, 99, sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kUnsupportedSchema,
            "unsupported schema was accepted");
    require(transaction.begin(
                12, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE + 1, crc) ==
                ConfigurationTransactionStatus::kTooLarge,
            "oversized transaction was accepted");
}

}  // namespace

int main() {
    test_schema_encoding();
    test_two_copy_recovery();
    test_interrupted_write_retains_previous_generation();
    test_transaction_validation();
    return 0;
}
