#include "adapter_configuration.h"
#include "configuration_storage.h"
#include "configuration_transaction.h"

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
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload,
                                         sizeof(payload)),
            "valid configuration did not encode");
    AdapterConfiguration decoded{};
    require(adapter_configuration_decode(payload, sizeof(payload),
                                         &decoded) &&
                decoded.pairing_window_seconds == 90,
            "configuration did not round trip");
    payload[2] = 1;
    require(!adapter_configuration_decode(payload, sizeof(payload),
                                          &decoded),
            "nonzero reserved configuration byte was accepted");
    configuration.pairing_window_seconds = 9;
    require(!adapter_configuration_encode(configuration, payload,
                                          sizeof(payload)),
            "out-of-range pairing window was accepted");
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
                transaction.append(8, 2, &payload[2], 2) ==
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
