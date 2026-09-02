#pragma once

#include <stddef.h>
#include <stdint.h>

#include "configuration_storage.h"

enum class ConfigurationTransactionStatus : uint8_t {
    kIdle = 0,
    kReceiving = 1,
    kPending = 2,
    kCommitted = 3,
    kUnchanged = 4,
    kMalformed = 5,
    kUnsupportedSchema = 6,
    kTooLarge = 7,
    kOutOfOrder = 8,
    kBadCrc = 9,
    kBusy = 10,
    kStorageError = 11,
};

struct ConfigurationTransactionSnapshot {
    uint32_t transaction_id = 0;
    uint16_t received_size = 0;
    uint16_t expected_size = 0;
    uint32_t expected_crc = 0;
    uint32_t stored_generation = 0;
    uint32_t stored_crc = 0;
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kIdle;
};

class ConfigurationTransaction {
public:
    ConfigurationTransactionStatus begin(uint32_t transaction_id,
                                         uint16_t schema_version,
                                         size_t payload_size,
                                         uint32_t payload_crc);
    ConfigurationTransactionStatus append(uint32_t transaction_id,
                                          size_t offset,
                                          const uint8_t* data,
                                          size_t size);
    ConfigurationTransactionStatus finish(uint32_t transaction_id);
    void set_result(ConfigurationTransactionStatus status,
                    uint32_t stored_generation, uint32_t stored_crc);
    void clear();

    const ConfigurationTransactionSnapshot& snapshot() const;
    uint16_t schema_version() const;
    const uint8_t* payload() const;

private:
    ConfigurationTransactionSnapshot snapshot_{};
    uint16_t schema_version_ = 0;
    uint8_t payload_[CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE]{};
};
