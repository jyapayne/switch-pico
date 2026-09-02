#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t CONFIGURATION_STORAGE_COPY_COUNT = 2;
constexpr size_t CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE = 512;
constexpr size_t CONFIGURATION_STORAGE_RECORD_HEADER_SIZE = 24;
constexpr size_t CONFIGURATION_STORAGE_MAX_PAGE_SIZE = 256;
constexpr size_t CONFIGURATION_STORAGE_MAX_RECORD_SIZE =
    CONFIGURATION_STORAGE_RECORD_HEADER_SIZE +
    CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE;
constexpr size_t CONFIGURATION_STORAGE_MAX_PROGRAM_SIZE =
    ((CONFIGURATION_STORAGE_MAX_RECORD_SIZE +
      CONFIGURATION_STORAGE_MAX_PAGE_SIZE - 1) /
     CONFIGURATION_STORAGE_MAX_PAGE_SIZE) *
    CONFIGURATION_STORAGE_MAX_PAGE_SIZE;

uint32_t configuration_crc32(const uint8_t* data, size_t size);

enum class ConfigurationStorageResult : uint8_t {
    kOk,
    kUnchanged,
    kInvalidArgument,
    kIoError,
};

struct ConfigurationStorageIo {
    void* context = nullptr;
    size_t sector_size = 0;
    size_t page_size = 0;
    bool (*read)(void* context, uint8_t copy, size_t offset,
                 uint8_t* output, size_t size) = nullptr;
    bool (*erase)(void* context, uint8_t copy) = nullptr;
    bool (*program)(void* context, uint8_t copy, size_t offset,
                    const uint8_t* data, size_t size) = nullptr;
};

struct ConfigurationStorageSnapshot {
    bool valid = false;
    uint16_t schema_version = 0;
    uint16_t payload_size = 0;
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
    uint8_t payload[CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE]{};
};

class ConfigurationStorage {
public:
    bool initialize(const ConfigurationStorageIo& io);
    const ConfigurationStorageSnapshot& snapshot() const;
    ConfigurationStorageResult commit(uint16_t schema_version,
                                      const uint8_t* payload,
                                      size_t payload_size);

private:
    bool read_copy(uint8_t copy, ConfigurationStorageSnapshot* output) const;

    ConfigurationStorageIo io_{};
    ConfigurationStorageSnapshot snapshot_{};
    uint8_t active_copy_ = 0;
    bool initialized_ = false;
};
