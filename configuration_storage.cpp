#include "configuration_storage.h"

#include <string.h>

namespace {

constexpr uint8_t kRecordMagic[4] = {'S', 'P', 'C', 'F'};
constexpr uint16_t kRecordFormatVersion = 1;
constexpr size_t kHeaderCrcOffset = 20;

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

bool generation_is_newer(uint32_t candidate, uint32_t current) {
    const uint32_t difference = candidate - current;
    return difference != 0 && difference < 0x80000000u;
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

bool ConfigurationStorage::initialize(const ConfigurationStorageIo& io) {
    io_ = io;
    snapshot_ = {};
    active_copy_ = 0;
    initialized_ = io_.read != nullptr && io_.erase != nullptr &&
                   io_.program != nullptr && io_.page_size != 0 &&
                   io_.page_size <= CONFIGURATION_STORAGE_MAX_PAGE_SIZE &&
                   io_.sector_size >= CONFIGURATION_STORAGE_MAX_RECORD_SIZE &&
                   io_.sector_size % io_.page_size == 0;
    if (!initialized_) {
        return false;
    }

    ConfigurationStorageSnapshot copies[CONFIGURATION_STORAGE_COPY_COUNT]{};
    const bool first_valid = read_copy(0, &copies[0]);
    const bool second_valid = read_copy(1, &copies[1]);
    if (first_valid && second_valid) {
        active_copy_ = generation_is_newer(copies[1].generation,
                                           copies[0].generation)
                           ? 1
                           : 0;
        snapshot_ = copies[active_copy_];
    } else if (first_valid) {
        active_copy_ = 0;
        snapshot_ = copies[0];
    } else if (second_valid) {
        active_copy_ = 1;
        snapshot_ = copies[1];
    }
    return true;
}

const ConfigurationStorageSnapshot& ConfigurationStorage::snapshot() const {
    return snapshot_;
}

bool ConfigurationStorage::read_copy(
    uint8_t copy, ConfigurationStorageSnapshot* output) const {
    uint8_t header[CONFIGURATION_STORAGE_RECORD_HEADER_SIZE]{};
    if (output == nullptr ||
        !io_.read(io_.context, copy, 0, header, sizeof(header)) ||
        memcmp(header, kRecordMagic, sizeof(kRecordMagic)) != 0 ||
        read_u16(&header[4]) != kRecordFormatVersion ||
        read_u16(&header[6]) != CONFIGURATION_STORAGE_RECORD_HEADER_SIZE ||
        configuration_crc32(header, kHeaderCrcOffset) !=
            read_u32(&header[kHeaderCrcOffset])) {
        return false;
    }

    const uint16_t payload_size = read_u16(&header[8]);
    const uint16_t schema_version = read_u16(&header[10]);
    if (payload_size == 0 ||
        payload_size > CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE ||
        schema_version == 0 ||
        CONFIGURATION_STORAGE_RECORD_HEADER_SIZE + payload_size >
            io_.sector_size) {
        return false;
    }

    ConfigurationStorageSnapshot candidate{};
    if (!io_.read(io_.context, copy,
                  CONFIGURATION_STORAGE_RECORD_HEADER_SIZE,
                  candidate.payload, payload_size)) {
        return false;
    }
    candidate.payload_crc = read_u32(&header[16]);
    if (configuration_crc32(candidate.payload, payload_size) !=
        candidate.payload_crc) {
        return false;
    }

    candidate.valid = true;
    candidate.payload_size = payload_size;
    candidate.schema_version = schema_version;
    candidate.generation = read_u32(&header[12]);
    *output = candidate;
    return true;
}

ConfigurationStorageResult ConfigurationStorage::commit(
    uint16_t schema_version, const uint8_t* payload, size_t payload_size) {
    if (!initialized_ || schema_version == 0 || payload == nullptr ||
        payload_size == 0 ||
        payload_size > CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE) {
        return ConfigurationStorageResult::kInvalidArgument;
    }

    const uint32_t payload_crc = configuration_crc32(payload, payload_size);
    if (snapshot_.valid && snapshot_.schema_version == schema_version &&
        snapshot_.payload_size == payload_size &&
        snapshot_.payload_crc == payload_crc &&
        memcmp(snapshot_.payload, payload, payload_size) == 0) {
        return ConfigurationStorageResult::kUnchanged;
    }

    const size_t program_size =
        ((CONFIGURATION_STORAGE_RECORD_HEADER_SIZE + payload_size +
          io_.page_size - 1) /
         io_.page_size) *
        io_.page_size;
    if (program_size > CONFIGURATION_STORAGE_MAX_PROGRAM_SIZE ||
        program_size > io_.sector_size) {
        return ConfigurationStorageResult::kInvalidArgument;
    }

    uint8_t record[CONFIGURATION_STORAGE_MAX_PROGRAM_SIZE];
    memset(record, 0xff, sizeof(record));
    memcpy(record, kRecordMagic, sizeof(kRecordMagic));
    write_u16(&record[4], kRecordFormatVersion);
    write_u16(&record[6], CONFIGURATION_STORAGE_RECORD_HEADER_SIZE);
    write_u16(&record[8], static_cast<uint16_t>(payload_size));
    write_u16(&record[10], schema_version);
    const uint32_t generation = snapshot_.valid
                                    ? snapshot_.generation + 1u
                                    : 1u;
    write_u32(&record[12], generation);
    write_u32(&record[16], payload_crc);
    write_u32(&record[kHeaderCrcOffset],
              configuration_crc32(record, kHeaderCrcOffset));
    memcpy(&record[CONFIGURATION_STORAGE_RECORD_HEADER_SIZE], payload,
           payload_size);

    const uint8_t target_copy = snapshot_.valid ? active_copy_ ^ 1u : 0u;
    if (!io_.erase(io_.context, target_copy)) {
        return ConfigurationStorageResult::kIoError;
    }
    for (size_t offset = 0; offset < program_size;
         offset += io_.page_size) {
        if (!io_.program(io_.context, target_copy, offset,
                         &record[offset], io_.page_size)) {
            return ConfigurationStorageResult::kIoError;
        }
    }

    ConfigurationStorageSnapshot committed{};
    if (!read_copy(target_copy, &committed) ||
        committed.generation != generation ||
        committed.payload_crc != payload_crc) {
        return ConfigurationStorageResult::kIoError;
    }
    active_copy_ = target_copy;
    snapshot_ = committed;
    return ConfigurationStorageResult::kOk;
}
