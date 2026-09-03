#include "profile_storage.h"

#include <string.h>

namespace {

constexpr uint8_t kRecordMagic[4] = {'S', 'P', 'P', 'F'};
constexpr uint16_t kRecordFormatVersion = 1;
constexpr size_t kHeaderFieldsSize = 24;
constexpr size_t kHeaderCrcOffset = 20;

uint16_t storage_read_u16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
           (static_cast<uint16_t>(input[1]) << 8);
}

uint32_t storage_read_u32(const uint8_t* input) {
    return static_cast<uint32_t>(input[0]) |
           (static_cast<uint32_t>(input[1]) << 8) |
           (static_cast<uint32_t>(input[2]) << 16) |
           (static_cast<uint32_t>(input[3]) << 24);
}

void storage_write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void storage_write_u32(uint8_t* output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t size) {
    for (size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^
                  (0xedb88320u &
                   static_cast<uint32_t>(
                       -static_cast<int32_t>(crc & 1u)));
        }
    }
    return crc;
}

bool generation_is_newer(uint32_t candidate, uint32_t current) {
    return static_cast<int32_t>(candidate - current) > 0;
}

struct DatabaseReadContext {
    const ProfileStorageIo* io;
    uint8_t bank;
};

bool read_database(void* context, size_t offset, uint8_t* output,
                   size_t size) {
    const auto* read_context =
        static_cast<const DatabaseReadContext*>(context);
    return read_context->io->read(
        read_context->io->context, read_context->bank,
        PROFILE_STORAGE_RECORD_HEADER_SIZE + offset, output, size);
}

}  // namespace

uint32_t profile_storage_crc32(const uint8_t* data, size_t size) {
    if (data == nullptr && size != 0) {
        return 0;
    }
    return ~crc32_update(0xffffffffu, data, size);
}

bool ProfileStorage::initialize(const ProfileStorageIo& io,
                                ControllerProfileDatabase* database) {
    io_ = io;
    snapshot_ = {};
    initialized_ =
        database != nullptr && io_.read != nullptr &&
        io_.replace_bank != nullptr &&
        io_.bank_size >= PROFILE_STORAGE_BANK_SIZE &&
        io_.sector_size == PROFILE_STORAGE_SECTOR_SIZE &&
        io_.page_size == PROFILE_STORAGE_PAGE_SIZE &&
        io_.bank_size % io_.sector_size == 0 &&
        io_.sector_size % io_.page_size == 0;
    if (!initialized_) {
        return false;
    }

    controller_profile_database_default(database);
    BankHeader headers[PROFILE_STORAGE_BANK_COUNT]{};
    bool valid[PROFILE_STORAGE_BANK_COUNT]{};
    for (uint8_t bank = 0; bank < PROFILE_STORAGE_BANK_COUNT; ++bank) {
        valid[bank] = read_header(bank, &headers[bank]) &&
                      validate_payload(bank, headers[bank].payload_crc);
    }

    uint8_t first = 0;
    uint8_t second = 1;
    if (valid[1] &&
        (!valid[0] || generation_is_newer(headers[1].generation,
                                          headers[0].generation))) {
        first = 1;
        second = 0;
    }
    const uint8_t order[PROFILE_STORAGE_BANK_COUNT] = {first, second};
    for (uint8_t candidate : order) {
        if (!valid[candidate] || !decode_bank(candidate, database)) {
            continue;
        }
        snapshot_.valid = true;
        snapshot_.generation = headers[candidate].generation;
        snapshot_.payload_crc = headers[candidate].payload_crc;
        snapshot_.active_bank = candidate;
        return true;
    }
    controller_profile_database_default(database);
    return true;
}

ProfileStorageResult ProfileStorage::commit(
    const ControllerProfileDatabase& database,
    uint8_t* encoded_database, size_t encoded_database_size) {
    if (!initialized_ || !controller_profile_database_validate(database) ||
        encoded_database == nullptr ||
        encoded_database_size < CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        !controller_profile_database_encode_range(
            database, 0, encoded_database,
            CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE)) {
        return ProfileStorageResult::kInvalidArgument;
    }
    if (snapshot_.valid &&
        payload_matches_encoded(
            snapshot_.active_bank, encoded_database,
            CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE)) {
        return ProfileStorageResult::kUnchanged;
    }

    const uint32_t crc = profile_storage_crc32(
        encoded_database, CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE);
    const uint8_t target_bank =
        snapshot_.valid ? snapshot_.active_bank ^ 1u : 0;
    const uint32_t generation =
        snapshot_.valid ? snapshot_.generation + 1u : 1u;
    uint8_t header[PROFILE_STORAGE_RECORD_HEADER_SIZE]{};
    memcpy(header, kRecordMagic, sizeof(kRecordMagic));
    storage_write_u16(&header[4], kRecordFormatVersion);
    storage_write_u16(
        &header[6], CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION);
    storage_write_u32(&header[8], generation);
    storage_write_u32(
        &header[12], CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE);
    storage_write_u32(&header[16], crc);
    storage_write_u32(
        &header[kHeaderCrcOffset],
        profile_storage_crc32(header, kHeaderCrcOffset));

    if (!io_.replace_bank(
            io_.context, target_bank, encoded_database,
            CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE, header,
            sizeof(header))) {
        return ProfileStorageResult::kIoError;
    }

    snapshot_.valid = true;
    snapshot_.generation = generation;
    snapshot_.payload_crc = crc;
    snapshot_.active_bank = target_bank;
    return ProfileStorageResult::kOk;
}

const ProfileStorageSnapshot& ProfileStorage::snapshot() const {
    return snapshot_;
}

bool ProfileStorage::read_header(uint8_t bank, BankHeader* output) const {
    uint8_t header[PROFILE_STORAGE_RECORD_HEADER_SIZE]{};
    if (bank >= PROFILE_STORAGE_BANK_COUNT || output == nullptr ||
        !io_.read(io_.context, bank, 0, header, sizeof(header)) ||
        memcmp(header, kRecordMagic, sizeof(kRecordMagic)) != 0 ||
        storage_read_u16(&header[4]) != kRecordFormatVersion ||
        (storage_read_u16(&header[6]) !=
             CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION &&
         storage_read_u16(&header[6]) !=
             CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION) ||
        storage_read_u32(&header[12]) !=
            CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        profile_storage_crc32(header, kHeaderCrcOffset) !=
            storage_read_u32(&header[kHeaderCrcOffset])) {
        return false;
    }
    for (size_t offset = kHeaderFieldsSize;
         offset < sizeof(header); ++offset) {
        if (header[offset] != 0) {
            return false;
        }
    }
    output->generation = storage_read_u32(&header[8]);
    output->payload_crc = storage_read_u32(&header[16]);
    return true;
}

bool ProfileStorage::validate_payload(uint8_t bank,
                                      uint32_t expected_crc) const {
    uint8_t page[PROFILE_STORAGE_PAGE_SIZE]{};
    uint32_t crc = 0xffffffffu;
    for (size_t offset = 0;
         offset < CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE;
         offset += sizeof(page)) {
        const size_t size =
            CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE - offset < sizeof(page)
                ? CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE - offset
                : sizeof(page);
        if (!io_.read(io_.context, bank,
                      PROFILE_STORAGE_RECORD_HEADER_SIZE + offset,
                      page, size)) {
            return false;
        }
        crc = crc32_update(crc, page, size);
    }
    return ~crc == expected_crc;
}

bool ProfileStorage::decode_bank(
    uint8_t bank, ControllerProfileDatabase* database) const {
    DatabaseReadContext context{&io_, bank};
    return controller_profile_database_decode(read_database, &context,
                                              database);
}

bool ProfileStorage::payload_matches_encoded(
    uint8_t bank, const uint8_t* payload,
    size_t payload_size) const {
    uint8_t stored[PROFILE_STORAGE_PAGE_SIZE]{};
    for (size_t offset = 0; offset < payload_size;
         offset += sizeof(stored)) {
        const size_t size =
            payload_size - offset < sizeof(stored)
                ? payload_size - offset
                : sizeof(stored);
        if (!io_.read(
                io_.context, bank,
                PROFILE_STORAGE_RECORD_HEADER_SIZE + offset,
                stored, size) ||
            memcmp(stored, &payload[offset], size) != 0) {
            return false;
        }
    }
    return true;
}
