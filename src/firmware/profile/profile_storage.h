#pragma once

#include <stddef.h>
#include <stdint.h>

#include "profile/controller_profile.h"

constexpr uint8_t PROFILE_STORAGE_BANK_COUNT = 2;
constexpr size_t PROFILE_STORAGE_SECTOR_SIZE = 4096;
constexpr size_t PROFILE_STORAGE_SECTORS_PER_BANK = 5;
constexpr size_t PROFILE_STORAGE_BANK_SIZE =
    PROFILE_STORAGE_SECTOR_SIZE * PROFILE_STORAGE_SECTORS_PER_BANK;
constexpr size_t PROFILE_STORAGE_TOTAL_SIZE =
    PROFILE_STORAGE_BANK_COUNT * PROFILE_STORAGE_BANK_SIZE;
constexpr size_t PROFILE_STORAGE_PAGE_SIZE = 256;
constexpr size_t PROFILE_STORAGE_RECORD_HEADER_SIZE =
    PROFILE_STORAGE_PAGE_SIZE;

static_assert(PROFILE_STORAGE_BANK_SIZE == 20 * 1024);
static_assert(PROFILE_STORAGE_TOTAL_SIZE == 40 * 1024);
static_assert(PROFILE_STORAGE_RECORD_HEADER_SIZE +
                      CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE <=
                  PROFILE_STORAGE_BANK_SIZE,
              "profile database does not fit a storage bank");

enum class ProfileStorageResult : uint8_t {
    kOk = 0,
    kUnchanged = 1,
    kInvalidArgument = 2,
    kIoError = 3,
};

struct ProfileStorageIo {
    void* context = nullptr;
    size_t bank_size = 0;
    size_t sector_size = 0;
    size_t page_size = 0;
    bool (*read)(void* context, uint8_t bank, size_t offset,
                 uint8_t* output, size_t size) = nullptr;
    // Replaces one bank and verifies the payload before publishing the
    // complete header page.
    bool (*replace_bank)(void* context, uint8_t bank,
                         const uint8_t* payload, size_t payload_size,
                         const uint8_t* header, size_t header_size) = nullptr;
};

struct ProfileStorageSnapshot {
    bool valid = false;
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
    uint8_t active_bank = 0;
};

uint32_t profile_storage_crc32(const uint8_t* data, size_t size);

class ProfileStorage {
public:
    bool initialize(const ProfileStorageIo& io,
                    ControllerProfileDatabase* database);
    ProfileStorageResult commit(
        const ControllerProfileDatabase& database,
        uint8_t* encoded_database,
        size_t encoded_database_size);
    const ProfileStorageSnapshot& snapshot() const;

private:
    struct BankHeader {
        uint32_t generation = 0;
        uint32_t payload_crc = 0;
    };

    bool read_header(uint8_t bank, BankHeader* output) const;
    bool validate_payload(uint8_t bank, uint32_t expected_crc) const;
    bool decode_bank(uint8_t bank,
                     ControllerProfileDatabase* database) const;
    bool payload_matches_encoded(uint8_t bank,
                                 const uint8_t* payload,
                                 size_t payload_size) const;

    ProfileStorageIo io_{};
    ProfileStorageSnapshot snapshot_{};
    bool initialized_ = false;
};
