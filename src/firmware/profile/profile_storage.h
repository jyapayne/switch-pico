#pragma once

#include <stddef.h>
#include <stdint.h>

#include "profile/controller_profile.h"

constexpr uint8_t PROFILE_STORAGE_ARENA_COUNT = 2;
constexpr size_t PROFILE_STORAGE_SECTOR_SIZE = 4096;
constexpr size_t PROFILE_STORAGE_PAGE_SIZE = 256;
constexpr size_t PROFILE_STORAGE_ARENA_SIZE = 128 * 1024;
constexpr size_t PROFILE_STORAGE_TOTAL_SIZE =
    PROFILE_STORAGE_ARENA_COUNT * PROFILE_STORAGE_ARENA_SIZE;
constexpr size_t PROFILE_STORAGE_SUPERBLOCK_SIZE = PROFILE_STORAGE_PAGE_SIZE;
constexpr size_t PROFILE_STORAGE_RECORD_SIZE = 2 * PROFILE_STORAGE_PAGE_SIZE;
constexpr size_t PROFILE_STORAGE_RECORDS_OFFSET = PROFILE_STORAGE_SECTOR_SIZE;
constexpr uint32_t PROFILE_STORAGE_NO_RECORD = UINT32_MAX;
constexpr uint8_t PROFILE_STORAGE_METADATA_MAX_BYTES = 31;
constexpr size_t PROFILE_STORAGE_METADATA_PAYLOAD_SIZE =
    PROFILE_STORAGE_METADATA_MAX_BYTES + 1;
constexpr size_t PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE =
    CONTROLLER_PROFILE_COUNT * PROFILE_STORAGE_METADATA_PAYLOAD_SIZE;
constexpr size_t PROFILE_STORAGE_MAX_LIVE_RECORDS =
    (CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) *
    (CONTROLLER_PROFILE_COUNT + 3);
constexpr size_t PROFILE_STORAGE_RECORD_CAPACITY =
    (PROFILE_STORAGE_ARENA_SIZE - PROFILE_STORAGE_RECORDS_OFFSET) /
    PROFILE_STORAGE_RECORD_SIZE;
static_assert(PROFILE_STORAGE_MAX_LIVE_RECORDS <=
              PROFILE_STORAGE_RECORD_CAPACITY,
              "all live profile records must fit during compaction");
static_assert(PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE == 256);
static_assert(PROFILE_STORAGE_RECORD_CAPACITY == 248);
static_assert(PROFILE_STORAGE_MAX_LIVE_RECORDS == 187);

// Layout of the retired v1/v2 whole-database store. The indexed catalog reads
// this region once during migration; new firmware never writes it.
constexpr uint8_t PROFILE_STORAGE_LEGACY_BANK_COUNT = 2;
constexpr size_t PROFILE_STORAGE_LEGACY_BANK_SIZE = 20 * 1024;
constexpr size_t PROFILE_STORAGE_LEGACY_TOTAL_SIZE =
    PROFILE_STORAGE_LEGACY_BANK_COUNT * PROFILE_STORAGE_LEGACY_BANK_SIZE;
constexpr size_t PROFILE_STORAGE_LEGACY_HEADER_SIZE = PROFILE_STORAGE_PAGE_SIZE;
constexpr uint8_t PROFILE_STORAGE_LEGACY_PROFILE_COUNT = 4;
constexpr size_t PROFILE_STORAGE_LEGACY_DATABASE_SIZE = 17696;

static_assert(PROFILE_STORAGE_ARENA_SIZE % PROFILE_STORAGE_SECTOR_SIZE == 0);
static_assert(PROFILE_STORAGE_RECORDS_OFFSET % PROFILE_STORAGE_RECORD_SIZE ==
              0);

struct ProfileStorageIo {
  void *context = nullptr;
  size_t arena_size = 0;
  size_t sector_size = 0;
  size_t page_size = 0;
  bool (*read)(void *context, uint8_t arena, size_t offset, uint8_t *output,
               size_t size) = nullptr;
  bool (*erase_arena)(void *context, uint8_t arena) = nullptr;
  bool (*program_page)(void *context, uint8_t arena, size_t offset,
                       const uint8_t *page, size_t size) = nullptr;
};

struct ProfileStorageSnapshot {
  bool valid = false;
  uint32_t generation = 0;
  uint32_t payload_crc = 0;
  uint8_t active_bank = 0;
};

enum class ProfileStorageResult : uint8_t {
  kOk = 0,
  kUnchanged = 1,
  kInvalidArgument = 2,
  kIoError = 3,
  kFull = 4,
};

struct ProfileStorageIdentityIndex {
  bool used = false;
  ControllerIdentity identity{};
  uint8_t active_profile = 0;
  uint32_t active_generation = 0;
  uint32_t profile_generation[CONTROLLER_PROFILE_COUNT]{};
  uint32_t profile_record[CONTROLLER_PROFILE_COUNT]{};
  uint32_t alias_generation = 0;
  uint32_t alias_record = PROFILE_STORAGE_NO_RECORD;
  uint32_t profile_names_generation = 0;
  uint32_t profile_names_record = PROFILE_STORAGE_NO_RECORD;
};

uint32_t profile_storage_crc32(const uint8_t *data, size_t size);

class ProfileStorage {
public:
  bool initialize(const ProfileStorageIo &io);
  ProfileStorageResult ensure_identity(const ControllerIdentity &identity);
  ProfileStorageResult ensure_joycon_pair(const ControllerIdentity &pair);
  ProfileStorageResult get(const ControllerIdentity &identity,
                           uint8_t profile_index,
                           ControllerProfile *output) const;
  ProfileStorageResult set(const ControllerIdentity &identity,
                           uint8_t profile_index,
                           const ControllerProfile &profile);
  ProfileStorageResult reset(const ControllerIdentity &identity,
                             uint8_t profile_index);
  ProfileStorageResult activate(const ControllerIdentity &identity,
                                uint8_t profile_index);
  ProfileStorageResult get_alias(
      const ControllerIdentity &identity, char *output,
      size_t output_size) const;
  ProfileStorageResult set_alias(
      const ControllerIdentity &identity, const char *value,
      size_t value_size);
  ProfileStorageResult get_profile_name(
      const ControllerIdentity &identity, uint8_t profile_index,
      char *output, size_t output_size) const;
  ProfileStorageResult set_profile_name(
      const ControllerIdentity &identity, uint8_t profile_index,
      const char *value, size_t value_size);

  uint8_t identity_count() const;
  const ProfileStorageIdentityIndex *identity(uint8_t index) const;
  const ProfileStorageIdentityIndex *
  find(const ControllerIdentity &identity) const;
  const ProfileStorageSnapshot &snapshot() const;

private:
  enum class RecordType : uint8_t {
    kProfile = 1,
    kReset = 2,
    kResetAll = 3,
    kActivate = 4,
    kAlias = 5,
    kProfileNames = 6,
    kSeedJoyConPair = 7,
  };

  bool valid_owner(const ControllerIdentity &identity) const;
  ProfileStorageResult scan_arena(
      uint8_t arena, uint16_t *version, uint32_t *epoch,
      uint32_t *generation, uint32_t *payload_crc, size_t *next_offset,
      ProfileStorageIdentityIndex *index, uint8_t *identity_count) const;
  bool validate_record(const uint8_t *record, uint16_t version) const;
  bool read_record_payload(uint32_t record, RecordType type, uint8_t *output,
                           size_t capacity, size_t *size = nullptr) const;
  bool read_profile_record(uint32_t record, ControllerProfile *output) const;
  bool read_metadata_record(uint32_t record, char *output,
                            size_t output_size) const;
  ProfileStorageResult append(RecordType type,
                              const ControllerIdentity &identity,
                              uint8_t profile_index, const uint8_t *payload,
                              size_t payload_size);
  ProfileStorageResult compact();
  ProfileStorageResult migrate_legacy();
  bool publish_arena(uint8_t arena, uint32_t epoch, size_t records_end) const;
  bool publish_empty_arena(uint8_t arena, uint32_t epoch);
  bool write_record(uint8_t arena, size_t offset, RecordType type,
                    const ControllerIdentity &identity, uint8_t profile_index,
                    uint32_t generation, const uint8_t *payload,
                    size_t payload_size) const;
  bool apply_record(ProfileStorageIdentityIndex *index, uint8_t *identity_count,
                    RecordType type, const ControllerIdentity &identity,
                    uint8_t profile_index, uint32_t generation,
                    uint32_t record) const;

  ProfileStorageIo io_{};
  ProfileStorageSnapshot snapshot_{};
  ProfileStorageIdentityIndex
      index_[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t identity_count_ = 0;
  uint32_t epoch_ = 0;
  uint16_t catalog_version_ = 0;
  size_t next_offset_ = PROFILE_STORAGE_RECORDS_OFFSET;
  bool initialized_ = false;
};
