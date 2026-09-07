#include "profile/profile_storage.h"

#include <string.h>

namespace {

constexpr uint8_t kSuperblockMagic[4] = {'S', 'P', 'C', 'A'};
constexpr uint8_t kRecordMagic[4] = {'S', 'P', 'C', 'R'};
constexpr uint8_t kLegacyStorageMagic[4] = {'S', 'P', 'P', 'F'};
constexpr uint8_t kLegacyDatabaseMagic[4] = {'S', 'P', 'D', 'B'};
constexpr uint16_t kLegacyCatalogVersion = 1;
constexpr uint16_t kExpandedCatalogVersion = 2;
constexpr uint16_t kCatalogVersion = 3;
constexpr size_t kRecordPayloadOffset = 128;
constexpr size_t kLegacyRecordPayloadOffset = 256;
static_assert(kRecordPayloadOffset + CONTROLLER_PROFILE_ENCODED_SIZE ==
              PROFILE_STORAGE_RECORD_SIZE);
constexpr size_t kRecordHeaderCrcOffset = 34;
constexpr size_t kLegacyStart =
    PROFILE_STORAGE_TOTAL_SIZE - PROFILE_STORAGE_LEGACY_TOTAL_SIZE;

uint16_t read_u16(const uint8_t *input) {
  return static_cast<uint16_t>(input[0]) | static_cast<uint16_t>(input[1] << 8);
}

uint32_t read_u32(const uint8_t *input) {
  return static_cast<uint32_t>(input[0]) |
         (static_cast<uint32_t>(input[1]) << 8) |
         (static_cast<uint32_t>(input[2]) << 16) |
         (static_cast<uint32_t>(input[3]) << 24);
}

void write_u16(uint8_t *output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t *output, uint32_t value) {
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8);
  output[2] = static_cast<uint8_t>(value >> 16);
  output[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t size) {
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return crc;
}

bool generation_is_newer(uint32_t candidate, uint32_t current) {
  return static_cast<int32_t>(candidate - current) > 0;
}

bool valid_identity(const ControllerIdentity &identity) {
  uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE]{};
  return (controller_identity_is_global(identity) || identity.stable) &&
         controller_identity_encode(identity, encoded, sizeof(encoded));
}

uint32_t pack_record(uint8_t arena, size_t offset) {
  return static_cast<uint32_t>(arena * PROFILE_STORAGE_ARENA_SIZE + offset);
}

uint8_t record_arena(uint32_t record) {
  return static_cast<uint8_t>(record / PROFILE_STORAGE_ARENA_SIZE);
}

size_t record_offset(uint32_t record) {
  return record % PROFILE_STORAGE_ARENA_SIZE;
}

bool bytes_are(uint8_t value, const uint8_t *data, size_t size) {
  for (size_t index = 0; index < size; ++index) {
    if (data[index] != value) {
      return false;
    }
  }
  return true;
}

bool metadata_value_valid(const uint8_t *payload, size_t size) {
  return payload != nullptr && size == PROFILE_STORAGE_METADATA_PAYLOAD_SIZE &&
         payload[0] <= PROFILE_STORAGE_METADATA_MAX_BYTES &&
         bytes_are(0, &payload[payload[0] + 1],
                   size - payload[0] - 1);
}

} // namespace

uint32_t profile_storage_crc32(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0) {
    return 0;
  }
  return ~crc32_update(0xffffffffu, data, size);
}

bool ProfileStorage::initialize(const ProfileStorageIo &io) {
  io_ = io;
  snapshot_ = {};
  identity_count_ = 0;
  epoch_ = 0;
  catalog_version_ = 0;
  next_offset_ = PROFILE_STORAGE_RECORDS_OFFSET;
  for (ProfileStorageIdentityIndex &entry : index_) {
    entry = {};
  }
  initialized_ = io_.read != nullptr && io_.erase_arena != nullptr &&
                 io_.program_page != nullptr &&
                 io_.arena_size == PROFILE_STORAGE_ARENA_SIZE &&
                 io_.sector_size == PROFILE_STORAGE_SECTOR_SIZE &&
                 io_.page_size == PROFILE_STORAGE_PAGE_SIZE;
  if (!initialized_) {
    return false;
  }

  for (uint8_t arena = 0; arena < PROFILE_STORAGE_ARENA_COUNT; ++arena) {
    ProfileStorageIdentityIndex
        candidate[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
    uint8_t count = 0;
    uint16_t version = 0;
    uint32_t epoch = 0;
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
    size_t offset = 0;
    const ProfileStorageResult scanned =
        scan_arena(arena, &version, &epoch, &generation, &payload_crc,
                   &offset, candidate, &count);
    if (scanned == ProfileStorageResult::kIoError) {
      initialized_ = false;
      return false;
    }
    if (scanned != ProfileStorageResult::kOk ||
        (snapshot_.valid && !generation_is_newer(epoch, epoch_))) {
      continue;
    }
    for (size_t index = 0;
         index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
      index_[index] = candidate[index];
    }
    identity_count_ = count;
    epoch_ = epoch;
    catalog_version_ = version;
    next_offset_ = offset;
    snapshot_.valid = true;
    snapshot_.active_bank = arena;
    snapshot_.generation = generation;
    snapshot_.payload_crc = payload_crc;
  }
  if (snapshot_.valid) {
    if (catalog_version_ != kCatalogVersion &&
        compact() != ProfileStorageResult::kOk) {
      initialized_ = false;
      return false;
    }
    return true;
  }

  const ProfileStorageResult migrated = migrate_legacy();
  if (migrated != ProfileStorageResult::kUnchanged) {
    initialized_ = migrated == ProfileStorageResult::kOk;
    return initialized_;
  }
  // Only pristine storage can become an empty catalog. In particular, an
  // unreadable or interrupted migration must never erase an old database.
  uint8_t page[PROFILE_STORAGE_PAGE_SIZE]{};
  for (uint8_t arena = 0; arena < PROFILE_STORAGE_ARENA_COUNT; ++arena) {
    for (size_t offset = 0; offset < PROFILE_STORAGE_ARENA_SIZE;
         offset += sizeof(page)) {
      if (!io_.read(io_.context, arena, offset, page, sizeof(page)) ||
          !bytes_are(0xff, page, sizeof(page))) {
        initialized_ = false;
        return false;
      }
    }
  }
  identity_count_ = 1;
  index_[0].used = true;
  index_[0].identity = controller_identity_global();
  for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
    index_[0].profile_record[profile] = PROFILE_STORAGE_NO_RECORD;
  }
  initialized_ = publish_empty_arena(0, 1);
  return initialized_;
}

bool ProfileStorage::valid_owner(const ControllerIdentity &identity) const {
  return valid_identity(identity) &&
         (!controller_identity_is_joycon_pair(identity) ||
          find(identity) != nullptr);
}

ProfileStorageResult
ProfileStorage::ensure_identity(const ControllerIdentity &identity) {
  if (!initialized_ || !valid_owner(identity)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  if (find(identity) != nullptr) {
    return ProfileStorageResult::kUnchanged;
  }
  if (identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  return append(RecordType::kActivate, identity, 0, nullptr, 0);
}

ProfileStorageResult
ProfileStorage::ensure_joycon_pair(const ControllerIdentity &pair) {
  if (!initialized_) {
    return ProfileStorageResult::kIoError;
  }
  ControllerIdentity left{};
  ControllerIdentity right{};
  if (!controller_identity_joycon_pair_members(pair, &left, &right)) {
    return ProfileStorageResult::kInvalidArgument;
  }
  if (find(pair) != nullptr) {
    return ProfileStorageResult::kUnchanged;
  }
  const uint8_t needed = 1 + (find(left) == nullptr) + (find(right) == nullptr);
  if (identity_count_ + needed > CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  ProfileStorageResult result = ensure_identity(left);
  if (result != ProfileStorageResult::kOk &&
      result != ProfileStorageResult::kUnchanged) {
    return result;
  }
  result = ensure_identity(right);
  if (result != ProfileStorageResult::kOk &&
      result != ProfileStorageResult::kUnchanged) {
    return result;
  }
  // One committed record snapshots the immutable left record references.
  // Defaults are identity-independent; later edits use normal copy-on-write.
  return append(RecordType::kSeedJoyConPair, pair,
                CONTROLLER_PROFILE_ALL, nullptr, 0);
}

ProfileStorageResult ProfileStorage::get(const ControllerIdentity &identity,
                                         uint8_t profile_index,
                                         ControllerProfile *output) const {
  if (!initialized_ || output == nullptr || !valid_owner(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry == nullptr ||
      entry->profile_record[profile_index] == PROFILE_STORAGE_NO_RECORD) {
    *output = controller_profile_default(identity, profile_index);
    return ProfileStorageResult::kOk;
  }
  return read_profile_record(entry->profile_record[profile_index], output)
             ? ProfileStorageResult::kOk
             : ProfileStorageResult::kIoError;
}

ProfileStorageResult ProfileStorage::set(const ControllerIdentity &identity,
                                         uint8_t profile_index,
                                         const ControllerProfile &profile) {
  if (!initialized_ || !valid_owner(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT ||
      !controller_profile_validate(profile)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  if (!controller_profile_encode(profile, encoded, sizeof(encoded))) {
    return ProfileStorageResult::kInvalidArgument;
  }
  ControllerProfile current{};
  uint8_t current_encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  if (get(identity, profile_index, &current) == ProfileStorageResult::kOk &&
      controller_profile_encode(current, current_encoded,
                                sizeof(current_encoded)) &&
      memcmp(encoded, current_encoded, sizeof(encoded)) == 0) {
    return ProfileStorageResult::kUnchanged;
  }
  if (find(identity) == nullptr &&
      identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  return append(RecordType::kProfile, identity, profile_index, encoded,
                sizeof(encoded));
}

ProfileStorageResult ProfileStorage::reset(const ControllerIdentity &identity,
                                           uint8_t profile_index) {
  if (!initialized_ || !valid_owner(identity) ||
      (profile_index != CONTROLLER_PROFILE_ALL &&
       profile_index >= CONTROLLER_PROFILE_COUNT)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry == nullptr) {
    return ProfileStorageResult::kUnchanged;
  }
  if (profile_index == CONTROLLER_PROFILE_ALL) {
    bool changed = entry->active_profile != 0;
    for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
      changed = changed ||
                entry->profile_record[profile] != PROFILE_STORAGE_NO_RECORD;
    }
    return changed ? append(RecordType::kResetAll, identity,
                            CONTROLLER_PROFILE_ALL, nullptr, 0)
                   : ProfileStorageResult::kUnchanged;
  }
  if (entry->profile_record[profile_index] == PROFILE_STORAGE_NO_RECORD) {
    return ProfileStorageResult::kUnchanged;
  }
  return append(RecordType::kReset, identity, profile_index, nullptr, 0);
}

ProfileStorageResult
ProfileStorage::activate(const ControllerIdentity &identity,
                         uint8_t profile_index) {
  if (!initialized_ || !valid_owner(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry != nullptr && entry->active_profile == profile_index) {
    return ProfileStorageResult::kUnchanged;
  }
  if (entry == nullptr &&
      identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  return append(RecordType::kActivate, identity, profile_index, nullptr, 0);
}

ProfileStorageResult ProfileStorage::get_alias(
    const ControllerIdentity &identity, char *output,
    size_t output_size) const {
  if (!initialized_ || output == nullptr || output_size == 0 ||
      !valid_owner(identity)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry == nullptr ||
      entry->alias_record == PROFILE_STORAGE_NO_RECORD) {
    output[0] = '\0';
    return ProfileStorageResult::kOk;
  }
  return read_metadata_record(entry->alias_record, output, output_size)
             ? ProfileStorageResult::kOk
             : ProfileStorageResult::kIoError;
}

ProfileStorageResult ProfileStorage::set_alias(
    const ControllerIdentity &identity, const char *value,
    size_t value_size) {
  if (!initialized_ || !valid_owner(identity) ||
      value_size > PROFILE_STORAGE_METADATA_MAX_BYTES ||
      (value_size != 0 && value == nullptr)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  char current[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  if (find(identity) == nullptr &&
      identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  if (get_alias(identity, current, sizeof(current)) ==
          ProfileStorageResult::kOk &&
      strlen(current) == value_size &&
      (value_size == 0 || memcmp(current, value, value_size) == 0)) {
    return ProfileStorageResult::kUnchanged;
  }
  uint8_t payload[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  payload[0] = static_cast<uint8_t>(value_size);
  if (value_size != 0) {
    memcpy(&payload[1], value, value_size);
  }
  return append(RecordType::kAlias, identity, CONTROLLER_PROFILE_ALL,
                payload, sizeof(payload));
}

ProfileStorageResult ProfileStorage::get_profile_name(
    const ControllerIdentity &identity, uint8_t profile_index,
    char *output, size_t output_size) const {
  if (!initialized_ || output == nullptr || output_size == 0 ||
      !valid_owner(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry == nullptr ||
      entry->profile_names_record == PROFILE_STORAGE_NO_RECORD) {
    output[0] = '\0';
    return ProfileStorageResult::kOk;
  }
  uint8_t payload[PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE]{};
  if (!read_record_payload(entry->profile_names_record,
                           RecordType::kProfileNames, payload,
                           sizeof(payload))) {
    return ProfileStorageResult::kIoError;
  }
  const size_t offset =
      profile_index * PROFILE_STORAGE_METADATA_PAYLOAD_SIZE;
  const size_t size = payload[offset];
  if (size > PROFILE_STORAGE_METADATA_MAX_BYTES ||
      output_size <= size) {
    return ProfileStorageResult::kInvalidArgument;
  }
  memcpy(output, &payload[offset + 1], size);
  output[size] = '\0';
  return ProfileStorageResult::kOk;
}

ProfileStorageResult ProfileStorage::set_profile_name(
    const ControllerIdentity &identity, uint8_t profile_index,
    const char *value, size_t value_size) {
  if (!initialized_ || !valid_owner(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT ||
      value_size > PROFILE_STORAGE_METADATA_MAX_BYTES ||
      (value_size != 0 && value == nullptr)) {
    return initialized_ ? ProfileStorageResult::kInvalidArgument
                        : ProfileStorageResult::kIoError;
  }
  const ProfileStorageIdentityIndex *entry = find(identity);
  if (entry == nullptr &&
      identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  char current[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  if (get_profile_name(identity, profile_index, current,
                       sizeof(current)) == ProfileStorageResult::kOk &&
      strlen(current) == value_size &&
      (value_size == 0 || memcmp(current, value, value_size) == 0)) {
    return ProfileStorageResult::kUnchanged;
  }
  uint8_t payload[PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE]{};
  if (entry != nullptr &&
      entry->profile_names_record != PROFILE_STORAGE_NO_RECORD &&
      !read_record_payload(entry->profile_names_record,
                           RecordType::kProfileNames, payload,
                           sizeof(payload))) {
    return ProfileStorageResult::kIoError;
  }
  const size_t offset =
      profile_index * PROFILE_STORAGE_METADATA_PAYLOAD_SIZE;
  memset(&payload[offset], 0, PROFILE_STORAGE_METADATA_PAYLOAD_SIZE);
  payload[offset] = static_cast<uint8_t>(value_size);
  if (value_size != 0) {
    memcpy(&payload[offset + 1], value, value_size);
  }
  return append(RecordType::kProfileNames, identity,
                CONTROLLER_PROFILE_ALL, payload, sizeof(payload));
}

uint8_t ProfileStorage::identity_count() const { return identity_count_; }

const ProfileStorageIdentityIndex *
ProfileStorage::identity(uint8_t index) const {
  return index < identity_count_ ? &index_[index] : nullptr;
}

const ProfileStorageIdentityIndex *
ProfileStorage::find(const ControllerIdentity &identity) const {
  for (uint8_t index = 0; index < identity_count_; ++index) {
    if (index_[index].used &&
        controller_identity_equal(index_[index].identity, identity)) {
      return &index_[index];
    }
  }
  return nullptr;
}

const ProfileStorageSnapshot &ProfileStorage::snapshot() const {
  return snapshot_;
}

ProfileStorageResult ProfileStorage::scan_arena(
    uint8_t arena, uint16_t *version, uint32_t *epoch, uint32_t *generation,
    uint32_t *payload_crc, size_t *next_offset,
    ProfileStorageIdentityIndex *index, uint8_t *identity_count) const {
  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  if (!io_.read(io_.context, arena, 0, superblock, sizeof(superblock))) {
    return ProfileStorageResult::kIoError;
  }
  *version = read_u16(&superblock[4]);
  if (memcmp(superblock, kSuperblockMagic, sizeof(kSuperblockMagic)) != 0 ||
      (*version != kCatalogVersion && *version != kExpandedCatalogVersion &&
       *version != kLegacyCatalogVersion) ||
      profile_storage_crc32(superblock, 12) != read_u32(&superblock[12]) ||
      !bytes_are(0, &superblock[16], sizeof(superblock) - 16)) {
    return ProfileStorageResult::kUnchanged;
  }
  *epoch = read_u32(&superblock[8]);
  *generation = 0;
  *payload_crc = 0;
  *next_offset = PROFILE_STORAGE_RECORDS_OFFSET;
  *identity_count = 1;
  index[0].used = true;
  index[0].identity = controller_identity_global();
  for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
    index[0].profile_record[profile] = PROFILE_STORAGE_NO_RECORD;
  }

  bool have_generation = false;
  for (size_t offset = PROFILE_STORAGE_RECORDS_OFFSET;
       offset + PROFILE_STORAGE_RECORD_SIZE <= PROFILE_STORAGE_ARENA_SIZE;
       offset += PROFILE_STORAGE_RECORD_SIZE) {
    uint8_t record[PROFILE_STORAGE_RECORD_SIZE]{};
    if (!io_.read(io_.context, arena, offset, record, sizeof(record))) {
      return ProfileStorageResult::kIoError;
    }
    // The second page is written first. Even a partial program anywhere in
    // that page consumes the slot when the header page is still erased.
    if (!bytes_are(0xff, record, sizeof(record))) {
      *next_offset = offset + PROFILE_STORAGE_RECORD_SIZE;
    }
    if (!validate_record(record, *version)) {
      continue;
    }
    ControllerIdentity identity_value{};
    controller_identity_decode(&record[20], CONTROLLER_IDENTITY_ENCODED_SIZE,
                               &identity_value);
    const uint32_t record_generation = read_u32(&record[8]);
    if (!apply_record(index, identity_count, static_cast<RecordType>(record[6]),
                      identity_value, record[7], record_generation,
                      pack_record(arena, offset))) {
      return ProfileStorageResult::kIoError;
    }
    if (!have_generation || generation_is_newer(record_generation, *generation)) {
      have_generation = true;
      *generation = record_generation;
      *payload_crc = read_u32(&record[16]);
    }
  }
  return ProfileStorageResult::kOk;
}

bool ProfileStorage::validate_record(const uint8_t *record,
                                      uint16_t version) const {
  const size_t payload_offset = version == kLegacyCatalogVersion
                                    ? kLegacyRecordPayloadOffset
                                    : kRecordPayloadOffset;
  if (memcmp(record, kRecordMagic, sizeof(kRecordMagic)) != 0 ||
      read_u16(&record[4]) != version ||
      profile_storage_crc32(record, kRecordHeaderCrcOffset) !=
          read_u32(&record[kRecordHeaderCrcOffset]) ||
      !bytes_are(0, &record[kRecordHeaderCrcOffset + 4],
                 payload_offset - kRecordHeaderCrcOffset - 4)) {
    return false;
  }
  const auto type = static_cast<RecordType>(record[6]);
  const uint8_t profile_index = record[7];
  const size_t payload_size = read_u16(&record[12]);
  const uint8_t *payload = &record[payload_offset];
  ControllerIdentity identity_value{};
  const bool known_type =
      type == RecordType::kProfile || type == RecordType::kReset ||
      type == RecordType::kResetAll || type == RecordType::kActivate ||
      type == RecordType::kAlias || type == RecordType::kProfileNames ||
      (version == kCatalogVersion && type == RecordType::kSeedJoyConPair);
  const bool indexed_profile =
      type == RecordType::kProfile || type == RecordType::kReset ||
      type == RecordType::kActivate;
  const size_t expected_payload_size =
      type == RecordType::kProfile
          ? (version == kLegacyCatalogVersion
                 ? CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE
                 : CONTROLLER_PROFILE_ENCODED_SIZE)
          : type == RecordType::kProfileNames
                ? PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE
                : type == RecordType::kAlias
                      ? PROFILE_STORAGE_METADATA_PAYLOAD_SIZE
                      : 0;
  if (!controller_identity_decode(
          &record[20], CONTROLLER_IDENTITY_ENCODED_SIZE, &identity_value) ||
      !valid_identity(identity_value) || !known_type ||
      (version != kCatalogVersion &&
       controller_identity_is_joycon_pair(identity_value)) ||
      (type == RecordType::kSeedJoyConPair &&
       (!controller_identity_is_joycon_pair(identity_value) ||
        profile_index != CONTROLLER_PROFILE_ALL)) ||
      (indexed_profile && profile_index >= CONTROLLER_PROFILE_COUNT) ||
      payload_size != expected_payload_size ||
      profile_storage_crc32(payload, payload_size) != read_u32(&record[16])) {
    return false;
  }
  if (type == RecordType::kProfile) {
    ControllerProfile decoded{};
    return read_u16(&record[14]) == read_u16(payload) &&
           controller_profile_decode(payload, payload_size, &decoded);
  }
  if (read_u16(&record[14]) != 0) {
    return false;
  }
  if (type == RecordType::kAlias) {
    return metadata_value_valid(payload, payload_size);
  }
  if (type == RecordType::kProfileNames) {
    for (size_t name = 0; name < CONTROLLER_PROFILE_COUNT; ++name) {
      if (!metadata_value_valid(
              &payload[name * PROFILE_STORAGE_METADATA_PAYLOAD_SIZE],
              PROFILE_STORAGE_METADATA_PAYLOAD_SIZE)) {
        return false;
      }
    }
  }
  return true;
}

bool ProfileStorage::read_record_payload(
    uint32_t record, RecordType type, uint8_t *output, size_t capacity,
    size_t *size) const {
  uint8_t header[kRecordHeaderCrcOffset + 4]{};
  if (record == PROFILE_STORAGE_NO_RECORD ||
      !io_.read(io_.context, record_arena(record), record_offset(record),
                header, sizeof(header)) ||
      memcmp(header, kRecordMagic, sizeof(kRecordMagic)) != 0 ||
      header[6] != static_cast<uint8_t>(type) ||
      profile_storage_crc32(header, kRecordHeaderCrcOffset) !=
          read_u32(&header[kRecordHeaderCrcOffset])) {
    return false;
  }
  const uint16_t version = read_u16(&header[4]);
  const size_t payload_offset = version == kLegacyCatalogVersion
                                    ? kLegacyRecordPayloadOffset
                                    : kRecordPayloadOffset;
  const size_t payload_size = read_u16(&header[12]);
  if ((version != kCatalogVersion && version != kExpandedCatalogVersion &&
       version != kLegacyCatalogVersion) ||
      payload_size > capacity ||
      payload_size > PROFILE_STORAGE_RECORD_SIZE - payload_offset ||
      !io_.read(io_.context, record_arena(record),
                record_offset(record) + payload_offset, output, payload_size) ||
      profile_storage_crc32(output, payload_size) != read_u32(&header[16])) {
    return false;
  }
  if (size != nullptr) {
    *size = payload_size;
  }
  return true;
}

bool ProfileStorage::read_profile_record(uint32_t record,
                                         ControllerProfile *output) const {
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  size_t size = 0;
  return read_record_payload(record, RecordType::kProfile, encoded,
                             sizeof(encoded), &size) &&
         controller_profile_decode(encoded, size, output);
}

bool ProfileStorage::read_metadata_record(
    uint32_t record, char *output, size_t output_size) const {
  uint8_t payload[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  if (!read_record_payload(record, RecordType::kAlias, payload,
                           sizeof(payload)) ||
      payload[0] > PROFILE_STORAGE_METADATA_MAX_BYTES ||
      output_size <= payload[0]) {
    return false;
  }
  memcpy(output, &payload[1], payload[0]);
  output[payload[0]] = '\0';
  return true;
}

ProfileStorageResult ProfileStorage::append(RecordType type,
                                            const ControllerIdentity &identity,
                                            uint8_t profile_index,
                                            const uint8_t *payload,
                                            size_t payload_size) {
  if (next_offset_ + PROFILE_STORAGE_RECORD_SIZE > PROFILE_STORAGE_ARENA_SIZE) {
    const ProfileStorageResult result = compact();
    if (result != ProfileStorageResult::kOk) {
      initialized_ = false;
      return result;
    }
  }
  const uint32_t generation = snapshot_.generation + 1u;
  const size_t offset = next_offset_;
  next_offset_ += PROFILE_STORAGE_RECORD_SIZE;
  if (!write_record(snapshot_.active_bank, offset, type, identity,
                    profile_index, generation, payload, payload_size)) {
    // The header may have committed even when program/readback reports an
    // error. Freeze all mutations until initialize() replays durable state;
    // otherwise a retry could seed from a different left snapshot.
    initialized_ = false;
    return ProfileStorageResult::kIoError;
  }
  if (!apply_record(index_, &identity_count_, type, identity, profile_index,
                    generation, pack_record(snapshot_.active_bank, offset))) {
    initialized_ = false;
    return ProfileStorageResult::kIoError;
  }
  snapshot_.generation = generation;
  snapshot_.payload_crc = profile_storage_crc32(payload, payload_size);
  return ProfileStorageResult::kOk;
}

ProfileStorageResult ProfileStorage::compact() {
  const uint8_t target = snapshot_.active_bank ^ 1u;
  if (!io_.erase_arena(io_.context, target)) {
    return ProfileStorageResult::kIoError;
  }
  size_t offset = PROFILE_STORAGE_RECORDS_OFFSET;
  uint32_t generation = snapshot_.generation;
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  for (uint8_t identity_index = 0; identity_index < identity_count_;
       ++identity_index) {
    const ProfileStorageIdentityIndex &entry = index_[identity_index];
    for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
      if (entry.profile_record[profile] == PROFILE_STORAGE_NO_RECORD) {
        continue;
      }
      ControllerProfile decoded{};
      if (!read_profile_record(entry.profile_record[profile], &decoded) ||
          !controller_profile_encode(decoded, encoded, sizeof(encoded)) ||
          !write_record(target, offset, RecordType::kProfile, entry.identity,
                        profile, ++generation, encoded, sizeof(encoded))) {
        return ProfileStorageResult::kIoError;
      }
      offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    uint8_t metadata[PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE]{};
    if (entry.alias_record != PROFILE_STORAGE_NO_RECORD) {
      if (!read_record_payload(entry.alias_record, RecordType::kAlias,
                               metadata, PROFILE_STORAGE_METADATA_PAYLOAD_SIZE) ||
          !write_record(
              target, offset, RecordType::kAlias, entry.identity,
              CONTROLLER_PROFILE_ALL, ++generation, metadata,
              PROFILE_STORAGE_METADATA_PAYLOAD_SIZE)) {
        return ProfileStorageResult::kIoError;
      }
      offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    if (entry.profile_names_record != PROFILE_STORAGE_NO_RECORD) {
      if (!read_record_payload(entry.profile_names_record,
                               RecordType::kProfileNames, metadata,
                               sizeof(metadata)) ||
          !write_record(
              target, offset, RecordType::kProfileNames,
              entry.identity, CONTROLLER_PROFILE_ALL, ++generation,
              metadata, sizeof(metadata))) {
        return ProfileStorageResult::kIoError;
      }
      offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    if (!write_record(target, offset, RecordType::kActivate, entry.identity,
                      entry.active_profile, ++generation, nullptr, 0)) {
      return ProfileStorageResult::kIoError;
    }
    offset += PROFILE_STORAGE_RECORD_SIZE;
  }

  if (!publish_arena(target, epoch_ + 1u, offset)) {
    return ProfileStorageResult::kIoError;
  }

  ProfileStorageIdentityIndex
      rebuilt[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t rebuilt_count = 0;
  uint16_t rebuilt_version = 0;
  uint32_t rebuilt_epoch = 0;
  uint32_t rebuilt_generation = 0;
  uint32_t rebuilt_payload_crc = 0;
  size_t rebuilt_offset = 0;
  if (scan_arena(target, &rebuilt_version, &rebuilt_epoch,
                 &rebuilt_generation, &rebuilt_payload_crc, &rebuilt_offset,
                 rebuilt, &rebuilt_count) != ProfileStorageResult::kOk ||
      rebuilt_count != identity_count_ || rebuilt_offset != offset ||
      rebuilt_generation != generation) {
    return ProfileStorageResult::kIoError;
  }
  for (size_t index = 0;
       index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
    index_[index] = rebuilt[index];
  }
  identity_count_ = rebuilt_count;
  epoch_ = rebuilt_epoch;
  catalog_version_ = rebuilt_version;
  next_offset_ = rebuilt_offset;
  snapshot_.active_bank = target;
  snapshot_.generation = rebuilt_generation;
  snapshot_.payload_crc = rebuilt_payload_crc;
  snapshot_.valid = true;
  return ProfileStorageResult::kOk;
}

ProfileStorageResult ProfileStorage::migrate_legacy() {
  struct LegacyHeader {
    bool valid = false;
    uint32_t generation = 0;
    uint32_t crc = 0;
    uint8_t bank = 0;
  } headers[PROFILE_STORAGE_LEGACY_BANK_COUNT]{};
  const uint8_t arena =
      static_cast<uint8_t>(kLegacyStart / PROFILE_STORAGE_ARENA_SIZE);
  const size_t arena_base = kLegacyStart % PROFILE_STORAGE_ARENA_SIZE;
  for (uint8_t bank = 0; bank < PROFILE_STORAGE_LEGACY_BANK_COUNT; ++bank) {
    uint8_t header[PROFILE_STORAGE_LEGACY_HEADER_SIZE]{};
    const size_t base = arena_base + bank * PROFILE_STORAGE_LEGACY_BANK_SIZE;
    if (!io_.read(io_.context, arena, base, header, sizeof(header))) {
      return ProfileStorageResult::kIoError;
    }
    if (memcmp(header, kLegacyStorageMagic, 4) != 0 ||
        read_u16(&header[4]) != 1 ||
        (read_u16(&header[6]) != 1 && read_u16(&header[6]) != 2) ||
        read_u32(&header[12]) != PROFILE_STORAGE_LEGACY_DATABASE_SIZE ||
        profile_storage_crc32(header, 20) != read_u32(&header[20]) ||
        !bytes_are(0, &header[24], sizeof(header) - 24)) {
      continue;
    }
    uint8_t page[PROFILE_STORAGE_PAGE_SIZE]{};
    uint32_t crc = 0xffffffffu;
    size_t remaining = PROFILE_STORAGE_LEGACY_DATABASE_SIZE;
    size_t payload_offset = base + PROFILE_STORAGE_LEGACY_HEADER_SIZE;
    while (remaining != 0) {
      const size_t size = remaining < sizeof(page) ? remaining : sizeof(page);
      if (!io_.read(io_.context, arena, payload_offset, page, size)) {
        return ProfileStorageResult::kIoError;
      }
      crc = crc32_update(crc, page, size);
      payload_offset += size;
      remaining -= size;
    }
    if (remaining == 0 && ~crc == read_u32(&header[16])) {
      headers[bank] = {true, read_u32(&header[8]), read_u32(&header[16]), bank};
    }
  }
  int selected = -1;
  for (uint8_t bank = 0; bank < PROFILE_STORAGE_LEGACY_BANK_COUNT; ++bank) {
    if (headers[bank].valid &&
        (selected < 0 || generation_is_newer(headers[bank].generation,
                                             headers[selected].generation))) {
      selected = bank;
    }
  }
  if (selected < 0) {
    return ProfileStorageResult::kUnchanged;
  }

  const size_t legacy_base =
      arena_base +
      static_cast<size_t>(selected) * PROFILE_STORAGE_LEGACY_BANK_SIZE +
      PROFILE_STORAGE_LEGACY_HEADER_SIZE;
  uint8_t database_header[32]{};
  if (!io_.read(io_.context, arena, legacy_base, database_header,
                sizeof(database_header)) ||
      memcmp(database_header, kLegacyDatabaseMagic, 4) != 0 ||
      (read_u16(&database_header[4]) != 1 &&
       read_u16(&database_header[4]) != 2) ||
      read_u16(&database_header[6]) != PROFILE_STORAGE_LEGACY_DATABASE_SIZE ||
      database_header[8] != CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY ||
      database_header[9] != PROFILE_STORAGE_LEGACY_PROFILE_COUNT ||
      database_header[10] >= PROFILE_STORAGE_LEGACY_PROFILE_COUNT) {
    return ProfileStorageResult::kIoError;
  }
  if (!io_.erase_arena(io_.context, 0)) {
    return ProfileStorageResult::kIoError;
  }
  size_t target_offset = PROFILE_STORAGE_RECORDS_OFFSET;
  uint32_t generation = headers[selected].generation;
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  ControllerProfile decoded{};
  const ControllerIdentity global = controller_identity_global();
  for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
       ++profile) {
    const size_t source =
        legacy_base + 32 + profile * CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
    if (!io_.read(io_.context, arena, source, encoded,
                   CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE) ||
        !controller_profile_decode(encoded,
                                   CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE,
                                   &decoded) ||
        !controller_profile_encode(decoded, encoded, sizeof(encoded)) ||
        !write_record(0, target_offset, RecordType::kProfile, global, profile,
                      ++generation, encoded, sizeof(encoded))) {
      return ProfileStorageResult::kIoError;
    }
    target_offset += PROFILE_STORAGE_RECORD_SIZE;
  }
  if (!write_record(0, target_offset, RecordType::kActivate, global,
                    database_header[10], ++generation, nullptr, 0)) {
    return ProfileStorageResult::kIoError;
  }
  target_offset += PROFILE_STORAGE_RECORD_SIZE;

  constexpr size_t kLegacyEntrySize =
      16 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT *
          CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
  const size_t entries_base =
      legacy_base + 32 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT *
          CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
  for (uint8_t entry_index = 0;
       entry_index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
       ++entry_index) {
    const size_t entry_base = entries_base + entry_index * kLegacyEntrySize;
    uint8_t entry_header[16]{};
    if (!io_.read(io_.context, arena, entry_base, entry_header,
                  sizeof(entry_header))) {
      return ProfileStorageResult::kIoError;
    }
    if (entry_header[15] == 0) {
      continue;
    }
    ControllerIdentity identity_value{};
    if (entry_header[15] != 1 ||
        entry_header[14] >= PROFILE_STORAGE_LEGACY_PROFILE_COUNT ||
        !controller_identity_decode(
            entry_header, CONTROLLER_IDENTITY_ENCODED_SIZE, &identity_value) ||
        !identity_value.stable ||
        controller_identity_is_global(identity_value) ||
        controller_identity_is_joycon_pair(identity_value)) {
      return ProfileStorageResult::kIoError;
    }
    for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
         ++profile) {
      const size_t source =
          entry_base + 16 + profile * CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
      if (!io_.read(io_.context, arena, source, encoded,
                     CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE) ||
          !controller_profile_decode(encoded,
                                     CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE,
                                     &decoded) ||
          !controller_profile_encode(decoded, encoded, sizeof(encoded)) ||
          !write_record(0, target_offset, RecordType::kProfile, identity_value,
                        profile, ++generation, encoded, sizeof(encoded))) {
        return ProfileStorageResult::kIoError;
      }
      target_offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    if (!write_record(0, target_offset, RecordType::kActivate, identity_value,
                      entry_header[14], ++generation, nullptr, 0)) {
      return ProfileStorageResult::kIoError;
    }
    target_offset += PROFILE_STORAGE_RECORD_SIZE;
  }

  if (!publish_arena(0, 1, target_offset)) {
    return ProfileStorageResult::kIoError;
  }

  ProfileStorageIdentityIndex
      rebuilt[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t rebuilt_count = 0;
  uint16_t rebuilt_version = 0;
  uint32_t rebuilt_epoch = 0;
  uint32_t rebuilt_generation = 0;
  uint32_t rebuilt_payload_crc = 0;
  size_t rebuilt_offset = 0;
  if (scan_arena(0, &rebuilt_version, &rebuilt_epoch, &rebuilt_generation,
                 &rebuilt_payload_crc, &rebuilt_offset, rebuilt,
                 &rebuilt_count) != ProfileStorageResult::kOk ||
      rebuilt_offset != target_offset || rebuilt_generation != generation) {
    return ProfileStorageResult::kIoError;
  }
  for (size_t index = 0;
       index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
    index_[index] = rebuilt[index];
  }
  identity_count_ = rebuilt_count;
  epoch_ = rebuilt_epoch;
  catalog_version_ = rebuilt_version;
  next_offset_ = rebuilt_offset;
  snapshot_.valid = true;
  snapshot_.active_bank = 0;
  snapshot_.generation = rebuilt_generation;
  snapshot_.payload_crc = rebuilt_payload_crc;
  return ProfileStorageResult::kOk;
}

bool ProfileStorage::publish_arena(uint8_t arena, uint32_t epoch,
                                    size_t records_end) const {
  // Verify every copied record before making the new arena discoverable.
  // The old published arena remains untouched throughout this operation.
  for (size_t offset = PROFILE_STORAGE_RECORDS_OFFSET; offset < records_end;
       offset += PROFILE_STORAGE_RECORD_SIZE) {
    uint8_t record[PROFILE_STORAGE_RECORD_SIZE]{};
    if (!io_.read(io_.context, arena, offset, record, sizeof(record)) ||
        !validate_record(record, kCatalogVersion)) {
      return false;
    }
  }
  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  memcpy(superblock, kSuperblockMagic, sizeof(kSuperblockMagic));
  write_u16(&superblock[4], kCatalogVersion);
  write_u32(&superblock[8], epoch);
  write_u32(&superblock[12], profile_storage_crc32(superblock, 12));
  uint8_t verified[PROFILE_STORAGE_PAGE_SIZE]{};
  return io_.program_page(io_.context, arena, 0, superblock,
                          sizeof(superblock)) &&
         io_.read(io_.context, arena, 0, verified, sizeof(verified)) &&
         memcmp(superblock, verified, sizeof(superblock)) == 0;
}

bool ProfileStorage::publish_empty_arena(uint8_t arena, uint32_t epoch) {
  if (!io_.erase_arena(io_.context, arena)) {
    return false;
  }
  if (!publish_arena(arena, epoch, PROFILE_STORAGE_RECORDS_OFFSET)) {
    return false;
  }
  epoch_ = epoch;
  catalog_version_ = kCatalogVersion;
  next_offset_ = PROFILE_STORAGE_RECORDS_OFFSET;
  snapshot_.valid = true;
  snapshot_.active_bank = arena;
  snapshot_.generation = 0;
  snapshot_.payload_crc = 0;
  return true;
}

bool ProfileStorage::write_record(uint8_t arena, size_t offset, RecordType type,
                                  const ControllerIdentity &identity,
                                  uint8_t profile_index, uint32_t generation,
                                  const uint8_t *payload,
                                  size_t payload_size) const {
  const size_t expected_payload_size =
      type == RecordType::kProfile
          ? CONTROLLER_PROFILE_ENCODED_SIZE
          : type == RecordType::kProfileNames
                ? PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE
                : type == RecordType::kAlias
                      ? PROFILE_STORAGE_METADATA_PAYLOAD_SIZE
                      : 0;
  if (arena >= PROFILE_STORAGE_ARENA_COUNT ||
      offset < PROFILE_STORAGE_RECORDS_OFFSET ||
      offset + PROFILE_STORAGE_RECORD_SIZE > PROFILE_STORAGE_ARENA_SIZE ||
      offset % PROFILE_STORAGE_RECORD_SIZE != 0 ||
      payload_size != expected_payload_size ||
      (payload_size != 0 && payload == nullptr)) {
    return false;
  }
  uint8_t record[PROFILE_STORAGE_RECORD_SIZE]{};
  memcpy(record, kRecordMagic, sizeof(kRecordMagic));
  write_u16(&record[4], kCatalogVersion);
  record[6] = static_cast<uint8_t>(type);
  record[7] = profile_index;
  write_u32(&record[8], generation);
  write_u16(&record[12], static_cast<uint16_t>(payload_size));
  write_u16(&record[14], type == RecordType::kProfile ? read_u16(payload) : 0);
  write_u32(&record[16], profile_storage_crc32(payload, payload_size));
  if (!controller_identity_encode(identity, &record[20],
                                  CONTROLLER_IDENTITY_ENCODED_SIZE)) {
    return false;
  }
  write_u32(&record[kRecordHeaderCrcOffset],
            profile_storage_crc32(record, kRecordHeaderCrcOffset));
  if (payload_size != 0) {
    memcpy(&record[kRecordPayloadOffset], payload, payload_size);
  }
  // Publish the header-containing page last, including for short metadata and
  // zero-payload records, so every interrupted slot is detectably consumed.
  if (!io_.program_page(io_.context, arena, offset + PROFILE_STORAGE_PAGE_SIZE,
                         &record[PROFILE_STORAGE_PAGE_SIZE],
                         PROFILE_STORAGE_PAGE_SIZE) ||
      !io_.program_page(io_.context, arena, offset, record,
                         PROFILE_STORAGE_PAGE_SIZE)) {
    return false;
  }
  uint8_t verified[PROFILE_STORAGE_RECORD_SIZE]{};
  return io_.read(io_.context, arena, offset, verified, sizeof(verified)) &&
         memcmp(record, verified, sizeof(record)) == 0;
}

bool ProfileStorage::apply_record(ProfileStorageIdentityIndex *index,
                                  uint8_t *identity_count, RecordType type,
                                  const ControllerIdentity &identity,
                                  uint8_t profile_index, uint32_t generation,
                                  uint32_t record) const {
  ProfileStorageIdentityIndex *entry = nullptr;
  for (uint8_t current = 0; current < *identity_count; ++current) {
    if (index[current].used &&
        controller_identity_equal(index[current].identity, identity)) {
      entry = &index[current];
      break;
    }
  }
  if (type == RecordType::kSeedJoyConPair) {
    if (entry != nullptr) {
      return true; // A duplicate seed can never overwrite pair-owned edits.
    }
    ControllerIdentity left{};
    ControllerIdentity right{};
    if (!controller_identity_joycon_pair_members(identity, &left, &right) ||
        *identity_count >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
      return false;
    }
    const ProfileStorageIdentityIndex *source = nullptr;
    bool have_right = false;
    for (uint8_t current = 0; current < *identity_count; ++current) {
      if (index[current].used &&
          controller_identity_equal(index[current].identity, left)) {
        source = &index[current];
      }
      if (index[current].used &&
          controller_identity_equal(index[current].identity, right)) {
        have_right = true;
      }
    }
    if (source == nullptr || !have_right) {
      return false;
    }
    entry = &index[(*identity_count)++];
    *entry = *source;
    entry->identity = identity;
    entry->alias_record = PROFILE_STORAGE_NO_RECORD;
    entry->alias_generation = 0;
    entry->active_generation = generation;
    entry->profile_names_generation = generation;
    for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
      entry->profile_generation[profile] = generation;
    }
    return true;
  }
  if (entry == nullptr) {
    if (*identity_count >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
      return false;
    }
    entry = &index[(*identity_count)++];
    *entry = {};
    entry->used = true;
    entry->identity = identity;
    for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
      entry->profile_record[profile] = PROFILE_STORAGE_NO_RECORD;
    }
  }
  if (type == RecordType::kActivate) {
    if (entry->active_generation == 0 ||
        generation_is_newer(generation, entry->active_generation)) {
      entry->active_profile = profile_index;
      entry->active_generation = generation;
    }
    return true;
  }
  if (type == RecordType::kAlias) {
    if (entry->alias_generation == 0 ||
        generation_is_newer(generation, entry->alias_generation)) {
      entry->alias_record = record;
      entry->alias_generation = generation;
    }
    return true;
  }
  if (type == RecordType::kProfileNames) {
    if (entry->profile_names_generation == 0 ||
        generation_is_newer(
            generation, entry->profile_names_generation)) {
      entry->profile_names_record = record;
      entry->profile_names_generation = generation;
    }
    return true;
  }
  if (type == RecordType::kResetAll) {
    for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
      if (entry->profile_generation[profile] == 0 ||
          generation_is_newer(generation, entry->profile_generation[profile])) {
        entry->profile_record[profile] = PROFILE_STORAGE_NO_RECORD;
        entry->profile_generation[profile] = generation;
      }
    }
    if (entry->active_generation == 0 ||
        generation_is_newer(generation, entry->active_generation)) {
      entry->active_profile = 0;
      entry->active_generation = generation;
    }
    return true;
  }
  if (entry->profile_generation[profile_index] == 0 ||
      generation_is_newer(generation,
                          entry->profile_generation[profile_index])) {
    entry->profile_record[profile_index] =
        type == RecordType::kProfile ? record : PROFILE_STORAGE_NO_RECORD;
    entry->profile_generation[profile_index] = generation;
  }
  return true;
}
