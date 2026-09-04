#include "profile/profile_storage.h"

#include <string.h>

namespace {

constexpr uint8_t kSuperblockMagic[4] = {'S', 'P', 'C', 'A'};
constexpr uint8_t kRecordMagic[4] = {'S', 'P', 'C', 'R'};
constexpr uint8_t kLegacyStorageMagic[4] = {'S', 'P', 'P', 'F'};
constexpr uint8_t kLegacyDatabaseMagic[4] = {'S', 'P', 'D', 'B'};
constexpr uint16_t kCatalogVersion = 1;
constexpr size_t kRecordPayloadOffset = PROFILE_STORAGE_PAGE_SIZE;
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
  next_offset_ = PROFILE_STORAGE_RECORDS_OFFSET;
  initialized_ = io_.read != nullptr && io_.erase_arena != nullptr &&
                 io_.program_page != nullptr &&
                 io_.arena_size == PROFILE_STORAGE_ARENA_SIZE &&
                 io_.sector_size == PROFILE_STORAGE_SECTOR_SIZE &&
                 io_.page_size == PROFILE_STORAGE_PAGE_SIZE;
  if (!initialized_) {
    return false;
  }

  ProfileStorageIdentityIndex
      candidate[PROFILE_STORAGE_ARENA_COUNT]
               [CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t counts[PROFILE_STORAGE_ARENA_COUNT]{};
  uint32_t epochs[PROFILE_STORAGE_ARENA_COUNT]{};
  uint32_t generations[PROFILE_STORAGE_ARENA_COUNT]{};
  uint32_t payload_crcs[PROFILE_STORAGE_ARENA_COUNT]{};
  size_t offsets[PROFILE_STORAGE_ARENA_COUNT]{};
  bool valid[PROFILE_STORAGE_ARENA_COUNT]{};
  for (uint8_t arena = 0; arena < PROFILE_STORAGE_ARENA_COUNT; ++arena) {
    valid[arena] = scan_arena(arena, &epochs[arena], &generations[arena],
                              &payload_crcs[arena], &offsets[arena],
                              candidate[arena], &counts[arena]);
  }

  uint8_t selected = 0;
  if (valid[1] && (!valid[0] || generation_is_newer(epochs[1], epochs[0]))) {
    selected = 1;
  }
  if (valid[selected]) {
    for (size_t index = 0;
         index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
      index_[index] = candidate[selected][index];
    }
    identity_count_ = counts[selected];
    epoch_ = epochs[selected];
    next_offset_ = offsets[selected];
    snapshot_.valid = true;
    snapshot_.active_bank = selected;
    snapshot_.generation = generations[selected];
    snapshot_.payload_crc = payload_crcs[selected];
    return true;
  }

  if (migrate_legacy()) {
    return true;
  }
  for (ProfileStorageIdentityIndex &entry : index_) {
    entry = {};
  }
  identity_count_ = 1;
  index_[0].used = true;
  index_[0].identity = controller_identity_global();
  for (uint8_t profile = 0; profile < CONTROLLER_PROFILE_COUNT; ++profile) {
    index_[0].profile_record[profile] = PROFILE_STORAGE_NO_RECORD;
  }
  return publish_empty_arena(0, 1);
}

ProfileStorageResult
ProfileStorage::ensure_identity(const ControllerIdentity &identity) {
  if (!initialized_ || !valid_identity(identity)) {
    return ProfileStorageResult::kInvalidArgument;
  }
  if (find(identity) != nullptr) {
    return ProfileStorageResult::kUnchanged;
  }
  if (identity_count_ >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
    return ProfileStorageResult::kFull;
  }
  return append(RecordType::kActivate, identity, 0, nullptr, 0);
}

ProfileStorageResult ProfileStorage::get(const ControllerIdentity &identity,
                                         uint8_t profile_index,
                                         ControllerProfile *output) const {
  if (!initialized_ || output == nullptr || !valid_identity(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT) {
    return ProfileStorageResult::kInvalidArgument;
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
  if (!initialized_ || !valid_identity(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT ||
      !controller_profile_validate(profile)) {
    return ProfileStorageResult::kInvalidArgument;
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
  if (!initialized_ || !valid_identity(identity) ||
      (profile_index != CONTROLLER_PROFILE_ALL &&
       profile_index >= CONTROLLER_PROFILE_COUNT)) {
    return ProfileStorageResult::kInvalidArgument;
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
  if (!initialized_ || !valid_identity(identity) ||
      profile_index >= CONTROLLER_PROFILE_COUNT) {
    return ProfileStorageResult::kInvalidArgument;
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

bool ProfileStorage::scan_arena(uint8_t arena, uint32_t *epoch,
                                uint32_t *generation, uint32_t *payload_crc,
                                size_t *next_offset,
                                ProfileStorageIdentityIndex *index,
                                uint8_t *identity_count) const {
  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  if (!io_.read(io_.context, arena, 0, superblock, sizeof(superblock)) ||
      memcmp(superblock, kSuperblockMagic, sizeof(kSuperblockMagic)) != 0 ||
      read_u16(&superblock[4]) != kCatalogVersion ||
      profile_storage_crc32(superblock, 12) != read_u32(&superblock[12]) ||
      !bytes_are(0, &superblock[16], sizeof(superblock) - 16)) {
    return false;
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

  for (size_t offset = PROFILE_STORAGE_RECORDS_OFFSET;
       offset + PROFILE_STORAGE_RECORD_SIZE <= PROFILE_STORAGE_ARENA_SIZE;
       offset += PROFILE_STORAGE_RECORD_SIZE) {
    uint8_t header[PROFILE_STORAGE_PAGE_SIZE]{};
    uint8_t payload_prefix[4]{};
    if (!io_.read(io_.context, arena, offset, header, sizeof(header)) ||
        !io_.read(io_.context, arena, offset + kRecordPayloadOffset,
                  payload_prefix, sizeof(payload_prefix))) {
      return false;
    }
    if (!bytes_are(0xff, header, 4) ||
        !bytes_are(0xff, payload_prefix, sizeof(payload_prefix))) {
      *next_offset = offset + PROFILE_STORAGE_RECORD_SIZE;
    }
    if (memcmp(header, kRecordMagic, sizeof(kRecordMagic)) != 0 ||
        read_u16(&header[4]) != kCatalogVersion ||
        profile_storage_crc32(header, kRecordHeaderCrcOffset) !=
            read_u32(&header[kRecordHeaderCrcOffset]) ||
        !bytes_are(0, &header[kRecordHeaderCrcOffset + 4],
                   sizeof(header) - kRecordHeaderCrcOffset - 4)) {
      continue;
    }
    const auto type = static_cast<RecordType>(header[6]);
    const uint8_t profile_index = header[7];
    const uint32_t record_generation = read_u32(&header[8]);
    const size_t payload_size = read_u16(&header[12]);
    ControllerIdentity identity_value{};
    if (!controller_identity_decode(
            &header[20], CONTROLLER_IDENTITY_ENCODED_SIZE, &identity_value) ||
        !valid_identity(identity_value) ||
        (type != RecordType::kProfile && type != RecordType::kReset &&
         type != RecordType::kResetAll && type != RecordType::kActivate) ||
        ((type == RecordType::kProfile || type == RecordType::kReset ||
          type == RecordType::kActivate) &&
         profile_index >= CONTROLLER_PROFILE_COUNT) ||
        (type == RecordType::kProfile &&
         payload_size != CONTROLLER_PROFILE_ENCODED_SIZE) ||
        (type != RecordType::kProfile && payload_size != 0)) {
      continue;
    }
    if (type == RecordType::kProfile) {
      uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
      ControllerProfile decoded{};
      if (!io_.read(io_.context, arena, offset + kRecordPayloadOffset, payload,
                    sizeof(payload)) ||
          read_u16(&header[14]) != read_u16(payload) ||
          profile_storage_crc32(payload, sizeof(payload)) !=
              read_u32(&header[16]) ||
          !controller_profile_decode(payload, sizeof(payload), &decoded)) {
        continue;
      }
    } else if (read_u16(&header[14]) != 0) {
      continue;
    }
    apply_record(index, identity_count, type, identity_value, profile_index,
                 record_generation, pack_record(arena, offset));
    if (generation_is_newer(record_generation, *generation)) {
      *generation = record_generation;
      *payload_crc = read_u32(&header[16]);
    }
  }
  return true;
}

bool ProfileStorage::read_profile_record(uint32_t record,
                                         ControllerProfile *output) const {
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  return record != PROFILE_STORAGE_NO_RECORD &&
         io_.read(io_.context, record_arena(record),
                  record_offset(record) + kRecordPayloadOffset, encoded,
                  sizeof(encoded)) &&
         controller_profile_decode(encoded, sizeof(encoded), output);
}

ProfileStorageResult ProfileStorage::append(RecordType type,
                                            const ControllerIdentity &identity,
                                            uint8_t profile_index,
                                            const uint8_t *payload,
                                            size_t payload_size) {
  if (next_offset_ + PROFILE_STORAGE_RECORD_SIZE > PROFILE_STORAGE_ARENA_SIZE) {
    const ProfileStorageResult result = compact();
    if (result != ProfileStorageResult::kOk) {
      return result;
    }
  }
  const uint32_t generation = snapshot_.generation + 1u;
  const size_t offset = next_offset_;
  next_offset_ += PROFILE_STORAGE_RECORD_SIZE;
  if (!write_record(snapshot_.active_bank, offset, type, identity,
                    profile_index, generation, payload, payload_size)) {
    return ProfileStorageResult::kIoError;
  }
  apply_record(index_, &identity_count_, type, identity, profile_index,
               generation, pack_record(snapshot_.active_bank, offset));
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
    if (!write_record(target, offset, RecordType::kActivate, entry.identity,
                      entry.active_profile, ++generation, nullptr, 0)) {
      return ProfileStorageResult::kIoError;
    }
    offset += PROFILE_STORAGE_RECORD_SIZE;
  }

  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  memcpy(superblock, kSuperblockMagic, sizeof(kSuperblockMagic));
  write_u16(&superblock[4], kCatalogVersion);
  write_u32(&superblock[8], epoch_ + 1u);
  write_u32(&superblock[12], profile_storage_crc32(superblock, 12));
  if (!io_.program_page(io_.context, target, 0, superblock,
                        sizeof(superblock))) {
    return ProfileStorageResult::kIoError;
  }

  ProfileStorageIdentityIndex
      rebuilt[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t rebuilt_count = 0;
  uint32_t rebuilt_epoch = 0;
  uint32_t rebuilt_generation = 0;
  uint32_t rebuilt_payload_crc = 0;
  size_t rebuilt_offset = 0;
  if (!scan_arena(target, &rebuilt_epoch, &rebuilt_generation,
                  &rebuilt_payload_crc, &rebuilt_offset, rebuilt,
                  &rebuilt_count)) {
    return ProfileStorageResult::kIoError;
  }
  for (size_t index = 0;
       index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
    index_[index] = rebuilt[index];
  }
  identity_count_ = rebuilt_count;
  epoch_ = rebuilt_epoch;
  next_offset_ = rebuilt_offset;
  snapshot_.active_bank = target;
  snapshot_.generation = rebuilt_generation;
  snapshot_.payload_crc = rebuilt_payload_crc;
  snapshot_.valid = true;
  return ProfileStorageResult::kOk;
}

bool ProfileStorage::migrate_legacy() {
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
    if (!io_.read(io_.context, arena, base, header, sizeof(header)) ||
        memcmp(header, kLegacyStorageMagic, 4) != 0 ||
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
        remaining = SIZE_MAX;
        break;
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
    return false;
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
    return false;
  }
  if (!io_.erase_arena(io_.context, 0)) {
    return false;
  }
  size_t target_offset = PROFILE_STORAGE_RECORDS_OFFSET;
  uint32_t generation = 0;
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  ControllerProfile decoded{};
  const ControllerIdentity global = controller_identity_global();
  for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
       ++profile) {
    const size_t source =
        legacy_base + 32 + profile * CONTROLLER_PROFILE_ENCODED_SIZE;
    if (!io_.read(io_.context, arena, source, encoded, sizeof(encoded)) ||
        !controller_profile_decode(encoded, sizeof(encoded), &decoded) ||
        !write_record(0, target_offset, RecordType::kProfile, global, profile,
                      ++generation, encoded, sizeof(encoded))) {
      return false;
    }
    target_offset += PROFILE_STORAGE_RECORD_SIZE;
  }
  if (!write_record(0, target_offset, RecordType::kActivate, global,
                    database_header[10], ++generation, nullptr, 0)) {
    return false;
  }
  target_offset += PROFILE_STORAGE_RECORD_SIZE;

  constexpr size_t kLegacyEntrySize =
      16 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
  const size_t entries_base =
      legacy_base + 32 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
  for (uint8_t entry_index = 0;
       entry_index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
       ++entry_index) {
    const size_t entry_base = entries_base + entry_index * kLegacyEntrySize;
    uint8_t entry_header[16]{};
    if (!io_.read(io_.context, arena, entry_base, entry_header,
                  sizeof(entry_header))) {
      return false;
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
        controller_identity_is_global(identity_value)) {
      return false;
    }
    for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
         ++profile) {
      const size_t source =
          entry_base + 16 + profile * CONTROLLER_PROFILE_ENCODED_SIZE;
      if (!io_.read(io_.context, arena, source, encoded, sizeof(encoded)) ||
          !controller_profile_decode(encoded, sizeof(encoded), &decoded) ||
          !write_record(0, target_offset, RecordType::kProfile, identity_value,
                        profile, ++generation, encoded, sizeof(encoded))) {
        return false;
      }
      target_offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    if (!write_record(0, target_offset, RecordType::kActivate, identity_value,
                      entry_header[14], ++generation, nullptr, 0)) {
      return false;
    }
    target_offset += PROFILE_STORAGE_RECORD_SIZE;
  }

  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  memcpy(superblock, kSuperblockMagic, sizeof(kSuperblockMagic));
  write_u16(&superblock[4], kCatalogVersion);
  write_u32(&superblock[8], 1);
  write_u32(&superblock[12], profile_storage_crc32(superblock, 12));
  if (!io_.program_page(io_.context, 0, 0, superblock, sizeof(superblock))) {
    return false;
  }

  ProfileStorageIdentityIndex
      rebuilt[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1]{};
  uint8_t rebuilt_count = 0;
  uint32_t rebuilt_epoch = 0;
  uint32_t rebuilt_generation = 0;
  uint32_t rebuilt_payload_crc = 0;
  size_t rebuilt_offset = 0;
  if (!scan_arena(0, &rebuilt_epoch, &rebuilt_generation, &rebuilt_payload_crc,
                  &rebuilt_offset, rebuilt, &rebuilt_count)) {
    return false;
  }
  for (size_t index = 0;
       index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1; ++index) {
    index_[index] = rebuilt[index];
  }
  identity_count_ = rebuilt_count;
  epoch_ = rebuilt_epoch;
  next_offset_ = rebuilt_offset;
  snapshot_.valid = true;
  snapshot_.active_bank = 0;
  snapshot_.generation = rebuilt_generation;
  snapshot_.payload_crc = rebuilt_payload_crc;
  return true;
}

bool ProfileStorage::publish_empty_arena(uint8_t arena, uint32_t epoch) {
  if (!io_.erase_arena(io_.context, arena)) {
    return false;
  }
  uint8_t superblock[PROFILE_STORAGE_PAGE_SIZE]{};
  memcpy(superblock, kSuperblockMagic, sizeof(kSuperblockMagic));
  write_u16(&superblock[4], kCatalogVersion);
  write_u32(&superblock[8], epoch);
  write_u32(&superblock[12], profile_storage_crc32(superblock, 12));
  if (!io_.program_page(io_.context, arena, 0, superblock,
                        sizeof(superblock))) {
    return false;
  }
  epoch_ = epoch;
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
  if (arena >= PROFILE_STORAGE_ARENA_COUNT ||
      offset < PROFILE_STORAGE_RECORDS_OFFSET ||
      offset + PROFILE_STORAGE_RECORD_SIZE > PROFILE_STORAGE_ARENA_SIZE ||
      offset % PROFILE_STORAGE_PAGE_SIZE != 0 ||
      (type == RecordType::kProfile
           ? payload == nullptr ||
                 payload_size != CONTROLLER_PROFILE_ENCODED_SIZE
           : payload_size != 0)) {
    return false;
  }
  if (payload_size != 0) {
    if (!io_.program_page(io_.context, arena, offset + kRecordPayloadOffset,
                          payload, PROFILE_STORAGE_PAGE_SIZE)) {
      return false;
    }
  }
  uint8_t header[PROFILE_STORAGE_PAGE_SIZE]{};
  memcpy(header, kRecordMagic, sizeof(kRecordMagic));
  write_u16(&header[4], kCatalogVersion);
  header[6] = static_cast<uint8_t>(type);
  header[7] = profile_index;
  write_u32(&header[8], generation);
  write_u16(&header[12], static_cast<uint16_t>(payload_size));
  write_u16(&header[14], type == RecordType::kProfile ? read_u16(payload) : 0);
  write_u32(&header[16], profile_storage_crc32(payload, payload_size));
  if (!controller_identity_encode(identity, &header[20],
                                  CONTROLLER_IDENTITY_ENCODED_SIZE)) {
    return false;
  }
  write_u32(&header[kRecordHeaderCrcOffset],
            profile_storage_crc32(header, kRecordHeaderCrcOffset));
  return io_.program_page(io_.context, arena, offset, header, sizeof(header));
}

void ProfileStorage::apply_record(ProfileStorageIdentityIndex *index,
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
  if (entry == nullptr) {
    if (*identity_count >= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1) {
      return;
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
    return;
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
    return;
  }
  if (entry->profile_generation[profile_index] == 0 ||
      generation_is_newer(generation,
                          entry->profile_generation[profile_index])) {
    entry->profile_record[profile_index] =
        type == RecordType::kProfile ? record : PROFILE_STORAGE_NO_RECORD;
    entry->profile_generation[profile_index] = generation;
  }
}
