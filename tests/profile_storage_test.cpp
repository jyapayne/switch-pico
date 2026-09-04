#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_storage.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeFlash {
  uint8_t bytes[PROFILE_STORAGE_ARENA_COUNT][PROFILE_STORAGE_ARENA_SIZE];
  int programs = 0;
  int erases = 0;
  int fail_after_programs = -1;
  bool corrupt_next_program = false;
};

FakeFlash flash{};

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void erase_all() {
  flash = FakeFlash{};
  memset(flash.bytes, 0xff, sizeof(flash.bytes));
  flash.fail_after_programs = -1;
}

bool fake_read(void *context, uint8_t arena, size_t offset, uint8_t *output,
               size_t size) {
  auto *storage = static_cast<FakeFlash *>(context);
  if (arena >= PROFILE_STORAGE_ARENA_COUNT || output == nullptr ||
      offset > PROFILE_STORAGE_ARENA_SIZE ||
      size > PROFILE_STORAGE_ARENA_SIZE - offset) {
    return false;
  }
  memcpy(output, &storage->bytes[arena][offset], size);
  return true;
}

bool fake_erase(void *context, uint8_t arena) {
  auto *storage = static_cast<FakeFlash *>(context);
  if (arena >= PROFILE_STORAGE_ARENA_COUNT) {
    return false;
  }
  memset(storage->bytes[arena], 0xff, PROFILE_STORAGE_ARENA_SIZE);
  ++storage->erases;
  return true;
}

bool fake_program(void *context, uint8_t arena, size_t offset,
                  const uint8_t *page, size_t size) {
  auto *storage = static_cast<FakeFlash *>(context);
  if (arena >= PROFILE_STORAGE_ARENA_COUNT || page == nullptr ||
      size != PROFILE_STORAGE_PAGE_SIZE ||
      offset % PROFILE_STORAGE_PAGE_SIZE != 0 ||
      offset + size > PROFILE_STORAGE_ARENA_SIZE ||
      (storage->fail_after_programs >= 0 &&
       storage->programs >= storage->fail_after_programs)) {
    return false;
  }
  for (size_t index = 0; index < size; ++index) {
    storage->bytes[arena][offset + index] &= page[index];
  }
  if (storage->corrupt_next_program) {
    storage->bytes[arena][offset] ^= 1;
    storage->corrupt_next_program = false;
  }
  ++storage->programs;
  return memcmp(&storage->bytes[arena][offset], page, size) == 0;
}

ProfileStorageIo fake_io() {
  return {
      &flash,
      PROFILE_STORAGE_ARENA_SIZE,
      PROFILE_STORAGE_SECTOR_SIZE,
      PROFILE_STORAGE_PAGE_SIZE,
      fake_read,
      fake_erase,
      fake_program,
  };
}

ControllerIdentity identity(uint8_t suffix) {
  ControllerIdentity result{};
  result.stable = true;
  result.transport = ControllerTransport::kClassic;
  result.address[5] = suffix;
  result.vendor_id = 0x1234;
  result.product_id = 0x5678;
  return result;
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

void install_legacy_database() {
  constexpr size_t kLegacyStart =
      PROFILE_STORAGE_TOTAL_SIZE - PROFILE_STORAGE_LEGACY_TOTAL_SIZE;
  constexpr uint8_t kArena = 1;
  constexpr size_t kArenaBase = kLegacyStart - PROFILE_STORAGE_ARENA_SIZE;
  constexpr uint8_t kBank = 1;
  constexpr size_t kBankBase =
      kArenaBase + kBank * PROFILE_STORAGE_LEGACY_BANK_SIZE;
  uint8_t *header = flash.bytes[kArena] + kBankBase;
  uint8_t *payload = header + PROFILE_STORAGE_LEGACY_HEADER_SIZE;
  memset(header, 0, PROFILE_STORAGE_LEGACY_HEADER_SIZE);
  memset(payload, 0, PROFILE_STORAGE_LEGACY_DATABASE_SIZE);

  memcpy(payload, "SPDB", 4);
  write_u16(&payload[4], 2);
  write_u16(&payload[6], PROFILE_STORAGE_LEGACY_DATABASE_SIZE);
  payload[8] = CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
  payload[9] = PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
  payload[10] = 3;
  payload[11] = 1;

  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  const ControllerIdentity global = controller_identity_global();
  for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
       ++profile) {
    ControllerProfile value = controller_profile_default(global, profile);
    value.weak_rumble_scale = static_cast<uint8_t>(20 + profile);
    require(controller_profile_encode(value, encoded, sizeof(encoded)),
            "legacy fallback profile did not encode");
    memcpy(payload + 32 + profile * sizeof(encoded), encoded, sizeof(encoded));
  }

  constexpr size_t kEntrySize = 16 + PROFILE_STORAGE_LEGACY_PROFILE_COUNT *
                                         CONTROLLER_PROFILE_ENCODED_SIZE;
  uint8_t *entry =
      payload + 32 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
  const ControllerIdentity stable = identity(7);
  require(controller_identity_encode(stable, entry,
                                     CONTROLLER_IDENTITY_ENCODED_SIZE),
          "legacy identity did not encode");
  entry[14] = 2;
  entry[15] = 1;
  for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
       ++profile) {
    ControllerProfile value = controller_profile_default(stable, profile);
    value.strong_rumble_scale = static_cast<uint8_t>(40 + profile);
    require(controller_profile_encode(value, encoded, sizeof(encoded)),
            "legacy stable profile did not encode");
    memcpy(entry + 16 + profile * sizeof(encoded), encoded, sizeof(encoded));
  }
  (void)kEntrySize;

  memcpy(header, "SPPF", 4);
  write_u16(&header[4], 1);
  write_u16(&header[6], 2);
  write_u32(&header[8], 41);
  write_u32(&header[12], PROFILE_STORAGE_LEGACY_DATABASE_SIZE);
  write_u32(&header[16], profile_storage_crc32(
                             payload, PROFILE_STORAGE_LEGACY_DATABASE_SIZE));
  write_u32(&header[20], profile_storage_crc32(header, 20));
}

void test_empty_catalog_and_eight_profiles() {
  erase_all();
  ProfileStorage storage;
  require(storage.initialize(fake_io()) && storage.snapshot().valid &&
              storage.identity_count() == 1,
          "erased flash did not initialize an empty catalog");
  const ControllerIdentity global = controller_identity_global();
  ControllerProfile profile = controller_profile_default(global, 7);
  profile.strong_rumble_scale = 77;
  require(storage.set(global, 7, profile) == ProfileStorageResult::kOk,
          "profile eight did not append");
  require(storage.activate(global, 7) == ProfileStorageResult::kOk,
          "profile eight did not activate");

  ProfileStorage reloaded;
  ControllerProfile recovered{};
  require(reloaded.initialize(fake_io()) && reloaded.find(global) != nullptr &&
              reloaded.find(global)->active_profile == 7 &&
              reloaded.get(global, 7, &recovered) ==
                  ProfileStorageResult::kOk &&
              recovered.strong_rumble_scale == 77,
          "profile eight did not survive reload");
}

void test_identity_capacity_and_defaults() {
  erase_all();
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "catalog did not initialize");
  for (uint8_t index = 1; index <= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
       ++index) {
    require(storage.ensure_identity(identity(index)) ==
                ProfileStorageResult::kOk,
            "stable identity was not indexed");
  }
  require(storage.ensure_identity(identity(99)) == ProfileStorageResult::kFull,
          "identity catalog accepted a seventeenth stable identity");
  ControllerProfile profile{};
  require(storage.get(identity(1), 7, &profile) == ProfileStorageResult::kOk &&
              controller_profile_validate(profile),
          "unstored profile eight did not resolve to its default");
}

void test_interrupted_and_corrupt_append_recovery() {
  erase_all();
  const ControllerIdentity global = controller_identity_global();
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "catalog did not initialize");
  ControllerProfile first = controller_profile_default(global, 0);
  first.weak_rumble_scale = 11;
  require(storage.set(global, 0, first) == ProfileStorageResult::kOk,
          "baseline profile did not append");

  ControllerProfile second = first;
  second.weak_rumble_scale = 22;
  flash.fail_after_programs = flash.programs + 1;
  require(storage.set(global, 0, second) == ProfileStorageResult::kIoError,
          "interrupted header publish reported success");
  flash.fail_after_programs = -1;
  ProfileStorage recovered;
  ControllerProfile value{};
  require(recovered.initialize(fake_io()) &&
              recovered.get(global, 0, &value) == ProfileStorageResult::kOk &&
              value.weak_rumble_scale == 11,
          "interrupted append displaced the previous record");

  flash.corrupt_next_program = true;
  second.weak_rumble_scale = 33;
  require(recovered.set(global, 0, second) == ProfileStorageResult::kIoError,
          "corrupt payload program reported success");
  ProfileStorage after_corruption;
  require(after_corruption.initialize(fake_io()) &&
              after_corruption.get(global, 0, &value) ==
                  ProfileStorageResult::kOk &&
              value.weak_rumble_scale == 11,
          "corrupt newest record displaced the previous record");
}

void test_compaction_preserves_latest_records() {
  erase_all();
  const ControllerIdentity global = controller_identity_global();
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "catalog did not initialize");
  ControllerProfile profile = controller_profile_default(global, 0);
  for (uint16_t write = 1; write <= 260; ++write) {
    profile.weak_rumble_scale = static_cast<uint8_t>(write);
    require(storage.set(global, 0, profile) == ProfileStorageResult::kOk,
            "catalog update failed while forcing compaction");
  }
  require(storage.snapshot().active_bank == 1 && flash.erases >= 2,
          "full arena did not compact into its peer");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(global, 0, &recovered) ==
                  ProfileStorageResult::kOk &&
              recovered.weak_rumble_scale == static_cast<uint8_t>(260),
          "compaction did not preserve the latest profile");
}

void test_legacy_migration_is_atomic_and_complete() {
  erase_all();
  install_legacy_database();
  ProfileStorage storage;
  require(storage.initialize(fake_io()) && storage.snapshot().valid &&
              storage.identity_count() == 2,
          "legacy database did not migrate");
  const ControllerIdentity global = controller_identity_global();
  ControllerProfile profile{};
  require(storage.find(global)->active_profile == 3 &&
              storage.get(global, 3, &profile) == ProfileStorageResult::kOk &&
              profile.weak_rumble_scale == 23,
          "legacy fallback profiles were not preserved");
  require(storage.get(global, 7, &profile) == ProfileStorageResult::kOk &&
              profile.weak_rumble_scale == UINT8_MAX,
          "new profile slots were not defaulted during migration");
  const ControllerIdentity stable = identity(7);
  require(storage.find(stable) != nullptr &&
              storage.find(stable)->active_profile == 2 &&
              storage.get(stable, 2, &profile) == ProfileStorageResult::kOk &&
              profile.strong_rumble_scale == 42,
          "legacy stable identity was not preserved");

  ProfileStorage reloaded;
  require(reloaded.initialize(fake_io()) && reloaded.find(stable) != nullptr &&
              reloaded.find(stable)->active_profile == 2,
          "migrated catalog did not survive reload");
}

} // namespace

int main() {
  test_empty_catalog_and_eight_profiles();
  test_identity_capacity_and_defaults();
  test_interrupted_and_corrupt_append_recovery();
  test_compaction_preserves_latest_records();
  test_legacy_migration_is_atomic_and_complete();
  std::cout << "profile storage tests passed\n";
  return 0;
}
