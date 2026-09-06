#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_storage.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <initializer_list>

namespace {

struct FakeFlash {
  uint8_t bytes[PROFILE_STORAGE_ARENA_COUNT][PROFILE_STORAGE_ARENA_SIZE];
  int programs = 0;
  int erases = 0;
  int fail_after_programs = -1;
  int fail_after_erases = -1;
  size_t torn_page_bytes = 0;
  int corrupt_previous_after_programs = -1;
  uint8_t corrupt_arena = 0;
  size_t corrupt_offset = 0;
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
  if (storage->fail_after_erases >= 0 &&
      storage->erases >= storage->fail_after_erases) {
    memset(storage->bytes[arena], 0xff, PROFILE_STORAGE_ARENA_SIZE / 2);
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
      offset + size > PROFILE_STORAGE_ARENA_SIZE) {
    return false;
  }
  if (storage->fail_after_programs >= 0 &&
      storage->programs >= storage->fail_after_programs) {
    for (size_t index = 0; index < storage->torn_page_bytes; ++index) {
      storage->bytes[arena][offset + index] &= page[index];
    }
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
  if (storage->programs == storage->corrupt_previous_after_programs) {
    storage->bytes[storage->corrupt_arena][storage->corrupt_offset] ^= 1;
  }
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

void encode_schema5(const ControllerProfile &profile, uint8_t *output) {
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(controller_profile_encode(profile, encoded, sizeof(encoded)),
          "profile fixture did not encode");
  memcpy(output, encoded, CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE);
  write_u16(output, CONTROLLER_PROFILE_SPARSE_MACRO_SCHEMA_VERSION);
  write_u16(output + 2, CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE);
}

void install_legacy_database(uint32_t generation = 41) {
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

  uint8_t encoded[CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE]{};
  const ControllerIdentity global = controller_identity_global();
  for (uint8_t profile = 0; profile < PROFILE_STORAGE_LEGACY_PROFILE_COUNT;
       ++profile) {
    ControllerProfile value = controller_profile_default(global, profile);
    value.weak_rumble_scale = static_cast<uint8_t>(20 + profile);
    encode_schema5(value, encoded);
    memcpy(payload + 32 + profile * sizeof(encoded), encoded, sizeof(encoded));
  }

  uint8_t *entry =
      payload + 32 +
      PROFILE_STORAGE_LEGACY_PROFILE_COUNT *
          CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
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
    encode_schema5(value, encoded);
    memcpy(entry + 16 + profile * sizeof(encoded), encoded, sizeof(encoded));
  }

  memcpy(header, "SPPF", 4);
  write_u16(&header[4], 1);
  write_u16(&header[6], 2);
  write_u32(&header[8], generation);
  write_u32(&header[12], PROFILE_STORAGE_LEGACY_DATABASE_SIZE);
  write_u32(&header[16], profile_storage_crc32(
                             payload, PROFILE_STORAGE_LEGACY_DATABASE_SIZE));
  write_u32(&header[20], profile_storage_crc32(header, 20));
}

ControllerIdentity catalog_identity(uint8_t index) {
  return index == 0 ? controller_identity_global() : identity(index);
}

ControllerProfile catalog_profile(uint8_t index, uint8_t slot, bool extended) {
  ControllerProfile profile =
      controller_profile_default(catalog_identity(index), slot);
  profile.weak_rumble_scale = static_cast<uint8_t>(10 + index);
  profile.strong_rumble_scale = static_cast<uint8_t>(30 + slot);
  profile.macros[0].trigger_mask = 1u << 10;
  profile.macros[0].step_count = 8;
  profile.macros[1].trigger_mask = 1u << 11;
  profile.macros[1].first_step = 8;
  profile.macros[1].step_count = 8;
  profile.macros[2].first_step = 16;
  profile.macros[3].first_step = 16;
  profile.macro_step_count = 16;
  // Fill 132 of the old 136 stream bytes, including the old page tail.
  for (uint8_t step = 0; step < profile.macro_step_count; ++step) {
    auto &value = profile.macro_steps[step];
    value.duration_ms = static_cast<uint16_t>(1 + step);
    if (step < 14) {
      value.override_flags = kControllerProfileOverrideButtons |
                             kControllerProfileOverrideLeftStick;
      value.output_button_mask = static_cast<uint16_t>(1u << (step % 16));
      value.left_stick_x = static_cast<int16_t>(100 * step - 500);
      value.left_stick_y = static_cast<int16_t>(100 * slot + index);
    }
  }
  if (extended) {
    profile.shortcuts.modifier = 4;
    const uint8_t selectors[8] = {0, 1, 2, 3, 12, 13, 14, 15};
    memcpy(profile.shortcuts.selectors, selectors, sizeof(selectors));
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    profile.shift.modifier = 17;
    profile.shift.button_map[0] = 15;
    profile.turbo_modes[0] = ControllerProfileTurboMode::kBurst;
    profile.turbo_defaults = {7, 33, 5};
    profile.turbo_override_mask = UINT16_MAX;
    for (uint8_t button = 0; button < 16; ++button) {
      profile.turbo_overrides[button] = {
          static_cast<uint8_t>(button + 1),
          static_cast<uint8_t>(button + 2),
          static_cast<uint8_t>(button + 3)};
    }
    profile.macros[0].mode = ControllerProfileMacroMode::kRepeat;
    profile.macros[0].repeat_count = 255;
  }
  return profile;
}

void install_catalog_record(uint16_t version, size_t offset, uint8_t type,
                            const ControllerIdentity &identity_value,
                            uint8_t slot, uint32_t generation,
                            const uint8_t *payload, size_t size) {
  uint8_t *record = &flash.bytes[0][offset];
  memset(record, 0, PROFILE_STORAGE_RECORD_SIZE);
  memcpy(record, "SPCR", 4);
  write_u16(record + 4, version);
  record[6] = type;
  record[7] = slot;
  write_u32(record + 8, generation);
  write_u16(record + 12, static_cast<uint16_t>(size));
  write_u16(record + 14, type == 1 ? (version == 1 ? 5 : 6) : 0);
  write_u32(record + 16, profile_storage_crc32(payload, size));
  require(controller_identity_encode(identity_value, record + 20,
                                     CONTROLLER_IDENTITY_ENCODED_SIZE),
          "catalog fixture identity did not encode");
  write_u32(record + 34, profile_storage_crc32(record, 34));
  if (size != 0) {
    memcpy(record + (version == 1 ? 256 : 128), payload, size);
  }
}

void install_populated_catalog(uint16_t version, bool fill_arena = false) {
  size_t offset = PROFILE_STORAGE_RECORDS_OFFSET;
  uint32_t generation = 1000;
  for (uint8_t index = 0;
       index <= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY; ++index) {
    const ControllerIdentity id = catalog_identity(index);
    for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
      const ControllerProfile profile = catalog_profile(index, slot, version == 2);
      uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
      if (version == 1) {
        encode_schema5(profile, payload);
      } else {
        require(controller_profile_encode(profile, payload, sizeof(payload)),
                "catalog2 fixture profile did not encode");
      }
      install_catalog_record(
          version, offset, 1, id, slot, ++generation, payload,
          version == 1 ? CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE
                       : CONTROLLER_PROFILE_ENCODED_SIZE);
      offset += PROFILE_STORAGE_RECORD_SIZE;
    }
    uint8_t alias[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
    alias[0] = 1;
    alias[1] = static_cast<uint8_t>('A' + index);
    install_catalog_record(version, offset, 5, id, CONTROLLER_PROFILE_ALL,
                           ++generation, alias, sizeof(alias));
    offset += PROFILE_STORAGE_RECORD_SIZE;
    uint8_t names[PROFILE_STORAGE_PROFILE_NAMES_PAYLOAD_SIZE]{};
    for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
      const size_t base = slot * PROFILE_STORAGE_METADATA_PAYLOAD_SIZE;
      names[base] = 2;
      names[base + 1] = static_cast<uint8_t>('A' + index);
      names[base + 2] = static_cast<uint8_t>('0' + slot);
    }
    install_catalog_record(version, offset, 6, id, CONTROLLER_PROFILE_ALL,
                           ++generation, names, sizeof(names));
    offset += PROFILE_STORAGE_RECORD_SIZE;
    install_catalog_record(version, offset, 4, id, index % 8,
                           ++generation, nullptr, 0);
    offset += PROFILE_STORAGE_RECORD_SIZE;
  }
  while (fill_arena && offset < PROFILE_STORAGE_ARENA_SIZE) {
    install_catalog_record(version, offset, 4, catalog_identity(0), 0,
                           ++generation, nullptr, 0);
    offset += PROFILE_STORAGE_RECORD_SIZE;
  }
  uint8_t *superblock = flash.bytes[0];
  memset(superblock, 0, PROFILE_STORAGE_SUPERBLOCK_SIZE);
  memcpy(superblock, "SPCA", 4);
  write_u16(superblock + 4, version);
  write_u32(superblock + 8, 20);
  write_u32(superblock + 12, profile_storage_crc32(superblock, 12));
}

void require_populated_catalog(const ProfileStorage &storage, bool extended) {
  require(storage.identity_count() ==
              CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1,
          "catalog lost a populated identity");
  for (uint8_t index = 0;
       index <= CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY; ++index) {
    const ControllerIdentity id = catalog_identity(index);
    const auto *entry = storage.find(id);
    require(entry != nullptr && entry->active_profile == index % 8,
            "catalog lost an active profile index");
    char value[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
    require(storage.get_alias(id, value, sizeof(value)) ==
                ProfileStorageResult::kOk &&
                value[0] == 'A' + index && value[1] == '\0',
            "catalog lost a controller alias");
    for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
      ControllerProfile actual{};
      const ControllerProfile expected = catalog_profile(index, slot, extended);
      uint8_t actual_bytes[CONTROLLER_PROFILE_ENCODED_SIZE]{};
      uint8_t expected_bytes[CONTROLLER_PROFILE_ENCODED_SIZE]{};
      require(storage.get(id, slot, &actual) == ProfileStorageResult::kOk &&
                  controller_profile_encode(actual, actual_bytes,
                                            sizeof(actual_bytes)) &&
                  controller_profile_encode(expected, expected_bytes,
                                            sizeof(expected_bytes)) &&
                  memcmp(actual_bytes, expected_bytes, sizeof(actual_bytes)) == 0,
              "catalog lost profile content or a macro at the old page tail");
      require(storage.get_profile_name(id, slot, value, sizeof(value)) ==
                  ProfileStorageResult::kOk &&
                  value[0] == 'A' + index && value[1] == '0' + slot &&
                  value[2] == '\0',
              "catalog lost a profile name");
    }
  }
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
  second.weak_rumble_scale = 44;
  require(after_corruption.set(global, 0, second) == ProfileStorageResult::kOk,
          "interrupted second-page slot was reused instead of skipped");
  ProfileStorage after_retry;
  require(after_retry.initialize(fake_io()) &&
              after_retry.get(global, 0, &value) == ProfileStorageResult::kOk &&
              value.weak_rumble_scale == 44,
          "append after interrupted and corrupt slots did not survive reload");
}

void test_profile_names_and_aliases_recover() {
  erase_all();
  const ControllerIdentity stable = identity(7);
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "metadata catalog did not initialize");
  require(storage.set_alias(stable, "Player one", 10) ==
              ProfileStorageResult::kOk &&
              storage.set_profile_name(stable, 0, "Zelda", 5) ==
                  ProfileStorageResult::kOk &&
              storage.set_profile_name(stable, 7, "Desktop", 7) ==
                  ProfileStorageResult::kOk,
          "profile metadata did not append");
  char value[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(storage.get_alias(stable, value, sizeof(value)) ==
                  ProfileStorageResult::kOk &&
              strcmp(value, "Player one") == 0,
          "controller alias did not read back");

  flash.fail_after_programs = flash.programs + 1;
  require(storage.set_alias(stable, "Interrupted", 11) ==
              ProfileStorageResult::kIoError,
          "interrupted alias append reported success");
  flash.fail_after_programs = -1;
  ProfileStorage reloaded;
  require(reloaded.initialize(fake_io()) &&
              reloaded.get_alias(stable, value, sizeof(value)) ==
                  ProfileStorageResult::kOk &&
              strcmp(value, "Player one") == 0,
          "interrupted alias displaced the previous value");
  require(reloaded.get_profile_name(stable, 0, value, sizeof(value)) ==
                  ProfileStorageResult::kOk &&
              strcmp(value, "Zelda") == 0 &&
              reloaded.get_profile_name(stable, 7, value, sizeof(value)) ==
                  ProfileStorageResult::kOk &&
              strcmp(value, "Desktop") == 0,
          "profile names did not survive reload");
}

void test_compaction_preserves_latest_records() {
  erase_all();
  const ControllerIdentity global = controller_identity_global();
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "catalog did not initialize");
  ControllerProfile profile = controller_profile_default(global, 0);
  require(storage.set_alias(global, "Fallback", 8) ==
                  ProfileStorageResult::kOk &&
              storage.set_profile_name(global, 0, "Compacted", 9) ==
                  ProfileStorageResult::kOk,
          "compaction metadata did not append");
  for (uint16_t write = 1; write <= 260; ++write) {
    profile.weak_rumble_scale = static_cast<uint8_t>(write);
    require(storage.set(global, 0, profile) == ProfileStorageResult::kOk,
            "catalog update failed while forcing compaction");
  }
  require(storage.snapshot().active_bank == 1 && flash.erases >= 2,
          "full arena did not compact into its peer");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  char metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(global, 0, &recovered) ==
                  ProfileStorageResult::kOk &&
              recovered.weak_rumble_scale == static_cast<uint8_t>(260) &&
              reloaded.get_alias(global, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "Fallback") == 0 &&
              reloaded.get_profile_name(global, 0, metadata,
                                        sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "Compacted") == 0,
          "compaction did not preserve profiles and metadata");
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

void test_populated_catalog_publication_power_loss() {
  static FakeFlash baseline;
  for (uint16_t version = 1; version <= 2; ++version) {
    erase_all();
    install_populated_catalog(version, true);
    baseline = flash;
    ProfileStorage completed;
    require(completed.initialize(fake_io()), "populated catalog did not load");
    require_populated_catalog(completed, version == 2);
    if (version == 2) {
      require(completed.set_alias(catalog_identity(0), "New", 3) ==
                  ProfileStorageResult::kOk,
              "populated catalog could not compact all 187 live records");
    }
    const int publication_programs = flash.programs;
    for (size_t torn_bytes : {size_t{0}, size_t{129}}) {
      // Include interruption during arena erase and before every program page,
      // including both pages of every record and the final superblock.
      for (int cut = -1; cut < publication_programs; ++cut) {
        flash = baseline;
        flash.torn_page_bytes = torn_bytes;
        ProfileStorage interrupted;
        if (version == 2) {
          require(interrupted.initialize(fake_io()),
                  "catalog2 power-loss baseline did not load");
        }
        if (cut < 0) {
          flash.fail_after_erases = 0;
        } else {
          flash.fail_after_programs = cut;
        }
        if (version == 1) {
          require(!interrupted.initialize(fake_io()),
                  "incomplete catalog1 migration silently initialized empty");
        } else {
          require(interrupted.set_alias(catalog_identity(0), "New", 3) ==
                      ProfileStorageResult::kIoError,
                  "incomplete compaction or append reported success");
        }
        require(memcmp(flash.bytes[0], baseline.bytes[0],
                       PROFILE_STORAGE_ARENA_SIZE) == 0,
                "interrupted publication modified the old published arena");
        flash.fail_after_erases = -1;
        flash.fail_after_programs = -1;
        flash.torn_page_bytes = 0;
        ProfileStorage recovered;
        require(recovered.initialize(fake_io()),
                "catalog did not recover after an interrupted publication");
        require_populated_catalog(recovered, version == 2);
        require(recovered.snapshot().generation >= 1248,
                "catalog publication regressed the stored generation");
      }
    }
  }
}

void test_migration_verifies_all_records_before_publication() {
  erase_all();
  install_populated_catalog(1);
  static FakeFlash baseline;
  baseline = flash;
  // Corrupt an already-verified first record while the last record is being
  // copied. A per-write readback alone would publish an incomplete catalog.
  flash.corrupt_previous_after_programs =
      2 * PROFILE_STORAGE_MAX_LIVE_RECORDS;
  flash.corrupt_arena = 1;
  flash.corrupt_offset = PROFILE_STORAGE_RECORDS_OFFSET + 128 + 283;
  ProfileStorage interrupted;
  require(!interrupted.initialize(fake_io()) &&
              flash.bytes[1][0] == 0xff &&
              memcmp(flash.bytes[0], baseline.bytes[0],
                     PROFILE_STORAGE_ARENA_SIZE) == 0,
          "migration published before fully verifying the copied arena");
  flash.corrupt_previous_after_programs = -1;
  ProfileStorage recovered;
  require(recovered.initialize(fake_io()),
          "migration did not retry after copy verification failed");
  require_populated_catalog(recovered, false);
}

void test_retired_bank_migration_power_loss() {
  erase_all();
  install_legacy_database();
  static FakeFlash baseline;
  baseline = flash;
  ProfileStorage complete;
  require(complete.initialize(fake_io()), "retired bank did not migrate");
  const int programs = flash.programs;
  for (int cut = -1; cut < programs; ++cut) {
    flash = baseline;
    flash.torn_page_bytes = 129;
    if (cut < 0) {
      flash.fail_after_erases = 0;
    } else {
      flash.fail_after_programs = cut;
    }
    ProfileStorage interrupted;
    require(!interrupted.initialize(fake_io()) &&
                memcmp(flash.bytes[1], baseline.bytes[1],
                       PROFILE_STORAGE_ARENA_SIZE) == 0,
            "interrupted retired-bank migration lost the old database");
    flash.fail_after_erases = -1;
    flash.fail_after_programs = -1;
    flash.torn_page_bytes = 0;
    ProfileStorage recovered;
    ControllerProfile profile{};
    require(recovered.initialize(fake_io()) &&
                recovered.find(identity(7)) != nullptr &&
                recovered.find(identity(7))->active_profile == 2 &&
                recovered.get(identity(7), 2, &profile) ==
                    ProfileStorageResult::kOk &&
                profile.strong_rumble_scale == 42 &&
                recovered.get(catalog_identity(0), 3, &profile) ==
                    ProfileStorageResult::kOk &&
                profile.weak_rumble_scale == 23 &&
                recovered.snapshot().generation > 41,
            "retired-bank migration did not recover its profiles/generation");
  }
}

void test_late_second_page_program_is_not_reused() {
  erase_all();
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "catalog did not initialize");
  // The first page and first bytes of page two are erased; only a late
  // default-rate byte was programmed when power was lost.
  flash.bytes[0][PROFILE_STORAGE_RECORDS_OFFSET + 128 + 283] = 0;
  ProfileStorage recovered;
  require(recovered.initialize(fake_io()), "partial record did not recover");
  const auto id = catalog_identity(0);
  ControllerProfile profile = catalog_profile(0, 7, true);
  require(recovered.set(id, 7, profile) == ProfileStorageResult::kOk,
          "scanner reused a partially programmed second page");
  profile.turbo_defaults.rate_hz = 23;
  require(recovered.set(id, 7, profile) == ProfileStorageResult::kOk,
          "newest profile did not append");
  const uint32_t newest = recovered.find(id)->profile_record[7];
  flash.bytes[0][newest + 128 + 283] ^= 1;
  ProfileStorage fallback;
  ControllerProfile value{};
  require(fallback.initialize(fake_io()) &&
              fallback.get(id, 7, &value) == ProfileStorageResult::kOk &&
              value.turbo_defaults.rate_hz == 7 &&
              value.macros[0].repeat_count == 255,
          "corrupted schema6 newest record did not fall back to its predecessor");
}

void test_unreadable_legacy_data_is_not_erased() {
  erase_all();
  install_legacy_database();
  constexpr size_t base = PROFILE_STORAGE_ARENA_SIZE -
                          PROFILE_STORAGE_LEGACY_BANK_SIZE;
  flash.bytes[1][base + PROFILE_STORAGE_LEGACY_HEADER_SIZE + 32] ^= 1;
  static FakeFlash baseline;
  baseline = flash;
  ProfileStorage storage;
  require(!storage.initialize(fake_io()) &&
              memcmp(flash.bytes, baseline.bytes, sizeof(flash.bytes)) == 0,
          "unreadable old data was silently replaced with an empty catalog");
}

void test_legacy_high_generation_remains_mutable() {
  erase_all();
  install_legacy_database(0x80000000u);
  ProfileStorage storage;
  require(storage.initialize(fake_io()),
          "high-generation legacy bank could not migrate");
  const auto global = controller_identity_global();
  auto changed = controller_profile_default(global, 0);
  changed.weak_rumble_scale = 77;
  require(storage.set(global, 0, changed) == ProfileStorageResult::kOk,
          "high-generation migrated profile could not be updated");
  ProfileStorage reloaded;
  ControllerProfile restored{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(global, 0, &restored) == ProfileStorageResult::kOk &&
              restored.weak_rumble_scale == 77,
          "high-generation update was lost after reload");
}

} // namespace

int main() {
  test_empty_catalog_and_eight_profiles();
  test_identity_capacity_and_defaults();
  test_interrupted_and_corrupt_append_recovery();
  test_profile_names_and_aliases_recover();
  test_compaction_preserves_latest_records();
  test_legacy_migration_is_atomic_and_complete();
  test_legacy_high_generation_remains_mutable();
  test_populated_catalog_publication_power_loss();
  test_migration_verifies_all_records_before_publication();
  test_retired_bank_migration_power_loss();
  test_late_second_page_program_is_not_reused();
  test_unreadable_legacy_data_is_not_erased();
  std::cout << "profile storage tests passed\n";
  return 0;
}
