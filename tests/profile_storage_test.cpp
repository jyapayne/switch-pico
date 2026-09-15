#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_storage.h"
#include "controller_profile_legacy_fixtures.h"

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
  int fail_read_after_programs = -1;
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
  if (storage->fail_read_after_programs >= 0 &&
      storage->programs >= storage->fail_read_after_programs) {
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

ControllerIdentity joycon(uint8_t suffix, bool left) {
  ControllerIdentity result = identity(suffix);
  result.transport = ControllerTransport::kBle;
  result.vendor_id = 0x057e;
  result.product_id = left ? 0x2067 : 0x2066;
  return result;
}

ControllerIdentity joycon_pair(const ControllerIdentity &left,
                              const ControllerIdentity &right) {
  ControllerIdentity pair{};
  require(controller_identity_make_joycon_pair(left, right, &pair),
          "pair fixture identity is invalid");
  return pair;
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
  write_u16(record + 14, type == 1
                            ? static_cast<uint16_t>(payload[0] | (payload[1] << 8))
                            : 0);
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
      const ControllerProfile profile = catalog_profile(index, slot, version != 1);
      uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
      if (version == 1) {
        encode_schema5(profile, payload);
      } else {
        require(controller_profile_encode(profile, payload, sizeof(payload)),
                "catalog2 fixture profile did not encode");
        // Keep catalog2 fixtures on the deployed schema6 wire format.
        write_u16(payload, CONTROLLER_PROFILE_EXPANDED_SCHEMA_VERSION);
        memset(payload + 344, 0, sizeof(payload) - 344);
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
  profile.extra_button_map[0] = 16;
  profile.shift.extra_button_map[6] = 15;
  profile.swing = {2, 2, 24};
  profile.nunchuk_swing = {255, 0, 18, 3};
  profile.combined_swing = {4, 255, 16};
  profile.combination_window_ms = 30;
  profile.macros[3].step_count = 1;
  profile.macros[3].mode = ControllerProfileMacroMode::kToggle;
  profile.macro_step_count = 1;
  profile.macro_steps[0].duration_ms = 50;
  ControllerProfile other = controller_profile_default(global, 7);
  other.swing = {15, 0, CONTROLLER_PROFILE_NO_BUTTON};
  other.nunchuk_swing = {1, 2, 24};
  other.combined_swing = {255, 0, 255};
  other.combination_window_ms = 200;
  other.macros[0].step_count = 1;
  other.macro_step_count = 1;
  other.macro_steps[0].duration_ms = 75;
  for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
    other.macros[index].first_step = 1;
  }
  const ControllerIdentity stable = identity(1);
  require(storage.set(global, 7, other) == ProfileStorageResult::kOk,
          "second swing profile did not append");
  other.swing = {255, 1, 16, 0};
  other.nunchuk_swing = {3, 1, 17};
  other.combined_swing = {14, 255, 24};
  other.combination_window_ms = 120;
  require(storage.set(stable, 0, other) == ProfileStorageResult::kOk,
          "stable identity swing profile did not append");
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
              recovered.extra_button_map[0] == 16 &&
              recovered.shift.extra_button_map[6] == 15 &&
              recovered.swing.button == 2 &&
              recovered.swing.sensitivity == 2 &&
              recovered.swing.modifier == 24 &&
              recovered.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.sensitivity == 0 &&
              recovered.nunchuk_swing.modifier == 18 &&
              recovered.nunchuk_swing.macro == 3 &&
              recovered.combined_swing.button == 4 &&
              recovered.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.modifier == 16 &&
              recovered.combination_window_ms == 30 &&
              recovered.macros[3].mode == ControllerProfileMacroMode::kToggle &&
              recovered.macro_steps[0].duration_ms == 50 &&
              reloaded.get_alias(global, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "Fallback") == 0 &&
              reloaded.get_profile_name(global, 0, metadata,
                                        sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "Compacted") == 0,
          "compaction did not preserve profiles and metadata");
  const uint32_t generation = reloaded.snapshot().generation;
  const int programs = flash.programs;
  recovered.nunchuk_swing.button = 0;
  require(reloaded.set(global, 0, recovered) ==
                  ProfileStorageResult::kInvalidArgument &&
              reloaded.snapshot().generation == generation &&
              flash.programs == programs &&
              reloaded.get(global, 0, &recovered) == ProfileStorageResult::kOk &&
              recovered.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.macro == 3,
          "rejected gesture action modified the stored profile");
  require(reloaded.get(global, 7, &recovered) == ProfileStorageResult::kOk &&
              recovered.swing.button == 15 &&
              recovered.swing.sensitivity == 0 &&
              recovered.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.button == 1 &&
              recovered.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.macro == 0 &&
              recovered.combination_window_ms == 200 &&
              recovered.macro_steps[0].duration_ms == 75 &&
              reloaded.get(stable, 0, &recovered) == ProfileStorageResult::kOk &&
              recovered.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.swing.macro == 0 &&
              recovered.swing.sensitivity == 1 &&
              recovered.swing.modifier == 16 &&
              recovered.nunchuk_swing.button == 3 &&
              recovered.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.button == 14 &&
              recovered.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combination_window_ms == 120 &&
              reloaded.get(global, 1, &recovered) == ProfileStorageResult::kOk &&
              recovered.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              recovered.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON,
          "compacted swing settings bled across profiles or identities");
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
  for (uint16_t version = 1; version <= 3; ++version) {
    erase_all();
    install_populated_catalog(version, true);
    baseline = flash;
    ProfileStorage completed;
    require(completed.initialize(fake_io()), "populated catalog did not load");
    require_populated_catalog(completed, version != 1);
    if (version == 3) {
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
        if (version == 3) {
          require(interrupted.initialize(fake_io()),
                  "catalog2 power-loss baseline did not load");
        }
        if (cut < 0) {
          flash.fail_after_erases = 0;
        } else {
          flash.fail_after_programs = cut;
        }
        if (version != 3) {
          require(!interrupted.initialize(fake_io()),
                  "incomplete old catalog migration silently initialized empty");
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
        require_populated_catalog(recovered, version != 1);
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

void test_schema6_read_migration_is_lazy_and_edit_preserves_metadata() {
  erase_all();
  install_populated_catalog(3);
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "schema6 catalog did not load");
  require_populated_catalog(storage, true);
  require(flash.programs == 0 && flash.erases == 0,
          "reading schema6 profiles rewrote the published flash arena");
  const auto id = catalog_identity(7);
  ControllerProfile profile{};
  uint8_t before[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(storage.get(id, 7, &profile) == ProfileStorageResult::kOk &&
              controller_profile_encode(profile, before, sizeof(before)) &&
              storage.set(id, 7, profile) == ProfileStorageResult::kUnchanged &&
              flash.programs == 0 && flash.erases == 0,
          "unchanged migrated profile caused an unnecessary flash write");
  profile.extra_button_map[0] = 16;
  profile.extra_button_map[6] = 15;
  profile.shift.extra_button_map[1] = 2;
  require(storage.set(id, 7, profile) == ProfileStorageResult::kOk,
          "schema6 profile could not add extra-control mappings");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  uint8_t after[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(id, 7, &recovered) == ProfileStorageResult::kOk &&
              controller_profile_encode(recovered, after, sizeof(after)) &&
              memcmp(before + 2, after + 2, 342) == 0 &&
              recovered.extra_button_map[0] == 16 &&
              recovered.extra_button_map[6] == 15 &&
              recovered.shift.extra_button_map[1] == 2,
          "editing migrated extras lost existing settings or the new mappings");
  char metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(reloaded.find(id)->active_profile == 7 &&
              reloaded.get_alias(id, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "H") == 0 &&
              reloaded.get_profile_name(id, 7, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "H7") == 0,
          "editing a schema6 profile lost its active index, alias, or name");
}

void test_schema8_swing_migration_is_lazy_and_preserves_existing_data() {
  erase_all();
  install_populated_catalog(3);
  const auto id = controller_identity_global();
  ControllerProfile original = catalog_profile(0, 0, true);
  original.extra_button_map[6] = 16;
  original.shift.extra_button_map[0] = 15;
  original.swing = {2, 0, 24};
  uint8_t legacy[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(controller_profile_encode(original, legacy, sizeof(legacy)),
          "schema8 storage fixture did not encode");
  write_u16(legacy, CONTROLLER_PROFILE_SWING_SCHEMA_VERSION);
  memset(legacy + 367, 0, sizeof(legacy) - 367);
  install_catalog_record(3, PROFILE_STORAGE_RECORDS_OFFSET, 1, id, 0, 1001,
                         legacy, sizeof(legacy));
  ProfileStorage storage;
  ControllerProfile migrated{};
  require(storage.initialize(fake_io()) &&
              storage.get(id, 0, &migrated) == ProfileStorageResult::kOk &&
              migrated.swing.button == 2 && migrated.swing.sensitivity == 0 &&
              migrated.swing.modifier == 24 &&
              migrated.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              migrated.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              migrated.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              migrated.combined_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
              migrated.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
              migrated.combination_window_ms == 100 &&
              storage.set(id, 0, migrated) == ProfileStorageResult::kUnchanged &&
              flash.programs == 0 && flash.erases == 0,
          "schema8 read enabled new gestures or unnecessarily rewrote flash");
  migrated.nunchuk_swing = {255, 2, 18, 1};
  migrated.combined_swing = {255, 0, 16};
  migrated.combination_window_ms = 200;
  require(storage.set(id, 0, migrated) == ProfileStorageResult::kOk,
          "schema8 profile could not add gesture macro actions");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  uint8_t upgraded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  char metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(id, 0, &recovered) == ProfileStorageResult::kOk &&
              controller_profile_encode(recovered, upgraded, sizeof(upgraded)) &&
              memcmp(legacy + 2, upgraded + 2, 365) == 0 &&
              recovered.nunchuk_swing.macro == 1 &&
              recovered.combined_swing.macro == 0 &&
              recovered.combination_window_ms == 200 &&
              reloaded.find(id)->active_profile == 0 &&
              reloaded.get_alias(id, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "A") == 0 &&
              reloaded.get_profile_name(id, 0, metadata, sizeof(metadata)) ==
                  ProfileStorageResult::kOk &&
              strcmp(metadata, "A0") == 0,
          "editing migrated gestures lost Remote, macros, extras, or metadata");
}

void test_schema9_native_layout_migration_preserves_profiles() {
  erase_all();
  install_populated_catalog(3);
  const auto id = controller_identity_global();
  uint8_t legacy[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  static_assert(sizeof(kLegacySchema9Profile) == sizeof(legacy));
  memcpy(legacy, kLegacySchema9Profile, sizeof(legacy));
  install_catalog_record(3, PROFILE_STORAGE_RECORDS_OFFSET, 1, id, 0, 1001,
                         legacy, sizeof(legacy));
  ProfileStorage storage;
  ControllerProfile migrated{};
  uint8_t migrated_bytes[sizeof(legacy)]{};
  require(storage.initialize(fake_io()) &&
              storage.get(id, 0, &migrated) == ProfileStorageResult::kOk &&
              migrated.native_joycon_layout == ControllerProfileNativeJoyconLayout::kPaired &&
              !migrated.swap_sticks &&
              controller_profile_encode(migrated, migrated_bytes, sizeof(migrated_bytes)) &&
              memcmp(legacy + 2, migrated_bytes + 2, sizeof(legacy) - 2) == 0 &&
              storage.set(id, 0, migrated) == ProfileStorageResult::kUnchanged &&
              flash.programs == 0 && flash.erases == 0,
          "reading schema9 changed bindings/gestures or rewrote persistent storage");
  migrated.native_joycon_layout = ControllerProfileNativeJoyconLayout::kRightSolo;
  migrated.swap_sticks = true;
  migrated.button_map[5] = CONTROLLER_PROFILE_RIGHT_SR_OUTPUT;
  require(storage.set(id, 0, migrated) == ProfileStorageResult::kOk,
          "migrated profile could not save solo layout and rail output");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  uint8_t expected[sizeof(legacy)]{}, actual[sizeof(legacy)]{};
  char metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(id, 0, &recovered) == ProfileStorageResult::kOk &&
              controller_profile_encode(migrated, expected, sizeof(expected)) &&
              controller_profile_encode(recovered, actual, sizeof(actual)) &&
              memcmp(expected, actual, sizeof(expected)) == 0 &&
              reloaded.find(id)->active_profile == 0 &&
              reloaded.get_alias(id, metadata, sizeof(metadata)) == ProfileStorageResult::kOk &&
              strcmp(metadata, "A") == 0 &&
              reloaded.get_profile_name(id, 0, metadata, sizeof(metadata)) == ProfileStorageResult::kOk &&
              strcmp(metadata, "A0") == 0,
          "solo-layout save/reload lost existing profile settings or metadata");
}

void test_schema10_direction_migration_preserves_layout_and_swap() {
  erase_all();
  install_populated_catalog(3);
  const auto id = controller_identity_global();
  install_catalog_record(3, PROFILE_STORAGE_RECORDS_OFFSET, 1, id, 0, 1001,
                         kLegacySchema10Profile, sizeof(kLegacySchema10Profile));
  ProfileStorage storage;
  ControllerProfile migrated{};
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(storage.initialize(fake_io()) &&
              storage.get(id, 0, &migrated) == ProfileStorageResult::kOk &&
              migrated.native_joycon_layout != ControllerProfileNativeJoyconLayout::kPaired &&
              migrated.swap_sticks &&
              controller_profile_encode(migrated, encoded, sizeof(encoded)) &&
              memcmp(encoded + 2, kLegacySchema10Profile + 2, sizeof(encoded) - 2) == 0 &&
              storage.set(id, 0, migrated) == ProfileStorageResult::kUnchanged &&
              flash.programs == 0 && flash.erases == 0,
          "schema10 read changed solo layout/swap/rails or rewrote storage");
  migrated.button_map[12] = CONTROLLER_PROFILE_LEFT_STICK_UP_OUTPUT;
  require(storage.set(id, 0, migrated) == ProfileStorageResult::kOk,
          "schema10 profile could not add a digital stick direction");
  ProfileStorage reloaded;
  ControllerProfile recovered{};
  uint8_t expected[sizeof(encoded)]{};
  char metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(reloaded.initialize(fake_io()) &&
              reloaded.get(id, 0, &recovered) == ProfileStorageResult::kOk &&
              controller_profile_encode(migrated, expected, sizeof(expected)) &&
              controller_profile_encode(recovered, encoded, sizeof(encoded)) &&
              memcmp(encoded, expected, sizeof(encoded)) == 0 &&
              reloaded.find(id)->active_profile == 0 &&
              reloaded.get_profile_name(id, 0, metadata, sizeof(metadata)) == ProfileStorageResult::kOk &&
              strcmp(metadata, "A0") == 0,
          "digital direction save lost other settings, active selection or metadata");
}

void require_pair_bank(const ProfileStorage &storage,
                       const ControllerIdentity &owner, uint8_t base,
                       uint8_t active, char name_prefix) {
  const auto *entry = storage.find(owner);
  require(entry != nullptr && entry->active_profile == active,
          "pair bank active selection changed");
  for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
    ControllerProfile actual{};
    char name[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
    require(storage.get(owner, slot, &actual) == ProfileStorageResult::kOk &&
                actual.weak_rumble_scale == base + slot &&
                storage.get_profile_name(owner, slot, name, sizeof(name)) ==
                    ProfileStorageResult::kOk &&
                name[0] == name_prefix && name[1] == '0' + slot &&
                name[2] == '\0',
            "pair bank profile or name bled into another owner");
  }
}

void write_pair_bank(ProfileStorage &storage, const ControllerIdentity &owner,
                     uint8_t base, char name_prefix) {
  for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
    ControllerProfile profile = controller_profile_default(owner, slot);
    profile.weak_rumble_scale = base + slot;
    const char name[2] = {name_prefix, static_cast<char>('0' + slot)};
    require(storage.set(owner, slot, profile) == ProfileStorageResult::kOk &&
                storage.set_profile_name(owner, slot, name, sizeof(name)) ==
                    ProfileStorageResult::kOk,
            "pair bank edit failed");
  }
}

void test_pair_seed_independence_reconnect_and_compaction() {
  erase_all();
  const auto left = joycon(1, true);
  const auto right = joycon(2, false);
  const auto pair = joycon_pair(left, right);
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "pair catalog did not initialize");
  write_pair_bank(storage, left, 20, 'L');
  write_pair_bank(storage, right, 40, 'R');
  require(storage.set_alias(left, "Solo", 4) == ProfileStorageResult::kOk &&
              storage.activate(left, 7) == ProfileStorageResult::kOk &&
              storage.activate(right, 3) == ProfileStorageResult::kOk,
          "solo bank setup failed");
  const int programs = flash.programs;
  const int erases = flash.erases;
  require(storage.ensure_joycon_pair(pair) == ProfileStorageResult::kOk &&
              flash.programs == programs + 2 && flash.erases == erases,
          "first pair did not seed with one atomic append");
  require_pair_bank(storage, pair, 20, 7, 'L');
  char alias[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(storage.get_alias(pair, alias, sizeof(alias)) ==
                  ProfileStorageResult::kOk && alias[0] == '\0' &&
              storage.get_alias(left, alias, sizeof(alias)) ==
                  ProfileStorageResult::kOk && strcmp(alias, "Solo") == 0,
          "pair inherited or changed the solo alias");
  write_pair_bank(storage, left, 60, 'S');
  require(storage.activate(left, 1) == ProfileStorageResult::kOk,
          "left selection edit failed");
  ProfileStorage replayed;
  require(replayed.initialize(fake_io()), "pair seed did not replay");
  require_pair_bank(replayed, pair, 20, 7, 'L');
  require_pair_bank(replayed, left, 60, 1, 'S');
  require_pair_bank(replayed, right, 40, 3, 'R');
  write_pair_bank(replayed, pair, 80, 'P');
  require(replayed.activate(pair, 6) == ProfileStorageResult::kOk &&
              replayed.set_alias(pair, "Together", 8) == ProfileStorageResult::kOk,
          "pair selection or alias edit failed");
  const uint32_t generation = replayed.snapshot().generation;
  require(replayed.ensure_joycon_pair(pair) == ProfileStorageResult::kUnchanged &&
              replayed.snapshot().generation == generation,
          "pair reconnect reseeded or wrote the bank");
  const auto other_pair = joycon_pair(left, joycon(3, false));
  require(replayed.ensure_joycon_pair(other_pair) == ProfileStorageResult::kOk,
          "different right member did not receive a separate bank");
  require_pair_bank(replayed, other_pair, 60, 1, 'S');
  require_pair_bank(replayed, pair, 80, 6, 'P');
  require_pair_bank(replayed, left, 60, 1, 'S');
  require_pair_bank(replayed, right, 40, 3, 'R');
  const uint8_t old_arena = replayed.snapshot().active_bank;
  for (size_t write = 0; write <= PROFILE_STORAGE_RECORD_CAPACITY; ++write) {
    require(replayed.activate(left, (write + 2) % CONTROLLER_PROFILE_COUNT) ==
                ProfileStorageResult::kOk,
            "pair compaction trigger failed");
  }
  require(replayed.snapshot().active_bank != old_arena,
          "pair catalog never compacted");
  ProfileStorage compacted;
  require(compacted.initialize(fake_io()) &&
              compacted.ensure_joycon_pair(pair) == ProfileStorageResult::kUnchanged,
          "materialized pair did not reconnect after compaction");
  require_pair_bank(compacted, pair, 80, 6, 'P');
  require_pair_bank(compacted, other_pair, 60, 1, 'S');
  require_pair_bank(compacted, right, 40, 3, 'R');
  require(compacted.get_alias(pair, alias, sizeof(alias)) ==
                  ProfileStorageResult::kOk && strcmp(alias, "Together") == 0,
          "compaction lost pair-owned alias");
  require(compacted.reset(pair, CONTROLLER_PROFILE_ALL) ==
              ProfileStorageResult::kOk,
          "pair reset failed");
  require_pair_bank(compacted, other_pair, 60, 1, 'S');
  require_pair_bank(compacted, right, 40, 3, 'R');
}

void test_pair_capacity_and_unseeded_mutations() {
  erase_all();
  const auto left = joycon(1, true);
  const auto right = joycon(2, false);
  const auto pair = joycon_pair(left, right);
  ProfileStorage storage;
  require(storage.initialize(fake_io()), "pair capacity catalog did not initialize");
  ControllerProfile profile = controller_profile_default(pair, 0);
  profile.weak_rumble_scale = 42;
  require(storage.ensure_identity(pair) == ProfileStorageResult::kInvalidArgument &&
              storage.set(pair, 0, profile) == ProfileStorageResult::kInvalidArgument &&
              storage.activate(pair, 7) == ProfileStorageResult::kInvalidArgument &&
              storage.set_alias(pair, "Pair", 4) == ProfileStorageResult::kInvalidArgument &&
              storage.set_profile_name(pair, 7, "Pair", 4) ==
                  ProfileStorageResult::kInvalidArgument &&
              storage.reset(pair, CONTROLLER_PROFILE_ALL) ==
                  ProfileStorageResult::kInvalidArgument &&
              storage.find(pair) == nullptr,
          "mutation implicitly created an unseeded pair");
  for (uint8_t index = 1; index <= 14; ++index) {
    require(storage.ensure_identity(identity(index)) == ProfileStorageResult::kOk,
            "capacity setup failed");
  }
  const int programs = flash.programs;
  require(storage.ensure_joycon_pair(pair) == ProfileStorageResult::kFull &&
              storage.find(left) == nullptr && storage.find(right) == nullptr &&
              storage.find(pair) == nullptr && flash.programs == programs,
          "capacity preflight changed owners before rejecting the pair");
  erase_all();
  require(storage.initialize(fake_io()), "exact-fit catalog did not initialize");
  for (uint8_t index = 1; index <= 13; ++index) {
    require(storage.ensure_identity(identity(index)) == ProfileStorageResult::kOk,
            "exact-fit setup failed");
  }
  require(storage.ensure_joycon_pair(pair) == ProfileStorageResult::kOk &&
              storage.identity_count() == CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1 &&
              storage.find(left) != nullptr && storage.find(right) != nullptr &&
              storage.get(pair, 7, &profile) == ProfileStorageResult::kOk &&
              profile.weak_rumble_scale ==
                  controller_profile_default(left, 7).weak_rumble_scale &&
              storage.ensure_joycon_pair(pair) == ProfileStorageResult::kUnchanged,
          "exact-capacity pair did not create both members and the default bank");
}

void test_pair_seed_interruption_and_ambiguous_readback() {
  erase_all();
  const auto left = joycon(1, true);
  const auto right = joycon(2, false);
  const auto pair = joycon_pair(left, right);
  ProfileStorage initial;
  require(initial.initialize(fake_io()), "seed fault catalog did not initialize");
  write_pair_bank(initial, left, 20, 'L');
  require(initial.activate(left, 7) == ProfileStorageResult::kOk,
          "seed fault active setup failed");
  static FakeFlash baseline;
  baseline = flash;
  // Right row and seed each use two pages. Exercise every cut, including a
  // fully committed header whose program call nevertheless reports failure.
  for (size_t torn_bytes : {size_t{0}, size_t{64}, size_t{256}}) {
    for (int cut = 0; cut < 4; ++cut) {
      flash = baseline;
      ProfileStorage interrupted;
      require(interrupted.initialize(fake_io()), "seed fault baseline did not load");
      flash.fail_after_programs = flash.programs + cut;
      flash.torn_page_bytes = torn_bytes;
      require(interrupted.ensure_joycon_pair(pair) == ProfileStorageResult::kIoError &&
                  interrupted.find(pair) == nullptr,
              "interrupted seed exposed a partially initialized bank");
      flash.fail_after_programs = -1;
      require(interrupted.ensure_joycon_pair(pair) == ProfileStorageResult::kIoError &&
                  interrupted.activate(left, 0) == ProfileStorageResult::kIoError,
              "ambiguous seed allowed retry or source mutation before replay");
      flash.torn_page_bytes = 0;
      ProfileStorage recovered;
      require(recovered.initialize(fake_io()), "interrupted seed did not recover");
      const auto result = recovered.ensure_joycon_pair(pair);
      require(result == ProfileStorageResult::kOk ||
                  result == ProfileStorageResult::kUnchanged,
              "seed retry failed after durable replay");
      require_pair_bank(recovered, pair, 20, 7, 'L');
      require_pair_bank(recovered, left, 20, 7, 'L');
    }
  }
  flash = baseline;
  ProfileStorage ambiguous;
  require(ambiguous.initialize(fake_io()) &&
              ambiguous.ensure_identity(right) == ProfileStorageResult::kOk,
          "readback fault setup failed");
  flash.fail_read_after_programs = flash.programs + 2;
  require(ambiguous.ensure_joycon_pair(pair) == ProfileStorageResult::kIoError &&
              ambiguous.find(pair) == nullptr,
          "unacknowledged seed was published in memory");
  ProfileStorage unreadable;
  require(!unreadable.initialize(fake_io()), "unreadable seed replay failed open");
  flash.fail_read_after_programs = -1;
  require(ambiguous.activate(left, 0) == ProfileStorageResult::kIoError,
          "unacknowledged seed did not freeze source changes");
  ProfileStorage recovered;
  require(recovered.initialize(fake_io()) &&
              recovered.ensure_joycon_pair(pair) == ProfileStorageResult::kUnchanged,
          "committed unacknowledged seed was reseeded");
  require_pair_bank(recovered, pair, 20, 7, 'L');
  write_pair_bank(recovered, left, 60, 'S');
  require(recovered.ensure_joycon_pair(pair) == ProfileStorageResult::kUnchanged,
          "recovered pair was reseeded after source edits");
  require_pair_bank(recovered, pair, 20, 7, 'L');
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
  test_schema6_read_migration_is_lazy_and_edit_preserves_metadata();
  test_schema8_swing_migration_is_lazy_and_preserves_existing_data();
  test_schema9_native_layout_migration_preserves_profiles();
  test_schema10_direction_migration_preserves_layout_and_swap();
  test_pair_seed_independence_reconnect_and_compaction();
  test_pair_capacity_and_unseeded_mutations();
  test_pair_seed_interruption_and_ambiguous_readback();
  std::cout << "profile storage tests passed\n";
  return 0;
}
