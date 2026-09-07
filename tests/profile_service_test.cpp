#include "core/controller_identity.h"
#include "platform/pico/pico_profile_storage.h"
#include "profile/controller_profile.h"
#include "profile/profile_service.h"
#include "profile/profile_storage.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeFlash {
  uint8_t bytes[PROFILE_STORAGE_ARENA_COUNT][PROFILE_STORAGE_ARENA_SIZE];
  bool fail_program_once = false;
};

FakeFlash flash{};
bool observe_pair_during_program = false;
ControllerIdentity pending_pair{};
ControllerIdentity pending_left{};

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
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
  if (observe_pair_during_program) {
    ProfileServiceActiveProfileSnapshot pair_snapshot{};
    ProfileServiceActiveProfileSnapshot left_snapshot{};
    profile_service_active_profile_snapshot(pending_pair, &pair_snapshot);
    profile_service_active_profile_snapshot(pending_left, &left_snapshot);
    require(!pair_snapshot.valid && left_snapshot.valid &&
                left_snapshot.profile_index == 7 &&
                left_snapshot.profile.weak_rumble_scale == 27,
            "pair seed exposed incomplete profiles while writing flash");
  }
  if (storage->fail_program_once) {
    storage->fail_program_once = false;
    return false;
  }
  for (size_t index = 0; index < size; ++index) {
    storage->bytes[arena][offset + index] &= page[index];
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

ControllerIdentity stable_identity() {
  ControllerIdentity identity{};
  identity.stable = true;
  identity.transport = ControllerTransport::kClassic;
  identity.address[5] = 7;
  identity.vendor_id = 0x054c;
  identity.product_id = 0x0ce6;
  return identity;
}

ProfileServiceTransactionSnapshot transaction_snapshot() {
  ProfileServiceTransactionSnapshot snapshot{};
  profile_service_transaction_snapshot(&snapshot);
  return snapshot;
}

ProfileServiceActiveProfileSnapshot
active_snapshot(const ControllerIdentity &identity) {
  ProfileServiceActiveProfileSnapshot snapshot{};
  profile_service_active_profile_snapshot(identity, &snapshot);
  return snapshot;
}

void write_profile(uint32_t transaction_id, const ControllerIdentity &identity,
                   uint8_t profile_index, const ControllerProfile &profile,
                   uint32_t now_ms) {
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(controller_profile_encode(profile, encoded, sizeof(encoded)),
          "profile did not encode");
  require(
      profile_service_begin(transaction_id, identity, profile_index,
                            CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
                            profile_storage_crc32(encoded, sizeof(encoded))) ==
          ConfigurationTransactionStatus::kReceiving,
      "profile transaction did not begin");
  require(profile_service_append(transaction_id, 0, encoded, 117) ==
                  ConfigurationTransactionStatus::kReceiving &&
              profile_service_append(transaction_id, 117, encoded + 117,
                                     sizeof(encoded) - 117) ==
                  ConfigurationTransactionStatus::kReceiving,
          "profile chunks were not accepted");
  require(profile_service_commit(transaction_id) ==
              ConfigurationTransactionStatus::kPending,
          "profile transaction did not become pending");
  profile_service_task_on_storage_core(now_ms);
  require(transaction_snapshot().transaction.status ==
              ConfigurationTransactionStatus::kCommitted,
          "profile transaction did not persist");
}

void test_eight_profile_transactions_and_active_cache() {
  memset(flash.bytes, 0xff, sizeof(flash.bytes));
  profile_service_prepare();
  profile_service_initialize_on_storage_core();
  const ControllerIdentity global = controller_identity_global();
  ProfileServiceActiveProfileSnapshot active = active_snapshot(global);
  require(active.valid && active.profile_index == 0 &&
              active.metadata.state == ProfileServiceState::kReady,
          "fallback active cache did not initialize");

  ControllerProfile eighth = controller_profile_default(global, 7);
  eighth.strong_rumble_scale = 37;
  eighth.shortcuts.modifier = 4;
  eighth.shortcuts.selectors[7] = 15;
  eighth.shift.mode = ControllerProfileShiftMode::kHold;
  eighth.shift.modifier = 17;
  eighth.shift.button_map[0] = 2;
  eighth.turbo_modes[0] = ControllerProfileTurboMode::kBurst;
  eighth.turbo_defaults = {11, 37, 9};
  eighth.turbo_override_mask = 1u << 15;
  eighth.turbo_overrides[15] = {30, 99, 255};
  write_profile(1, global, 7, eighth, 0);
  require(profile_service_select(global, 7) ==
              ConfigurationTransactionStatus::kCommitted,
          "profile eight was not selectable");
  ProfileServiceSelectedSnapshot selected{};
  profile_service_selected_snapshot(&selected);
  require(selected.valid && selected.profile_index == 7 &&
              selected.profile.strong_rumble_scale == 37 &&
              selected.profile.shortcuts.selectors[7] == 15 &&
              selected.profile.shift.button_map[0] == 2 &&
              selected.profile.turbo_overrides[15].burst_count == 255,
          "selected profile eight lost its schema6 extension");

  require(profile_service_activate(2, global, 7) ==
              ConfigurationTransactionStatus::kPending,
          "profile eight activation was not queued");
  profile_service_task_on_storage_core(1000);
  require(transaction_snapshot().transaction.status ==
              ConfigurationTransactionStatus::kCommitted,
          "profile eight activation did not persist");
  active = active_snapshot(global);
  require(active.valid && active.profile_index == 7 &&
              active.profile.strong_rumble_scale == 37 &&
              active.profile.shift.modifier == 17 &&
              active.profile.turbo_defaults.duty_percent == 37,
          "active cache did not publish the complete schema6 profile");

  const ControllerIdentity connected = stable_identity();
  require(profile_service_observe_identity_on_storage_core(connected),
          "stable identity was not added to the catalog");
  ProfileServiceListSnapshot list{};
  profile_service_list_snapshot(&list);
  require(list.count == 2 &&
              controller_identity_equal(list.rows[1].identity, connected),
          "profile list did not publish the stable identity");
  active = active_snapshot(connected);
  require(active.valid && active.profile_index == 0,
          "new identity did not publish its default active profile");

  ControllerProfile seventh = controller_profile_default(connected, 6);
  seventh.weak_rumble_scale = 61;
  write_profile(3, connected, 6, seventh, 3000);
  require(profile_service_activate_internal(0x80000007u, connected, 6) ==
              ConfigurationTransactionStatus::kPending,
          "controller activation was not queued");
  const auto host_before_activation = transaction_snapshot();
  profile_service_task_on_storage_core(3999);
  active = active_snapshot(connected);
  require(active.valid && active.profile_index == 0,
          "internal activation published before the one-second commit interval");
  profile_service_task_on_storage_core(4000);
  active = active_snapshot(connected);
  require(active.valid && active.profile_index == 6 &&
              active.profile.weak_rumble_scale == 61,
          "controller activation did not refresh the active cache");
  require(transaction_snapshot().transaction.transaction_id ==
              host_before_activation.transaction.transaction_id &&
              transaction_snapshot().transaction.status ==
                  host_before_activation.transaction.status,
          "internal activation replaced the host-visible transaction snapshot");
  const uint32_t activated_generation = active.metadata.generation;
  require(profile_service_activate_internal(0x80000008u, connected, 6) ==
              ConfigurationTransactionStatus::kPending,
          "unchanged internal activation was not accepted");
  profile_service_task_on_storage_core(5000);
  require(active_snapshot(connected).metadata.generation == activated_generation,
          "unchanged internal activation published a fake commit");

  require(profile_service_reset(4, global, CONTROLLER_PROFILE_ALL) ==
              ConfigurationTransactionStatus::kPending,
          "reset-all was not queued");
  profile_service_task_on_storage_core(5000);
  active = active_snapshot(global);
  require(active.valid && active.profile_index == 0 &&
              active.profile.strong_rumble_scale == UINT8_MAX,
          "reset-all did not restore defaults and activation");

  require(profile_service_set_metadata(
              5, connected, CONTROLLER_PROFILE_ALL,
              "Desk pad", 8) ==
              ConfigurationTransactionStatus::kPending,
          "controller alias was not queued");
  profile_service_task_on_storage_core(6000);
  require(transaction_snapshot().transaction.status ==
              ConfigurationTransactionStatus::kCommitted,
          "controller alias did not persist");
  require(profile_service_set_metadata(
              6, connected, 6, "Desktop", 7) ==
              ConfigurationTransactionStatus::kPending,
          "profile name was not queued");
  profile_service_task_on_storage_core(7000);
  require(profile_service_select(connected, 6) ==
              ConfigurationTransactionStatus::kCommitted,
          "named profile was not selected");
  ProfileServiceMetadataSnapshot metadata{};
  profile_service_metadata_snapshot(&metadata);
  require(metadata.valid && strcmp(metadata.alias, "Desk pad") == 0 &&
              strcmp(metadata.profile_names[6], "Desktop") == 0,
          "profile metadata snapshot lost persisted values");

  ProfileStorage reloaded;
  ControllerProfile persisted{};
  char stored_metadata[PROFILE_STORAGE_METADATA_PAYLOAD_SIZE]{};
  require(
      reloaded.initialize(fake_io()) && reloaded.find(connected) != nullptr &&
          reloaded.find(connected)->active_profile == 6 &&
          reloaded.get(connected, 6, &persisted) == ProfileStorageResult::kOk &&
          persisted.weak_rumble_scale == 61 &&
          reloaded.get_alias(connected, stored_metadata,
                             sizeof(stored_metadata)) ==
              ProfileStorageResult::kOk &&
          strcmp(stored_metadata, "Desk pad") == 0 &&
          reloaded.get_profile_name(connected, 6, stored_metadata,
                                    sizeof(stored_metadata)) ==
              ProfileStorageResult::kOk &&
          strcmp(stored_metadata, "Desktop") == 0,
      "service mutations and metadata did not survive catalog reload");
}

void test_profile_bounds_and_transaction_namespace() {
  const ControllerIdentity global = controller_identity_global();
  require(profile_service_select(global, 8) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_activate(10, global, 8) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_reset(11, global, 8) ==
                  ConfigurationTransactionStatus::kMalformed,
          "profile index beyond eight was admitted");
  require(profile_service_activate(0x80000001u, global, 0) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_activate_internal(12, global, 0) ==
                  ConfigurationTransactionStatus::kMalformed,
          "transaction namespaces were not enforced");
}

void test_schema7_validation_and_atomic_selection() {
  const ControllerIdentity id = stable_identity();
  require(profile_service_select(id, 6) ==
              ConfigurationTransactionStatus::kCommitted,
          "transaction validation baseline was not selected");
  ProfileServiceSelectedSnapshot old_selection{};
  profile_service_selected_snapshot(&old_selection);
  ControllerProfile updated = old_selection.profile;
  updated.shortcuts.modifier = 18;
  updated.shortcuts.selectors[6] = 14;
  updated.turbo_defaults = {30, 1, 255};
  updated.extra_button_map[6] = 16;
  updated.shift.extra_button_map[0] = 3;
  updated.macros[0].trigger_mask = 1u << 24;
  updated.macros[0].cancel_control = 19;
  updated.macros[0].step_count = 1;
  updated.macros[0].mode = ControllerProfileMacroMode::kRepeat;
  updated.macros[0].repeat_count = 255;
  updated.macro_step_count = 1;
  for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index)
    updated.macros[index].first_step = 1;
  updated.macro_steps[0].duration_ms = 7;
  uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(controller_profile_encode(updated, encoded, sizeof(encoded)),
          "extended service profile did not encode");
  require(profile_service_begin(20, id, 6, 6, 384, 0) ==
              ConfigurationTransactionStatus::kUnsupportedSchema &&
              profile_service_begin(21, id, 6, 7, 256, 0) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_begin(22, id, 6, 7, 385, 0) ==
                  ConfigurationTransactionStatus::kTooLarge,
          "service admitted old-schema or incorrectly-sized writes");

  // A valid transport CRC cannot authorize an invalid extension.
  encoded[283] = 0;
  require(profile_service_begin(
              23, id, 6, CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
              profile_storage_crc32(encoded, sizeof(encoded))) ==
              ConfigurationTransactionStatus::kReceiving &&
              profile_service_append(23, 0, encoded, sizeof(encoded)) ==
                  ConfigurationTransactionStatus::kReceiving &&
              profile_service_commit(23) ==
                  ConfigurationTransactionStatus::kMalformed,
          "service admitted invalid turbo settings");
  ProfileServiceSelectedSnapshot selected{};
  profile_service_selected_snapshot(&selected);
  require(selected.valid &&
              selected.metadata.generation == old_selection.metadata.generation &&
              selected.profile.turbo_defaults.rate_hz ==
                  old_selection.profile.turbo_defaults.rate_hz,
          "rejected extension replaced the old selected snapshot");

  require(controller_profile_encode(updated, encoded, sizeof(encoded)),
          "valid replacement did not encode");
  require(profile_service_begin(
              24, id, 6, CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
              profile_storage_crc32(encoded, sizeof(encoded))) ==
              ConfigurationTransactionStatus::kReceiving &&
              profile_service_append(24, 0, encoded, 256) ==
                  ConfigurationTransactionStatus::kReceiving &&
              profile_service_append(24, 256, encoded + 256, 128) ==
                  ConfigurationTransactionStatus::kReceiving &&
              profile_service_commit(24) ==
                  ConfigurationTransactionStatus::kPending,
          "service did not receive both parts of the profile payload");
  profile_service_selected_snapshot(&selected);
  require(selected.metadata.generation == old_selection.metadata.generation &&
              active_snapshot(id).profile.turbo_defaults.rate_hz ==
                  old_selection.profile.turbo_defaults.rate_hz,
          "pending write replaced a selected or active profile before commit");
  profile_service_task_on_storage_core(8000);
  profile_service_selected_snapshot(&selected);
  require(transaction_snapshot().transaction.status ==
              ConfigurationTransactionStatus::kCommitted &&
              selected.valid && selected.profile.shortcuts.selectors[6] == 14 &&
              selected.profile.macros[0].repeat_count == 255 &&
              selected.profile.extra_button_map[6] == 16 &&
              selected.profile.shift.extra_button_map[0] == 3 &&
              active_snapshot(id).profile.macros[0].trigger_mask == (1u << 24) &&
              active_snapshot(id).profile.macros[0].cancel_control == 19 &&
              active_snapshot(id).profile.turbo_defaults.rate_hz == 30,
          "committed schema7 profile did not atomically refresh snapshots");
}

void test_catalog1_selected_and_active_snapshots_migrate() {
  memset(flash.bytes, 0xff, sizeof(flash.bytes));
  const ControllerIdentity id = controller_identity_global();
  ControllerProfile original = controller_profile_default(id, 7);
  original.weak_rumble_scale = 73;
  original.macros[0].trigger_mask = 1u << 10;
  original.macros[0].step_count = 1;
  original.macro_step_count = 1;
  for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index)
    original.macros[index].first_step = 1;
  original.macro_steps[0].duration_ms = 125;
  uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
  require(controller_profile_encode(original, payload, sizeof(payload)),
          "old service snapshot fixture did not encode");
  payload[0] = 5;
  payload[2] = 0;
  payload[3] = 1;
  const auto put_u32 = [](uint8_t *output, uint32_t value) {
    for (uint8_t byte = 0; byte < 4; ++byte) {
      output[byte] = static_cast<uint8_t>(value >> (8 * byte));
    }
  };
  for (uint8_t record_index = 0; record_index < 2; ++record_index) {
    uint8_t *record = &flash.bytes[0][PROFILE_STORAGE_RECORDS_OFFSET +
                                    record_index * PROFILE_STORAGE_RECORD_SIZE];
    memset(record, 0, PROFILE_STORAGE_RECORD_SIZE);
    memcpy(record, "SPCR", 4);
    record[4] = 1;
    record[6] = record_index == 0 ? 1 : 4;
    record[7] = 7;
    put_u32(record + 8, 41 + record_index);
    if (record_index == 0) {
      record[13] = 1;
      record[14] = 5;
      memcpy(record + 256, payload, 256);
      put_u32(record + 16, profile_storage_crc32(payload, 256));
    }
    require(controller_identity_encode(id, record + 20,
                                       CONTROLLER_IDENTITY_ENCODED_SIZE),
            "old service identity did not encode");
    put_u32(record + 34, profile_storage_crc32(record, 34));
  }
  uint8_t *superblock = flash.bytes[0];
  memset(superblock, 0, PROFILE_STORAGE_SUPERBLOCK_SIZE);
  memcpy(superblock, "SPCA", 4);
  superblock[4] = 1;
  put_u32(superblock + 8, 9);
  put_u32(superblock + 12, profile_storage_crc32(superblock, 12));
  profile_service_initialize_on_storage_core();
  require(profile_service_select(id, 7) ==
              ConfigurationTransactionStatus::kCommitted,
          "migrated profile could not be selected");
  ProfileServiceSelectedSnapshot selected{};
  profile_service_selected_snapshot(&selected);
  const auto active = active_snapshot(id);
  require(selected.valid && active.valid && active.profile_index == 7 &&
              selected.profile.weak_rumble_scale == 73 &&
              active.profile.macro_steps[0].duration_ms == 125 &&
              selected.profile.shortcuts.modifier == CONTROLLER_PROFILE_NO_BUTTON &&
              selected.profile.shift.mode == ControllerProfileShiftMode::kOff &&
              selected.profile.macros[0].mode == ControllerProfileMacroMode::kOnce &&
              selected.metadata.generation > 42,
          "old selection/activation snapshots lost migrated content or defaults");
}

void test_pair_publication_failure_recovery_and_independence() {
  memset(flash.bytes, 0xff, sizeof(flash.bytes));
  ControllerIdentity left = stable_identity();
  left.transport = ControllerTransport::kBle;
  left.vendor_id = 0x057e;
  left.product_id = 0x2067;
  ControllerIdentity right = left;
  right.product_id = 0x2066;
  right.address[5] = 8;
  ControllerIdentity pair{};
  require(controller_identity_make_joycon_pair(left, right, &pair),
          "service pair identity is invalid");
  ProfileStorage setup;
  require(setup.initialize(fake_io()), "service pair fixture did not initialize");
  for (uint8_t slot = 0; slot < CONTROLLER_PROFILE_COUNT; ++slot) {
    ControllerProfile profile = controller_profile_default(left, slot);
    profile.weak_rumble_scale = 20 + slot;
    const char name[2] = {'L', static_cast<char>('0' + slot)};
    require(setup.set(left, slot, profile) == ProfileStorageResult::kOk &&
                setup.set_profile_name(left, slot, name, sizeof(name)) ==
                    ProfileStorageResult::kOk,
            "service solo fixture did not persist");
  }
  require(setup.activate(left, 7) == ProfileStorageResult::kOk &&
              setup.set_alias(left, "Solo", 4) == ProfileStorageResult::kOk,
          "service solo activation did not persist");
  profile_service_initialize_on_storage_core();
  require(!active_snapshot(pair).valid &&
              !profile_service_observe_identity_on_storage_core(pair) &&
              profile_service_select(pair, 0) == ConfigurationTransactionStatus::kMalformed &&
              profile_service_activate(100, pair, 0) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_activate_internal(0x80000001u, pair, 0) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_reset(101, pair, CONTROLLER_PROFILE_ALL) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_set_metadata(102, pair, 7, "Premature", 9) ==
                  ConfigurationTransactionStatus::kMalformed &&
              profile_service_begin(103, pair, 0, CONTROLLER_PROFILE_SCHEMA_VERSION,
                                    CONTROLLER_PROFILE_ENCODED_SIZE, 0) ==
                  ConfigurationTransactionStatus::kMalformed,
          "service admitted an unseeded pair or supplied its fallback profile");
  ProfileServiceListSnapshot before{};
  profile_service_list_snapshot(&before);
  pending_pair = pair;
  pending_left = left;
  observe_pair_during_program = true;
  flash.fail_program_once = true;
  require(!profile_service_observe_joycon_pair_on_storage_core(pair),
          "failed seed published a ready pair");
  observe_pair_during_program = false;
  ProfileServiceListSnapshot failed{};
  profile_service_list_snapshot(&failed);
  const auto solo_after_failure = active_snapshot(left);
  require(failed.count == before.count &&
              failed.metadata.state == ProfileServiceState::kStorageError &&
              failed.metadata.generation == before.metadata.generation &&
              !active_snapshot(pair).valid && solo_after_failure.valid &&
              solo_after_failure.profile_index == 7 &&
              solo_after_failure.profile.weak_rumble_scale == 27 &&
              !profile_service_observe_joycon_pair_on_storage_core(pair),
          "rejected pair replaced the last committed solo publication");
  profile_service_initialize_on_storage_core();
  observe_pair_during_program = true;
  require(profile_service_observe_joycon_pair_on_storage_core(pair),
          "pair seed did not retry after recovery");
  observe_pair_during_program = false;
  ProfileServiceListSnapshot seeded{};
  profile_service_list_snapshot(&seeded);
  require(seeded.count == before.count + 2 &&
              seeded.metadata.state == ProfileServiceState::kReady &&
              active_snapshot(pair).valid &&
              active_snapshot(pair).profile_index == 7 &&
              active_snapshot(pair).profile.weak_rumble_scale == 27,
          "seeded pair list and active profile were not published together");
  require(profile_service_select(pair, 7) ==
              ConfigurationTransactionStatus::kCommitted,
          "seeded pair could not be selected");
  ProfileServiceMetadataSnapshot metadata{};
  profile_service_metadata_snapshot(&metadata);
  require(metadata.valid && metadata.alias[0] == '\0' &&
              strcmp(metadata.profile_names[7], "L7") == 0,
          "service did not expose independent seeded metadata");
  auto profile = controller_profile_default(pair, 7);
  profile.weak_rumble_scale = 81;
  write_profile(104, pair, 7, profile, 9000);
  require(active_snapshot(pair).profile.weak_rumble_scale == 81 &&
              active_snapshot(left).profile.weak_rumble_scale == 27 &&
              profile_service_activate(105, pair, 3) ==
                  ConfigurationTransactionStatus::kPending,
          "pair edit changed the solo active profile");
  profile_service_task_on_storage_core(10000);
  require(active_snapshot(pair).profile_index == 3 &&
              active_snapshot(left).profile_index == 7,
          "pair activation changed the solo selection");
  require(profile_service_set_metadata(106, pair, 7, "Pair", 4) ==
              ConfigurationTransactionStatus::kPending,
          "pair metadata edit did not queue");
  profile_service_task_on_storage_core(11000);
  require(profile_service_observe_joycon_pair_on_storage_core(pair),
          "existing pair reconnect failed");
  profile_service_metadata_snapshot(&metadata);
  require(metadata.valid && strcmp(metadata.profile_names[7], "Pair") == 0 &&
              profile_service_select(left, 7) ==
                  ConfigurationTransactionStatus::kCommitted,
          "pair reconnect overwrote pair-owned metadata");
  profile_service_metadata_snapshot(&metadata);
  require(metadata.valid && strcmp(metadata.profile_names[7], "L7") == 0 &&
              strcmp(metadata.alias, "Solo") == 0,
          "pair metadata edit changed solo metadata");
  profile_service_initialize_on_storage_core();
  require(profile_service_observe_joycon_pair_on_storage_core(pair) &&
              active_snapshot(pair).profile_index == 3 &&
              active_snapshot(left).profile_index == 7,
          "service reload reseeded the pair active selection");
}

} // namespace

ProfileStorageIo pico_profile_storage_io() { return fake_io(); }

int main() {
  test_eight_profile_transactions_and_active_cache();
  test_profile_bounds_and_transaction_namespace();
  test_schema7_validation_and_atomic_selection();
  test_catalog1_selected_and_active_snapshots_migrate();
  test_pair_publication_failure_recovery_and_independence();
  std::cout << "profile service tests passed\n";
  return 0;
}
