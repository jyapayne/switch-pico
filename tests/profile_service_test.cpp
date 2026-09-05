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
};

FakeFlash flash{};

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
  write_profile(1, global, 7, eighth, 0);
  require(profile_service_select(global, 7) ==
              ConfigurationTransactionStatus::kCommitted,
          "profile eight was not selectable");
  ProfileServiceSelectedSnapshot selected{};
  profile_service_selected_snapshot(&selected);
  require(selected.valid && selected.profile_index == 7 &&
              selected.profile.strong_rumble_scale == 37,
          "selected profile eight was not decoded on demand");

  require(profile_service_activate(2, global, 7) ==
              ConfigurationTransactionStatus::kPending,
          "profile eight activation was not queued");
  profile_service_task_on_storage_core(1000);
  require(transaction_snapshot().transaction.status ==
              ConfigurationTransactionStatus::kCommitted,
          "profile eight activation did not persist");
  active = active_snapshot(global);
  require(active.valid && active.profile_index == 7 &&
              active.profile.strong_rumble_scale == 37,
          "active cache did not publish profile eight");

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
  profile_service_task_on_storage_core(4000);
  active = active_snapshot(connected);
  require(active.valid && active.profile_index == 6 &&
              active.profile.weak_rumble_scale == 61,
          "controller activation did not refresh the active cache");

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

} // namespace

ProfileStorageIo pico_profile_storage_io() { return fake_io(); }

int main() {
  test_eight_profile_transactions_and_active_cache();
  test_profile_bounds_and_transaction_namespace();
  std::cout << "profile service tests passed\n";
  return 0;
}
