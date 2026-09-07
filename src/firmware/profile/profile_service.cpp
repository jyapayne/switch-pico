#include "profile/profile_service.h"
#include <string.h>

#include "pico/critical_section.h"
#include "platform/pico/pico_profile_storage.h"
#include "profile/profile_storage.h"

namespace {

constexpr uint32_t kMinimumCommitIntervalMs = 1000;
constexpr uint32_t kInternalTransactionIdMask = 0x80000000u;
static_assert(PROFILE_SERVICE_METADATA_MAX_BYTES ==
              PROFILE_STORAGE_METADATA_MAX_BYTES);

enum class PendingCommandType : uint8_t {
  kNone = 0,
  kReset = 1,
  kActivate = 2,
  kSetAlias = 3,
  kSetProfileName = 4,
};

struct PendingCommand {
  PendingCommandType type = PendingCommandType::kNone;
  uint32_t transaction_id = 0;
  ControllerIdentity identity{};
  uint8_t profile_index = 0;
  uint8_t value_size = 0;
  char value[PROFILE_SERVICE_METADATA_MAX_BYTES]{};
};

struct ProfileTransaction {
  ControllerIdentity identity{};
  uint8_t profile_index = 0;
  ConfigurationTransactionSnapshot snapshot{};
  uint8_t payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};
};

struct PublishedActiveProfile {
  ControllerIdentity identity{};
  uint8_t profile_index = 0;
  ControllerProfile profile{};
};

critical_section_t g_lock;
bool g_prepared = false;
ProfileStorage g_storage;
ProfileServiceMetadata g_metadata;
uint32_t g_published_generation = 0;
ProfileServiceListSnapshot g_list;
ProfileServiceSelectedSnapshot g_selected;
PublishedActiveProfile g_active_profiles[PROFILE_SERVICE_LIST_CAPACITY]{};
uint8_t g_active_profile_count = 0;
ProfileTransaction g_transaction;
PendingCommand g_command;
PendingCommand g_internal_activation;
bool g_has_committed = false;
uint32_t g_last_commit_ms = 0;

bool valid_identity(const ControllerIdentity &identity) {
  uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE]{};
  return (controller_identity_is_global(identity) || identity.stable) &&
         controller_identity_encode(identity, encoded, sizeof(encoded));
}

bool valid_owner_locked(const ControllerIdentity &identity) {
  if (!valid_identity(identity)) {
    return false;
  }
  if (!controller_identity_is_joycon_pair(identity)) {
    return true;
  }
  if (g_metadata.state != ProfileServiceState::kReady) {
    return false;
  }
  for (uint8_t index = 0; index < g_list.count; ++index) {
    if (controller_identity_equal(g_list.rows[index].identity, identity)) {
      return true;
    }
  }
  return false;
}

void refresh_list_locked() {
  g_list = ProfileServiceListSnapshot{};
  g_list.metadata = g_metadata;
  for (uint8_t index = 0; index < g_storage.identity_count() &&
                          g_list.count < PROFILE_SERVICE_LIST_CAPACITY;
       ++index) {
    const ProfileStorageIdentityIndex *entry = g_storage.identity(index);
    if (entry == nullptr || !entry->used) {
      continue;
    }
    ProfileServiceListRow &row = g_list.rows[g_list.count++];
    if (g_storage.get_alias(
            entry->identity, row.alias,
            sizeof(row.alias)) != ProfileStorageResult::kOk) {
      row.alias[0] = '\0';
    }
    row.identity = entry->identity;
    row.active_profile = entry->active_profile;
  }
}

void refresh_active_profiles_locked() {
  g_active_profile_count = 0;
  for (uint8_t index = 0;
       index < g_storage.identity_count() &&
       g_active_profile_count < PROFILE_SERVICE_LIST_CAPACITY;
       ++index) {
    const ProfileStorageIdentityIndex *entry = g_storage.identity(index);
    if (entry == nullptr || !entry->used) {
      continue;
    }
    PublishedActiveProfile &active = g_active_profiles[g_active_profile_count];
    active.identity = entry->identity;
    active.profile_index = entry->active_profile;
    if (g_storage.get(entry->identity, entry->active_profile,
                      &active.profile) != ProfileStorageResult::kOk) {
      continue;
    }
    ++g_active_profile_count;
  }
}

void refresh_selected_locked() {
  g_selected.metadata = g_metadata;
  if (g_storage.get(g_selected.identity, g_selected.profile_index,
                    &g_selected.profile) != ProfileStorageResult::kOk) {
    g_selected.valid = false;
    g_selected.status = ConfigurationTransactionStatus::kMalformed;
    return;
  }
  g_selected.valid = true;
  g_selected.status = ConfigurationTransactionStatus::kCommitted;
}

void refresh_metadata_locked(ProfileServiceState state) {
  const ProfileStorageSnapshot &stored = g_storage.snapshot();
  if (state == ProfileServiceState::kStorageError) {
    // Keep the last committed publication usable by already-connected slots.
    // In particular, a rejected pair setup must not replace either solo's
    // profile with defaults or expose newly appended member rows.
    g_metadata.state = state;
    g_list.metadata = g_metadata;
    g_selected.metadata = g_metadata;
    g_selected.valid = false;
    g_selected.status = ConfigurationTransactionStatus::kStorageError;
    return;
  }
  g_metadata.state = state;
  g_metadata.generation = stored.valid ? stored.generation : 0;
  g_metadata.payload_crc = stored.valid ? stored.payload_crc : 0;
  refresh_active_profiles_locked();
  refresh_list_locked();
  refresh_selected_locked();
  __atomic_store_n(&g_published_generation, g_metadata.generation,
                   __ATOMIC_RELEASE);
}

ConfigurationTransactionStatus
storage_result_status(ProfileStorageResult result) {
  switch (result) {
  case ProfileStorageResult::kOk:
    return ConfigurationTransactionStatus::kCommitted;
  case ProfileStorageResult::kUnchanged:
    return ConfigurationTransactionStatus::kUnchanged;
  case ProfileStorageResult::kInvalidArgument:
    return ConfigurationTransactionStatus::kMalformed;
  case ProfileStorageResult::kFull:
    return ConfigurationTransactionStatus::kTooLarge;
  case ProfileStorageResult::kIoError:
    return ConfigurationTransactionStatus::kStorageError;
  }
  return ConfigurationTransactionStatus::kStorageError;
}

bool mutation_ready(uint32_t now_ms) {
  return !g_has_committed || static_cast<uint32_t>(now_ms - g_last_commit_ms) >=
                                 kMinimumCommitIntervalMs;
}

void finish_mutation(ConfigurationTransactionStatus status,
                     bool clear_command) {
  critical_section_enter_blocking(&g_lock);
  g_transaction.snapshot.status = status;
  g_transaction.snapshot.stored_generation =
      g_storage.snapshot().valid ? g_storage.snapshot().generation : 0;
  g_transaction.snapshot.stored_crc =
      g_storage.snapshot().valid ? g_storage.snapshot().payload_crc : 0;
  if (clear_command) {
    g_command = {};
  }
  refresh_metadata_locked(status ==
                                  ConfigurationTransactionStatus::kStorageError
                              ? ProfileServiceState::kStorageError
                              : ProfileServiceState::kReady);
  critical_section_exit(&g_lock);
}

void finish_internal_activation(ConfigurationTransactionStatus status) {
  critical_section_enter_blocking(&g_lock);
  g_internal_activation = {};
  refresh_metadata_locked(status ==
                                  ConfigurationTransactionStatus::kStorageError
                              ? ProfileServiceState::kStorageError
                              : ProfileServiceState::kReady);
  critical_section_exit(&g_lock);
}

} // namespace

void profile_service_prepare() {
  if (g_prepared) {
    return;
  }
  critical_section_init(&g_lock);
  g_metadata = {};
  __atomic_store_n(&g_published_generation, 0, __ATOMIC_RELAXED);
  g_list = ProfileServiceListSnapshot{};
  g_selected = {};
  g_active_profiles[0] = {};
  g_active_profile_count = 0;
  g_selected.identity = controller_identity_global();
  g_selected.profile_index = 0;
  g_transaction = {};
  g_command = {};
  g_internal_activation = {};
  g_has_committed = false;
  g_last_commit_ms = 0;
  g_prepared = true;
}

void profile_service_initialize_on_storage_core() {
  if (!g_prepared) {
    profile_service_prepare();
  }
  const bool initialized = g_storage.initialize(pico_profile_storage_io());
  critical_section_enter_blocking(&g_lock);
  refresh_metadata_locked(initialized ? ProfileServiceState::kReady
                                      : ProfileServiceState::kStorageError);
  critical_section_exit(&g_lock);
}

bool profile_service_observe_identity_on_storage_core(
    const ControllerIdentity &identity) {
  if (!g_prepared || !identity.stable ||
      controller_identity_is_global(identity) || !valid_identity(identity) ||
      controller_identity_is_joycon_pair(identity)) {
    return false;
  }
  critical_section_enter_blocking(&g_lock);
  const bool ready = g_metadata.state == ProfileServiceState::kReady;
  critical_section_exit(&g_lock);
  if (!ready) {
    return false;
  }
  const ProfileStorageResult result = g_storage.ensure_identity(identity);
  if (result != ProfileStorageResult::kOk &&
      result != ProfileStorageResult::kUnchanged) {
    if (result == ProfileStorageResult::kIoError) {
      critical_section_enter_blocking(&g_lock);
      refresh_metadata_locked(ProfileServiceState::kStorageError);
      critical_section_exit(&g_lock);
    }
    return false;
  }
  critical_section_enter_blocking(&g_lock);
  refresh_metadata_locked(ProfileServiceState::kReady);
  critical_section_exit(&g_lock);
  return true;
}

bool profile_service_observe_joycon_pair_on_storage_core(
    const ControllerIdentity &pair) {
  if (!g_prepared || !controller_identity_is_joycon_pair(pair)) {
    return false;
  }
  critical_section_enter_blocking(&g_lock);
  const bool ready = g_metadata.state == ProfileServiceState::kReady;
  critical_section_exit(&g_lock);
  if (!ready) {
    return false;
  }
  // Flash writes and the seed snapshot must not hold either core's state lock.
  const ProfileStorageResult result = g_storage.ensure_joycon_pair(pair);
  if (result != ProfileStorageResult::kOk &&
      result != ProfileStorageResult::kUnchanged) {
    if (result == ProfileStorageResult::kIoError) {
      critical_section_enter_blocking(&g_lock);
      refresh_metadata_locked(ProfileServiceState::kStorageError);
      critical_section_exit(&g_lock);
    }
    return false;
  }
  critical_section_enter_blocking(&g_lock);
  refresh_metadata_locked(ProfileServiceState::kReady);
  critical_section_exit(&g_lock);
  return true;
}

void profile_service_task_on_storage_core(uint32_t now_ms) {
  PendingCommand command{};
  bool process_write = false;
  bool process_internal_activation = false;
  ControllerIdentity write_identity{};
  uint8_t write_profile_index = 0;
  uint8_t write_payload[CONTROLLER_PROFILE_ENCODED_SIZE]{};

  critical_section_enter_blocking(&g_lock);
  if (mutation_ready(now_ms)) {
    if (g_command.type != PendingCommandType::kNone) {
      command = g_command;
    } else if (g_transaction.snapshot.status ==
               ConfigurationTransactionStatus::kPending) {
      process_write = true;
      write_identity = g_transaction.identity;
      write_profile_index = g_transaction.profile_index;
      memcpy(write_payload, g_transaction.payload, sizeof(write_payload));
    } else if (g_internal_activation.type != PendingCommandType::kNone) {
      command = g_internal_activation;
      process_internal_activation = true;
    }
  }
  critical_section_exit(&g_lock);

  if (!process_write && !process_internal_activation &&
      command.type == PendingCommandType::kNone) {
    return;
  }

  ProfileStorageResult storage_result = ProfileStorageResult::kInvalidArgument;
  if (process_write) {
    ControllerProfile profile{};
    if (controller_profile_decode(write_payload, sizeof(write_payload),
                                  &profile)) {
      storage_result =
          g_storage.set(write_identity, write_profile_index, profile);
    }
  } else if (command.type == PendingCommandType::kReset) {
    storage_result = g_storage.reset(command.identity, command.profile_index);
  } else if (command.type == PendingCommandType::kActivate) {
    storage_result =
        g_storage.activate(command.identity, command.profile_index);
  } else if (command.type == PendingCommandType::kSetAlias) {
    storage_result = g_storage.set_alias(
        command.identity, command.value, command.value_size);
  } else if (command.type == PendingCommandType::kSetProfileName) {
    storage_result = g_storage.set_profile_name(
        command.identity, command.profile_index,
        command.value, command.value_size);
  }
  const ConfigurationTransactionStatus status =
      storage_result_status(storage_result);
  if (status == ConfigurationTransactionStatus::kCommitted) {
    g_has_committed = true;
    g_last_commit_ms = now_ms;
  }
  if (process_internal_activation) {
    finish_internal_activation(status);
  } else {
    finish_mutation(status, !process_write);
  }
}

ConfigurationTransactionStatus
profile_service_select(const ControllerIdentity &identity,
                       uint8_t profile_index) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  ConfigurationTransactionStatus status =
      ConfigurationTransactionStatus::kCommitted;
  if (!valid_owner_locked(identity) || profile_index >= CONTROLLER_PROFILE_COUNT) {
    status = ConfigurationTransactionStatus::kMalformed;
    g_selected.metadata = g_metadata;
    g_selected.identity = identity;
    g_selected.profile_index = profile_index;
    g_selected.valid = false;
    g_selected.status = status;
  } else if (g_metadata.state != ProfileServiceState::kReady) {
    status = g_metadata.state == ProfileServiceState::kLoading
                 ? ConfigurationTransactionStatus::kPending
                 : ConfigurationTransactionStatus::kStorageError;
    g_selected.metadata = g_metadata;
    g_selected.identity = identity;
    g_selected.profile_index = profile_index;
    g_selected.valid = false;
    g_selected.status = status;
  } else {
    g_selected.identity = identity;
    g_selected.profile_index = profile_index;
    refresh_selected_locked();
    status = g_selected.status;
  }
  critical_section_exit(&g_lock);
  return status;
}

ConfigurationTransactionStatus
profile_service_begin(uint32_t transaction_id,
                      const ControllerIdentity &identity, uint8_t profile_index,
                      uint16_t schema_version, size_t payload_size,
                      uint32_t payload_crc) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  if (g_command.type != PendingCommandType::kNone ||
      g_internal_activation.type != PendingCommandType::kNone ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kReceiving ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kPending) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  g_transaction = {};
  g_transaction.snapshot.transaction_id = transaction_id;
  g_transaction.identity = identity;
  g_transaction.profile_index = profile_index;
  if (transaction_id == 0 ||
      (transaction_id & kInternalTransactionIdMask) != 0 ||
      !valid_owner_locked(identity) || profile_index >= CONTROLLER_PROFILE_COUNT ||
      payload_size == 0) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kMalformed;
  } else if (schema_version != CONTROLLER_PROFILE_SCHEMA_VERSION) {
    g_transaction.snapshot.status =
        ConfigurationTransactionStatus::kUnsupportedSchema;
  } else if (payload_size > CONTROLLER_PROFILE_ENCODED_SIZE) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kTooLarge;
  } else if (payload_size != CONTROLLER_PROFILE_ENCODED_SIZE) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kMalformed;
  } else {
    g_transaction.snapshot.expected_size = static_cast<uint16_t>(payload_size);
    g_transaction.snapshot.expected_crc = payload_crc;
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kReceiving;
  }
  const ConfigurationTransactionStatus status = g_transaction.snapshot.status;
  critical_section_exit(&g_lock);
  return status;
}

ConfigurationTransactionStatus profile_service_append(uint32_t transaction_id,
                                                      size_t offset,
                                                      const uint8_t *data,
                                                      size_t size) {
  if ((transaction_id & kInternalTransactionIdMask) != 0) {
    return ConfigurationTransactionStatus::kMalformed;
  }
  critical_section_enter_blocking(&g_lock);
  if (g_transaction.snapshot.status !=
      ConfigurationTransactionStatus::kReceiving) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  if (transaction_id != g_transaction.snapshot.transaction_id ||
      data == nullptr || size == 0 ||
      offset != g_transaction.snapshot.received_size ||
      offset > g_transaction.snapshot.expected_size ||
      size > g_transaction.snapshot.expected_size - offset) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kOutOfOrder;
  } else {
    memcpy(&g_transaction.payload[offset], data, size);
    g_transaction.snapshot.received_size = static_cast<uint16_t>(offset + size);
  }
  const ConfigurationTransactionStatus status = g_transaction.snapshot.status;
  critical_section_exit(&g_lock);
  return status;
}

ConfigurationTransactionStatus profile_service_commit(uint32_t transaction_id) {
  if ((transaction_id & kInternalTransactionIdMask) != 0) {
    return ConfigurationTransactionStatus::kMalformed;
  }
  critical_section_enter_blocking(&g_lock);
  if (g_transaction.snapshot.status !=
          ConfigurationTransactionStatus::kReceiving ||
      transaction_id != g_transaction.snapshot.transaction_id ||
      g_transaction.snapshot.received_size !=
          g_transaction.snapshot.expected_size) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kOutOfOrder;
  } else if (profile_storage_crc32(g_transaction.payload,
                                   g_transaction.snapshot.expected_size) !=
             g_transaction.snapshot.expected_crc) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kBadCrc;
  } else {
    ControllerProfile profile{};
    g_transaction.snapshot.status =
        controller_profile_decode(g_transaction.payload,
                                  g_transaction.snapshot.expected_size,
                                  &profile)
            ? ConfigurationTransactionStatus::kPending
            : ConfigurationTransactionStatus::kMalformed;
  }
  const ConfigurationTransactionStatus status = g_transaction.snapshot.status;
  critical_section_exit(&g_lock);
  return status;
}

ConfigurationTransactionStatus
profile_service_reset(uint32_t transaction_id,
                      const ControllerIdentity &identity,
                      uint8_t profile_index) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  if (g_command.type != PendingCommandType::kNone ||
      g_internal_activation.type != PendingCommandType::kNone ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kReceiving ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kPending) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  g_transaction = {};
  g_transaction.snapshot.transaction_id = transaction_id;
  g_transaction.identity = identity;
  g_transaction.profile_index = profile_index;
  if (transaction_id == 0 ||
      (transaction_id & kInternalTransactionIdMask) != 0 ||
      !valid_owner_locked(identity) ||
      (profile_index != CONTROLLER_PROFILE_ALL &&
       profile_index >= CONTROLLER_PROFILE_COUNT)) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kMalformed;
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kMalformed;
  }
  g_transaction.snapshot.status = ConfigurationTransactionStatus::kPending;
  g_command.transaction_id = transaction_id;
  g_command.type = PendingCommandType::kReset;
  g_command.identity = identity;
  g_command.profile_index = profile_index;
  critical_section_exit(&g_lock);
  return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus
profile_service_activate(uint32_t transaction_id,
                         const ControllerIdentity &identity,
                         uint8_t profile_index) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  if (g_command.type != PendingCommandType::kNone ||
      g_internal_activation.type != PendingCommandType::kNone ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kReceiving ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kPending) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  g_transaction = {};
  g_transaction.snapshot.transaction_id = transaction_id;
  g_transaction.identity = identity;
  g_transaction.profile_index = profile_index;
  if (transaction_id == 0 ||
      (transaction_id & kInternalTransactionIdMask) != 0 ||
      !valid_owner_locked(identity) || profile_index >= CONTROLLER_PROFILE_COUNT) {
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kMalformed;
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kMalformed;
  }
  g_transaction.snapshot.status = ConfigurationTransactionStatus::kPending;
  g_command.transaction_id = transaction_id;
  g_command.type = PendingCommandType::kActivate;
  g_command.identity = identity;
  g_command.profile_index = profile_index;
  critical_section_exit(&g_lock);
  return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus profile_service_set_metadata(
    uint32_t transaction_id, const ControllerIdentity &identity,
    uint8_t profile_index, const char *value, size_t value_size) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  if (g_command.type != PendingCommandType::kNone ||
      g_internal_activation.type != PendingCommandType::kNone ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kReceiving ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kPending) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  g_transaction = {};
  g_transaction.snapshot.transaction_id = transaction_id;
  g_transaction.identity = identity;
  g_transaction.profile_index = profile_index;
  bool malformed =
      transaction_id == 0 ||
      (transaction_id & kInternalTransactionIdMask) != 0 ||
      !valid_owner_locked(identity) ||
      (profile_index != CONTROLLER_PROFILE_ALL &&
       profile_index >= CONTROLLER_PROFILE_COUNT) ||
      value_size > PROFILE_SERVICE_METADATA_MAX_BYTES ||
      (value_size != 0 && value == nullptr);
  for (size_t index = 0; index < value_size && !malformed; ++index) {
    malformed = value[index] == '\0';
  }
  if (malformed) {
    g_transaction.snapshot.status =
        ConfigurationTransactionStatus::kMalformed;
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kMalformed;
  }
  g_transaction.snapshot.status = ConfigurationTransactionStatus::kPending;
  g_command.transaction_id = transaction_id;
  g_command.type = profile_index == CONTROLLER_PROFILE_ALL
                       ? PendingCommandType::kSetAlias
                       : PendingCommandType::kSetProfileName;
  g_command.identity = identity;
  g_command.profile_index = profile_index;
  g_command.value_size = static_cast<uint8_t>(value_size);
  if (value_size != 0) {
    memcpy(g_command.value, value, value_size);
  }
  critical_section_exit(&g_lock);
  return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus
profile_service_activate_internal(uint32_t transaction_id,
                                  const ControllerIdentity &identity,
                                  uint8_t profile_index) {
  if (!g_prepared) {
    profile_service_prepare();
  }
  critical_section_enter_blocking(&g_lock);
  if (g_command.type != PendingCommandType::kNone ||
      g_internal_activation.type != PendingCommandType::kNone ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kReceiving ||
      g_transaction.snapshot.status ==
          ConfigurationTransactionStatus::kPending) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kBusy;
  }
  if ((transaction_id & kInternalTransactionIdMask) == 0 ||
      !valid_owner_locked(identity) || profile_index >= CONTROLLER_PROFILE_COUNT) {
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kMalformed;
  }
  g_internal_activation.type = PendingCommandType::kActivate;
  g_internal_activation.transaction_id = transaction_id;
  g_internal_activation.identity = identity;
  g_internal_activation.profile_index = profile_index;
  critical_section_exit(&g_lock);
  return ConfigurationTransactionStatus::kPending;
}

void profile_service_list_snapshot(ProfileServiceListSnapshot *output) {
  if (output == nullptr) {
    return;
  }
  critical_section_enter_blocking(&g_lock);
  *output = g_list;
  critical_section_exit(&g_lock);
}

void profile_service_selected_snapshot(ProfileServiceSelectedSnapshot *output) {
  if (output == nullptr) {
    return;
  }
  critical_section_enter_blocking(&g_lock);
  *output = g_selected;
  critical_section_exit(&g_lock);
}

void profile_service_metadata_snapshot(
    ProfileServiceMetadataSnapshot *output) {
  if (output == nullptr) {
    return;
  }
  *output = {};
  critical_section_enter_blocking(&g_lock);
  output->metadata = g_metadata;
  output->identity = g_selected.identity;
  output->status = g_selected.status;
  bool valid =
      g_metadata.state == ProfileServiceState::kReady &&
      g_storage.get_alias(
          g_selected.identity, output->alias,
          sizeof(output->alias)) == ProfileStorageResult::kOk;
  for (uint8_t profile = 0;
       profile < CONTROLLER_PROFILE_COUNT && valid; ++profile) {
    valid = g_storage.get_profile_name(
                g_selected.identity, profile,
                output->profile_names[profile],
                sizeof(output->profile_names[profile])) ==
            ProfileStorageResult::kOk;
  }
  output->valid = valid;
  if (!valid && output->status ==
                    ConfigurationTransactionStatus::kCommitted) {
    output->status = ConfigurationTransactionStatus::kStorageError;
  }
  critical_section_exit(&g_lock);
}

void profile_service_transaction_snapshot(
    ProfileServiceTransactionSnapshot *output) {
  if (output == nullptr) {
    return;
  }
  critical_section_enter_blocking(&g_lock);
  output->metadata = g_metadata;
  output->identity = g_transaction.identity;
  output->profile_index = g_transaction.profile_index;
  output->transaction = g_transaction.snapshot;
  critical_section_exit(&g_lock);
}

uint32_t profile_service_database_generation() {
  return __atomic_load_n(&g_published_generation, __ATOMIC_ACQUIRE);
}

void profile_service_active_profile_snapshot(
    const ControllerIdentity &identity,
    ProfileServiceActiveProfileSnapshot *output) {
  if (output == nullptr) {
    return;
  }
  *output = {};
  if (!valid_identity(identity)) {
    return;
  }

  critical_section_enter_blocking(&g_lock);
  output->metadata = g_metadata;
  if ((g_metadata.state == ProfileServiceState::kReady ||
       g_metadata.state == ProfileServiceState::kStorageError) &&
      g_active_profile_count != 0) {
    const PublishedActiveProfile *active =
        controller_identity_is_joycon_pair(identity) ? nullptr
                                                    : &g_active_profiles[0];
    for (uint8_t index = 1; index < g_active_profile_count; ++index) {
      if (controller_identity_equal(g_active_profiles[index].identity,
                                    identity)) {
        active = &g_active_profiles[index];
        break;
      }
    }
    if (active != nullptr) {
      output->profile_index = active->profile_index;
      output->profile = active->profile;
      output->valid = true;
    }
  }
  critical_section_exit(&g_lock);
}
