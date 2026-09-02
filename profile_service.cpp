#include "profile_service.h"

#include <string.h>

#include "pico/critical_section.h"
#include "pico_profile_storage.h"
#include "profile_storage.h"

namespace {

constexpr uint32_t kMinimumCommitIntervalMs = 1000;

enum class PendingCommandType : uint8_t {
    kNone = 0,
    kReset = 1,
    kActivate = 2,
};

struct PendingCommand {
    PendingCommandType type = PendingCommandType::kNone;
    ControllerIdentity identity{};
    uint8_t profile_index = 0;
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
ControllerProfileDatabase g_database;
ProfileServiceMetadata g_metadata;
uint32_t g_published_generation = 0;
ProfileServiceListSnapshot g_list;
ProfileServiceSelectedSnapshot g_selected;
PublishedActiveProfile
    g_active_profiles[PROFILE_SERVICE_LIST_CAPACITY]{};
uint8_t g_active_profile_count = 0;
ProfileTransaction g_transaction;
PendingCommand g_command;
bool g_identity_dirty = false;
bool g_has_committed = false;
uint32_t g_last_commit_ms = 0;

bool valid_identity(const ControllerIdentity& identity) {
    uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE]{};
    return (controller_identity_is_global(identity) || identity.stable) &&
           controller_identity_encode(identity, encoded, sizeof(encoded));
}

void refresh_list_locked() {
    g_list = {};
    g_list.metadata = g_metadata;
    g_list.count = 1;
    g_list.rows[0].identity = controller_identity_global();
    g_list.rows[0].active_profile =
        g_database.fallback_active_profile;
    for (const ControllerProfileDatabaseEntry& entry : g_database.entries) {
        if (!entry.used || g_list.count >= PROFILE_SERVICE_LIST_CAPACITY) {
            continue;
        }
        ProfileServiceListRow& row = g_list.rows[g_list.count++];
        row.identity = entry.identity;
        row.active_profile = entry.active_profile;
    }
}

void refresh_active_profiles_locked() {
    g_active_profile_count = 1;
    g_active_profiles[0].identity = controller_identity_global();
    g_active_profiles[0].profile_index =
        g_database.fallback_active_profile;
    g_active_profiles[0].profile =
        g_database.fallback_profiles[g_database.fallback_active_profile];
    for (const ControllerProfileDatabaseEntry& entry : g_database.entries) {
        if (!entry.used ||
            g_active_profile_count >= PROFILE_SERVICE_LIST_CAPACITY) {
            continue;
        }
        PublishedActiveProfile& active =
            g_active_profiles[g_active_profile_count++];
        active.identity = entry.identity;
        active.profile_index = entry.active_profile;
        active.profile = entry.profiles[entry.active_profile];
    }
}

void refresh_selected_locked() {
    g_selected.metadata = g_metadata;
    const ControllerProfile* profile = controller_profile_database_get(
        g_database, g_selected.identity, g_selected.profile_index);
    if (profile == nullptr) {
        g_selected.valid = false;
        g_selected.status = ConfigurationTransactionStatus::kMalformed;
        return;
    }
    g_selected.profile = *profile;
    g_selected.valid = true;
    g_selected.status = ConfigurationTransactionStatus::kCommitted;
}

void refresh_metadata_locked(ProfileServiceState state) {
    const ProfileStorageSnapshot& stored = g_storage.snapshot();
    g_metadata.state = state;
    g_metadata.generation = stored.valid ? stored.generation : 0;
    g_metadata.payload_crc = stored.valid ? stored.payload_crc : 0;
    refresh_active_profiles_locked();
    refresh_list_locked();
    refresh_selected_locked();
    __atomic_store_n(&g_published_generation, g_metadata.generation,
                     __ATOMIC_RELEASE);
}

ConfigurationTransactionStatus database_result_status(
    ControllerProfileDatabaseResult result) {
    switch (result) {
        case ControllerProfileDatabaseResult::kOk:
            return ConfigurationTransactionStatus::kPending;
        case ControllerProfileDatabaseResult::kFull:
            return ConfigurationTransactionStatus::kTooLarge;
        case ControllerProfileDatabaseResult::kInvalidArgument:
            return ConfigurationTransactionStatus::kMalformed;
    }
    return ConfigurationTransactionStatus::kStorageError;
}

bool mutation_ready(uint32_t now_ms) {
    return !g_has_committed ||
           static_cast<uint32_t>(now_ms - g_last_commit_ms) >=
               kMinimumCommitIntervalMs;
}

void finish_mutation(ConfigurationTransactionStatus status,
                     bool clear_command) {
    const ProfileStorageSnapshot& stored = g_storage.snapshot();
    critical_section_enter_blocking(&g_lock);
    g_transaction.snapshot.status = status;
    g_transaction.snapshot.stored_generation =
        stored.valid ? stored.generation : 0;
    g_transaction.snapshot.stored_crc =
        stored.valid ? stored.payload_crc : 0;
    if (clear_command) {
        g_command = {};
    }
    refresh_metadata_locked(
        status == ConfigurationTransactionStatus::kStorageError
            ? ProfileServiceState::kStorageError
            : ProfileServiceState::kReady);
    critical_section_exit(&g_lock);
}

}  // namespace

void profile_service_prepare() {
    if (g_prepared) {
        return;
    }
    critical_section_init(&g_lock);
    g_metadata = {};
    __atomic_store_n(&g_published_generation, 0, __ATOMIC_RELAXED);
    g_list = {};
    g_selected = {};
    g_active_profiles[0] = {};
    g_active_profile_count = 0;
    g_selected.identity = controller_identity_global();
    g_selected.profile_index = 0;
    g_transaction = {};
    g_command = {};
    g_identity_dirty = false;
    g_has_committed = false;
    g_last_commit_ms = 0;
    g_prepared = true;
}

void profile_service_initialize_on_storage_core() {
    if (!g_prepared) {
        profile_service_prepare();
    }
    const bool initialized =
        g_storage.initialize(pico_profile_storage_io(), &g_database);
    critical_section_enter_blocking(&g_lock);
    refresh_metadata_locked(initialized ? ProfileServiceState::kReady
                                        : ProfileServiceState::kStorageError);
    critical_section_exit(&g_lock);
}

bool profile_service_observe_identity_on_storage_core(
    const ControllerIdentity& identity) {
    if (!g_prepared || !identity.stable ||
        controller_identity_is_global(identity) ||
        !valid_identity(identity)) {
        return false;
    }
    critical_section_enter_blocking(&g_lock);
    if (g_metadata.state != ProfileServiceState::kReady) {
        critical_section_exit(&g_lock);
        return false;
    }
    ControllerProfileDatabaseEntry* entry =
        controller_profile_database_find(&g_database, identity);
    if (entry != nullptr) {
        critical_section_exit(&g_lock);
        return true;
    }
    const ControllerProfileDatabaseResult result =
        controller_profile_database_ensure(&g_database, identity, &entry);
    if (result == ControllerProfileDatabaseResult::kOk) {
        g_identity_dirty = true;
        refresh_list_locked();
    }
    critical_section_exit(&g_lock);
    return result == ControllerProfileDatabaseResult::kOk;
}


void profile_service_task_on_storage_core(uint32_t now_ms) {
    PendingCommand command{};
    bool process_write = false;
    bool process_identity = false;
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
            memcpy(write_payload, g_transaction.payload,
                   sizeof(write_payload));
        } else if (g_identity_dirty) {
            process_identity = true;
        }
    }
    critical_section_exit(&g_lock);

    if (!process_write && !process_identity &&
        command.type == PendingCommandType::kNone) {
        return;
    }

    ControllerProfileDatabaseResult database_result =
        ControllerProfileDatabaseResult::kInvalidArgument;
    if (process_identity) {
        database_result = ControllerProfileDatabaseResult::kOk;
    } else if (process_write) {
        ControllerProfile profile{};
        if (controller_profile_decode(write_payload, sizeof(write_payload),
                                      &profile)) {
            critical_section_enter_blocking(&g_lock);
            database_result = controller_profile_database_set(
                &g_database, write_identity, write_profile_index, profile);
            critical_section_exit(&g_lock);
        }
    } else if (command.type == PendingCommandType::kReset) {
        critical_section_enter_blocking(&g_lock);
        database_result = controller_profile_database_reset(
            &g_database, command.identity, command.profile_index);
        critical_section_exit(&g_lock);
    } else if (command.type == PendingCommandType::kActivate) {
        critical_section_enter_blocking(&g_lock);
        database_result = controller_profile_database_activate(
            &g_database, command.identity, command.profile_index);
        critical_section_exit(&g_lock);
    }

    if (database_result != ControllerProfileDatabaseResult::kOk) {
        finish_mutation(database_result_status(database_result),
                        !process_write);
        return;
    }

    const ProfileStorageResult storage_result = g_storage.commit(g_database);
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kStorageError;
    if (storage_result == ProfileStorageResult::kOk) {
        status = ConfigurationTransactionStatus::kCommitted;
        g_has_committed = true;
        g_last_commit_ms = now_ms;
    } else if (storage_result == ProfileStorageResult::kUnchanged) {
        status = ConfigurationTransactionStatus::kUnchanged;
    } else {
        critical_section_enter_blocking(&g_lock);
        const bool restored =
            g_storage.initialize(pico_profile_storage_io(), &g_database);
        critical_section_exit(&g_lock);
        if (!restored) {
            status = ConfigurationTransactionStatus::kStorageError;
        }
    }
    if (process_identity) {
        critical_section_enter_blocking(&g_lock);
        g_identity_dirty = false;
        refresh_metadata_locked(
            status == ConfigurationTransactionStatus::kStorageError
                ? ProfileServiceState::kStorageError
                : ProfileServiceState::kReady);
        critical_section_exit(&g_lock);
        return;
    }
    critical_section_enter_blocking(&g_lock);
    g_identity_dirty = false;
    critical_section_exit(&g_lock);
    finish_mutation(status, !process_write);
}

ConfigurationTransactionStatus profile_service_select(
    const ControllerIdentity& identity, uint8_t profile_index) {
    if (!g_prepared) {
        profile_service_prepare();
    }
    critical_section_enter_blocking(&g_lock);
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kCommitted;
    if (!valid_identity(identity) ||
        profile_index >= CONTROLLER_PROFILE_COUNT) {
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

ConfigurationTransactionStatus profile_service_begin(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc) {
    if (!g_prepared) {
        profile_service_prepare();
    }
    critical_section_enter_blocking(&g_lock);
    if (g_command.type != PendingCommandType::kNone ||
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
    if (transaction_id == 0 || !valid_identity(identity) ||
        profile_index >= CONTROLLER_PROFILE_COUNT || payload_size == 0) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kMalformed;
    } else if (schema_version != CONTROLLER_PROFILE_SCHEMA_VERSION) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kUnsupportedSchema;
    } else if (payload_size > CONTROLLER_PROFILE_ENCODED_SIZE) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kTooLarge;
    } else if (payload_size != CONTROLLER_PROFILE_ENCODED_SIZE) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kMalformed;
    } else {
        g_transaction.snapshot.expected_size =
            static_cast<uint16_t>(payload_size);
        g_transaction.snapshot.expected_crc = payload_crc;
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kReceiving;
    }
    const ConfigurationTransactionStatus status =
        g_transaction.snapshot.status;
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus profile_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
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
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kOutOfOrder;
    } else {
        memcpy(&g_transaction.payload[offset], data, size);
        g_transaction.snapshot.received_size =
            static_cast<uint16_t>(offset + size);
    }
    const ConfigurationTransactionStatus status =
        g_transaction.snapshot.status;
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus profile_service_commit(
    uint32_t transaction_id) {
    critical_section_enter_blocking(&g_lock);
    if (g_transaction.snapshot.status !=
            ConfigurationTransactionStatus::kReceiving ||
        transaction_id != g_transaction.snapshot.transaction_id ||
        g_transaction.snapshot.received_size !=
            g_transaction.snapshot.expected_size) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kOutOfOrder;
    } else if (profile_storage_crc32(
                   g_transaction.payload,
                   g_transaction.snapshot.expected_size) !=
               g_transaction.snapshot.expected_crc) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kBadCrc;
    } else {
        ControllerProfile profile{};
        g_transaction.snapshot.status =
            controller_profile_decode(
                g_transaction.payload,
                g_transaction.snapshot.expected_size, &profile)
                ? ConfigurationTransactionStatus::kPending
                : ConfigurationTransactionStatus::kMalformed;
    }
    const ConfigurationTransactionStatus status =
        g_transaction.snapshot.status;
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus profile_service_reset(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index) {
    if (!g_prepared) {
        profile_service_prepare();
    }
    critical_section_enter_blocking(&g_lock);
    if (g_command.type != PendingCommandType::kNone ||
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
    if (transaction_id == 0 || !valid_identity(identity) ||
        (profile_index != CONTROLLER_PROFILE_ALL &&
         profile_index >= CONTROLLER_PROFILE_COUNT)) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kMalformed;
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kMalformed;
    }
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kPending;
    g_command.type = PendingCommandType::kReset;
    g_command.identity = identity;
    g_command.profile_index = profile_index;
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kPending;
}

ConfigurationTransactionStatus profile_service_activate(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index) {
    if (!g_prepared) {
        profile_service_prepare();
    }
    critical_section_enter_blocking(&g_lock);
    if (g_command.type != PendingCommandType::kNone ||
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
    if (transaction_id == 0 || !valid_identity(identity) ||
        profile_index >= CONTROLLER_PROFILE_COUNT) {
        g_transaction.snapshot.status =
            ConfigurationTransactionStatus::kMalformed;
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kMalformed;
    }
    g_transaction.snapshot.status = ConfigurationTransactionStatus::kPending;
    g_command.type = PendingCommandType::kActivate;
    g_command.identity = identity;
    g_command.profile_index = profile_index;
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kPending;
}

void profile_service_list_snapshot(ProfileServiceListSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    critical_section_enter_blocking(&g_lock);
    *output = g_list;
    critical_section_exit(&g_lock);
}

void profile_service_selected_snapshot(
    ProfileServiceSelectedSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    critical_section_enter_blocking(&g_lock);
    *output = g_selected;
    critical_section_exit(&g_lock);
}

void profile_service_transaction_snapshot(
    ProfileServiceTransactionSnapshot* output) {
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
    const ControllerIdentity& identity,
    ProfileServiceActiveProfileSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    *output = {};
    if (!valid_identity(identity)) {
        return;
    }

    critical_section_enter_blocking(&g_lock);
    output->metadata = g_metadata;
    if (g_metadata.state == ProfileServiceState::kReady &&
        g_active_profile_count != 0) {
        const PublishedActiveProfile* active = &g_active_profiles[0];
        for (uint8_t index = 1; index < g_active_profile_count; ++index) {
            if (controller_identity_equal(
                    g_active_profiles[index].identity, identity)) {
                active = &g_active_profiles[index];
                break;
            }
        }
        output->profile_index = active->profile_index;
        output->profile = active->profile;
        output->valid = true;
    }
    critical_section_exit(&g_lock);
}
