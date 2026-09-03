#include "configuration_service.h"

#include <string.h>

#include "pico/critical_section.h"
#include "pico_configuration_storage.h"

namespace {

constexpr uint32_t kMinimumCommitIntervalMs = 1000;

enum class PendingWriteOwner : uint8_t {
    kNone,
    kHost,
    kInternal,
    kMigration,
};

critical_section_t g_lock;
bool g_prepared = false;
bool g_pre_usb_initialized = false;
bool g_storage_initialized = false;
bool g_storage_core_adopted = false;
bool g_migration_needed = false;
bool g_migration_pending = false;
ConfigurationStorage g_storage;
ConfigurationTransaction g_transaction;
ConfigurationServiceSnapshot g_snapshot;
ConfigurationModeTransactionSnapshot g_host_mode_transaction;
ConfigurationModeTransactionSnapshot g_internal_mode_transaction;
uint8_t g_internal_payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
uint8_t g_migration_payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
uint32_t g_published_reset_generation = 0;
bool g_has_committed = false;
uint32_t g_last_commit_ms = 0;
uint64_t g_latest_mode_transaction_serial = 0;
bool g_recovery_reserved = false;

bool transaction_active(ConfigurationTransactionStatus status) {
    return status == ConfigurationTransactionStatus::kReceiving ||
           status == ConfigurationTransactionStatus::kPending;
}

bool internal_write_pending() {
    return g_migration_pending ||
           g_internal_mode_transaction.status ==
               ConfigurationTransactionStatus::kPending;
}

uint64_t advance_mode_transaction_serial() {
    ++g_latest_mode_transaction_serial;
    if (g_latest_mode_transaction_serial == 0) {
        ++g_latest_mode_transaction_serial;
    }
    return g_latest_mode_transaction_serial;
}

bool decode_storage_configuration(
    const ConfigurationStorageSnapshot& stored,
    AdapterConfiguration* configuration,
    bool* migration_needed) {
    *configuration = adapter_configuration_default();
    *migration_needed = false;
    if (!stored.valid) {
        return true;
    }
    if (!adapter_configuration_decode(stored.schema_version, stored.payload,
                                      stored.payload_size, configuration)) {
        return false;
    }
    *migration_needed =
        stored.schema_version == ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION;
    return true;
}

void publish_storage_snapshot(bool initialized) {
    const ConfigurationStorageSnapshot& stored = g_storage.snapshot();
    AdapterConfiguration configuration = adapter_configuration_default();
    bool migration_needed = false;
    const bool decoded = initialized &&
                         decode_storage_configuration(
                             stored, &configuration, &migration_needed);

    critical_section_enter_blocking(&g_lock);
    g_storage_initialized = initialized;
    g_migration_needed = decoded && migration_needed;
    g_snapshot.state = decoded ? ConfigurationServiceState::kReady
                               : ConfigurationServiceState::kStorageError;
    g_snapshot.configuration = configuration;
    g_snapshot.generation = stored.valid ? stored.generation : 0;
    g_snapshot.payload_crc = stored.valid ? stored.payload_crc : 0;
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
}


void publish_mode_transaction(
    const ConfigurationModeTransactionSnapshot& transaction) {
    g_snapshot.mode_transaction = transaction;
}
const ConfigurationModeTransactionSnapshot& mode_transaction_for_id(
    uint32_t transaction_id) {
    return (transaction_id &
            CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0
               ? g_internal_mode_transaction
               : g_host_mode_transaction;
}

bool mode_transaction_succeeded(
    const ConfigurationModeTransactionSnapshot& transaction) {
    return transaction.status == ConfigurationTransactionStatus::kCommitted ||
           transaction.status == ConfigurationTransactionStatus::kUnchanged;
}

ConfigurationTransactionStatus validate_mode_request(
    AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    if (!adapter_requested_mode_valid(requested_mode)) {
        return ConfigurationTransactionStatus::kMalformed;
    }
    if (!adapter_requested_mode_available(requested_mode, availability)) {
        return ConfigurationTransactionStatus::kUnsupportedSchema;
    }
    return ConfigurationTransactionStatus::kIdle;
}

void update_snapshot_from_storage(ConfigurationStorageResult result) {
    const ConfigurationStorageSnapshot& stored = g_storage.snapshot();
    AdapterConfiguration configuration = adapter_configuration_default();
    bool migration_needed = false;
    const bool decoded = decode_storage_configuration(
        stored, &configuration, &migration_needed);
    g_snapshot.state = decoded ? ConfigurationServiceState::kReady
                               : ConfigurationServiceState::kStorageError;
    if (!stored.valid && result == ConfigurationStorageResult::kIoError) {
        g_snapshot.state = ConfigurationServiceState::kStorageError;
    }
    g_snapshot.configuration = configuration;
    g_snapshot.generation = stored.valid ? stored.generation : 0;
    g_snapshot.payload_crc = stored.valid ? stored.payload_crc : 0;
    g_migration_needed = decoded && migration_needed;
}

}  // namespace

void configuration_service_prepare() {
    if (g_prepared) {
        return;
    }
    critical_section_init(&g_lock);
    g_snapshot = {};
    g_snapshot.configuration = adapter_configuration_default();
    g_transaction.clear();
    g_host_mode_transaction = {};
    g_internal_mode_transaction = {};
    g_latest_mode_transaction_serial = 0;
    g_recovery_reserved = false;
    __atomic_store_n(&g_published_reset_generation, 0, __ATOMIC_RELAXED);
    g_prepared = true;
}

void configuration_service_initialize_pre_usb() {
    if (!g_prepared) {
        configuration_service_prepare();
    }

    critical_section_enter_blocking(&g_lock);
    const bool already_initialized = g_pre_usb_initialized;
    if (!already_initialized) {
        g_pre_usb_initialized = true;
    }
    critical_section_exit(&g_lock);
    if (already_initialized) {
        return;
    }

    const bool initialized =
        g_storage.initialize(pico_configuration_storage_io());
    publish_storage_snapshot(initialized);
}

void configuration_service_initialize_on_storage_core() {
    if (!g_prepared) {
        configuration_service_prepare();
    }
    critical_section_enter_blocking(&g_lock);
    const bool pre_usb_initialized = g_pre_usb_initialized;
    critical_section_exit(&g_lock);
    if (!pre_usb_initialized) {
        configuration_service_initialize_pre_usb();
    }

    critical_section_enter_blocking(&g_lock);
    if (g_storage_core_adopted) {
        critical_section_exit(&g_lock);
        return;
    }
    g_storage_core_adopted = g_storage_initialized;
    if (g_storage_core_adopted && g_migration_needed) {
        const AdapterConfiguration configuration = g_snapshot.configuration;
        if (adapter_configuration_encode(configuration, g_migration_payload,
                                         sizeof(g_migration_payload))) {
            g_migration_pending = true;
        } else {
            g_snapshot.state = ConfigurationServiceState::kStorageError;
        }
    }
    critical_section_exit(&g_lock);
}

void configuration_service_task_on_storage_core(uint32_t now_ms) {
    uint8_t payload[CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE]{};
    uint16_t payload_size = 0;
    uint16_t schema_version = ADAPTER_CONFIGURATION_SCHEMA_VERSION;
    uint32_t transaction_id = 0;
    PendingWriteOwner owner = PendingWriteOwner::kNone;

    critical_section_enter_blocking(&g_lock);
    const bool rate_limited =
        g_has_committed &&
        static_cast<uint32_t>(now_ms - g_last_commit_ms) <
            kMinimumCommitIntervalMs;
    if (g_storage_core_adopted && !rate_limited) {
        if (g_migration_pending) {
            owner = PendingWriteOwner::kMigration;
            payload_size = sizeof(g_migration_payload);
            memcpy(payload, g_migration_payload, payload_size);
        } else if (g_internal_mode_transaction.status ==
                   ConfigurationTransactionStatus::kPending) {
            owner = PendingWriteOwner::kInternal;
            transaction_id = g_internal_mode_transaction.transaction_id;
            payload_size = sizeof(g_internal_payload);
            memcpy(payload, g_internal_payload, payload_size);
        } else {
            const ConfigurationTransactionSnapshot transaction =
                g_transaction.snapshot();
            if (transaction.status ==
                ConfigurationTransactionStatus::kPending) {
                owner = PendingWriteOwner::kHost;
                transaction_id = transaction.transaction_id;
                payload_size = transaction.expected_size;
                schema_version = g_transaction.schema_version();
                memcpy(payload, g_transaction.payload(), payload_size);
            }
        }
    }
    critical_section_exit(&g_lock);
    if (owner == PendingWriteOwner::kNone) {
        return;
    }

    const ConfigurationStorageResult result =
        g_storage.commit(schema_version, payload, payload_size);
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kStorageError;
    if (result == ConfigurationStorageResult::kOk) {
        status = ConfigurationTransactionStatus::kCommitted;
        g_has_committed = true;
        g_last_commit_ms = now_ms;
    } else if (result == ConfigurationStorageResult::kUnchanged) {
        status = ConfigurationTransactionStatus::kUnchanged;
    }

    critical_section_enter_blocking(&g_lock);
    update_snapshot_from_storage(result);
    if (owner == PendingWriteOwner::kHost) {
        const ConfigurationTransactionSnapshot transaction =
            g_transaction.snapshot();
        if (transaction.transaction_id == transaction_id &&
            transaction.status ==
                ConfigurationTransactionStatus::kPending) {
            g_transaction.set_result(status, g_snapshot.generation,
                                     g_snapshot.payload_crc);
            if (g_host_mode_transaction.transaction_id == transaction_id &&
                g_host_mode_transaction.status ==
                    ConfigurationTransactionStatus::kPending) {
                g_host_mode_transaction.status = status;
                g_host_mode_transaction.stored_generation =
                    g_snapshot.generation;
                if (!g_snapshot.mode_transaction.internal &&
                    g_snapshot.mode_transaction.transaction_id ==
                        transaction_id) {
                    publish_mode_transaction(g_host_mode_transaction);
                }
            }
        }
        g_snapshot.transaction = g_transaction.snapshot();
    } else if (owner == PendingWriteOwner::kInternal) {
        if (g_internal_mode_transaction.transaction_id == transaction_id &&
            g_internal_mode_transaction.status ==
                ConfigurationTransactionStatus::kPending) {
            g_internal_mode_transaction.status = status;
            g_internal_mode_transaction.stored_generation =
                g_snapshot.generation;
            if (g_snapshot.mode_transaction.internal &&
                g_snapshot.mode_transaction.transaction_id ==
                    transaction_id) {
                publish_mode_transaction(g_internal_mode_transaction);
            }
        }
    } else {
        g_migration_pending = false;
        if (status == ConfigurationTransactionStatus::kCommitted ||
            status == ConfigurationTransactionStatus::kUnchanged) {
            g_migration_needed = false;
        }
    }
    critical_section_exit(&g_lock);
}

ConfigurationTransactionStatus configuration_service_begin(
    uint32_t transaction_id, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc) {
    if ((transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    critical_section_enter_blocking(&g_lock);
    if (!g_storage_core_adopted || g_recovery_reserved ||
        internal_write_pending()) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    const ConfigurationTransactionStatus status = g_transaction.begin(
        transaction_id, schema_version, payload_size, payload_crc);
    if (status == ConfigurationTransactionStatus::kReceiving) {
        advance_mode_transaction_serial();
    }
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
    if ((transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    critical_section_enter_blocking(&g_lock);
    if (g_recovery_reserved || internal_write_pending()) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    const ConfigurationTransactionStatus status =
        g_transaction.append(transaction_id, offset, data, size);
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_commit(
    uint32_t transaction_id) {
    if ((transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    critical_section_enter_blocking(&g_lock);
    if (g_recovery_reserved || internal_write_pending()) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    const ConfigurationTransactionStatus status =
        g_transaction.finish(transaction_id);
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_reset(
    uint32_t transaction_id) {
    if (transaction_id == 0 ||
        (transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    const AdapterConfiguration defaults = adapter_configuration_default();
    if (!adapter_configuration_encode(defaults, payload, sizeof(payload))) {
        return ConfigurationTransactionStatus::kMalformed;
    }
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));

    critical_section_enter_blocking(&g_lock);
    if (!g_storage_core_adopted || g_recovery_reserved ||
        internal_write_pending()) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    ConfigurationTransactionStatus status = g_transaction.begin(
        transaction_id, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
        sizeof(payload), crc);
    if (status == ConfigurationTransactionStatus::kReceiving) {
        status = g_transaction.append(transaction_id, 0, payload,
                                      sizeof(payload));
    }
    if (status == ConfigurationTransactionStatus::kReceiving) {
        status = g_transaction.finish(transaction_id);
    }
    if (status == ConfigurationTransactionStatus::kPending) {
        advance_mode_transaction_serial();
    }
    if (status == ConfigurationTransactionStatus::kPending &&
        g_snapshot.reset_generation != UINT32_MAX) {
        ++g_snapshot.reset_generation;
    }
    g_snapshot.transaction = g_transaction.snapshot();
    if (status == ConfigurationTransactionStatus::kPending) {
        __atomic_store_n(&g_published_reset_generation,
                         g_snapshot.reset_generation, __ATOMIC_RELEASE);
    }
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_set_mode(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    if (transaction_id == 0 ||
        (transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) != 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    critical_section_enter_blocking(&g_lock);
    if (g_recovery_reserved) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    const ConfigurationTransactionStatus validation =
        validate_mode_request(requested_mode, availability);
    if (validation != ConfigurationTransactionStatus::kIdle) {
        critical_section_exit(&g_lock);
        return validation;
    }
    if (g_host_mode_transaction.transaction_id == transaction_id) {
        const ConfigurationTransactionStatus status =
            g_host_mode_transaction.requested_mode == requested_mode
                ? g_host_mode_transaction.status
                : ConfigurationTransactionStatus::kMalformed;
        critical_section_exit(&g_lock);
        return status;
    }
    if (!g_storage_core_adopted || internal_write_pending() ||
        transaction_active(g_transaction.snapshot().status)) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    if (g_snapshot.state != ConfigurationServiceState::kReady) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kStorageError;
    }

    AdapterConfiguration configuration = g_snapshot.configuration;
    configuration.requested_mode = requested_mode;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    if (!adapter_configuration_encode(configuration, payload,
                                      sizeof(payload))) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kMalformed;
    }
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));
    ConfigurationTransactionStatus status = g_transaction.begin(
        transaction_id, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
        sizeof(payload), crc);
    if (status == ConfigurationTransactionStatus::kReceiving) {
        status = g_transaction.append(transaction_id, 0, payload,
                                      sizeof(payload));
    }
    if (status == ConfigurationTransactionStatus::kReceiving) {
        status = g_transaction.finish(transaction_id);
    }
    if (status == ConfigurationTransactionStatus::kPending) {
        g_host_mode_transaction = {
            transaction_id,
            requested_mode,
            0,
            advance_mode_transaction_serial(),
            status,
            false,
        };
        publish_mode_transaction(g_host_mode_transaction);
    }
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_set_mode_internal(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    if ((transaction_id &
         CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK) == 0) {
        return ConfigurationTransactionStatus::kMalformed;
    }

    critical_section_enter_blocking(&g_lock);
    if (g_recovery_reserved &&
        requested_mode != AdapterRequestedMode::kAuto) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    const ConfigurationTransactionStatus validation =
        validate_mode_request(requested_mode, availability);
    if (validation != ConfigurationTransactionStatus::kIdle) {
        critical_section_exit(&g_lock);
        return validation;
    }
    if (g_internal_mode_transaction.transaction_id == transaction_id) {
        const ConfigurationTransactionStatus status =
            g_internal_mode_transaction.requested_mode == requested_mode
                ? g_internal_mode_transaction.status
                : ConfigurationTransactionStatus::kMalformed;
        critical_section_exit(&g_lock);
        return status;
    }
    if (!g_storage_core_adopted || g_migration_pending ||
        g_internal_mode_transaction.status ==
            ConfigurationTransactionStatus::kPending ||
        transaction_active(g_transaction.snapshot().status)) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kBusy;
    }
    if (g_snapshot.state != ConfigurationServiceState::kReady) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kStorageError;
    }

    AdapterConfiguration configuration = g_snapshot.configuration;
    configuration.requested_mode = requested_mode;
    if (!adapter_configuration_encode(configuration, g_internal_payload,
                                      sizeof(g_internal_payload))) {
        critical_section_exit(&g_lock);
        return ConfigurationTransactionStatus::kMalformed;
    }
    g_internal_mode_transaction = {
        transaction_id,
        requested_mode,
        0,
        advance_mode_transaction_serial(),
        ConfigurationTransactionStatus::kPending,
        true,
    };
    publish_mode_transaction(g_internal_mode_transaction);
    critical_section_exit(&g_lock);
    return ConfigurationTransactionStatus::kPending;
}

void configuration_service_reserve_for_recovery() {
    critical_section_enter_blocking(&g_lock);
    if (!g_recovery_reserved) {
        g_recovery_reserved = true;
        advance_mode_transaction_serial();
    }

    const ConfigurationTransactionSnapshot transaction =
        g_transaction.snapshot();
    if (transaction_active(transaction.status)) {
        g_transaction.set_result(ConfigurationTransactionStatus::kBusy, 0, 0);
        if (g_host_mode_transaction.transaction_id ==
                transaction.transaction_id &&
            transaction_active(g_host_mode_transaction.status)) {
            g_host_mode_transaction.status =
                ConfigurationTransactionStatus::kBusy;
            g_host_mode_transaction.stored_generation = 0;
            if (!g_snapshot.mode_transaction.internal &&
                g_snapshot.mode_transaction.transaction_id ==
                    transaction.transaction_id) {
                publish_mode_transaction(g_host_mode_transaction);
            }
        }
        g_snapshot.transaction = g_transaction.snapshot();
    }
    critical_section_exit(&g_lock);
}

bool configuration_service_mode_transaction_status(
    uint32_t transaction_id, ConfigurationTransactionStatus* output) {
    if (transaction_id == 0 || output == nullptr) {
        return false;
    }

    critical_section_enter_blocking(&g_lock);
    const ConfigurationModeTransactionSnapshot& transaction =
        mode_transaction_for_id(transaction_id);
    const bool found = transaction.transaction_id == transaction_id;
    if (found) {
        *output = transaction.status;
    }
    critical_section_exit(&g_lock);
    return found;
}

bool configuration_service_mode_transaction_reboot_ready(
    uint32_t transaction_id) {
    if (transaction_id == 0) {
        return false;
    }

    critical_section_enter_blocking(&g_lock);
    const ConfigurationModeTransactionSnapshot& transaction =
        mode_transaction_for_id(transaction_id);
    const bool ready =
        transaction.transaction_id == transaction_id &&
        mode_transaction_succeeded(transaction) &&
        transaction.accepted_serial ==
            g_latest_mode_transaction_serial &&
        transaction.stored_generation == g_snapshot.generation &&
        transaction.requested_mode == g_snapshot.configuration.requested_mode;
    critical_section_exit(&g_lock);
    return ready;
}

void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    critical_section_enter_blocking(&g_lock);
    *output = g_snapshot;
    critical_section_exit(&g_lock);
}

uint32_t configuration_service_reset_generation() {
    return __atomic_load_n(&g_published_reset_generation, __ATOMIC_ACQUIRE);
}
