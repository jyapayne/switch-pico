#include "configuration_service.h"

#include <string.h>

#include "pico/critical_section.h"
#include "pico_configuration_storage.h"

namespace {

constexpr uint32_t kMinimumCommitIntervalMs = 1000;

critical_section_t g_lock;
bool g_prepared = false;
ConfigurationStorage g_storage;
ConfigurationTransaction g_transaction;
ConfigurationServiceSnapshot g_snapshot;
bool g_has_committed = false;
uint32_t g_last_commit_ms = 0;

void publish_storage_snapshot(ConfigurationServiceState state) {
    const ConfigurationStorageSnapshot& stored = g_storage.snapshot();
    AdapterConfiguration configuration = adapter_configuration_default();
    if (stored.valid &&
        (stored.schema_version != ADAPTER_CONFIGURATION_SCHEMA_VERSION ||
         !adapter_configuration_decode(stored.payload,
                                       stored.payload_size,
                                       &configuration))) {
        state = ConfigurationServiceState::kStorageError;
    }

    critical_section_enter_blocking(&g_lock);
    g_snapshot.state = state;
    g_snapshot.configuration = configuration;
    g_snapshot.generation = stored.valid ? stored.generation : 0;
    g_snapshot.payload_crc = stored.valid ? stored.payload_crc : 0;
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
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
    g_prepared = true;
}

void configuration_service_initialize_on_storage_core() {
    if (!g_prepared) {
        configuration_service_prepare();
    }
    const bool initialized =
        g_storage.initialize(pico_configuration_storage_io());
    publish_storage_snapshot(initialized
                                 ? ConfigurationServiceState::kReady
                                 : ConfigurationServiceState::kStorageError);
}

void configuration_service_task_on_storage_core(uint32_t now_ms) {
    uint8_t payload[CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE]{};
    uint16_t payload_size = 0;
    uint16_t schema_version = 0;

    critical_section_enter_blocking(&g_lock);
    const ConfigurationTransactionSnapshot transaction =
        g_transaction.snapshot();
    if (transaction.status == ConfigurationTransactionStatus::kPending &&
        (!g_has_committed ||
         static_cast<uint32_t>(now_ms - g_last_commit_ms) >=
             kMinimumCommitIntervalMs)) {
        payload_size = transaction.expected_size;
        schema_version = g_transaction.schema_version();
        memcpy(payload, g_transaction.payload(), payload_size);
    }
    critical_section_exit(&g_lock);
    if (payload_size == 0) {
        return;
    }

    const ConfigurationStorageResult result =
        g_storage.commit(schema_version, payload, payload_size);
    const ConfigurationStorageSnapshot& stored = g_storage.snapshot();
    ConfigurationTransactionStatus transaction_status =
        ConfigurationTransactionStatus::kStorageError;
    if (result == ConfigurationStorageResult::kOk) {
        transaction_status = ConfigurationTransactionStatus::kCommitted;
        g_has_committed = true;
        g_last_commit_ms = now_ms;
    } else if (result == ConfigurationStorageResult::kUnchanged) {
        transaction_status = ConfigurationTransactionStatus::kUnchanged;
    }

    AdapterConfiguration configuration{};
    ConfigurationServiceState service_state =
        ConfigurationServiceState::kStorageError;
    if (stored.valid &&
        stored.schema_version == ADAPTER_CONFIGURATION_SCHEMA_VERSION &&
        adapter_configuration_decode(stored.payload, stored.payload_size,
                                     &configuration)) {
        service_state = ConfigurationServiceState::kReady;
    } else if (!stored.valid) {
        configuration = adapter_configuration_default();
    }

    critical_section_enter_blocking(&g_lock);
    g_transaction.set_result(transaction_status,
                             stored.valid ? stored.generation : 0,
                             stored.valid ? stored.payload_crc : 0);
    g_snapshot.state = service_state;
    g_snapshot.configuration = configuration;
    g_snapshot.generation = stored.valid ? stored.generation : 0;
    g_snapshot.payload_crc = stored.valid ? stored.payload_crc : 0;
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
}

ConfigurationTransactionStatus configuration_service_begin(
    uint32_t transaction_id, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc) {
    critical_section_enter_blocking(&g_lock);
    const ConfigurationTransactionStatus status = g_transaction.begin(
        transaction_id, schema_version, payload_size, payload_crc);
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size) {
    critical_section_enter_blocking(&g_lock);
    const ConfigurationTransactionStatus status =
        g_transaction.append(transaction_id, offset, data, size);
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_commit(
    uint32_t transaction_id) {
    critical_section_enter_blocking(&g_lock);
    const ConfigurationTransactionStatus status =
        g_transaction.finish(transaction_id);
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

ConfigurationTransactionStatus configuration_service_reset(
    uint32_t transaction_id) {
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    const AdapterConfiguration defaults = adapter_configuration_default();
    if (!adapter_configuration_encode(defaults, payload, sizeof(payload))) {
        return ConfigurationTransactionStatus::kMalformed;
    }
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));

    critical_section_enter_blocking(&g_lock);
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
    if (status == ConfigurationTransactionStatus::kPending &&
        g_snapshot.reset_generation != UINT32_MAX) {
        ++g_snapshot.reset_generation;
    }
    g_snapshot.transaction = g_transaction.snapshot();
    critical_section_exit(&g_lock);
    return status;
}

void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    if (output == nullptr) {
        return;
    }
    critical_section_enter_blocking(&g_lock);
    *output = g_snapshot;
    critical_section_exit(&g_lock);
}
