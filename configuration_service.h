#pragma once

#include <stddef.h>
#include <stdint.h>

#include "adapter_configuration.h"
#include "configuration_transaction.h"

enum class ConfigurationServiceState : uint8_t {
    kLoading = 0,
    kReady = 1,
    kStorageError = 2,
};

struct ConfigurationServiceSnapshot {
    ConfigurationServiceState state = ConfigurationServiceState::kLoading;
    AdapterConfiguration configuration{};
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
    ConfigurationTransactionSnapshot transaction{};
};

void configuration_service_prepare();
void configuration_service_initialize_on_storage_core();
void configuration_service_task_on_storage_core(uint32_t now_ms);

ConfigurationTransactionStatus configuration_service_begin(
    uint32_t transaction_id, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc);
ConfigurationTransactionStatus configuration_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size);
ConfigurationTransactionStatus configuration_service_commit(
    uint32_t transaction_id);
ConfigurationTransactionStatus configuration_service_reset(
    uint32_t transaction_id);
void configuration_service_snapshot(ConfigurationServiceSnapshot* output);
