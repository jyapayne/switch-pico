#pragma once

#include <stddef.h>
#include <stdint.h>

#include "configuration/adapter_configuration.h"
#include "configuration/configuration_transaction.h"

constexpr uint32_t CONFIGURATION_SERVICE_INTERNAL_TRANSACTION_ID_MASK =
    0x80000000u;

enum class ConfigurationServiceState : uint8_t {
    kLoading = 0,
    kReady = 1,
    kStorageError = 2,
};

struct ConfigurationModeTransactionSnapshot {
    uint32_t transaction_id = 0;
    AdapterRequestedMode requested_mode = AdapterRequestedMode::kAuto;
    uint32_t stored_generation = 0;
    uint64_t accepted_serial = 0;
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kIdle;
    bool internal = false;
};

struct ConfigurationServiceSnapshot {
    ConfigurationServiceState state = ConfigurationServiceState::kLoading;
    AdapterConfiguration configuration{};
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
    uint32_t reset_generation = 0;
    ConfigurationTransactionSnapshot transaction{};
    ConfigurationModeTransactionSnapshot mode_transaction{};
};

void configuration_service_prepare();
void configuration_service_initialize_pre_usb();
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
ConfigurationTransactionStatus configuration_service_set_mode(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability);
ConfigurationTransactionStatus configuration_service_set_mode_internal(
    uint32_t transaction_id, AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability);
// Permanently reserves configuration writes for physical recovery. Active
// host work is canceled; only an internal Auto mode transaction remains legal.
void configuration_service_reserve_for_recovery();
bool configuration_service_mode_transaction_status(
    uint32_t transaction_id, ConfigurationTransactionStatus* output);
bool configuration_service_mode_transaction_reboot_ready(
    uint32_t transaction_id);
void configuration_service_snapshot(ConfigurationServiceSnapshot* output);

// Lock-free publication for the report path. The value changes as soon as a
// configuration reset is accepted.
uint32_t configuration_service_reset_generation();
