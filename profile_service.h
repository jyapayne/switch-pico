#pragma once

#include <stddef.h>
#include <stdint.h>

#include "configuration_transaction.h"
#include "controller_profile.h"

constexpr uint8_t PROFILE_SERVICE_LIST_CAPACITY =
    CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY + 1;

enum class ProfileServiceState : uint8_t {
    kLoading = 0,
    kReady = 1,
    kStorageError = 2,
};

struct ProfileServiceMetadata {
    ProfileServiceState state = ProfileServiceState::kLoading;
    uint32_t generation = 0;
    uint32_t payload_crc = 0;
};

struct ProfileServiceListRow {
    ControllerIdentity identity{};
    uint8_t active_profile = 0;
};

struct ProfileServiceListSnapshot {
    ProfileServiceMetadata metadata{};
    uint8_t count = 0;
    ProfileServiceListRow rows[PROFILE_SERVICE_LIST_CAPACITY]{};
};

struct ProfileServiceSelectedSnapshot {
    ProfileServiceMetadata metadata{};
    bool valid = false;
    ConfigurationTransactionStatus status =
        ConfigurationTransactionStatus::kIdle;
    ControllerIdentity identity{};
    uint8_t profile_index = 0;
    ControllerProfile profile{};
};

struct ProfileServiceTransactionSnapshot {
    ProfileServiceMetadata metadata{};
    ControllerIdentity identity{};
    uint8_t profile_index = 0;
    ConfigurationTransactionSnapshot transaction{};
};

void profile_service_prepare();
void profile_service_initialize_on_storage_core();
void profile_service_task_on_storage_core(uint32_t now_ms);
bool profile_service_observe_identity_on_storage_core(
    const ControllerIdentity& identity);

ConfigurationTransactionStatus profile_service_select(
    const ControllerIdentity& identity, uint8_t profile_index);
ConfigurationTransactionStatus profile_service_begin(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index, uint16_t schema_version, size_t payload_size,
    uint32_t payload_crc);
ConfigurationTransactionStatus profile_service_append(
    uint32_t transaction_id, size_t offset, const uint8_t* data,
    size_t size);
ConfigurationTransactionStatus profile_service_commit(
    uint32_t transaction_id);
ConfigurationTransactionStatus profile_service_reset(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index);
ConfigurationTransactionStatus profile_service_activate(
    uint32_t transaction_id, const ControllerIdentity& identity,
    uint8_t profile_index);

void profile_service_list_snapshot(ProfileServiceListSnapshot* output);
void profile_service_selected_snapshot(
    ProfileServiceSelectedSnapshot* output);
void profile_service_transaction_snapshot(
    ProfileServiceTransactionSnapshot* output);
bool profile_service_active_profile(const ControllerIdentity& identity,
                                    ControllerProfile* output,
                                    uint8_t* profile_index);
