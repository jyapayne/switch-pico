#include "controller_identity.h"
#include "controller_profile.h"
#include "pico_profile_storage.h"
#include "profile_service.h"
#include "profile_storage.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeFlash {
    uint8_t bytes[PROFILE_STORAGE_BANK_COUNT][PROFILE_STORAGE_BANK_SIZE];
};

FakeFlash flash{};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

bool fake_read(void* context, uint8_t bank, size_t offset,
               uint8_t* output, size_t size) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (bank >= PROFILE_STORAGE_BANK_COUNT || output == nullptr ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        size > PROFILE_STORAGE_BANK_SIZE - offset) {
        return false;
    }
    memcpy(output, &storage->bytes[bank][offset], size);
    return true;
}

bool fake_erase_sector(void* context, uint8_t bank, size_t offset) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (bank >= PROFILE_STORAGE_BANK_COUNT ||
        offset % PROFILE_STORAGE_SECTOR_SIZE != 0 ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        PROFILE_STORAGE_SECTOR_SIZE > PROFILE_STORAGE_BANK_SIZE - offset) {
        return false;
    }
    memset(&storage->bytes[bank][offset], 0xff,
           PROFILE_STORAGE_SECTOR_SIZE);
    return true;
}

bool fake_program(void* context, uint8_t bank, size_t offset,
                  const uint8_t* data, size_t size) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (bank >= PROFILE_STORAGE_BANK_COUNT || data == nullptr ||
        size != PROFILE_STORAGE_PAGE_SIZE ||
        offset % PROFILE_STORAGE_PAGE_SIZE != 0 ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        size > PROFILE_STORAGE_BANK_SIZE - offset) {
        return false;
    }
    for (size_t index = 0; index < size; ++index) {
        storage->bytes[bank][offset + index] &= data[index];
    }
    return true;
}

ProfileStorageIo fake_io() {
    return {
        &flash,
        PROFILE_STORAGE_BANK_SIZE,
        PROFILE_STORAGE_SECTOR_SIZE,
        PROFILE_STORAGE_PAGE_SIZE,
        fake_read,
        fake_erase_sector,
        fake_program,
    };
}

ProfileServiceTransactionSnapshot transaction_snapshot() {
    ProfileServiceTransactionSnapshot snapshot{};
    profile_service_transaction_snapshot(&snapshot);
    return snapshot;
}

ProfileServiceActiveProfileSnapshot active_profile_snapshot(
    const ControllerIdentity& identity) {
    ProfileServiceActiveProfileSnapshot snapshot{};
    profile_service_active_profile_snapshot(identity, &snapshot);
    return snapshot;
}

ControllerProfileDatabase reload_database(
    const ProfileServiceTransactionSnapshot& transaction,
    uint32_t expected_generation) {
    ControllerProfileDatabase recovered{};
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &recovered) &&
                storage.snapshot().valid &&
                storage.snapshot().generation == expected_generation &&
                storage.snapshot().generation ==
                    transaction.transaction.stored_generation &&
                storage.snapshot().payload_crc ==
                    transaction.transaction.stored_crc,
            "terminal transaction status did not identify persisted storage");
    return recovered;
}

void test_pending_commands_are_not_decoded_as_profile_writes() {
    memset(flash.bytes, 0xff, sizeof(flash.bytes));
    profile_service_prepare();
    profile_service_initialize_on_storage_core();
    ProfileServiceActiveProfileSnapshot active =
        active_profile_snapshot(controller_identity_global());
    require(active.valid &&
                active.metadata.state == ProfileServiceState::kReady &&
                active.metadata.generation == 0 &&
                profile_service_database_generation() == 0 &&
                active.profile_index == 0,
            "initial active profile snapshot was not coherent");

    const ControllerIdentity identity = controller_identity_global();
    constexpr uint8_t kProfileIndex = 2;
    ControllerProfile customized =
        controller_profile_default(identity, kProfileIndex);
    customized.strong_rumble_scale = 17;
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(customized, encoded, sizeof(encoded)),
            "customized profile did not encode");

    constexpr uint32_t kWriteTransactionId = 0x10203040;
    require(profile_service_begin(
                kWriteTransactionId, identity, kProfileIndex,
                CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
                profile_storage_crc32(encoded, sizeof(encoded))) ==
                ConfigurationTransactionStatus::kReceiving &&
                profile_service_append(kWriteTransactionId, 0, encoded,
                                       sizeof(encoded)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                profile_service_commit(kWriteTransactionId) ==
                    ConfigurationTransactionStatus::kPending,
            "profile write did not reach pending");
    profile_service_task_on_storage_core(0);
    require(transaction_snapshot().transaction.status ==
                ConfigurationTransactionStatus::kCommitted,
            "profile write baseline did not commit");
    active = active_profile_snapshot(identity);
    require(active.valid && active.metadata.generation == 1 &&
                profile_service_database_generation() == 1 &&
                active.profile_index == 0,
            "profile write did not publish one coherent generation");

    constexpr uint32_t kResetTransactionId = 0xa5a55a5a;
    require(profile_service_reset(kResetTransactionId, identity,
                                  kProfileIndex) ==
                ConfigurationTransactionStatus::kPending,
            "profile reset did not reach pending");
    ProfileServiceTransactionSnapshot reset = transaction_snapshot();
    require(reset.transaction.transaction_id == kResetTransactionId &&
                reset.transaction.status ==
                    ConfigurationTransactionStatus::kPending,
            "pending reset lost its transaction identity");

    profile_service_task_on_storage_core(1000);
    reset = transaction_snapshot();
    require(reset.transaction.transaction_id == kResetTransactionId &&
                reset.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted,
            "one reset tick decoded profile payload or published a malformed result");
    ControllerProfileDatabase recovered = reload_database(reset, 2);
    require(recovered.fallback_profiles[kProfileIndex].strong_rumble_scale ==
                UINT8_MAX,
            "terminal reset status was published before reset persisted");
    active = active_profile_snapshot(identity);
    require(active.valid && active.metadata.generation == 2 &&
                profile_service_database_generation() == 2 &&
                active.profile_index == 0,
            "profile reset did not refresh the active snapshot generation");

    constexpr uint32_t kActivateTransactionId = 0x50607080;
    constexpr uint8_t kActivatedProfile = 3;
    require(profile_service_activate(kActivateTransactionId, identity,
                                     kActivatedProfile) ==
                ConfigurationTransactionStatus::kPending,
            "profile activation did not reach pending");
    ProfileServiceTransactionSnapshot activate = transaction_snapshot();
    require(activate.transaction.transaction_id == kActivateTransactionId &&
                activate.transaction.status ==
                    ConfigurationTransactionStatus::kPending,
            "pending activation lost its transaction identity");
    active = active_profile_snapshot(identity);
    require(active.valid && active.metadata.generation == 2 &&
                active.profile_index == 0,
            "pending activation leaked an uncommitted active profile");

    profile_service_task_on_storage_core(2000);
    activate = transaction_snapshot();
    require(activate.transaction.transaction_id == kActivateTransactionId &&
                activate.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted,
            "one activation tick decoded profile payload or published a malformed result");
    recovered = reload_database(activate, 3);
    require(recovered.fallback_active_profile == kActivatedProfile,
            "terminal activation status was published before activation persisted");
    active = active_profile_snapshot(identity);
    require(active.valid && active.metadata.generation == 3 &&
                profile_service_database_generation() == 3 &&
                active.profile_index == kActivatedProfile &&
                active.profile.strong_rumble_scale ==
                    recovered.fallback_profiles[kActivatedProfile]
                        .strong_rumble_scale,
            "activation did not publish profile, index, and generation together");
}

void test_host_and_controller_mutations_are_serialized() {
    const ControllerIdentity identity = controller_identity_global();
    ControllerProfile profile =
        controller_profile_default(identity, 1);
    profile.weak_rumble_scale = 23;
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)),
            "serialization fixture profile did not encode");

    constexpr uint32_t kHostTransactionId = 0x11223344;
    constexpr uint32_t kInternalTransactionId = 0x80000019;
    require(profile_service_begin(
                kHostTransactionId, identity, 1,
                CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
                profile_storage_crc32(encoded, sizeof(encoded))) ==
                ConfigurationTransactionStatus::kReceiving,
            "host write did not acquire the profile mutation boundary");
    require(profile_service_activate_internal(
                kInternalTransactionId, identity, 2) ==
                ConfigurationTransactionStatus::kBusy,
            "controller activation raced a receiving host write");
    ProfileServiceTransactionSnapshot snapshot =
        transaction_snapshot();
    require(snapshot.transaction.transaction_id ==
                    kHostTransactionId &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kReceiving,
            "busy controller activation replaced the host transaction");

    require(profile_service_append(
                kHostTransactionId, 0, encoded, sizeof(encoded)) ==
                ConfigurationTransactionStatus::kReceiving &&
                profile_service_commit(kHostTransactionId) ==
                    ConfigurationTransactionStatus::kPending,
            "host write did not reach pending after controller contention");
    profile_service_task_on_storage_core(3000);
    require(transaction_snapshot().transaction.status ==
                ConfigurationTransactionStatus::kCommitted,
            "serialized host write did not commit");

    constexpr uint32_t kHostActivationTransactionId = 0x22334455;
    require(profile_service_activate(
                kHostActivationTransactionId, identity, 0) ==
                ConfigurationTransactionStatus::kPending,
            "host activation did not acquire the released boundary");
    require(profile_service_activate_internal(
                kInternalTransactionId, identity, 2) ==
                ConfigurationTransactionStatus::kBusy,
            "controller activation raced a pending host activation");
    snapshot = transaction_snapshot();
    require(snapshot.transaction.transaction_id ==
                    kHostActivationTransactionId &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kPending &&
                active_profile_snapshot(identity).profile_index == 3,
            "pending host activation was replaced or leaked before commit");
    profile_service_task_on_storage_core(4000);
    require(active_profile_snapshot(identity).profile_index == 0,
            "serialized host activation did not commit");
    require(profile_service_activate_internal(
                0x19, identity, 2) ==
                ConfigurationTransactionStatus::kMalformed,
            "internal activation admitted a transaction without the high bit");

    require(profile_service_activate_internal(
                kInternalTransactionId, identity, 2) ==
                ConfigurationTransactionStatus::kPending,
            "controller activation did not acquire the released boundary");
    require(profile_service_begin(
                0x55667788, identity, 0,
                CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
                profile_storage_crc32(encoded, sizeof(encoded))) ==
                ConfigurationTransactionStatus::kBusy,
            "host write raced a pending controller activation");
    snapshot = transaction_snapshot();
    require(snapshot.transaction.transaction_id ==
                    kHostActivationTransactionId &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted &&
                active_profile_snapshot(identity).profile_index == 0,
            "pending controller activation replaced host-visible status or leaked before commit");

    profile_service_task_on_storage_core(5000);
    const ProfileServiceActiveProfileSnapshot active =
        active_profile_snapshot(identity);
    snapshot = transaction_snapshot();
    require(active.valid && active.profile_index == 2 &&
                snapshot.transaction.transaction_id ==
                    kHostActivationTransactionId &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted,
            "controller activation did not publish while preserving host-visible status");
}

}  // namespace

ProfileStorageIo pico_profile_storage_io() {
    return fake_io();
}

int main() {
    test_pending_commands_are_not_decoded_as_profile_writes();
    test_host_and_controller_mutations_are_serialized();
    return 0;
}
