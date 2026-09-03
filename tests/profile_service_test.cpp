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
    int bank_replacements = 0;
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

bool fake_replace_bank(void* context, uint8_t bank,
                       const uint8_t* payload, size_t payload_size,
                       const uint8_t* header, size_t header_size) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (bank >= PROFILE_STORAGE_BANK_COUNT || payload == nullptr ||
        header == nullptr ||
        payload_size != CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        header_size != PROFILE_STORAGE_RECORD_HEADER_SIZE) {
        return false;
    }
    ++storage->bank_replacements;
    memset(storage->bytes[bank], 0xff, PROFILE_STORAGE_BANK_SIZE);
    memcpy(
        &storage->bytes[bank][PROFILE_STORAGE_RECORD_HEADER_SIZE],
        payload, payload_size);
    memcpy(storage->bytes[bank], header, header_size);
    return memcmp(
               &storage->bytes[bank][
                   PROFILE_STORAGE_RECORD_HEADER_SIZE],
               payload, payload_size) == 0 &&
           memcmp(storage->bytes[bank], header, header_size) == 0;
}

ProfileStorageIo fake_io() {
    return {
        &flash,
        PROFILE_STORAGE_BANK_SIZE,
        PROFILE_STORAGE_SECTOR_SIZE,
        PROFILE_STORAGE_PAGE_SIZE,
        fake_read,
        fake_replace_bank,
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

void test_completed_write_then_dirty_identity_activation() {
    const int replacements_before = flash.bank_replacements;
    const ControllerIdentity global = controller_identity_global();
    constexpr uint8_t kWrittenProfileIndex = 1;
    ControllerProfile customized =
        controller_profile_default(global, kWrittenProfileIndex);
    customized.strong_rumble_scale = 31;
    customized.weak_rumble_scale = 47;
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(customized, encoded, sizeof(encoded)),
            "sequential mutation fixture profile did not encode");

    constexpr uint32_t kWriteTransactionId = 0x31415926;
    require(profile_service_begin(
                kWriteTransactionId, global, kWrittenProfileIndex,
                CONTROLLER_PROFILE_SCHEMA_VERSION, sizeof(encoded),
                profile_storage_crc32(encoded, sizeof(encoded))) ==
                ConfigurationTransactionStatus::kReceiving &&
                profile_service_append(kWriteTransactionId, 0, encoded,
                                       sizeof(encoded)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                profile_service_commit(kWriteTransactionId) ==
                    ConfigurationTransactionStatus::kPending,
            "sequential profile write did not reach pending");
    profile_service_task_on_storage_core(6000);
    const ProfileServiceTransactionSnapshot written =
        transaction_snapshot();
    require(written.transaction.transaction_id == kWriteTransactionId &&
                written.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted &&
                flash.bank_replacements == replacements_before + 1,
            "completed write lost correlation or used multiple bank replacements");

    ControllerIdentity connected{};
    connected.stable = true;
    connected.transport = ControllerTransport::kClassic;
    connected.address[0] = 0x10;
    connected.address[1] = 0x20;
    connected.address[2] = 0x30;
    connected.address[3] = 0x40;
    connected.address[4] = 0x50;
    connected.address[5] = 0x60;
    connected.vendor_id = 0x1234;
    connected.product_id = 0xabcd;
    require(profile_service_observe_identity_on_storage_core(connected),
            "connected identity did not enter the dirty database");

    constexpr uint32_t kActivateTransactionId = 0x27182818;
    constexpr uint8_t kActivatedProfileIndex = 2;
    require(profile_service_activate(
                kActivateTransactionId, connected,
                kActivatedProfileIndex) ==
                ConfigurationTransactionStatus::kPending,
            "activation after completed write did not reach pending");
    const ProfileServiceTransactionSnapshot pending =
        transaction_snapshot();
    require(pending.transaction.transaction_id ==
                    kActivateTransactionId &&
                pending.transaction.status ==
                    ConfigurationTransactionStatus::kPending &&
                pending.transaction.stored_generation == 0 &&
                pending.transaction.stored_crc == 0,
            "pending activation was not correlated to its own transaction");

    profile_service_task_on_storage_core(7000);
    const ProfileServiceTransactionSnapshot activated =
        transaction_snapshot();
    require(activated.transaction.transaction_id ==
                    kActivateTransactionId &&
                activated.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted &&
                activated.transaction.stored_generation ==
                    written.transaction.stored_generation + 1 &&
                flash.bank_replacements == replacements_before + 2,
            "activation did not complete as one next correlated bank replacement");

    const ControllerProfileDatabase recovered = reload_database(
        activated, activated.transaction.stored_generation);
    const ControllerProfileDatabaseEntry* connected_entry =
        controller_profile_database_find(recovered, connected);
    require(connected_entry != nullptr &&
                connected_entry->active_profile ==
                    kActivatedProfileIndex &&
                recovered.fallback_profiles[kWrittenProfileIndex]
                        .strong_rumble_scale ==
                    customized.strong_rumble_scale &&
                recovered.fallback_profiles[kWrittenProfileIndex]
                        .weak_rumble_scale ==
                    customized.weak_rumble_scale,
            "activation did not atomically persist the dirty identity and prior write");
    const ProfileServiceActiveProfileSnapshot active =
        active_profile_snapshot(connected);
    require(active.valid &&
                active.metadata.generation ==
                    activated.transaction.stored_generation &&
                active.profile_index == kActivatedProfileIndex,
            "completed activation did not publish the dirty identity");
}

}  // namespace

ProfileStorageIo pico_profile_storage_io() {
    return fake_io();
}

int main() {
    test_pending_commands_are_not_decoded_as_profile_writes();
    test_host_and_controller_mutations_are_serialized();
    test_completed_write_then_dirty_identity_activation();
    return 0;
}
