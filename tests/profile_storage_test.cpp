#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "profile/profile_storage.h"
#include "controller_profile_legacy_fixtures.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeFlash {
    uint8_t bytes[PROFILE_STORAGE_BANK_COUNT][PROFILE_STORAGE_BANK_SIZE];
    int successful_programs = 0;
    int fail_after_programs = -1;
    bool corrupt_next_program = false;
    int corrupt_header_padding_offset = -1;
    bool fail_reads_after_header_program = false;
    bool header_programmed = false;
    int erase_count = 0;
    int bank_replacements = 0;
};

FakeFlash flash{};
ControllerProfileDatabase database{};
ControllerProfileDatabase recovered_database{};
uint8_t encoded_database[CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE]{};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void erase_all() {
    memset(flash.bytes, 0xff, sizeof(flash.bytes));
    flash.successful_programs = 0;
    flash.fail_after_programs = -1;
    flash.corrupt_next_program = false;
    flash.corrupt_header_padding_offset = -1;
    flash.fail_reads_after_header_program = false;
    flash.header_programmed = false;
    flash.erase_count = 0;
    flash.bank_replacements = 0;
}

bool fake_read(void* context, uint8_t bank, size_t offset,
               uint8_t* output, size_t size) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (storage->fail_reads_after_header_program &&
        storage->header_programmed) {
        return false;
    }
    if (bank >= PROFILE_STORAGE_BANK_COUNT ||
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
    storage->erase_count +=
        static_cast<int>(PROFILE_STORAGE_SECTORS_PER_BANK);

    uint8_t final_page[PROFILE_STORAGE_PAGE_SIZE]{};
    for (size_t offset = 0; offset < payload_size;
         offset += PROFILE_STORAGE_PAGE_SIZE) {
        if (storage->fail_after_programs >= 0 &&
            storage->successful_programs >=
                storage->fail_after_programs) {
            return false;
        }
        const size_t remaining = payload_size - offset;
        const uint8_t* page = &payload[offset];
        if (remaining < PROFILE_STORAGE_PAGE_SIZE) {
            memcpy(final_page, page, remaining);
            page = final_page;
        }
        memcpy(
            &storage->bytes[bank][
                PROFILE_STORAGE_RECORD_HEADER_SIZE + offset],
            page, PROFILE_STORAGE_PAGE_SIZE);
        if (storage->corrupt_next_program) {
            storage->bytes[bank][
                PROFILE_STORAGE_RECORD_HEADER_SIZE + offset] ^= 1;
            storage->corrupt_next_program = false;
        }
        ++storage->successful_programs;
    }
    if (memcmp(
            &storage->bytes[bank][PROFILE_STORAGE_RECORD_HEADER_SIZE],
            payload, payload_size) != 0) {
        return false;
    }
    if (storage->fail_after_programs >= 0 &&
        storage->successful_programs >= storage->fail_after_programs) {
        return false;
    }

    memcpy(storage->bytes[bank], header, header_size);
    storage->header_programmed = true;
    ++storage->successful_programs;
    if (storage->corrupt_header_padding_offset >= 24 &&
        static_cast<size_t>(
            storage->corrupt_header_padding_offset) < header_size) {
        storage->bytes[bank][
            static_cast<size_t>(
                storage->corrupt_header_padding_offset)] ^= 1;
    }
    return memcmp(storage->bytes[bank], header, header_size) == 0;
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

uint16_t fixture_read_u16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
           static_cast<uint16_t>(input[1] << 8);
}

void fixture_write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void fixture_write_u32(uint8_t* output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

void install_legacy_database_bank_fixture() {
    erase_all();
    constexpr uint8_t kBank = 1;
    constexpr uint32_t kGeneration = 41;
    constexpr size_t kFallbackOffset =
        CONTROLLER_PROFILE_DATABASE_HEADER_SIZE;
    constexpr size_t kEntryOffset =
        kFallbackOffset +
        CONTROLLER_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
    constexpr uint8_t kEntryHeader[CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE] = {
        1, 1, 7, 0, 1, 2, 3, 4, 5, 6, 0x7e, 0x05, 0x09, 0x20, 3, 1,
    };

    uint8_t* const record = flash.bytes[kBank];
    uint8_t* const payload = record + PROFILE_STORAGE_RECORD_HEADER_SIZE;
    memset(record, 0, PROFILE_STORAGE_RECORD_HEADER_SIZE);
    memset(payload, 0, CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE);

    memcpy(payload, "SPDB", 4);
    fixture_write_u16(&payload[4],
                      CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION);
    fixture_write_u16(
        &payload[6],
        static_cast<uint16_t>(CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE));
    payload[8] = CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
    payload[9] = CONTROLLER_PROFILE_COUNT;
    payload[10] = 2;
    payload[11] = 1;
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const uint8_t* fixture =
            profile_index == 0
                ? kLegacyNarrowRawRangeProfile
                : profile_index == 1 ? kLegacyCustomThresholdProfile
                                     : kLegacyDefaultProfile;
        memcpy(&payload[kFallbackOffset +
                        profile_index * CONTROLLER_PROFILE_ENCODED_SIZE],
               fixture, CONTROLLER_PROFILE_ENCODED_SIZE);
    }

    memcpy(&payload[kEntryOffset], kEntryHeader, sizeof(kEntryHeader));
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const uint8_t* fixture =
            profile_index == 0
                ? kLegacyNarrowRawRangeProfile
                : profile_index == 3 ? kLegacyCustomThresholdProfile
                                     : kLegacyDefaultProfile;
        memcpy(&payload[kEntryOffset +
                        CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
                        profile_index * CONTROLLER_PROFILE_ENCODED_SIZE],
               fixture, CONTROLLER_PROFILE_ENCODED_SIZE);
    }

    const uint32_t payload_crc = profile_storage_crc32(
        payload, CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE);
    memcpy(record, "SPPF", 4);
    fixture_write_u16(&record[4], 1);
    fixture_write_u16(&record[6],
                      CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION);
    fixture_write_u32(&record[8], kGeneration);
    fixture_write_u32(
        &record[12],
        static_cast<uint32_t>(CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE));
    fixture_write_u32(&record[16], payload_crc);
    fixture_write_u32(&record[20], profile_storage_crc32(record, 20));
}

void test_initialize_requires_batch_replacement() {
    erase_all();
    ProfileStorageIo io = fake_io();
    io.replace_bank = nullptr;
    ProfileStorage storage;
    require(!storage.initialize(io, &database),
            "profile storage initialized without bank replacement");
}

void test_two_bank_recovery() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                !storage.snapshot().valid,
            "erased profile storage did not initialize empty");
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk &&
                storage.snapshot().generation == 1,
            "first profile database did not commit");
    const int programs_after_first = flash.successful_programs;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kUnchanged &&
                flash.successful_programs == programs_after_first,
            "unchanged profile database consumed flash writes");

    database.fallback_profiles[0].button_map[0] = 1;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk &&
                storage.snapshot().generation == 2,
            "second profile database generation did not commit");
    ProfileStorage reloaded;
    require(reloaded.initialize(fake_io(), &recovered_database) &&
                reloaded.snapshot().generation == 2 &&
                recovered_database.fallback_profiles[0].button_map[0] == 1,
            "latest profile database did not survive reload");

    const uint8_t newest_bank = reloaded.snapshot().active_bank;
    flash.bytes[newest_bank][PROFILE_STORAGE_RECORD_HEADER_SIZE + 4] ^= 1;
    ProfileStorage after_corruption;
    require(after_corruption.initialize(fake_io(), &recovered_database) &&
                after_corruption.snapshot().generation == 1 &&
                recovered_database.fallback_profiles[0].button_map[0] == 0,
            "corrupt newest profile bank did not roll back");
}

void test_interrupted_commit_retains_previous_bank() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk,
            "interruption baseline did not commit");
    database.fallback_profiles[1].button_map[2] = 3;
    flash.fail_after_programs = flash.successful_programs + 1;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                ProfileStorageResult::kIoError,
            "interrupted profile write reported success");

    flash.fail_after_programs = -1;
    ProfileStorage recovered;
    require(recovered.initialize(fake_io(), &recovered_database) &&
                recovered.snapshot().generation == 1 &&
                recovered_database.fallback_profiles[1].button_map[2] == 2,
            "interrupted profile write replaced previous bank");
}

void test_successful_header_program_is_commit_point() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk,
            "commit-point baseline did not commit");

    database.fallback_profiles[1].strong_rumble_scale = 17;
    flash.header_programmed = false;
    flash.fail_reads_after_header_program = true;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk &&
                storage.snapshot().generation == 2,
            "successful header program was rolled back by a later read");

    flash.fail_reads_after_header_program = false;
    ProfileStorage recovered;
    require(recovered.initialize(fake_io(), &recovered_database) &&
                recovered.snapshot().generation == 2 &&
                recovered_database.fallback_profiles[1]
                        .strong_rumble_scale == 17,
            "committed header did not recover after transient read failure");
}

void test_payload_corruption_prevents_header_publication() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk,
            "corruption baseline did not commit");
    const ProfileStorageSnapshot previous = storage.snapshot();
    const uint8_t target_bank = previous.active_bank ^ 1u;
    const int programs_before_corruption = flash.successful_programs;
    constexpr int kPayloadProgramCount =
        (CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE +
         PROFILE_STORAGE_PAGE_SIZE - 1) /
        PROFILE_STORAGE_PAGE_SIZE;

    database.fallback_profiles[1].button_map[2] = 3;
    flash.corrupt_next_program = true;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kIoError &&
                flash.successful_programs ==
                    programs_before_corruption + kPayloadProgramCount,
            "corrupt payload programming reached the header program");
    for (size_t index = 0; index < PROFILE_STORAGE_RECORD_HEADER_SIZE;
         ++index) {
        require(flash.bytes[target_bank][index] == 0xff,
                "rejected corrupt payload published a discoverable header");
    }
    require(storage.snapshot().valid == previous.valid &&
                storage.snapshot().generation == previous.generation &&
                storage.snapshot().payload_crc == previous.payload_crc &&
                storage.snapshot().active_bank == previous.active_bank,
            "rejected corrupt programming changed the storage snapshot");

    ProfileStorage recovered;
    require(recovered.initialize(fake_io(), &recovered_database) &&
                recovered.snapshot().generation == previous.generation &&
                recovered.snapshot().active_bank == previous.active_bank &&
                recovered_database.fallback_profiles[1].button_map[2] == 2,
            "headerless corrupt payload was recovered");
}

void test_batched_bank_replacement_is_one_atomic_operation() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    constexpr int kPayloadProgramCount =
        (CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE +
         PROFILE_STORAGE_PAGE_SIZE - 1) /
        PROFILE_STORAGE_PAGE_SIZE;
    require(storage.initialize(fake_io(), &database) &&
                storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk &&
                flash.bank_replacements == 1 &&
                flash.erase_count == static_cast<int>(
                    PROFILE_STORAGE_SECTORS_PER_BANK) &&
                flash.successful_programs ==
                    kPayloadProgramCount + 1,
            "batched commit did not replace one bank in one operation");

    const ProfileStorageSnapshot previous = storage.snapshot();
    const uint8_t target_bank = previous.active_bank ^ 1u;
    const int programs_before_corruption = flash.successful_programs;
    database.fallback_profiles[1].button_map[2] = 3;
    flash.corrupt_next_program = true;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kIoError &&
                flash.bank_replacements == 2 &&
                flash.successful_programs ==
                    programs_before_corruption +
                        kPayloadProgramCount,
            "corrupt batched payload reached header publication");
    for (size_t index = 0; index < PROFILE_STORAGE_RECORD_HEADER_SIZE;
         ++index) {
        require(flash.bytes[target_bank][index] == 0xff,
                "failed batched replacement published a header");
    }
    require(storage.snapshot().generation == previous.generation &&
                storage.snapshot().payload_crc ==
                    previous.payload_crc &&
                storage.snapshot().active_bank ==
                    previous.active_bank,
            "failed batched replacement changed the committed snapshot");

    ProfileStorage recovered;
    require(recovered.initialize(fake_io(), &recovered_database) &&
                recovered.snapshot().generation ==
                    previous.generation &&
                recovered.snapshot().active_bank ==
                    previous.active_bank &&
                recovered_database.fallback_profiles[1]
                        .button_map[2] == 2,
            "headerless batched payload replaced the prior bank");
}

void test_header_padding_corruption_fails_commit_and_recovery() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk,
            "header padding baseline did not commit");

    const ProfileStorageSnapshot previous = storage.snapshot();
    const uint8_t previous_scale =
        database.fallback_profiles[1].strong_rumble_scale;
    database.fallback_profiles[1].strong_rumble_scale = 17;
    for (size_t offset = 24;
         offset < PROFILE_STORAGE_RECORD_HEADER_SIZE; ++offset) {
        flash.corrupt_header_padding_offset =
            static_cast<int>(offset);
        require(storage.commit(database, encoded_database,
                               sizeof(encoded_database)) ==
                    ProfileStorageResult::kIoError,
                "corrupt header padding did not fail the commit");

        ProfileStorage recovered;
        require(recovered.initialize(fake_io(), &recovered_database) &&
                    recovered.snapshot().generation ==
                        previous.generation &&
                    recovered.snapshot().active_bank ==
                        previous.active_bank &&
                    recovered_database.fallback_profiles[1]
                            .strong_rumble_scale == previous_scale,
                "corrupt header padding was accepted on recovery");
    }

    flash.corrupt_header_padding_offset = -1;
    require(storage.commit(database, encoded_database,
                           sizeof(encoded_database)) ==
                ProfileStorageResult::kOk,
            "valid full header page did not commit");
    for (size_t offset = 24;
         offset < PROFILE_STORAGE_RECORD_HEADER_SIZE; ++offset) {
        require(
            flash.bytes[storage.snapshot().active_bank][offset] == 0,
            "valid committed header contained nonzero padding");
    }
    ProfileStorage recovered;
    require(recovered.initialize(fake_io(), &recovered_database) &&
                recovered.snapshot().generation ==
                    previous.generation + 1u &&
                recovered_database.fallback_profiles[1]
                        .strong_rumble_scale == 17,
            "valid full header page was rejected on recovery");
}

void test_legacy_database_bank_migration() {
    install_legacy_database_bank_fixture();
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &recovered_database) &&
                storage.snapshot().valid &&
                storage.snapshot().active_bank == 1 &&
                storage.snapshot().generation == 41,
            "legacy v1 database bank was not selected");
    require(flash.erase_count == 0,
            "legacy bank admission erased flash");
    require(recovered_database.fallback_active_profile == 2,
            "legacy fallback active profile was not preserved");

    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const uint16_t expected_left =
            profile_index == 1
                ? 0x1234
                : CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        const uint16_t expected_right =
            profile_index == 1
                ? 0xabcd
                : CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        require(recovered_database.fallback_profiles[profile_index]
                        .triggers[0]
                        .digital_threshold == expected_left &&
                    recovered_database.fallback_profiles[profile_index]
                            .triggers[1]
                            .digital_threshold == expected_right,
                "legacy fallback thresholds were not selectively migrated");
    }
    require(recovered_database.fallback_profiles[0]
                        .triggers[0]
                        .lower_deadzone == 30000 &&
                recovered_database.fallback_profiles[0]
                        .triggers[0]
                        .upper_saturation == 40000 &&
                recovered_database.fallback_profiles[0]
                        .triggers[1]
                        .lower_deadzone == 30000 &&
                recovered_database.fallback_profiles[0]
                        .triggers[1]
                        .upper_saturation == 40000,
            "legacy fallback raw trigger ranges were not preserved");

    const ControllerProfileDatabaseEntry& entry =
        recovered_database.entries[0];
    require(entry.used && entry.active_profile == 3 &&
                entry.identity.stable &&
                entry.identity.transport == ControllerTransport::kClassic &&
                entry.identity.address_type == 7 &&
                entry.identity.address[0] == 1 &&
                entry.identity.address[5] == 6 &&
                entry.identity.vendor_id == 0x057e &&
                entry.identity.product_id == 0x2009,
            "legacy entry identity or active profile was not preserved");
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const uint16_t expected_left =
            profile_index == 3
                ? 0x1234
                : CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        const uint16_t expected_right =
            profile_index == 3
                ? 0xabcd
                : CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        require(entry.profiles[profile_index]
                            .triggers[0]
                            .digital_threshold == expected_left &&
                    entry.profiles[profile_index]
                            .triggers[1]
                            .digital_threshold == expected_right,
                "legacy entry thresholds were not selectively migrated");
    }
    require(entry.profiles[0].triggers[0].lower_deadzone == 30000 &&
                entry.profiles[0].triggers[0].upper_saturation == 40000 &&
                entry.profiles[0].triggers[1].lower_deadzone == 30000 &&
                entry.profiles[0].triggers[1].upper_saturation == 40000,
            "legacy entry raw trigger ranges were not preserved");

    recovered_database.fallback_profiles[2].weak_rumble_scale = 17;
    require(storage.commit(recovered_database, encoded_database,
                           sizeof(encoded_database)) ==
                    ProfileStorageResult::kOk &&
                storage.snapshot().active_bank == 0 &&
                storage.snapshot().generation == 42,
            "mutation after legacy admission did not commit");
    const uint8_t* const current_record = flash.bytes[0];
    const uint8_t* const current_payload =
        current_record + PROFILE_STORAGE_RECORD_HEADER_SIZE;
    require(fixture_read_u16(&current_record[6]) ==
                    CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION &&
                fixture_read_u16(&current_payload[4]) ==
                    CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION,
            "post-migration commit did not emit v2 storage schemas");
    constexpr size_t kFallbackOffset =
        CONTROLLER_PROFILE_DATABASE_HEADER_SIZE;
    constexpr size_t kEntryOffset =
        kFallbackOffset +
        CONTROLLER_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        require(fixture_read_u16(
                    &current_payload[kFallbackOffset +
                                     profile_index *
                                         CONTROLLER_PROFILE_ENCODED_SIZE]) ==
                    CONTROLLER_PROFILE_SCHEMA_VERSION &&
                    fixture_read_u16(
                        &current_payload[
                            kEntryOffset +
                            CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
                            profile_index *
                                CONTROLLER_PROFILE_ENCODED_SIZE]) ==
                        CONTROLLER_PROFILE_SCHEMA_VERSION,
                "post-migration commit retained a v1 profile");
    }
    require(fixture_read_u16(&flash.bytes[1][6]) ==
                    CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION &&
                fixture_read_u16(
                    &flash.bytes[1][PROFILE_STORAGE_RECORD_HEADER_SIZE + 4]) ==
                    CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION,
            "post-migration commit erased or rewrote the admitted legacy bank");

    ProfileStorage reloaded;
    require(reloaded.initialize(fake_io(), &database) &&
                reloaded.snapshot().generation == 42 &&
                database.fallback_profiles[2].weak_rumble_scale == 17 &&
                database.fallback_profiles[0]
                        .triggers[0]
                        .lower_deadzone == 30000 &&
                database.fallback_profiles[0]
                        .triggers[0]
                        .upper_saturation == 40000 &&
                database.fallback_profiles[0]
                        .triggers[0]
                        .digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                database.fallback_profiles[1]
                        .triggers[0]
                        .digital_threshold == 0x1234 &&
                database.fallback_profiles[1]
                        .triggers[1]
                        .digital_threshold == 0xabcd &&
                database.entries[0].used &&
                database.entries[0].active_profile == 3 &&
                database.entries[0]
                        .profiles[0]
                        .triggers[0]
                        .lower_deadzone == 30000 &&
                database.entries[0]
                        .profiles[0]
                        .triggers[0]
                        .upper_saturation == 40000 &&
                database.entries[0]
                        .profiles[0]
                        .triggers[0]
                        .digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                database.entries[0]
                        .profiles[3]
                        .triggers[0]
                        .digital_threshold == 0x1234 &&
                database.entries[0]
                        .profiles[3]
                        .triggers[1]
                        .digital_threshold == 0xabcd,
            "v2 migration commit did not reload without data loss");
}

}  // namespace
int main() {
    test_two_bank_recovery();
    test_initialize_requires_batch_replacement();
    test_interrupted_commit_retains_previous_bank();
    test_successful_header_program_is_commit_point();
    test_payload_corruption_prevents_header_publication();
    test_batched_bank_replacement_is_one_atomic_operation();
    test_header_padding_corruption_fails_commit_and_recovery();
    test_legacy_database_bank_migration();
    return 0;
}
