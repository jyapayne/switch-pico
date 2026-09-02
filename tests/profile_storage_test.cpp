#include "controller_identity.h"
#include "controller_profile.h"
#include "profile_storage.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

struct FakeFlash {
    uint8_t bytes[PROFILE_STORAGE_BANK_COUNT][PROFILE_STORAGE_BANK_SIZE];
    int successful_programs = 0;
    int fail_after_programs = -1;
    bool corrupt_next_program = false;
    bool fail_reads_after_header_program = false;
    bool header_programmed = false;
};

FakeFlash flash{};
ControllerProfileDatabase database{};
ControllerProfileDatabase recovered_database{};

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
    flash.fail_reads_after_header_program = false;
    flash.header_programmed = false;
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

bool fake_erase_sector(void* context, uint8_t bank, size_t offset) {
    auto* storage = static_cast<FakeFlash*>(context);
    if (bank >= PROFILE_STORAGE_BANK_COUNT ||
        offset % PROFILE_STORAGE_SECTOR_SIZE != 0 ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        PROFILE_STORAGE_SECTOR_SIZE >
            PROFILE_STORAGE_BANK_SIZE - offset) {
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
    if (storage->fail_after_programs >= 0 &&
        storage->successful_programs >= storage->fail_after_programs) {
        return false;
    }
    for (size_t index = 0; index < size; ++index) {
        storage->bytes[bank][offset + index] &= data[index];
    }
    if (storage->corrupt_next_program) {
        storage->bytes[bank][offset] ^= 1;
        storage->corrupt_next_program = false;
    }
    if (offset == 0) {
        storage->header_programmed = true;
    }
    ++storage->successful_programs;
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

void test_two_bank_recovery() {
    erase_all();
    controller_profile_database_default(&database);
    ProfileStorage storage;
    require(storage.initialize(fake_io(), &database) &&
                !storage.snapshot().valid,
            "erased profile storage did not initialize empty");
    require(storage.commit(database) == ProfileStorageResult::kOk &&
                storage.snapshot().generation == 1,
            "first profile database did not commit");
    const int programs_after_first = flash.successful_programs;
    require(storage.commit(database) == ProfileStorageResult::kUnchanged &&
                flash.successful_programs == programs_after_first,
            "unchanged profile database consumed flash writes");

    database.fallback_profiles[0].button_map[0] = 1;
    require(storage.commit(database) == ProfileStorageResult::kOk &&
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
                storage.commit(database) == ProfileStorageResult::kOk,
            "interruption baseline did not commit");
    database.fallback_profiles[1].button_map[2] = 3;
    flash.fail_after_programs = flash.successful_programs + 1;
    require(storage.commit(database) == ProfileStorageResult::kIoError,
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
                storage.commit(database) == ProfileStorageResult::kOk,
            "commit-point baseline did not commit");

    database.fallback_profiles[1].strong_rumble_scale = 17;
    flash.header_programmed = false;
    flash.fail_reads_after_header_program = true;
    require(storage.commit(database) == ProfileStorageResult::kOk &&
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
                storage.commit(database) == ProfileStorageResult::kOk,
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
    require(storage.commit(database) == ProfileStorageResult::kIoError &&
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

}  // namespace
int main() {
    test_two_bank_recovery();
    test_interrupted_commit_retains_previous_bank();
    test_successful_header_program_is_commit_point();
    test_payload_corruption_prevents_header_publication();
    return 0;
}
