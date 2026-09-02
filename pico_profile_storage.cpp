#include "pico_profile_storage.h"

#include <string.h>

#include "configuration_storage.h"
#include "hardware/flash.h"
#include "pico/btstack_flash_bank.h"
#include "pico/flash.h"
#include "pico/platform.h"

extern "C" char __flash_binary_end;

namespace {

constexpr size_t kConfigurationStorageSize =
    CONFIGURATION_STORAGE_COPY_COUNT * FLASH_SECTOR_SIZE;
constexpr uint32_t kConfigurationStorageOffset =
    PICO_FLASH_BANK_STORAGE_OFFSET - kConfigurationStorageSize;
constexpr uint32_t kProfileStorageOffset =
    kConfigurationStorageOffset - PROFILE_STORAGE_TOTAL_SIZE;

static_assert(FLASH_SECTOR_SIZE == PROFILE_STORAGE_SECTOR_SIZE,
              "profile storage sector size does not match Pico flash");
static_assert(FLASH_PAGE_SIZE == PROFILE_STORAGE_PAGE_SIZE,
              "profile storage page size does not match Pico flash");
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET >=
                  kConfigurationStorageSize + PROFILE_STORAGE_TOTAL_SIZE,
              "profile storage offset underflows flash");
static_assert(kProfileStorageOffset + PROFILE_STORAGE_TOTAL_SIZE <=
                  kConfigurationStorageOffset,
              "profile storage overlaps adapter configuration storage");
static_assert(kConfigurationStorageOffset + kConfigurationStorageSize <=
                  PICO_FLASH_BANK_STORAGE_OFFSET,
              "adapter configuration storage overlaps BTstack bonds");
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET +
                      PICO_FLASH_BANK_TOTAL_SIZE <=
                  PICO_FLASH_SIZE_BYTES,
              "BTstack storage exceeds flash");

struct FlashMutation {
    bool erase;
    uint32_t offset;
    const uint8_t* data;
};

void perform_flash_mutation(void* context) {
    const auto* mutation = static_cast<const FlashMutation*>(context);
    if (mutation->erase) {
        flash_range_erase(mutation->offset, FLASH_SECTOR_SIZE);
    } else {
        flash_range_program(mutation->offset, mutation->data,
                            FLASH_PAGE_SIZE);
    }
}

bool storage_region_available() {
    const uintptr_t binary_end =
        reinterpret_cast<uintptr_t>(&__flash_binary_end) - XIP_BASE;
    return binary_end <= kProfileStorageOffset;
}

bool read_storage(void*, uint8_t bank, size_t offset, uint8_t* output,
                  size_t size) {
    if (bank >= PROFILE_STORAGE_BANK_COUNT || output == nullptr ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        size > PROFILE_STORAGE_BANK_SIZE - offset ||
        !storage_region_available()) {
        return false;
    }
    const uintptr_t address =
        XIP_BASE + kProfileStorageOffset +
        bank * PROFILE_STORAGE_BANK_SIZE + offset;
    memcpy(output, reinterpret_cast<const void*>(address), size);
    return true;
}

bool erase_storage_sector(void*, uint8_t bank, size_t offset) {
    if (bank >= PROFILE_STORAGE_BANK_COUNT ||
        offset % FLASH_SECTOR_SIZE != 0 ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        FLASH_SECTOR_SIZE > PROFILE_STORAGE_BANK_SIZE - offset ||
        !storage_region_available()) {
        return false;
    }
    FlashMutation mutation{
        true,
        static_cast<uint32_t>(
            kProfileStorageOffset + bank * PROFILE_STORAGE_BANK_SIZE + offset),
        nullptr,
    };
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              UINT32_MAX) == PICO_OK;
}

bool program_storage(void*, uint8_t bank, size_t offset,
                     const uint8_t* data, size_t size) {
    if (bank >= PROFILE_STORAGE_BANK_COUNT || data == nullptr ||
        size != FLASH_PAGE_SIZE || offset % FLASH_PAGE_SIZE != 0 ||
        offset > PROFILE_STORAGE_BANK_SIZE ||
        size > PROFILE_STORAGE_BANK_SIZE - offset ||
        !storage_region_available()) {
        return false;
    }
    FlashMutation mutation{
        false,
        static_cast<uint32_t>(
            kProfileStorageOffset + bank * PROFILE_STORAGE_BANK_SIZE + offset),
        data,
    };
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              UINT32_MAX) == PICO_OK;
}

}  // namespace

ProfileStorageIo pico_profile_storage_io() {
    return {
        nullptr,
        PROFILE_STORAGE_BANK_SIZE,
        FLASH_SECTOR_SIZE,
        FLASH_PAGE_SIZE,
        read_storage,
        erase_storage_sector,
        program_storage,
    };
}
