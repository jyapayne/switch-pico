#include "platform/pico/pico_configuration_storage.h"
#include <string.h>

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
constexpr uint32_t kFlashSafeExecuteTimeoutMs = 5000;

static_assert(PICO_FLASH_BANK_STORAGE_OFFSET >= kConfigurationStorageSize,
              "configuration storage offset underflows flash");
static_assert(kConfigurationStorageOffset + kConfigurationStorageSize <=
                  PICO_FLASH_BANK_STORAGE_OFFSET,
              "configuration storage overlaps BTstack bonds");
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET +
                      PICO_FLASH_BANK_TOTAL_SIZE <=
                  PICO_FLASH_SIZE_BYTES,
              "BTstack storage exceeds flash");
static_assert(CONFIGURATION_STORAGE_MAX_RECORD_SIZE <= FLASH_SECTOR_SIZE,
              "configuration record does not fit one flash sector");
static_assert(CONFIGURATION_STORAGE_MAX_PAGE_SIZE == FLASH_PAGE_SIZE,
              "configuration page buffer does not match Pico flash");

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
    return binary_end <= kConfigurationStorageOffset;
}

bool read_storage(void*, uint8_t copy, size_t offset, uint8_t* output,
                  size_t size) {
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT || output == nullptr ||
        offset + size > FLASH_SECTOR_SIZE || !storage_region_available()) {
        return false;
    }
    const uintptr_t address = XIP_BASE + kConfigurationStorageOffset +
                              copy * FLASH_SECTOR_SIZE + offset;
    memcpy(output, reinterpret_cast<const void*>(address), size);
    return true;
}

bool erase_storage(void*, uint8_t copy) {
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT ||
        !storage_region_available()) {
        return false;
    }
    FlashMutation mutation{
        true,
        static_cast<uint32_t>(kConfigurationStorageOffset +
                              copy * FLASH_SECTOR_SIZE),
        nullptr,
    };
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              kFlashSafeExecuteTimeoutMs) == PICO_OK;
}

bool program_storage(void*, uint8_t copy, size_t offset,
                     const uint8_t* data, size_t size) {
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT || data == nullptr ||
        size != FLASH_PAGE_SIZE || offset % FLASH_PAGE_SIZE != 0 ||
        offset + size > FLASH_SECTOR_SIZE || !storage_region_available()) {
        return false;
    }
    FlashMutation mutation{
        false,
        static_cast<uint32_t>(kConfigurationStorageOffset +
                              copy * FLASH_SECTOR_SIZE + offset),
        data,
    };
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              kFlashSafeExecuteTimeoutMs) == PICO_OK;
}

}  // namespace

ConfigurationStorageIo pico_configuration_storage_io() {
    return {
        nullptr,
        FLASH_SECTOR_SIZE,
        FLASH_PAGE_SIZE,
        read_storage,
        erase_storage,
        program_storage,
    };
}
