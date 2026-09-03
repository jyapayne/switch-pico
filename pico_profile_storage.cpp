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
constexpr uint32_t kFlashSafeExecuteTimeoutMs = 5000;

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

struct FlashBankReplacement {
    uint8_t bank;
    const uint8_t* payload;
    size_t payload_size;
    const uint8_t* header;
    bool replaced;
};

void perform_flash_bank_replacement(void* context) {
    auto* replacement =
        static_cast<FlashBankReplacement*>(context);
    replacement->replaced = false;
    const uint32_t bank_offset =
        kProfileStorageOffset +
        replacement->bank * PROFILE_STORAGE_BANK_SIZE;

    for (size_t offset = 0; offset < PROFILE_STORAGE_BANK_SIZE;
         offset += FLASH_SECTOR_SIZE) {
        flash_range_erase(bank_offset + offset, FLASH_SECTOR_SIZE);
    }

    uint8_t final_page[FLASH_PAGE_SIZE]{};
    for (size_t offset = 0; offset < replacement->payload_size;
         offset += FLASH_PAGE_SIZE) {
        const size_t remaining =
            replacement->payload_size - offset;
        const uint8_t* page = &replacement->payload[offset];
        if (remaining < FLASH_PAGE_SIZE) {
            memcpy(final_page, page, remaining);
            page = final_page;
        }
        flash_range_program(
            bank_offset + PROFILE_STORAGE_RECORD_HEADER_SIZE + offset,
            page, FLASH_PAGE_SIZE);
    }

    const auto* stored_payload = reinterpret_cast<const uint8_t*>(
        XIP_BASE + bank_offset +
        PROFILE_STORAGE_RECORD_HEADER_SIZE);
    if (memcmp(stored_payload, replacement->payload,
               replacement->payload_size) != 0) {
        return;
    }

    flash_range_program(bank_offset, replacement->header,
                        PROFILE_STORAGE_RECORD_HEADER_SIZE);
    const auto* stored_header = reinterpret_cast<const uint8_t*>(
        XIP_BASE + bank_offset);
    replacement->replaced =
        memcmp(stored_header, replacement->header,
               PROFILE_STORAGE_RECORD_HEADER_SIZE) == 0;
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

bool replace_storage_bank(void*, uint8_t bank,
                          const uint8_t* payload,
                          size_t payload_size,
                          const uint8_t* header,
                          size_t header_size) {
    if (bank >= PROFILE_STORAGE_BANK_COUNT || payload == nullptr ||
        header == nullptr ||
        payload_size != CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        header_size != PROFILE_STORAGE_RECORD_HEADER_SIZE ||
        !storage_region_available()) {
        return false;
    }
    FlashBankReplacement replacement{
        bank,
        payload,
        payload_size,
        header,
        false,
    };
    return flash_safe_execute(
               perform_flash_bank_replacement, &replacement,
               kFlashSafeExecuteTimeoutMs) == PICO_OK &&
           replacement.replaced;
}

}  // namespace

ProfileStorageIo pico_profile_storage_io() {
    return {
        nullptr,
        PROFILE_STORAGE_BANK_SIZE,
        FLASH_SECTOR_SIZE,
        FLASH_PAGE_SIZE,
        read_storage,
        replace_storage_bank,
    };
}
