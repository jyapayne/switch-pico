#include "platform/pico/pico_profile_storage.h"

#include <string.h>

#include "configuration/configuration_storage.h"
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

static_assert(FLASH_SECTOR_SIZE == PROFILE_STORAGE_SECTOR_SIZE);
static_assert(FLASH_PAGE_SIZE == PROFILE_STORAGE_PAGE_SIZE);
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET >=
              kConfigurationStorageSize + PROFILE_STORAGE_TOTAL_SIZE);
static_assert(kProfileStorageOffset + PROFILE_STORAGE_TOTAL_SIZE <=
              kConfigurationStorageOffset);
static_assert(kConfigurationStorageOffset + kConfigurationStorageSize <=
              PICO_FLASH_BANK_STORAGE_OFFSET);
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET + PICO_FLASH_BANK_TOTAL_SIZE <=
              PICO_FLASH_SIZE_BYTES);

struct EraseOperation {
  uint32_t offset;
};

struct ProgramOperation {
  uint32_t offset;
  const uint8_t *page;
};

void perform_erase(void *context) {
  const auto *operation = static_cast<const EraseOperation *>(context);
  flash_range_erase(operation->offset, PROFILE_STORAGE_ARENA_SIZE);
}

void perform_program(void *context) {
  const auto *operation = static_cast<const ProgramOperation *>(context);
  flash_range_program(operation->offset, operation->page,
                      PROFILE_STORAGE_PAGE_SIZE);
}

bool storage_region_available() {
  const uintptr_t binary_end =
      reinterpret_cast<uintptr_t>(&__flash_binary_end) - XIP_BASE;
  return binary_end <= kProfileStorageOffset;
}

bool read_storage(void *, uint8_t arena, size_t offset, uint8_t *output,
                  size_t size) {
  if (arena >= PROFILE_STORAGE_ARENA_COUNT || output == nullptr ||
      offset > PROFILE_STORAGE_ARENA_SIZE ||
      size > PROFILE_STORAGE_ARENA_SIZE - offset ||
      !storage_region_available()) {
    return false;
  }
  const uintptr_t address = XIP_BASE + kProfileStorageOffset +
                            arena * PROFILE_STORAGE_ARENA_SIZE + offset;
  memcpy(output, reinterpret_cast<const void *>(address), size);
  return true;
}

bool erase_arena(void *, uint8_t arena) {
  if (arena >= PROFILE_STORAGE_ARENA_COUNT || !storage_region_available()) {
    return false;
  }
  EraseOperation operation{
      kProfileStorageOffset + arena * PROFILE_STORAGE_ARENA_SIZE,
  };
  if (flash_safe_execute(perform_erase, &operation,
                         kFlashSafeExecuteTimeoutMs) != PICO_OK) {
    return false;
  }
  const auto *stored =
      reinterpret_cast<const uint8_t *>(XIP_BASE + operation.offset);
  for (size_t offset = 0; offset < PROFILE_STORAGE_ARENA_SIZE; ++offset) {
    if (stored[offset] != 0xff) {
      return false;
    }
  }
  return true;
}

bool program_page(void *, uint8_t arena, size_t offset, const uint8_t *page,
                  size_t size) {
  if (arena >= PROFILE_STORAGE_ARENA_COUNT || page == nullptr ||
      size != PROFILE_STORAGE_PAGE_SIZE ||
      offset % PROFILE_STORAGE_PAGE_SIZE != 0 ||
      offset > PROFILE_STORAGE_ARENA_SIZE ||
      size > PROFILE_STORAGE_ARENA_SIZE - offset ||
      !storage_region_available()) {
    return false;
  }
  ProgramOperation operation{
      kProfileStorageOffset + arena * PROFILE_STORAGE_ARENA_SIZE + offset,
      page,
  };
  return flash_safe_execute(perform_program, &operation,
                            kFlashSafeExecuteTimeoutMs) == PICO_OK &&
         memcmp(reinterpret_cast<const void *>(XIP_BASE + operation.offset),
                page, size) == 0;
}

} // namespace

ProfileStorageIo pico_profile_storage_io() {
  return {
      nullptr,           PROFILE_STORAGE_ARENA_SIZE,
      FLASH_SECTOR_SIZE, FLASH_PAGE_SIZE,
      read_storage,      erase_arena,
      program_page,
  };
}
