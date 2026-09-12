#include "storage.h"
#include "model.h"

#include <string.h>

#include "configuration/configuration_storage.h"
#include "hardware/flash.h"
#include "pico/btstack_flash_bank.h"
#include "pico/flash.h"
#include "pico/platform.h"
#include "profile/profile_storage.h"

extern "C" char __flash_binary_end;

namespace {

constexpr size_t kSlotCount = 2;
constexpr size_t kMaximumPayloadSize = 512;
constexpr size_t kStorageSize = kSlotCount * FLASH_SECTOR_SIZE;
constexpr size_t kReservedStorageSize = 2 * kStorageSize;
constexpr size_t kConfigurationStorageSize =
    CONFIGURATION_STORAGE_COPY_COUNT * FLASH_SECTOR_SIZE;
constexpr size_t kConfigurationStorageOffset =
    PICO_FLASH_BANK_STORAGE_OFFSET - kConfigurationStorageSize;
constexpr size_t kProfileStorageOffset =
    kConfigurationStorageOffset - PROFILE_STORAGE_TOTAL_SIZE;
// Keep the original right bank adjacent to profiles; reserve the left bank below.
constexpr uint32_t kRightStorageOffset = kProfileStorageOffset - kStorageSize;
constexpr uint32_t kLeftStorageOffset = kRightStorageOffset - kStorageSize;
constexpr uint32_t kFlashSafeTimeoutMs = 5000;
constexpr uint32_t kFormatVersion = 1;

// Each sector owns one record. Integers are little-endian; all padding is ff.
// Page 0: magic[16], version:u32, absolute sector offset:u32, maximum payload:u32,
//         page size:u32, sector size:u32, CRC32(bytes 0..35):u32, padding.
// Page 1 starts the body: magic[8], generation:u32, ~generation:u32,
//         length:u32, payload CRC32:u32, header size:u32, header CRC32:u32,
//         payload[length], padding to the fixed body-area boundary.
// The separate commit page is programmed LAST: magic[16], generation:u32,
//         header CRC32:u32, payload CRC32:u32, length:u32, sector offset:u32,
//         CRC32(bytes 0..35):u32, padding. The rest of the sector stays erased.
// CRC32 is the project's IEEE CRC32 (also compatible with zlib.crc32).
constexpr uint8_t kOwnerMagic[16] = {
    'S', '2', 'P', 'R', 'O', 'B', 'E', '-',
    'P', 'A', 'I', 'R', 'I', 'N', 'G', 0,
};
constexpr uint8_t kBodyMagic[8] = {'S', '2', 'P', 'A', 'I', 'R', '0', '1'};
constexpr uint8_t kCommitMagic[16] = {
    'S', '2', 'P', 'A', 'I', 'R', '-', 'C',
    'O', 'M', 'M', 'I', 'T', 'T', 'E', 'D',
};
constexpr size_t kDescriptorCrcOffset = 36;
constexpr size_t kDescriptorSize = kDescriptorCrcOffset + sizeof(uint32_t);
constexpr size_t kBodyOffset = FLASH_PAGE_SIZE;
constexpr size_t kBodyHeaderCrcOffset = 28;
constexpr size_t kBodyHeaderSize = kBodyHeaderCrcOffset + sizeof(uint32_t);
constexpr size_t kBodySize =
    ((kBodyHeaderSize + kMaximumPayloadSize + FLASH_PAGE_SIZE - 1) /
     FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
constexpr size_t kPayloadOffset = kBodyOffset + kBodyHeaderSize;
constexpr size_t kCommitOffset = kBodyOffset + kBodySize;
constexpr size_t kRecordFootprint = kCommitOffset + FLASH_PAGE_SIZE;

static_assert(FLASH_SECTOR_SIZE == PROFILE_STORAGE_SECTOR_SIZE);
static_assert(FLASH_PAGE_SIZE == PROFILE_STORAGE_PAGE_SIZE);
static_assert(FLASH_SECTOR_SIZE % FLASH_PAGE_SIZE == 0);
static_assert(kDescriptorSize <= FLASH_PAGE_SIZE);
static_assert(kRecordFootprint <= FLASH_SECTOR_SIZE);
static_assert(PROFILE_STORAGE_TOTAL_SIZE % FLASH_SECTOR_SIZE == 0);
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET % FLASH_SECTOR_SIZE == 0);
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET >=
              kConfigurationStorageSize + PROFILE_STORAGE_TOTAL_SIZE +
                  kReservedStorageSize,
              "pairing storage offset underflows flash");
static_assert(kLeftStorageOffset + kStorageSize == kRightStorageOffset);
static_assert(kRightStorageOffset + kStorageSize == kProfileStorageOffset);
static_assert(kLeftStorageOffset + kReservedStorageSize == kProfileStorageOffset);
static_assert(kProfileStorageOffset + PROFILE_STORAGE_TOTAL_SIZE ==
              kConfigurationStorageOffset);
static_assert(kConfigurationStorageOffset + kConfigurationStorageSize ==
              PICO_FLASH_BANK_STORAGE_OFFSET);
static_assert(PICO_FLASH_BANK_STORAGE_OFFSET <= PICO_FLASH_SIZE_BYTES);
static_assert(PICO_FLASH_BANK_TOTAL_SIZE <=
              PICO_FLASH_SIZE_BYTES - PICO_FLASH_BANK_STORAGE_OFFSET,
              "BTstack storage exceeds flash");

// Avoid a large USB callback stack frame. All program sources, including any
// input originally backed by XIP, are staged before the first flash mutation.
alignas(FLASH_PAGE_SIZE) uint8_t staging[kRecordFootprint];

enum class SlotKind { Erased, Unknown, OwnedIncomplete, Committed };

struct Slot {
    SlotKind kind;
    const uint8_t *bytes;
    uint32_t generation;
    size_t size;
};

struct FlashMutation {
    uint32_t offset;
    const uint8_t *page; // Null means erase one sector.
};

uint32_t read_u32(const uint8_t *input) {
    return static_cast<uint32_t>(input[0]) |
           (static_cast<uint32_t>(input[1]) << 8) |
           (static_cast<uint32_t>(input[2]) << 16) |
           (static_cast<uint32_t>(input[3]) << 24);
}

void write_u32(uint8_t *output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

bool is_erased(const uint8_t *bytes, size_t size) {
    for (size_t index = 0; index < size; ++index) {
        if (bytes[index] != 0xff) {
            return false;
        }
    }
    return true;
}

bool storage_region_available(uint32_t storage_offset) {
    const uintptr_t binary_end = reinterpret_cast<uintptr_t>(&__flash_binary_end);
    return binary_end >= XIP_BASE &&
           binary_end - XIP_BASE <= kLeftStorageOffset &&
           storage_offset % FLASH_SECTOR_SIZE == 0 &&
           storage_offset <= PICO_FLASH_SIZE_BYTES &&
           kStorageSize <= PICO_FLASH_SIZE_BYTES - storage_offset &&
           storage_offset >= kLeftStorageOffset &&
           storage_offset + kStorageSize <= kProfileStorageOffset;
}

uint32_t slot_offset(uint32_t storage_offset, size_t slot) {
    return static_cast<uint32_t>(storage_offset + slot * FLASH_SECTOR_SIZE);
}

const uint8_t *slot_bytes(uint32_t storage_offset, size_t slot) {
    return reinterpret_cast<const uint8_t *>(XIP_BASE + slot_offset(storage_offset, slot));
}

bool owner_valid(const uint8_t *bytes, uint32_t offset) {
    return memcmp(bytes, kOwnerMagic, sizeof(kOwnerMagic)) == 0 &&
           read_u32(bytes + 16) == kFormatVersion &&
           read_u32(bytes + 20) == offset &&
           read_u32(bytes + 24) == kMaximumPayloadSize &&
           read_u32(bytes + 28) == FLASH_PAGE_SIZE &&
           read_u32(bytes + 32) == FLASH_SECTOR_SIZE &&
           read_u32(bytes + kDescriptorCrcOffset) ==
               configuration_crc32(bytes, kDescriptorCrcOffset) &&
           is_erased(bytes + kDescriptorSize,
                     FLASH_PAGE_SIZE - kDescriptorSize);
}

bool body_valid(const uint8_t *bytes) {
    const uint8_t *body = bytes + kBodyOffset;
    const size_t size = read_u32(body + 16);
    return memcmp(body, kBodyMagic, sizeof(kBodyMagic)) == 0 &&
           read_u32(body + 12) == ~read_u32(body + 8) &&
           size != 0 && size <= kMaximumPayloadSize &&
           read_u32(body + 24) == kBodyHeaderSize &&
           read_u32(body + kBodyHeaderCrcOffset) ==
               configuration_crc32(body, kBodyHeaderCrcOffset) &&
           read_u32(body + 20) ==
               configuration_crc32(bytes + kPayloadOffset, size) &&
           is_erased(bytes + kPayloadOffset + size,
                     kBodySize - kBodyHeaderSize - size);
}

bool commit_valid(const uint8_t *bytes, uint32_t offset) {
    const uint8_t *body = bytes + kBodyOffset;
    const uint8_t *commit = bytes + kCommitOffset;
    return memcmp(commit, kCommitMagic, sizeof(kCommitMagic)) == 0 &&
           read_u32(commit + 16) == read_u32(body + 8) &&
           read_u32(commit + 20) == read_u32(body + kBodyHeaderCrcOffset) &&
           read_u32(commit + 24) == read_u32(body + 20) &&
           read_u32(commit + 28) == read_u32(body + 16) &&
           read_u32(commit + 32) == offset &&
           read_u32(commit + kDescriptorCrcOffset) ==
               configuration_crc32(commit, kDescriptorCrcOffset) &&
           is_erased(commit + kDescriptorSize,
                     FLASH_PAGE_SIZE - kDescriptorSize);
}

Slot inspect_slot(uint32_t storage_offset, size_t index) {
    const uint8_t *bytes = slot_bytes(storage_offset, index);
    Slot slot{SlotKind::Unknown, bytes, 0, 0};
    if (!owner_valid(bytes, slot_offset(storage_offset, index))) {
        if (is_erased(bytes, FLASH_SECTOR_SIZE)) {
            slot.kind = SlotKind::Erased;
        }
        // A torn ownership page or erase is deliberately NOT guessed to be
        // ours. Recovery may need an externally verified backup in that case.
        return slot;
    }
    if (!is_erased(bytes + kRecordFootprint,
                   FLASH_SECTOR_SIZE - kRecordFootprint)) {
        return slot;
    }
    // A complete ownership page plus an erased tail proves ownership of the
    // bounded body/commit area, even if either subsequent write was interrupted.
    slot.kind = SlotKind::OwnedIncomplete;
    if (body_valid(bytes) && commit_valid(bytes, slot_offset(storage_offset, index))) {
        slot.kind = SlotKind::Committed;
        slot.generation = read_u32(bytes + kBodyOffset + 8);
        slot.size = read_u32(bytes + kBodyOffset + 16);
    }
    return slot;
}

bool newest_slot(const Slot (&slots)[kSlotCount], int *index) {
    *index = -1;
    for (size_t candidate = 0; candidate < kSlotCount; ++candidate) {
        if (slots[candidate].kind != SlotKind::Committed) {
            continue;
        }
        if (*index < 0) {
            *index = static_cast<int>(candidate);
            continue;
        }
        const Slot &current = slots[*index];
        const Slot &next = slots[candidate];
        const uint32_t difference = next.generation - current.generation;
        if (difference == 0) {
            if (next.size != current.size ||
                memcmp(next.bytes + kPayloadOffset,
                       current.bytes + kPayloadOffset, next.size) != 0) {
                return false; // Conflicting records with no ordering.
            }
        } else if (difference == 0x80000000u) {
            return false; // Exactly half a generation cycle is ambiguous.
        } else if (difference < 0x80000000u) {
            *index = static_cast<int>(candidate);
        }
    }
    return true;
}

void perform_flash_mutation(void *context) {
    const auto *mutation = static_cast<const FlashMutation *>(context);
    if (mutation->page == nullptr) {
        flash_range_erase(mutation->offset, FLASH_SECTOR_SIZE);
    } else {
        flash_range_program(mutation->offset, mutation->page, FLASH_PAGE_SIZE);
    }
}

// The caller has classified BOTH sectors before permitting any erase. Only
// the inactive, explicitly owned sector is passed here; the active one survives.
bool erase_slot(uint32_t storage_offset, size_t index) {
    if (index >= kSlotCount || !storage_region_available(storage_offset)) {
        return false;
    }
    FlashMutation mutation{slot_offset(storage_offset, index), nullptr};
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              kFlashSafeTimeoutMs) == PICO_OK &&
           is_erased(slot_bytes(storage_offset, index), FLASH_SECTOR_SIZE);
}

bool program_page(uint32_t storage_offset, size_t index, size_t offset, const uint8_t *page) {
    if (index >= kSlotCount || offset % FLASH_PAGE_SIZE != 0 ||
        offset > kRecordFootprint - FLASH_PAGE_SIZE ||
        !storage_region_available(storage_offset) ||
        !is_erased(slot_bytes(storage_offset, index) + offset, FLASH_PAGE_SIZE)) {
        return false;
    }
    FlashMutation mutation{
        static_cast<uint32_t>(slot_offset(storage_offset, index) + offset), page,
    };
    return flash_safe_execute(perform_flash_mutation, &mutation,
                              kFlashSafeTimeoutMs) == PICO_OK &&
           memcmp(slot_bytes(storage_offset, index) + offset, page, FLASH_PAGE_SIZE) == 0;
}

void prepare_record(uint32_t storage_offset, size_t target, uint32_t generation,
                    const uint8_t *data, size_t size) {
    memset(staging, 0xff, sizeof(staging));
    memcpy(staging, kOwnerMagic, sizeof(kOwnerMagic));
    write_u32(staging + 16, kFormatVersion);
    write_u32(staging + 20, slot_offset(storage_offset, target));
    write_u32(staging + 24, kMaximumPayloadSize);
    write_u32(staging + 28, FLASH_PAGE_SIZE);
    write_u32(staging + 32, FLASH_SECTOR_SIZE);
    write_u32(staging + kDescriptorCrcOffset,
              configuration_crc32(staging, kDescriptorCrcOffset));

    uint8_t *body = staging + kBodyOffset;
    memcpy(body, kBodyMagic, sizeof(kBodyMagic));
    write_u32(body + 8, generation);
    write_u32(body + 12, ~generation);
    write_u32(body + 16, static_cast<uint32_t>(size));
    memcpy(staging + kPayloadOffset, data, size);
    const uint32_t payload_crc = configuration_crc32(staging + kPayloadOffset, size);
    write_u32(body + 20, payload_crc);
    write_u32(body + 24, kBodyHeaderSize);
    const uint32_t header_crc = configuration_crc32(body, kBodyHeaderCrcOffset);
    write_u32(body + kBodyHeaderCrcOffset, header_crc);

    uint8_t *commit = staging + kCommitOffset;
    memcpy(commit, kCommitMagic, sizeof(kCommitMagic));
    write_u32(commit + 16, generation);
    write_u32(commit + 20, header_crc);
    write_u32(commit + 24, payload_crc);
    write_u32(commit + 28, static_cast<uint32_t>(size));
    write_u32(commit + 32, slot_offset(storage_offset, target));
    write_u32(commit + kDescriptorCrcOffset,
              configuration_crc32(commit, kDescriptorCrcOffset));
}

} // namespace

bool probe_storage_load(uint8_t instance, uint8_t *output, size_t size) {
    if (instance >= PROBE_CONTROLLER_COUNT) return false;
    const uint32_t storage_offset = probe_storage_offset(instance);
    if (output == nullptr || size == 0 || size > kMaximumPayloadSize ||
        !storage_region_available(storage_offset)) {
        return false;
    }
    const Slot slots[kSlotCount] = {
        inspect_slot(storage_offset, 0), inspect_slot(storage_offset, 1),
    };
    int active;
    if (!newest_slot(slots, &active) || active < 0 || slots[active].size != size) {
        return false;
    }
    memcpy(output, slots[active].bytes + kPayloadOffset, size);
    return true;
}

bool probe_storage_save(uint8_t instance, const uint8_t *data, size_t size) {
    if (instance >= PROBE_CONTROLLER_COUNT) return false;
    const uint32_t storage_offset = probe_storage_offset(instance);
    if (data == nullptr || size == 0 || size > kMaximumPayloadSize ||
        !storage_region_available(storage_offset)) {
        return false;
    }
    const Slot slots[kSlotCount] = {
        inspect_slot(storage_offset, 0), inspect_slot(storage_offset, 1),
    };
    if (slots[0].kind == SlotKind::Unknown || slots[1].kind == SlotKind::Unknown) {
        return false; // Never erase through an unrecognized region.
    }
    int active;
    if (!newest_slot(slots, &active)) {
        return false;
    }
    if (active >= 0 && slots[active].size == size &&
        memcmp(slots[active].bytes + kPayloadOffset, data, size) == 0) {
        return true;
    }
    const size_t target = active >= 0
                              ? static_cast<size_t>(active) ^ 1u
                              : (slots[0].kind == SlotKind::Erased ? 0u : 1u);
    const uint32_t generation = active >= 0 ? slots[active].generation + 1u : 1u;
    prepare_record(storage_offset, target, generation, data, size);

    if (slots[target].kind != SlotKind::Erased && !erase_slot(storage_offset, target)) {
        return false;
    }
    if (!program_page(storage_offset, target, 0, staging)) {
        return false;
    }
    for (size_t offset = kBodyOffset; offset < kCommitOffset;
         offset += FLASH_PAGE_SIZE) {
        if (!is_erased(staging + offset, FLASH_PAGE_SIZE) &&
            !program_page(storage_offset, target, offset, staging + offset)) {
            return false;
        }
    }
    if (!body_valid(slot_bytes(storage_offset, target)) ||
        !program_page(storage_offset, target, kCommitOffset, staging + kCommitOffset)) {
        return false;
    }
    const Slot committed = inspect_slot(storage_offset, target);
    return committed.kind == SlotKind::Committed &&
           committed.generation == generation && committed.size == size &&
           memcmp(committed.bytes + kPayloadOffset,
                  staging + kPayloadOffset, size) == 0;
}

uint32_t probe_storage_offset(uint8_t instance) {
    if (instance >= PROBE_CONTROLLER_COUNT) return UINT32_MAX;
    return probe_model_is_left(instance) ? kLeftStorageOffset : kRightStorageOffset;
}
