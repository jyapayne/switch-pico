#include "storage.h"
#include "protocol.h"
#include "configuration/configuration_storage.h"
#include "profile/profile_storage.h"
#include "hardware/flash.h"
#include "pico/btstack_flash_bank.h"
#include "pico/flash.h"
#include "pico/platform.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

extern "C" {
alignas(FLASH_SECTOR_SIZE) uint8_t probe_test_flash[PICO_FLASH_SIZE_BYTES];
}

namespace {
constexpr size_t kBankSize = 2 * FLASH_SECTOR_SIZE;
constexpr uint32_t kProfileOffset = PICO_FLASH_BANK_STORAGE_OFFSET -
    CONFIGURATION_STORAGE_COPY_COUNT * FLASH_SECTOR_SIZE - PROFILE_STORAGE_TOTAL_SIZE;
constexpr uint32_t kReservedOffset = kProfileOffset -
    (PROBE_CONTROLLER_COUNT > 2 ? PROBE_CONTROLLER_COUNT : 2) * kBankSize;
using Blob = std::array<uint8_t, PROBE_PAIRING_BLOB_SIZE>;
using Blobs = std::array<Blob, PROBE_CONTROLLER_COUNT>;
using Image = std::vector<uint8_t>;

struct Mutation {
    uint32_t offset;
    size_t size;
    bool erase;
};
std::vector<Mutation> mutations;
int safe_calls;
int fail_at = -1;
size_t torn_bytes;
size_t mutation_limit = std::numeric_limits<size_t>::max();
bool inside_safe;
bool fault_hit;

void reset_fault() {
    mutations.clear();
    safe_calls = 0;
    fail_at = -1;
    fault_hit = false;
}

Image image() {
    return Image(probe_test_flash, probe_test_flash + sizeof(probe_test_flash));
}

void restore(const Image& saved) {
    std::memcpy(probe_test_flash, saved.data(), saved.size());
    reset_fault();
}

Blob blob(uint8_t instance, unsigned generation) {
    Blob result;
    for (size_t i = 0; i < result.size(); ++i)
        result[i] = static_cast<uint8_t>(instance * 31 + generation * 83 + i * 7);
    return result;
}

void expect_blob(uint8_t instance, const Blob& expected) {
    Blob result;
    result.fill(0xa5);
    assert(probe_storage_load(instance, result.data(), result.size()));
    assert(result == expected);
}

void expect_siblings(uint8_t target, const Blobs& expected) {
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        if (instance != target) expect_blob(instance, expected[instance]);
}

void expect_outside_unchanged(uint8_t target, const Image& before) {
    const uint32_t offset = probe_storage_offset(target);
    assert(std::memcmp(probe_test_flash, before.data(), offset) == 0);
    assert(std::memcmp(probe_test_flash + offset + kBankSize,
                       before.data() + offset + kBankSize,
                       sizeof(probe_test_flash) - offset - kBankSize) == 0);
}

void erase_fixture() {
    reset_fault();
    // Non-erased sentinels protect firmware, profiles, configuration and BTstack.
    std::memset(probe_test_flash, 0xa5, sizeof(probe_test_flash));
    std::memset(probe_test_flash + kReservedOffset, 0xff, kProfileOffset - kReservedOffset);
}

Blobs seed() {
    erase_fixture();
    Blobs expected;
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        expected[instance] = blob(instance, 1);
        assert(probe_storage_save(instance, expected[instance].data(), expected[instance].size()));
    }
    reset_fault();
    return expected;
}

void test_offsets_and_isolation() {
    // These are the pre-experiment R/L offsets for the 2 MiB stub geometry.
    const uint32_t original_offsets[] = {0x1ba000, 0x1b8000, 0x1b6000, 0x1b4000};
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        const unsigned bank = PROBE_CONTROLLER_COUNT == 1 ? SWITCH2_PROBE_JOYCON_LEFT : instance;
        assert(probe_storage_offset(instance) == original_offsets[bank]);
    }
    assert(probe_storage_offset(PROBE_CONTROLLER_COUNT) == UINT32_MAX);
    assert(probe_storage_offset(UINT8_MAX) == UINT32_MAX);

    auto expected = seed();
    for (unsigned generation = 2; generation <= 3; ++generation) {
        for (unsigned remaining = PROBE_CONTROLLER_COUNT; remaining; --remaining) {
            const uint8_t instance = static_cast<uint8_t>(remaining - 1);
            const auto before = image();
            expected[instance] = blob(instance, generation);
            assert(probe_storage_save(instance, expected[instance].data(), expected[instance].size()));
            expect_blob(instance, expected[instance]);
            expect_siblings(instance, expected);
            expect_outside_unchanged(instance, before);
            reset_fault();
            assert(probe_storage_save(instance, expected[instance].data(), expected[instance].size()));
            assert(mutations.empty()); // Identical saves must not wear flash.
        }
    }
    Blob output;
    output.fill(0xa5);
    const Blob untouched = output;
    const auto before = image();
    assert(!probe_storage_load(PROBE_CONTROLLER_COUNT, output.data(), output.size()));
    assert(!probe_storage_save(PROBE_CONTROLLER_COUNT, output.data(), output.size()));
    assert(!probe_storage_load(UINT8_MAX, output.data(), output.size()));
    assert(!probe_storage_save(UINT8_MAX, output.data(), output.size()));
    assert(!probe_storage_load(0, output.data(), output.size() - 1));
    assert(output == untouched);
    assert(image() == before);
    assert(mutations.empty());
}

void test_interrupted_updates() {
    for (uint8_t target = 0; target < PROBE_CONTROLLER_COUNT; ++target) {
        // An erased inactive slot needs only programming. A reused inactive slot
        // must first erase its old owned record; cover both atomic transitions.
        for (bool reuse : {false, true}) {
            auto expected = seed();
            if (reuse) {
                expected[target] = blob(target, 2);
                assert(probe_storage_save(target, expected[target].data(), expected[target].size()));
            }
            const auto before = image();
            const Blob replacement = blob(target, 3);
            reset_fault();
            assert(probe_storage_save(target, replacement.data(), replacement.size()));
            const auto successful_mutations = mutations;
            assert(!successful_mutations.empty());
            for (size_t cut = 0; cut < successful_mutations.size(); ++cut) {
                const Mutation interrupted = successful_mutations[cut];
                const size_t partials[] = {0, 1, FLASH_PAGE_SIZE / 2, interrupted.size};
                for (size_t partial : partials) {
                    restore(before);
                    fail_at = static_cast<int>(cut);
                    torn_bytes = partial;
                    assert(!probe_storage_save(target, replacement.data(), replacement.size()));
                    assert(fault_hit);
                    expect_outside_unchanged(target, before);
                    expect_siblings(target, expected);
                    Blob recovered;
                    assert(probe_storage_load(target, recovered.data(), recovered.size()));
                    // A fully programmed commit may survive despite an ambiguous
                    // flash-safe return. Only the complete old OR new blob is legal.
                    assert(recovered == expected[target] || recovered == replacement);

                    // A torn owner/erase cannot prove ownership and must refuse
                    // further writes. Complete ownership allows body/commit recovery.
                    const bool unknown = interrupted.erase ?
                        partial != 0 && partial < interrupted.size :
                        interrupted.offset % FLASH_SECTOR_SIZE == 0 && partial != 0 && partial < 40;
                    const auto after_failure = image();
                    reset_fault();
                    const bool saved = probe_storage_save(target, replacement.data(), replacement.size());
                    assert(saved != unknown);
                    if (unknown) {
                        assert(mutations.empty());
                        assert(image() == after_failure);
                    } else {
                        expect_blob(target, replacement);
                    }
                    expect_siblings(target, expected);
                    expect_outside_unchanged(target, before);
                    if (unknown && PROBE_CONTROLLER_COUNT > 1) {
                        const uint8_t sibling = (target + 1) % PROBE_CONTROLLER_COUNT;
                        const auto before_sibling = image();
                        const Blob sibling_replacement = blob(sibling, 4);
                        assert(probe_storage_save(sibling, sibling_replacement.data(), sibling_replacement.size()));
                        expect_blob(sibling, sibling_replacement);
                        expect_outside_unchanged(sibling, before_sibling);
                    }
                }
            }
        }
    }
}

void test_unknown_sectors() {
    for (uint8_t target = 0; target < PROBE_CONTROLLER_COUNT; ++target) {
        for (unsigned slot = 0; slot < 2; ++slot) {
            const auto expected = seed();
            // Neither an arbitrary sector nor a record copied from a different
            // absolute bank may be claimed just because another child owns it.
            const uint32_t offset = probe_storage_offset(target) + slot * FLASH_SECTOR_SIZE;
            if (slot == 1 && PROBE_CONTROLLER_COUNT > 1) {
                const uint8_t sibling = (target + 1) % PROBE_CONTROLLER_COUNT;
                std::memcpy(probe_test_flash + offset,
                            probe_test_flash + probe_storage_offset(sibling), FLASH_SECTOR_SIZE);
            } else {
                probe_test_flash[offset] ^= 0x55;
            }
            const auto before = image();
            const Blob replacement = blob(target, 2);
            assert(!probe_storage_save(target, replacement.data(), replacement.size()));
            assert(mutations.empty());
            assert(image() == before);
            Blob output;
            output.fill(0xa5);
            const Blob untouched = output;
            if (slot == 0) {
                assert(!probe_storage_load(target, output.data(), output.size()));
                assert(output == untouched);
            } else {
                expect_blob(target, expected[target]);
            }
            expect_siblings(target, expected);
        }
    }
}

void test_reserved_range_overlap() {
    erase_fixture();
    const auto before = image();
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        Blob output;
        output.fill(0xa5);
        const Blob untouched = output;
        assert(!probe_storage_load(instance, output.data(), output.size()));
        assert(output == untouched);
        assert(!probe_storage_save(instance, output.data(), output.size()));
    }
    assert(mutations.empty());
    assert(image() == before);
}
} // namespace

void flash_range_erase(uint32_t offset, size_t count) {
    assert(inside_safe);
    assert(offset % FLASH_SECTOR_SIZE == 0 && count == FLASH_SECTOR_SIZE);
    assert(offset <= PICO_FLASH_SIZE_BYTES && count <= PICO_FLASH_SIZE_BYTES - offset);
    mutations.push_back({offset, count, true});
    std::memset(probe_test_flash + offset, 0xff, std::min(count, mutation_limit));
}

void flash_range_program(uint32_t offset, const uint8_t* data, size_t count) {
    assert(inside_safe);
    assert(offset % FLASH_PAGE_SIZE == 0 && count == FLASH_PAGE_SIZE);
    assert(offset <= PICO_FLASH_SIZE_BYTES && count <= PICO_FLASH_SIZE_BYTES - offset);
    mutations.push_back({offset, count, false});
    for (size_t i = 0; i < std::min(count, mutation_limit); ++i)
        probe_test_flash[offset + i] &= data[i];
}

int flash_safe_execute(void (*function)(void*), void* parameter, uint32_t timeout_ms) {
    assert(timeout_ms != 0 && !inside_safe);
    const bool fail = safe_calls++ == fail_at;
    mutation_limit = fail ? torn_bytes : std::numeric_limits<size_t>::max();
    fault_hit |= fail;
    inside_safe = true;
    function(parameter);
    inside_safe = false;
    return fail ? -1 : PICO_OK;
}

int main() {
    if (PROBE_TEST_STORAGE_OVERLAP) {
        test_reserved_range_overlap();
    } else {
        test_offsets_and_isolation();
        test_interrupted_updates();
        test_unknown_sectors();
    }
    std::puts("switch2 probe pairing storage tests passed");
    return 0;
}
