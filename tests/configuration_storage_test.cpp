#include "configuration/adapter_configuration.h"
#include "configuration/configuration_storage.h"
#include "configuration/configuration_transaction.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

constexpr size_t kSectorSize = 4096;
constexpr size_t kPageSize = 256;

struct FakeFlash {
    uint8_t bytes[CONFIGURATION_STORAGE_COPY_COUNT][kSectorSize];
    int successful_programs = 0;
    int fail_after_programs = -1;

    FakeFlash() { memset(bytes, 0xff, sizeof(bytes)); }
};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

bool fake_read(void* context, uint8_t copy, size_t offset,
               uint8_t* output, size_t size) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT ||
        offset + size > kSectorSize) {
        return false;
    }
    memcpy(output, &flash->bytes[copy][offset], size);
    return true;
}

bool fake_erase(void* context, uint8_t copy) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT) {
        return false;
    }
    memset(flash->bytes[copy], 0xff, kSectorSize);
    return true;
}

bool fake_program(void* context, uint8_t copy, size_t offset,
                  const uint8_t* data, size_t size) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT || size != kPageSize ||
        offset + size > kSectorSize) {
        return false;
    }
    if (flash->fail_after_programs >= 0 &&
        flash->successful_programs >= flash->fail_after_programs) {
        return false;
    }
    for (size_t index = 0; index < size; ++index) {
        flash->bytes[copy][offset + index] &= data[index];
    }
    ++flash->successful_programs;
    return true;
}

ConfigurationStorageIo fake_io(FakeFlash* flash) {
    return {
        flash,
        kSectorSize,
        kPageSize,
        fake_read,
        fake_erase,
        fake_program,
    };
}

ControllerIdentity nintendo_identity(uint8_t address_suffix,
                                     uint16_t product_id = 0x2009) {
    ControllerIdentity identity{};
    identity.stable = true;
    identity.transport = ControllerTransport::kClassic;
    identity.address[0] = 0x02;
    identity.address[5] = address_suffix;
    identity.vendor_id = 0x057e;
    identity.product_id = product_id;
    return identity;
}

void test_schema_encoding() {
    AdapterConfiguration configuration{};
    configuration.pairing_window_seconds = 90;
    configuration.requested_mode = AdapterRequestedMode::kXInput;
    configuration.joycon_mode = JoyConMode::kIndividual;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload,
                                         sizeof(payload)),
            "valid v4 configuration did not encode");
    const uint8_t expected[ADAPTER_CONFIGURATION_ENCODED_SIZE] = {
        90,
        0,
        static_cast<uint8_t>(AdapterRequestedMode::kXInput),
        0,
        static_cast<uint8_t>(JoyConMode::kIndividual),
        0,
        0,
        0,
    };
    require(memcmp(payload, expected, sizeof(expected)) == 0,
            "v4 configuration bytes are not canonical");

    AdapterConfiguration decoded{};
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                sizeof(payload), &decoded) &&
                decoded.pairing_window_seconds == 90 &&
                decoded.requested_mode == AdapterRequestedMode::kXInput &&
                decoded.joycon_mode == JoyConMode::kIndividual &&
                decoded.native_switch_controller_count == 0,
            "v4 configuration did not round trip");

    const AdapterRequestedMode valid_modes[] = {
        AdapterRequestedMode::kAuto,
        AdapterRequestedMode::kSwitch,
        AdapterRequestedMode::kXInput,
        AdapterRequestedMode::kDInput,
        AdapterRequestedMode::kMac,
    };
    for (AdapterRequestedMode mode : valid_modes) {
        configuration.requested_mode = mode;
        require(adapter_configuration_encode(configuration, payload,
                                             sizeof(payload)) &&
                    adapter_configuration_decode(
                        ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                        sizeof(payload), &decoded) &&
                    decoded.requested_mode == mode &&
                    decoded.joycon_mode == JoyConMode::kIndividual,
                "valid requested mode did not round trip");
    }

    AdapterModeAvailability availability{};
    require(!adapter_requested_mode_available(
                AdapterRequestedMode::kDInput, availability) &&
                !adapter_requested_mode_available(
                    AdapterRequestedMode::kMac, availability),
            "future modes were available by default");
    availability.dinput_mode = true;
    availability.mac_mode = true;
    require(adapter_requested_mode_available(
                AdapterRequestedMode::kDInput, availability) &&
                adapter_requested_mode_available(
                    AdapterRequestedMode::kMac, availability),
            "availability API could not enable future modes");

    const uint8_t legacy[] = {120, 0, 0, 0};
    decoded.native_switch_controller_count = 1;
    decoded.native_switch_controllers[0] = nintendo_identity(1);
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION, legacy,
                sizeof(legacy), &decoded) &&
                decoded.pairing_window_seconds == 120 &&
                decoded.requested_mode == AdapterRequestedMode::kAuto &&
                decoded.joycon_mode == JoyConMode::kPaired &&
                decoded.native_switch_controller_count == 0,
            "v1 configuration did not migrate without approvals");
    uint8_t malformed_legacy[sizeof(legacy)];
    memcpy(malformed_legacy, legacy, sizeof(legacy));
    malformed_legacy[3] = 1;
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION,
                malformed_legacy, sizeof(malformed_legacy), &decoded),
            "v1 nonzero reserved byte was accepted");
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION, legacy,
                sizeof(legacy) - 1, &decoded),
            "v1 record with wrong size was accepted");

    const uint8_t v2[] = {
        120, 0, static_cast<uint8_t>(AdapterRequestedMode::kMac), 0, 0, 0, 0, 0,
    };
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION, v2, sizeof(v2),
                &decoded) &&
                decoded.pairing_window_seconds == 120 &&
                decoded.requested_mode == AdapterRequestedMode::kMac &&
                decoded.joycon_mode == JoyConMode::kPaired &&
                decoded.native_switch_controller_count == 0,
            "v2 migration changed settings or granted native rumble");
    for (size_t index = 3; index < sizeof(v2); ++index) {
        uint8_t malformed[sizeof(v2)];
        memcpy(malformed, v2, sizeof(v2));
        malformed[index] = 1;
        require(!adapter_configuration_decode(
                    ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION, malformed,
                    sizeof(malformed), &decoded),
                "v2 nonzero reserved byte was accepted");
    }
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION, v2, sizeof(v2) - 1,
                &decoded),
            "v2 record with wrong size was accepted");

    for (size_t index = 3; index < sizeof(payload); ++index) {
        if (index == 4) {
            continue;
        }
        uint8_t malformed[sizeof(payload)];
        memcpy(malformed, payload, sizeof(payload));
        malformed[index] = 1;
        require(!adapter_configuration_decode(
                    ADAPTER_CONFIGURATION_SCHEMA_VERSION, malformed,
                    sizeof(malformed), &decoded),
                "v4 nonzero reserved or unused byte was accepted");
    }
    uint8_t invalid_mode[sizeof(payload)];
    memcpy(invalid_mode, payload, sizeof(payload));
    invalid_mode[2] = 5;
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, invalid_mode,
                sizeof(invalid_mode), &decoded),
            "out-of-range requested mode was accepted");
    const uint8_t invalid_joycon_modes[] = {2, 0xff};
    for (uint8_t mode : invalid_joycon_modes) {
        memcpy(invalid_mode, payload, sizeof(payload));
        invalid_mode[4] = mode;
        require(!adapter_configuration_decode(invalid_mode, sizeof(invalid_mode),
                                              &decoded),
                "out-of-range Joy-Con2 mode was accepted");
        configuration.joycon_mode = static_cast<JoyConMode>(mode);
        require(!adapter_configuration_encode(configuration, payload,
                                              sizeof(payload)),
                "out-of-range Joy-Con2 mode encoded");
    }
    configuration.joycon_mode = JoyConMode::kPaired;
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)) &&
                adapter_configuration_decode(payload, sizeof(payload), &decoded) &&
                decoded.joycon_mode == JoyConMode::kPaired,
            "Paired mode did not round trip");
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                sizeof(payload) - 1, &decoded),
            "short v4 record was accepted");
    uint8_t oversized[ADAPTER_CONFIGURATION_ENCODED_SIZE + 1]{};
    memcpy(oversized, payload, sizeof(payload));
    require(!adapter_configuration_decode(
                ADAPTER_CONFIGURATION_SCHEMA_VERSION, oversized,
                sizeof(oversized), &decoded),
            "oversized v4 record was accepted");

    configuration.pairing_window_seconds = 9;
    require(!adapter_configuration_encode(configuration, payload,
                                          sizeof(payload)),
            "out-of-range pairing window was accepted");
    configuration.pairing_window_seconds = 90;
    configuration.requested_mode =
        static_cast<AdapterRequestedMode>(5);
    require(!adapter_configuration_encode(configuration, payload,
                                          sizeof(payload)),
            "out-of-range requested mode encoded");
    configuration.requested_mode = AdapterRequestedMode::kAuto;
    require(!adapter_configuration_encode(configuration, oversized,
                                          sizeof(oversized)),
            "v4 encoder accepted a noncanonical output size");
}

void test_native_switch_approval_identity_and_canonical_encoding() {
    const ControllerIdentity pro = nintendo_identity(3);
    const ControllerIdentity left = nintendo_identity(1, 0x2006);
    const ControllerIdentity right = nintendo_identity(2, 0x2007);
    AdapterConfiguration configuration{};
    require(!adapter_configuration_native_switch_approved(configuration, pro),
            "a supported model was approved without an explicit identity");
    configuration.native_switch_controller_count = 3;
    configuration.native_switch_controllers[0] = pro;
    configuration.native_switch_controllers[1] = left;
    configuration.native_switch_controllers[2] = right;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)),
            "original Switch controller approvals did not encode");
    AdapterConfiguration decoded{};
    require(adapter_configuration_decode(payload, sizeof(payload), &decoded) &&
                adapter_configuration_native_switch_approved(decoded, pro) &&
                adapter_configuration_native_switch_approved(decoded, left) &&
                adapter_configuration_native_switch_approved(decoded, right) &&
                !adapter_configuration_native_switch_approved(
                    decoded, nintendo_identity(4)) &&
                !adapter_configuration_native_switch_approved(
                    decoded, nintendo_identity(3, 0x2006)),
            "approval did not remain specific to the complete controller identity");

    decoded.joycon_mode = JoyConMode::kIndividual;
    require(adapter_configuration_decode(
                ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION, payload,
                sizeof(payload), &decoded) &&
                decoded.joycon_mode == JoyConMode::kPaired &&
                adapter_configuration_native_switch_approved(decoded, pro) &&
                adapter_configuration_native_switch_approved(decoded, left) &&
                adapter_configuration_native_switch_approved(decoded, right),
            "v3 migration erased approvals or retained a stale player mode");
    for (size_t index = 4; index < ADAPTER_CONFIGURATION_HEADER_SIZE; ++index) {
        uint8_t malformed[sizeof(payload)];
        memcpy(malformed, payload, sizeof(payload));
        malformed[index] = 1;
        require(!adapter_configuration_decode(
                    ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION, malformed,
                    sizeof(malformed), &decoded),
                "v3 reserved header byte was accepted");
    }

    configuration.native_switch_controllers[0] = right;
    configuration.native_switch_controllers[1] = pro;
    configuration.native_switch_controllers[2] = left;
    uint8_t reordered[sizeof(payload)]{};
    require(adapter_configuration_encode(configuration, reordered,
                                         sizeof(reordered)) &&
                memcmp(payload, reordered, sizeof(payload)) == 0,
            "approval insertion order changed the persisted bytes or CRC");

    configuration.native_switch_controller_count = 0;
    require(!adapter_configuration_native_switch_approved(configuration, pro) &&
                adapter_configuration_encode(configuration, reordered,
                                             sizeof(reordered)) &&
                adapter_configuration_decode(reordered, sizeof(reordered),
                                             &decoded) &&
                !adapter_configuration_native_switch_approved(decoded, pro),
            "revoked array entries remained approved after encode and decode");

    configuration.native_switch_controller_count =
        ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY;
    for (size_t index = 0;
         index < configuration.native_switch_controller_count; ++index) {
        configuration.native_switch_controllers[index] =
            nintendo_identity(static_cast<uint8_t>(
                configuration.native_switch_controller_count - index));
    }
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)) &&
                adapter_configuration_decode(payload, sizeof(payload), &decoded) &&
                adapter_configuration_native_switch_approved(
                    decoded, nintendo_identity(1)) &&
                adapter_configuration_native_switch_approved(
                    decoded, nintendo_identity(16)),
            "a full approval array lost its boundary identities");
    ++configuration.native_switch_controller_count;
    require(!adapter_configuration_encode(configuration, payload, sizeof(payload)) &&
                !adapter_configuration_native_switch_approved(configuration, pro),
            "an oversized approval array was accepted");
    payload[3] = configuration.native_switch_controller_count;
    require(!adapter_configuration_decode(payload, sizeof(payload), &decoded),
            "an oversized encoded approval count was accepted");
}

void test_native_switch_approval_rejects_invalid_records() {
    AdapterConfiguration configuration{};
    configuration.native_switch_controller_count = 2;
    configuration.native_switch_controllers[0] = nintendo_identity(1);
    configuration.native_switch_controllers[1] = nintendo_identity(2);
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)),
            "approval validation fixture did not encode");
    configuration.native_switch_controllers[1] =
        configuration.native_switch_controllers[0];
    uint8_t malformed[sizeof(payload)]{};
    require(!adapter_configuration_encode(configuration, malformed,
                                          sizeof(malformed)),
            "duplicate approval identities encoded");
    memcpy(malformed, payload, sizeof(payload));
    constexpr size_t first = ADAPTER_CONFIGURATION_HEADER_SIZE;
    constexpr size_t second = first + CONTROLLER_IDENTITY_ENCODED_SIZE;
    memcpy(malformed + second, malformed + first,
           CONTROLLER_IDENTITY_ENCODED_SIZE);
    AdapterConfiguration decoded{};
    require(!adapter_configuration_decode(malformed, sizeof(malformed), &decoded),
            "duplicate encoded approval identities were accepted");
    memcpy(malformed + first, payload + second, CONTROLLER_IDENTITY_ENCODED_SIZE);
    memcpy(malformed + second, payload + first, CONTROLLER_IDENTITY_ENCODED_SIZE);
    require(!adapter_configuration_decode(malformed, sizeof(malformed), &decoded),
            "noncanonical approval ordering was accepted");

    configuration.native_switch_controller_count = 1;
    ControllerIdentity invalid[] = {
        controller_identity_global(), nintendo_identity(1),
        nintendo_identity(1), nintendo_identity(1), nintendo_identity(1, 0x2069),
    };
    invalid[1].stable = false;
    invalid[2].transport = ControllerTransport::kBle;
    invalid[3].vendor_id = 0x1234;
    for (const ControllerIdentity& identity : invalid) {
        configuration.native_switch_controllers[0] = identity;
        require(!adapter_configuration_encode(configuration, malformed,
                                              sizeof(malformed)) &&
                    !adapter_configuration_native_switch_approved(
                        configuration, identity),
                "global, unstable, BLE, or ineligible controller was approved");
    }
    const uint8_t invalid_fields[][2] = {
        {0, 0}, {0, 2}, {1, 0}, {1, 2}, {1, 3}, {3, 1},
        {10, 0x34}, {12, 0x69},
    };
    for (const auto& field : invalid_fields) {
        memcpy(malformed, payload, sizeof(payload));
        malformed[first + field[0]] = field[1];
        require(!adapter_configuration_decode(malformed, sizeof(malformed),
                                              &decoded),
                "malformed or ineligible encoded identity was accepted");
    }
    memcpy(malformed, payload, sizeof(payload));
    malformed[sizeof(malformed) - 1] = 1;
    require(!adapter_configuration_decode(malformed, sizeof(malformed), &decoded),
            "nonzero unused approval bytes were accepted");
}

void test_legacy_migration_power_loss() {
    const uint16_t versions[] = {
        ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION,
        ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION,
        ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION,
    };
    for (uint16_t version : versions) {
        const bool v1 = version == ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION;
        const bool v3 = version == ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION;
        AdapterConfiguration source{};
        source.pairing_window_seconds = 90;
        source.requested_mode = v1 ? AdapterRequestedMode::kAuto
                                   : AdapterRequestedMode::kXInput;
        if (v3) {
            source.native_switch_controller_count = 2;
            source.native_switch_controllers[0] = nintendo_identity(1);
            source.native_switch_controllers[1] = nintendo_identity(2, 0x2006);
        }
        uint8_t legacy[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
        require(adapter_configuration_encode(source, legacy, sizeof(legacy)),
                "legacy migration fixture did not encode");
        const size_t legacy_size =
            v3 ? ADAPTER_CONFIGURATION_ENCODED_SIZE
               : (v1 ? ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE
                     : ADAPTER_CONFIGURATION_V2_ENCODED_SIZE);
        FakeFlash flash;
        ConfigurationStorage store;
        require(store.initialize(fake_io(&flash)) &&
                    store.commit(version, legacy, legacy_size) ==
                        ConfigurationStorageResult::kOk,
                "legacy migration seed did not persist");
        AdapterConfiguration migrated{};
        uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
        require(adapter_configuration_decode(version, legacy, legacy_size,
                                             &migrated) &&
                    adapter_configuration_encode(migrated, payload,
                                                 sizeof(payload)),
                "legacy settings did not convert to schema4");
        flash.fail_after_programs = flash.successful_programs;
        require(store.commit(ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                             sizeof(payload)) ==
                    ConfigurationStorageResult::kIoError,
                "failed migration reported success");
        flash.fail_after_programs = -1;
        ConfigurationStorage recovered;
        require(recovered.initialize(fake_io(&flash)) &&
                    recovered.snapshot().schema_version == version &&
                    recovered.snapshot().payload_size == legacy_size &&
                    memcmp(recovered.snapshot().payload, legacy, legacy_size) == 0,
                "migration power loss destroyed the legacy settings");
        require(recovered.commit(ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                                 sizeof(payload)) ==
                    ConfigurationStorageResult::kOk,
                "migration retry failed");
        ConfigurationStorage rebooted;
        AdapterConfiguration decoded{};
        require(rebooted.initialize(fake_io(&flash)) &&
                    rebooted.snapshot().schema_version ==
                        ADAPTER_CONFIGURATION_SCHEMA_VERSION &&
                    adapter_configuration_decode(
                        rebooted.snapshot().schema_version,
                        rebooted.snapshot().payload,
                        rebooted.snapshot().payload_size, &decoded) &&
                    decoded.pairing_window_seconds == 90 &&
                    decoded.requested_mode == migrated.requested_mode &&
                    decoded.joycon_mode == JoyConMode::kPaired &&
                    decoded.native_switch_controller_count ==
                        source.native_switch_controller_count &&
                    (!v3 ||
                     (adapter_configuration_native_switch_approved(
                          decoded, source.native_switch_controllers[0]) &&
                      adapter_configuration_native_switch_approved(
                          decoded, source.native_switch_controllers[1]))),
                "migration retry changed settings or controller approvals");
    }
}

void test_two_copy_recovery() {
    FakeFlash flash;
    ConfigurationStorage store;
    require(store.initialize(fake_io(&flash)),
            "storage did not initialize");
    require(!store.snapshot().valid,
            "erased storage appeared valid");

    const uint8_t first[] = {1, 2, 3, 4};
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kOk &&
                store.snapshot().generation == 1,
            "first generation did not commit");
    const int programs_after_first = flash.successful_programs;
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kUnchanged &&
                flash.successful_programs == programs_after_first,
            "identical configuration consumed a flash write");

    const uint8_t second[] = {5, 6, 7, 8};
    require(store.commit(1, second, sizeof(second)) ==
                ConfigurationStorageResult::kOk &&
                store.snapshot().generation == 2,
            "second generation did not commit");

    ConfigurationStorage after_power_cycle;
    require(after_power_cycle.initialize(fake_io(&flash)) &&
                after_power_cycle.snapshot().generation == 2 &&
                memcmp(after_power_cycle.snapshot().payload, second,
                       sizeof(second)) == 0,
            "latest generation did not survive reinitialization");

    flash.bytes[1][CONFIGURATION_STORAGE_RECORD_HEADER_SIZE] ^= 0x01;
    ConfigurationStorage after_corruption;
    require(after_corruption.initialize(fake_io(&flash)) &&
                after_corruption.snapshot().generation == 1 &&
                memcmp(after_corruption.snapshot().payload, first,
                       sizeof(first)) == 0,
            "corrupt newest generation did not roll back");
}

void test_interrupted_write_retains_previous_generation() {
    FakeFlash flash;
    ConfigurationStorage store;
    require(store.initialize(fake_io(&flash)),
            "storage did not initialize for interruption test");
    const uint8_t first[] = {9, 8, 7, 6};
    require(store.commit(1, first, sizeof(first)) ==
                ConfigurationStorageResult::kOk,
            "baseline generation did not commit");

    uint8_t large[300];
    memset(large, 0x5a, sizeof(large));
    flash.fail_after_programs = flash.successful_programs + 1;
    require(store.commit(1, large, sizeof(large)) ==
                ConfigurationStorageResult::kIoError,
            "interrupted multi-page write reported success");

    flash.fail_after_programs = -1;
    ConfigurationStorage recovered;
    require(recovered.initialize(fake_io(&flash)) &&
                recovered.snapshot().generation == 1 &&
                recovered.snapshot().payload_size == sizeof(first) &&
                memcmp(recovered.snapshot().payload, first,
                       sizeof(first)) == 0,
            "interrupted write replaced the previous generation");
}

void test_transaction_validation() {
    AdapterConfiguration configuration{};
    configuration.pairing_window_seconds = 120;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload,
                                         sizeof(payload)),
            "transaction payload did not encode");
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));

    ConfigurationTransaction transaction;
    require(transaction.begin(7, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kReceiving,
            "valid transaction did not begin");
    require(transaction.append(7, 1, payload, 1) ==
                ConfigurationTransactionStatus::kOutOfOrder,
            "out-of-order chunk was accepted");

    require(transaction.begin(8, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc ^ 1u) ==
                ConfigurationTransactionStatus::kReceiving,
            "replacement transaction did not begin");
    require(transaction.append(8, 0, payload, 2) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(8, 2, &payload[2],
                                   sizeof(payload) - 2) ==
                    ConfigurationTransactionStatus::kReceiving,
            "ordered chunks were rejected");
    require(transaction.finish(8) ==
                ConfigurationTransactionStatus::kBadCrc,
            "bad transaction CRC was accepted");

    require(transaction.begin(9, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(9, 0, payload, sizeof(payload)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                transaction.finish(9) ==
                    ConfigurationTransactionStatus::kPending,
            "valid transaction did not reach pending commit");
    require(transaction.begin(10, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                              sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kBusy,
            "pending transaction was replaced");

    transaction.set_result(ConfigurationTransactionStatus::kCommitted,
                           4, crc);
    require(transaction.begin(11, 99, sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kUnsupportedSchema,
            "unsupported schema was accepted");
    require(transaction.begin(
                12, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                CONFIGURATION_STORAGE_MAX_PAYLOAD_SIZE + 1, crc) ==
                ConfigurationTransactionStatus::kTooLarge,
            "oversized transaction was accepted");
    require(transaction.begin(
                13, ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION,
                ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE, 0) ==
                ConfigurationTransactionStatus::kUnsupportedSchema &&
                transaction.begin(
                    14, ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION,
                    ADAPTER_CONFIGURATION_V2_ENCODED_SIZE, 0) ==
                    ConfigurationTransactionStatus::kUnsupportedSchema &&
                transaction.begin(
                    15, ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION,
                    ADAPTER_CONFIGURATION_ENCODED_SIZE, 0) ==
                    ConfigurationTransactionStatus::kUnsupportedSchema,
            "legacy host writes could silently erase stored settings");
    payload[3] = ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY + 1;
    require(transaction.begin(
                16, ADAPTER_CONFIGURATION_SCHEMA_VERSION, sizeof(payload),
                configuration_crc32(payload, sizeof(payload))) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(16, 0, payload, sizeof(payload)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                transaction.finish(16) ==
                    ConfigurationTransactionStatus::kMalformed,
            "valid CRC allowed a malformed approval list to reach storage");
    payload[3] = 0;
    payload[4] = 2;
    require(transaction.begin(
                17, ADAPTER_CONFIGURATION_SCHEMA_VERSION, sizeof(payload),
                configuration_crc32(payload, sizeof(payload))) ==
                ConfigurationTransactionStatus::kReceiving &&
                transaction.append(17, 0, payload, sizeof(payload)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                transaction.finish(17) ==
                    ConfigurationTransactionStatus::kMalformed,
            "valid CRC allowed an invalid Joy-Con2 mode to reach storage");
}

}  // namespace

int main() {
    test_schema_encoding();
    test_native_switch_approval_identity_and_canonical_encoding();
    test_native_switch_approval_rejects_invalid_records();
    test_legacy_migration_power_loss();
    test_two_copy_recovery();
    test_interrupted_write_retains_previous_generation();
    test_transaction_validation();
    return 0;
}
