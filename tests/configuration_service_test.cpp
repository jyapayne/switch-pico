#include "configuration/adapter_configuration.h"
#include "configuration/configuration_service.h"
#include "configuration/configuration_storage.h"
#include "platform/pico/pico_configuration_storage.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

constexpr size_t kSectorSize = 4096;
constexpr size_t kPageSize = 256;

struct FakeFlash {
    uint8_t bytes[CONFIGURATION_STORAGE_COPY_COUNT][kSectorSize];
    int read_count = 0;
    int erase_count = 0;
    int program_count = 0;
    bool fail_program = false;

    FakeFlash() { memset(bytes, 0xff, sizeof(bytes)); }
};

FakeFlash g_flash;
bool g_reserve_recovery_during_program = false;

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
    ++flash->read_count;
    return true;
}

bool fake_erase(void* context, uint8_t copy) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT) {
        return false;
    }
    memset(flash->bytes[copy], 0xff, kSectorSize);
    ++flash->erase_count;
    return true;
}

bool fake_program(void* context, uint8_t copy, size_t offset,
                  const uint8_t* data, size_t size) {
    auto* flash = static_cast<FakeFlash*>(context);
    if (copy >= CONFIGURATION_STORAGE_COPY_COUNT || size != kPageSize ||
        offset + size > kSectorSize || flash->fail_program) {
        return false;
    }
    if (g_reserve_recovery_during_program) {
        g_reserve_recovery_during_program = false;
        configuration_service_reserve_for_recovery();
    }
    for (size_t index = 0; index < size; ++index) {
        flash->bytes[copy][offset + index] &= data[index];
    }
    ++flash->program_count;
    return true;
}

ConfigurationStorageIo fake_io() {
    return {
        &g_flash,
        kSectorSize,
        kPageSize,
        fake_read,
        fake_erase,
        fake_program,
    };
}

void require_mode_status(uint32_t transaction_id,
                         ConfigurationTransactionStatus expected,
                         const char* message) {
    ConfigurationTransactionStatus actual =
        ConfigurationTransactionStatus::kIdle;
    require(configuration_service_mode_transaction_status(transaction_id,
                                                          &actual) &&
                actual == expected,
            message);
}

}  // namespace

ConfigurationStorageIo pico_configuration_storage_io() {
    return fake_io();
}

namespace {

void seed_legacy_configuration(bool v2) {
    ConfigurationStorage seed;
    const uint8_t legacy[] = {
        90, 0, static_cast<uint8_t>(v2 ? AdapterRequestedMode::kXInput
                                     : AdapterRequestedMode::kAuto),
        0, 0, 0, 0, 0,
    };
    require(seed.initialize(fake_io()), "legacy seed storage init failed");
    require(seed.commit(
                v2 ? ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION
                   : ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION,
                legacy, v2 ? ADAPTER_CONFIGURATION_V2_ENCODED_SIZE
                           : ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE) ==
                ConfigurationStorageResult::kOk,
            "legacy seed commit failed");
}

void test_service_lifecycle_and_mutations(bool v2) {
    seed_legacy_configuration(v2);
    const AdapterRequestedMode original_mode =
        v2 ? AdapterRequestedMode::kXInput : AdapterRequestedMode::kAuto;
    const int programs_after_seed = g_flash.program_count;
    const int erases_after_seed = g_flash.erase_count;

    configuration_service_prepare();
    configuration_service_initialize_pre_usb();

    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    require(snapshot.state == ConfigurationServiceState::kReady &&
                snapshot.configuration.pairing_window_seconds == 90 &&
                snapshot.configuration.requested_mode == original_mode &&
                snapshot.configuration.joycon_mode == JoyConMode::kPaired &&
                snapshot.configuration.native_switch_controller_count == 0,
            "Core 0 did not preserve legacy settings without approval");
    require(g_flash.program_count == programs_after_seed &&
                g_flash.erase_count == erases_after_seed,
            "pre-USB initialization wrote flash");

    const AdapterModeAvailability implemented{true, true, true, true};
    require(configuration_service_set_mode(
                1, AdapterRequestedMode::kSwitch, implemented) ==
                ConfigurationTransactionStatus::kBusy,
            "host mutation was accepted before storage-core adoption");

    const int reads_before_adoption = g_flash.read_count;
    configuration_service_initialize_on_storage_core();
    require(g_flash.read_count == reads_before_adoption &&
                g_flash.program_count == programs_after_seed &&
                g_flash.erase_count == erases_after_seed,
            "Core 1 reread or wrote instead of adopting Core 0 state");
    require(configuration_service_set_mode(
                2, AdapterRequestedMode::kSwitch, implemented) ==
                ConfigurationTransactionStatus::kBusy,
            "host mutation displaced the pending legacy migration");

    configuration_service_task_on_storage_core(0);
    configuration_service_snapshot(&snapshot);
    require(snapshot.state == ConfigurationServiceState::kReady &&
                snapshot.configuration.requested_mode == original_mode &&
                snapshot.configuration.joycon_mode == JoyConMode::kPaired &&
                snapshot.configuration.native_switch_controller_count == 0 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kIdle,
            "legacy migration changed configuration or host transaction state");

    ConfigurationStorage after_migration;
    require(after_migration.initialize(fake_io()) &&
                after_migration.snapshot().valid &&
                after_migration.snapshot().schema_version ==
                    ADAPTER_CONFIGURATION_SCHEMA_VERSION &&
                after_migration.snapshot().payload_size ==
                    ADAPTER_CONFIGURATION_ENCODED_SIZE,
            "power cycle did not observe the migrated v4 record");
    AdapterConfiguration migrated{};
    require(adapter_configuration_decode(
                after_migration.snapshot().schema_version,
                after_migration.snapshot().payload,
                after_migration.snapshot().payload_size, &migrated) &&
                migrated.pairing_window_seconds == 90 &&
                migrated.requested_mode == original_mode &&
                migrated.joycon_mode == JoyConMode::kPaired &&
                migrated.native_switch_controller_count == 0,
            "migrated v4 bytes did not preserve legacy configuration");

    require(configuration_service_set_mode(
                10, AdapterRequestedMode::kSwitch, implemented) ==
                ConfigurationTransactionStatus::kPending,
            "host mode transaction was not queued");
    require_mode_status(10, ConfigurationTransactionStatus::kPending,
                        "host mode correlation was not pending");
    require(configuration_service_set_mode_internal(
                0x80000001u, AdapterRequestedMode::kXInput, implemented) ==
                ConfigurationTransactionStatus::kBusy,
            "internal mode displaced a pending host mode");
    configuration_service_task_on_storage_core(999);
    require_mode_status(10, ConfigurationTransactionStatus::kPending,
                        "write-rate guard committed host mode too early");
    configuration_service_task_on_storage_core(1000);
    require_mode_status(10, ConfigurationTransactionStatus::kCommitted,
                        "host mode transaction did not commit");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds == 90 &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kSwitch &&
                snapshot.transaction.transaction_id == 10 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted,
            "mode-only commit changed pairing state or host status");

    const int programs_before_rejections = g_flash.program_count;
    const AdapterModeAvailability unavailable{true, true, false, false};
    require(configuration_service_set_mode(
                11, AdapterRequestedMode::kDInput, unavailable) ==
                ConfigurationTransactionStatus::kUnsupportedSchema &&
                configuration_service_set_mode(
                    11, static_cast<AdapterRequestedMode>(5), implemented) ==
                    ConfigurationTransactionStatus::kMalformed &&
                configuration_service_set_mode(
                    0x80000011u, AdapterRequestedMode::kXInput,
                    implemented) ==
                    ConfigurationTransactionStatus::kMalformed &&
                configuration_service_begin(
                    0x80000012u, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                    ADAPTER_CONFIGURATION_ENCODED_SIZE, 0) ==
                    ConfigurationTransactionStatus::kMalformed &&
                configuration_service_reset(0x80000013u) ==
                    ConfigurationTransactionStatus::kMalformed,
            "unavailable, invalid, or internal-range host request was accepted");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.requested_mode ==
                AdapterRequestedMode::kSwitch &&
                snapshot.transaction.transaction_id == 10 &&
                g_flash.program_count == programs_before_rejections,
            "rejected mode request corrupted persisted or host-visible state");

    require(configuration_service_set_mode(
                12, AdapterRequestedMode::kSwitch, implemented) ==
                ConfigurationTransactionStatus::kPending,
            "unchanged host mode was not serialized");
    const int programs_before_unchanged = g_flash.program_count;
    configuration_service_task_on_storage_core(2000);
    require_mode_status(12, ConfigurationTransactionStatus::kUnchanged,
                        "unchanged host mode did not terminate unchanged");
    require(g_flash.program_count == programs_before_unchanged,
            "unchanged mode consumed a flash program");
    require(configuration_service_mode_transaction_reboot_ready(12),
            "current unchanged host mode could not authorize reboot");

    constexpr uint32_t kInternalXInput = 0x80000020u;
    require(configuration_service_set_mode_internal(
                kInternalXInput, AdapterRequestedMode::kXInput,
                implemented) == ConfigurationTransactionStatus::kPending &&
                configuration_service_set_mode_internal(
                    kInternalXInput, AdapterRequestedMode::kXInput,
                    implemented) == ConfigurationTransactionStatus::kPending,
            "accepted internal mode was not idempotently pending");
    require(!configuration_service_mode_transaction_reboot_ready(12),
            "accepted internal mode did not immediately supersede host reboot");
    configuration_service_snapshot(&snapshot);
    const ConfigurationTransactionSnapshot preserved_host =
        snapshot.transaction;
    require(configuration_service_begin(
                20, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                ADAPTER_CONFIGURATION_ENCODED_SIZE, 0) ==
                ConfigurationTransactionStatus::kBusy &&
                configuration_service_reset(21) ==
                    ConfigurationTransactionStatus::kBusy,
            "host mutation was not blocked by pending internal mode");
    configuration_service_task_on_storage_core(2000);
    require_mode_status(12, ConfigurationTransactionStatus::kUnchanged,
                        "internal commit overwrote retained host status");
    require_mode_status(kInternalXInput,
                        ConfigurationTransactionStatus::kCommitted,
                        "internal mode transaction did not commit");
    require(!configuration_service_mode_transaction_reboot_ready(12) &&
                configuration_service_mode_transaction_reboot_ready(
                    kInternalXInput),
            "stale host generation authorized reboot after internal commit");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds == 90 &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kXInput &&
                snapshot.transaction.transaction_id ==
                    preserved_host.transaction_id &&
                snapshot.transaction.status == preserved_host.status &&
                snapshot.mode_transaction.transaction_id == kInternalXInput &&
                snapshot.mode_transaction.stored_generation ==
                    snapshot.generation,
            "internal mode changed host status or was not published");
    require(configuration_service_set_mode(
                13, AdapterRequestedMode::kAuto, implemented) ==
                ConfigurationTransactionStatus::kPending,
            "host mode did not serialize after internal mode");
    require(!configuration_service_mode_transaction_reboot_ready(
                kInternalXInput),
            "accepted host mode did not immediately supersede internal reboot");
    configuration_service_task_on_storage_core(3000);
    require_mode_status(kInternalXInput,
                        ConfigurationTransactionStatus::kCommitted,
                        "host commit overwrote retained internal status");
    require_mode_status(13, ConfigurationTransactionStatus::kCommitted,
                        "interleaved host mode transaction did not commit");
    require(!configuration_service_mode_transaction_reboot_ready(
                kInternalXInput) &&
                configuration_service_mode_transaction_reboot_ready(13),
            "stale internal generation authorized reboot after host commit");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.requested_mode ==
                AdapterRequestedMode::kAuto &&
                snapshot.mode_transaction.transaction_id == 13 &&
                !snapshot.mode_transaction.internal &&
                snapshot.mode_transaction.stored_generation ==
                    snapshot.generation,
            "latest host mode transaction was not published");

    AdapterConfiguration generic = snapshot.configuration;
    generic.pairing_window_seconds = 120;
    uint8_t generic_payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(generic, generic_payload,
                                         sizeof(generic_payload)),
            "generic test configuration did not encode");
    const uint32_t generic_crc =
        configuration_crc32(generic_payload, sizeof(generic_payload));
    require(configuration_service_begin(
                30, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                sizeof(generic_payload), generic_crc) ==
                ConfigurationTransactionStatus::kReceiving,
            "generic host transaction did not begin");
    require(!configuration_service_mode_transaction_reboot_ready(13),
            "accepted generic configuration did not invalidate mode reboot");
    require(configuration_service_set_mode_internal(
                0x80000021u, AdapterRequestedMode::kAuto, implemented) ==
                ConfigurationTransactionStatus::kBusy,
            "internal mode displaced receiving host configuration");
    require(configuration_service_append(
                30, 0, generic_payload, sizeof(generic_payload)) ==
                ConfigurationTransactionStatus::kReceiving &&
                configuration_service_commit(30) ==
                    ConfigurationTransactionStatus::kPending,
            "generic host configuration did not reach pending");
    configuration_service_task_on_storage_core(4000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds == 120 &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kAuto &&
                snapshot.transaction.transaction_id == 30 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted,
            "generic host commit did not preserve requested mode");
    require_mode_status(13, ConfigurationTransactionStatus::kCommitted,
                        "generic commit erased retained host mode status");
    require(!configuration_service_mode_transaction_reboot_ready(13),
            "host mode authorized reboot for a stale stored generation");
    ConfigurationTransactionStatus ignored{};
    require(!configuration_service_mode_transaction_status(30, &ignored),
            "generic transaction was misidentified as a mode transaction");

    constexpr uint32_t kFailingInternal = 0x80000022u;
    require(configuration_service_set_mode_internal(
                kFailingInternal, AdapterRequestedMode::kXInput,
                implemented) == ConfigurationTransactionStatus::kPending,
            "failing internal mode was not queued");
    g_flash.fail_program = true;
    configuration_service_task_on_storage_core(5000);
    g_flash.fail_program = false;
    require_mode_status(kFailingInternal,
                        ConfigurationTransactionStatus::kStorageError,
                        "failed internal mode did not report storage error");
    configuration_service_snapshot(&snapshot);
    require(snapshot.state == ConfigurationServiceState::kReady &&
                snapshot.configuration.pairing_window_seconds == 120 &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kAuto &&
                snapshot.transaction.transaction_id == 30 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted &&
                !configuration_service_mode_transaction_reboot_ready(
                    kFailingInternal),
            "failed internal mode did not roll back coherently");

    const uint32_t reset_generation_before =
        configuration_service_reset_generation();
    require(configuration_service_reset(40) ==
                ConfigurationTransactionStatus::kPending &&
                configuration_service_reset_generation() ==
                    reset_generation_before + 1,
            "configuration reset acceptance semantics changed");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds == 120 &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kAuto,
            "reset published defaults before durable commit");
    configuration_service_task_on_storage_core(6000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds ==
                ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kAuto &&
                snapshot.transaction.transaction_id == 40 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kCommitted &&
                snapshot.reset_generation == reset_generation_before + 1,
            "configuration reset did not durably restore v3 defaults");

    constexpr uint32_t kCapturedHostMode = 50;
    require(configuration_service_set_mode(
                kCapturedHostMode, AdapterRequestedMode::kSwitch,
                implemented) == ConfigurationTransactionStatus::kPending,
            "host mode was not queued for the storage completion race");
    g_reserve_recovery_during_program = true;
    configuration_service_task_on_storage_core(7000);
    require_mode_status(
        kCapturedHostMode, ConfigurationTransactionStatus::kBusy,
        "in-flight host mode completion revived its canceled status");
    configuration_service_snapshot(&snapshot);
    require(snapshot.transaction.transaction_id == kCapturedHostMode &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kBusy &&
                snapshot.configuration.requested_mode ==
                    AdapterRequestedMode::kSwitch &&
                !configuration_service_mode_transaction_reboot_ready(
                    kCapturedHostMode),
            "captured host completion escaped recovery ownership");

    const uint8_t blocked_payload = 0;
    require(configuration_service_append(
                kCapturedHostMode, 0, &blocked_payload,
                sizeof(blocked_payload)) ==
                ConfigurationTransactionStatus::kBusy &&
                configuration_service_commit(kCapturedHostMode) ==
                    ConfigurationTransactionStatus::kBusy &&
                configuration_service_begin(
                    51, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                    ADAPTER_CONFIGURATION_ENCODED_SIZE, 0) ==
                    ConfigurationTransactionStatus::kBusy &&
                configuration_service_reset(52) ==
                    ConfigurationTransactionStatus::kBusy &&
                configuration_service_set_mode(
                    53, AdapterRequestedMode::kSwitch, implemented) ==
                    ConfigurationTransactionStatus::kBusy,
            "host mutation escaped recovery reservation");
    require(configuration_service_set_mode_internal(
                0x80000030u, AdapterRequestedMode::kSwitch,
                implemented) == ConfigurationTransactionStatus::kBusy,
            "non-Auto internal mode escaped recovery reservation");

    constexpr uint32_t kRecoveryAuto = 0x80000031u;
    require(configuration_service_set_mode_internal(
                kRecoveryAuto, AdapterRequestedMode::kAuto,
                implemented) == ConfigurationTransactionStatus::kPending &&
                !configuration_service_mode_transaction_reboot_ready(
                    kRecoveryAuto),
            "recovery Auto was blocked or authorized before persistence");
    configuration_service_task_on_storage_core(8000);
    require_mode_status(kRecoveryAuto,
                        ConfigurationTransactionStatus::kCommitted,
                        "recovery Auto did not overwrite canceled host mode");
    require(configuration_service_mode_transaction_reboot_ready(
                kRecoveryAuto),
            "latest recovery Auto did not authorize reboot");

    constexpr uint32_t kLatestRecoveryAuto = 0x80000032u;
    require(configuration_service_set_mode_internal(
                kLatestRecoveryAuto, AdapterRequestedMode::kAuto,
                implemented) == ConfigurationTransactionStatus::kPending &&
                !configuration_service_mode_transaction_reboot_ready(
                    kRecoveryAuto),
            "newer recovery Auto did not immediately supersede reboot");
    configuration_service_task_on_storage_core(9000);
    require(!configuration_service_mode_transaction_reboot_ready(
                kRecoveryAuto) &&
                configuration_service_mode_transaction_reboot_ready(
                    kLatestRecoveryAuto),
            "recovery reboot authorization did not remain latest-only");
}

void test_abandoned_host_receive_does_not_block_recovery() {
    AdapterConfiguration original{};
    original.pairing_window_seconds = 90;
    original.requested_mode = AdapterRequestedMode::kSwitch;
    original.joycon_mode = JoyConMode::kIndividual;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    ConfigurationStorage seed;
    require(adapter_configuration_encode(original, payload, sizeof(payload)) &&
                seed.initialize(fake_io()) &&
                seed.commit(ADAPTER_CONFIGURATION_SCHEMA_VERSION, payload,
                            sizeof(payload)) == ConfigurationStorageResult::kOk,
            "recovery configuration fixture did not persist");
    configuration_service_prepare();
    configuration_service_initialize_pre_usb();
    configuration_service_initialize_on_storage_core();

    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    AdapterConfiguration abandoned = snapshot.configuration;
    abandoned.pairing_window_seconds = 180;
    abandoned.joycon_mode = JoyConMode::kPaired;
    require(adapter_configuration_encode(abandoned, payload,
                                         sizeof(payload)),
            "abandoned recovery-race configuration did not encode");
    const uint32_t crc = configuration_crc32(payload, sizeof(payload));
    require(configuration_service_begin(
                60, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                sizeof(payload), crc) ==
                ConfigurationTransactionStatus::kReceiving,
            "abandoned host configuration did not begin receiving");

    configuration_service_reserve_for_recovery();
    configuration_service_reserve_for_recovery();
    configuration_service_snapshot(&snapshot);
    require(snapshot.transaction.transaction_id == 60 &&
                snapshot.transaction.status ==
                    ConfigurationTransactionStatus::kBusy,
            "recovery reservation did not terminally cancel host receive");

    const AdapterModeAvailability implemented{true, true, true, true};
    constexpr uint32_t kRecoveryAuto = 0x80000040u;
    require(configuration_service_set_mode_internal(
                kRecoveryAuto, AdapterRequestedMode::kAuto,
                implemented) == ConfigurationTransactionStatus::kPending,
            "abandoned host receive blocked recovery Auto");
    configuration_service_task_on_storage_core(0);
    require_mode_status(kRecoveryAuto,
                        ConfigurationTransactionStatus::kCommitted,
                        "recovery Auto did not commit after abandoned receive");
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.requested_mode == AdapterRequestedMode::kAuto &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                snapshot.configuration.pairing_window_seconds == 90,
            "recovery Auto replaced durable settings with defaults or abandoned data");
    require(configuration_service_mode_transaction_reboot_ready(
                kRecoveryAuto),
            "recovery Auto did not become latest reboot authority");
}

void queue_configuration(uint32_t transaction_id,
                         const AdapterConfiguration& configuration) {
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)),
            "service approval fixture did not encode");
    require(configuration_service_begin(
                transaction_id, ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                sizeof(payload), configuration_crc32(payload, sizeof(payload))) ==
                ConfigurationTransactionStatus::kReceiving &&
                configuration_service_append(transaction_id, 0, payload,
                                             sizeof(payload)) ==
                    ConfigurationTransactionStatus::kReceiving &&
                configuration_service_commit(transaction_id) ==
                    ConfigurationTransactionStatus::kPending,
            "approval configuration did not reach pending commit");
}

void test_native_switch_approval_preservation_and_revocation(bool migrate_v3) {
    ControllerIdentity pro{};
    pro.stable = true;
    pro.transport = ControllerTransport::kClassic;
    pro.address[0] = 0x02;
    pro.address[5] = 1;
    pro.vendor_id = 0x057e;
    pro.product_id = 0x2009;
    ControllerIdentity left = pro;
    left.address[5] = 2;
    left.product_id = 0x2006;
    AdapterConfiguration configuration{};
    configuration.pairing_window_seconds = 90;
    configuration.joycon_mode =
        migrate_v3 ? JoyConMode::kPaired : JoyConMode::kIndividual;
    configuration.native_switch_controller_count = 2;
    configuration.native_switch_controllers[0] = pro;
    configuration.native_switch_controllers[1] = left;
    uint8_t payload[ADAPTER_CONFIGURATION_ENCODED_SIZE]{};
    ConfigurationStorage seed;
    require(adapter_configuration_encode(configuration, payload, sizeof(payload)) &&
                seed.initialize(fake_io()) &&
                seed.commit(migrate_v3 ? ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION
                                       : ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                            payload, sizeof(payload)) ==
                    ConfigurationStorageResult::kOk,
            "approved controller configuration did not persist");
    configuration_service_prepare();
    configuration_service_initialize_pre_usb();
    ConfigurationServiceSnapshot snapshot{};
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.joycon_mode == configuration.joycon_mode &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro) &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "pre-USB snapshot did not expose persisted approvals");
    configuration_service_initialize_on_storage_core();
    if (migrate_v3) {
        configuration_service_task_on_storage_core(0);
        ConfigurationStorage migrated;
        AdapterConfiguration decoded{};
        require(migrated.initialize(fake_io()) &&
                    migrated.snapshot().schema_version ==
                        ADAPTER_CONFIGURATION_SCHEMA_VERSION &&
                    adapter_configuration_decode(
                        migrated.snapshot().schema_version,
                        migrated.snapshot().payload,
                        migrated.snapshot().payload_size, &decoded) &&
                    decoded.joycon_mode == JoyConMode::kPaired &&
                    decoded.pairing_window_seconds == 90 &&
                    adapter_configuration_native_switch_approved(decoded, pro) &&
                    adapter_configuration_native_switch_approved(decoded, left),
                "v3 service migration changed settings or approvals");
        configuration.joycon_mode = JoyConMode::kIndividual;
        queue_configuration(100, configuration);
        configuration_service_snapshot(&snapshot);
        require(snapshot.configuration.joycon_mode == JoyConMode::kPaired,
                "player mode changed before durable commit");
        configuration_service_task_on_storage_core(1000);
        configuration_service_snapshot(&snapshot);
        require(snapshot.configuration.joycon_mode == JoyConMode::kIndividual,
                "committed Individual mode was not published");
    }
    const uint32_t start_ms = migrate_v3 ? 2000 : 0;

    const AdapterModeAvailability implemented{true, true, true, true};
    require(configuration_service_set_mode(
                1, AdapterRequestedMode::kSwitch, implemented) ==
                ConfigurationTransactionStatus::kPending,
            "approved configuration blocked host mode selection");
    configuration_service_task_on_storage_core(start_ms);
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.requested_mode == AdapterRequestedMode::kSwitch &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro) &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "host mode selection erased approval");
    require(configuration_service_set_mode_internal(
                0x80000002u, AdapterRequestedMode::kXInput, implemented) ==
                ConfigurationTransactionStatus::kPending,
            "approved configuration blocked internal mode selection");
    configuration_service_task_on_storage_core(start_ms + 1000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.requested_mode == AdapterRequestedMode::kXInput &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro) &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "internal mode selection erased approval");

    configuration = snapshot.configuration;
    configuration.pairing_window_seconds = 120;
    queue_configuration(3, configuration);
    configuration_service_task_on_storage_core(start_ms + 2000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.configuration.pairing_window_seconds == 120 &&
                snapshot.configuration.requested_mode == AdapterRequestedMode::kXInput &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro) &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "pairing-window update erased approvals or requested mode");
    const uint32_t approved_generation = snapshot.generation;
    const uint32_t approved_crc = snapshot.payload_crc;

    configuration = snapshot.configuration;
    configuration.native_switch_controller_count = 1;
    configuration.native_switch_controllers[0] = left;
    queue_configuration(4, configuration);
    configuration_service_snapshot(&snapshot);
    require(adapter_configuration_native_switch_approved(
                snapshot.configuration, pro),
            "approval was revoked before durable commit");
    g_flash.fail_program = true;
    configuration_service_task_on_storage_core(start_ms + 3000);
    g_flash.fail_program = false;
    configuration_service_snapshot(&snapshot);
    require(snapshot.transaction.status ==
                ConfigurationTransactionStatus::kStorageError &&
                snapshot.generation == approved_generation &&
                snapshot.payload_crc == approved_crc &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro),
            "failed revocation changed the published durable configuration");
    ConfigurationStorage after_failure;
    AdapterConfiguration recovered{};
    require(after_failure.initialize(fake_io()) &&
                adapter_configuration_decode(
                    after_failure.snapshot().schema_version,
                    after_failure.snapshot().payload,
                    after_failure.snapshot().payload_size, &recovered) &&
                recovered.joycon_mode == JoyConMode::kIndividual &&
                adapter_configuration_native_switch_approved(recovered, pro) &&
                adapter_configuration_native_switch_approved(recovered, left),
            "interrupted revocation destroyed persisted approvals");

    queue_configuration(5, configuration);
    configuration_service_task_on_storage_core(start_ms + 3000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.transaction.status ==
                ConfigurationTransactionStatus::kCommitted &&
                snapshot.configuration.pairing_window_seconds == 120 &&
                snapshot.configuration.requested_mode == AdapterRequestedMode::kXInput &&
                snapshot.configuration.joycon_mode == JoyConMode::kIndividual &&
                !adapter_configuration_native_switch_approved(
                    snapshot.configuration, pro) &&
                adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "successful revocation did not preserve other approval and settings");
    ConfigurationStorage after_revocation;
    require(after_revocation.initialize(fake_io()) &&
                adapter_configuration_decode(
                    after_revocation.snapshot().schema_version,
                    after_revocation.snapshot().payload,
                    after_revocation.snapshot().payload_size, &recovered) &&
                recovered.joycon_mode == JoyConMode::kIndividual &&
                !adapter_configuration_native_switch_approved(recovered, pro) &&
                adapter_configuration_native_switch_approved(recovered, left),
            "controller-specific revocation did not survive power cycle");

    require(configuration_service_reset(6) ==
                ConfigurationTransactionStatus::kPending,
            "approved configuration reset was not queued");
    configuration_service_snapshot(&snapshot);
    require(adapter_configuration_native_switch_approved(
                snapshot.configuration, left),
            "reset erased approval before commit");
    configuration_service_task_on_storage_core(start_ms + 4000);
    configuration_service_snapshot(&snapshot);
    require(snapshot.transaction.status == ConfigurationTransactionStatus::kCommitted &&
                snapshot.configuration.joycon_mode == JoyConMode::kPaired &&
                !adapter_configuration_native_switch_approved(
                    snapshot.configuration, left),
            "configuration reset did not revoke persisted approval");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || strcmp(argv[1], "lifecycle") == 0) {
        test_service_lifecycle_and_mutations(false);
    } else if (strcmp(argv[1], "v2-migration") == 0) {
        test_service_lifecycle_and_mutations(true);
    } else if (strcmp(argv[1], "native-approvals") == 0) {
        test_native_switch_approval_preservation_and_revocation(false);
    } else if (strcmp(argv[1], "v3-migration") == 0) {
        test_native_switch_approval_preservation_and_revocation(true);
    } else {
        require(strcmp(argv[1], "abandoned-receive") == 0,
                "unknown service test scenario");
        test_abandoned_host_receive_does_not_block_recovery();
    }
    return 0;
}
