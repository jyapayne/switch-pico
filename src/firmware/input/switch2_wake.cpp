#include "input/switch2_wake.h"

#include <string.h>

#include <btstack.h>

#if !defined(SWITCH2_WAKE_CONFIGURED)
#if __has_include("platform/pico/switch2_wake_config.h")
#include "platform/pico/switch2_wake_config.h"
#else
#define SWITCH2_WAKE_CONFIGURED 0
#endif
#endif

namespace {

constexpr uint32_t kRetryIntervalMs = 5;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr uint32_t kBurstDurationMs = 2000;
constexpr uint16_t kAdvertisingIntervalUnits = 0x0020;  // 20 ms + BLE delay.
constexpr uint8_t kAdvertisingTypeNonConnectable = 3;
constexpr uint16_t kWritePublicAddressOpcode = 0xfc01;

#if SWITCH2_WAKE_CONFIGURED
const uint8_t kWakeAddress[6] = SWITCH2_WAKE_SOURCE_ADDRESS_BYTES;
uint8_t kWakeAdvertisement[31] = SWITCH2_WAKE_ADVERTISEMENT_DATA_BYTES;
#endif

const hci_cmd_t kWritePublicAddress = {kWritePublicAddressOpcode, "B"};
const bd_addr_t kUnusedPeerAddress{};

enum class Phase : uint8_t {
    kDisabled,
    kIdle,
    kSetWakeAddress,
    kSetParameters,
    kSetData,
    kEnableAdvertising,
    kAdvertising,
    kDisableAdvertising,
    kRestoreAddress,
    kFailed,
};

Phase g_phase = Phase::kDisabled;
bd_addr_t g_original_address{};
btstack_packet_callback_registration_t g_event_registration{};
btstack_timer_source_t g_timer{};
uint16_t g_pending_opcode = 0;
uint32_t g_command_deadline_ms = 0;
uint32_t g_burst_deadline_ms = 0;
uint32_t g_accepted_requests = 0;
uint32_t g_completed_bursts = 0;
uint32_t g_failures = 0;
bool g_initialized = false;
bool g_configured = false;
bool g_timer_armed = false;
bool g_address_changed = false;
bool g_advertising = false;

bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

bool phase_needs_work() {
    return g_phase != Phase::kDisabled &&
           g_phase != Phase::kIdle &&
           g_phase != Phase::kFailed;
}

void cancel_task() {
    if (g_timer_armed) {
        btstack_run_loop_remove_timer(&g_timer);
        g_timer_armed = false;
    }
}

void schedule_task(uint32_t delay_ms) {
    cancel_task();
    btstack_run_loop_set_timer(&g_timer, delay_ms);
    btstack_run_loop_add_timer(&g_timer);
    g_timer_armed = true;
}

void schedule_for_phase(uint32_t now_ms) {
    if (g_pending_opcode != 0) {
        schedule_task(
            deadline_reached(now_ms, g_command_deadline_ms)
                ? 0
                : g_command_deadline_ms - now_ms);
    } else if (g_phase == Phase::kAdvertising) {
        schedule_task(
            deadline_reached(now_ms, g_burst_deadline_ms)
                ? 0
                : g_burst_deadline_ms - now_ms);
    } else if (phase_needs_work()) {
        schedule_task(kRetryIntervalMs);
    } else {
        cancel_task();
    }
}

#if SWITCH2_WAKE_CONFIGURED
bool configured_packet_valid() {
    static const uint8_t prefix[] = {
        0x02, 0x01, 0x06, 0x1b, 0xff, 0x53, 0x05,
        0x01, 0x00, 0x03, 0x7e, 0x05,
    };
    static const uint8_t suffix[] = {
        0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    uint8_t address_any = 0;
    uint8_t target_any = 0;
    for (uint8_t value : kWakeAddress) {
        address_any |= value;
    }
    for (size_t index = 17; index < 23; ++index) {
        target_any |= kWakeAdvertisement[index];
    }
    return address_any != 0 && target_any != 0 &&
           memcmp(kWakeAdvertisement, prefix, sizeof(prefix)) == 0 &&
           kWakeAdvertisement[14] == 0x00 &&
           kWakeAdvertisement[15] == 0x01 &&
           kWakeAdvertisement[16] == 0x81 &&
           memcmp(&kWakeAdvertisement[23], suffix, sizeof(suffix)) == 0;
}
#endif

void recover_from_failure() {
    ++g_failures;
    g_pending_opcode = 0;
    if (g_advertising) {
        g_phase = Phase::kDisableAdvertising;
    } else if (g_address_changed) {
        g_phase = Phase::kRestoreAddress;
    } else {
        g_phase = Phase::kIdle;
    }
}

void begin_command(uint16_t opcode, uint32_t now_ms) {
    g_pending_opcode = opcode;
    g_command_deadline_ms = now_ms + kCommandTimeoutMs;
}

void submit_phase_command(uint32_t now_ms) {
    if (g_pending_opcode != 0 || !hci_can_send_command_packet_now()) {
        return;
    }

    uint8_t result = ERROR_CODE_SUCCESS;
    switch (g_phase) {
        case Phase::kSetWakeAddress:
            begin_command(kWritePublicAddress.opcode, now_ms);
#if SWITCH2_WAKE_CONFIGURED
            result = hci_send_cmd(&kWritePublicAddress, kWakeAddress);
#endif
            break;
        case Phase::kSetParameters:
            begin_command(hci_le_set_advertising_parameters.opcode, now_ms);
            result = hci_send_cmd(
                &hci_le_set_advertising_parameters,
                kAdvertisingIntervalUnits, kAdvertisingIntervalUnits,
                kAdvertisingTypeNonConnectable,
                BD_ADDR_TYPE_LE_PUBLIC, BD_ADDR_TYPE_LE_PUBLIC,
                kUnusedPeerAddress, 7, 0);
            break;
        case Phase::kSetData:
            begin_command(hci_le_set_advertising_data.opcode, now_ms);
#if SWITCH2_WAKE_CONFIGURED
            result = hci_send_cmd(
                &hci_le_set_advertising_data,
                static_cast<uint8_t>(sizeof(kWakeAdvertisement)),
                kWakeAdvertisement);
#endif
            break;
        case Phase::kEnableAdvertising:
        case Phase::kDisableAdvertising:
            begin_command(hci_le_set_advertise_enable.opcode, now_ms);
            result = hci_send_cmd(
                &hci_le_set_advertise_enable,
                g_phase == Phase::kEnableAdvertising ? 1 : 0);
            break;
        case Phase::kRestoreAddress:
            begin_command(kWritePublicAddress.opcode, now_ms);
            result = hci_send_cmd(&kWritePublicAddress, g_original_address);
            break;
        default:
            return;
    }

    if (result != ERROR_CODE_SUCCESS) {
        recover_from_failure();
    }
}

void handle_command_complete(uint8_t* packet, uint16_t size) {
    if (size < 6 || g_pending_opcode == 0) {
        return;
    }
    const uint16_t opcode =
        hci_event_command_complete_get_command_opcode(packet);
    if (opcode != g_pending_opcode) {
        return;
    }
    const uint8_t status =
        hci_event_command_complete_get_return_parameters(packet)[0];
    g_pending_opcode = 0;
    if (status != ERROR_CODE_SUCCESS) {
        recover_from_failure();
        schedule_for_phase(btstack_run_loop_get_time_ms());
        return;
    }

    switch (g_phase) {
        case Phase::kSetWakeAddress:
            g_address_changed = true;
            g_phase = Phase::kSetParameters;
            break;
        case Phase::kSetParameters:
            g_phase = Phase::kSetData;
            break;
        case Phase::kSetData:
            g_phase = Phase::kEnableAdvertising;
            break;
        case Phase::kEnableAdvertising:
            g_advertising = true;
            g_burst_deadline_ms =
                btstack_run_loop_get_time_ms() + kBurstDurationMs;
            g_phase = Phase::kAdvertising;
            break;
        case Phase::kDisableAdvertising:
            g_advertising = false;
            g_phase = Phase::kRestoreAddress;
            break;
        case Phase::kRestoreAddress:
            g_address_changed = false;
            ++g_completed_bursts;
            g_phase = Phase::kIdle;
            break;
        default:
            recover_from_failure();
            break;
    }
    schedule_for_phase(btstack_run_loop_get_time_ms());
}

void handle_hci_event(uint8_t packet_type, uint16_t,
                      uint8_t* packet, uint16_t size) {
    if (packet_type == HCI_EVENT_PACKET && size >= 2 &&
        hci_event_packet_get_type(packet) == HCI_EVENT_COMMAND_COMPLETE) {
        handle_command_complete(packet, size);
    }
}

void task(btstack_timer_source_t*) {
    g_timer_armed = false;
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    if (g_pending_opcode != 0 &&
        deadline_reached(now_ms, g_command_deadline_ms)) {
        recover_from_failure();
    }
    if (g_phase == Phase::kAdvertising &&
        deadline_reached(now_ms, g_burst_deadline_ms)) {
        g_phase = Phase::kDisableAdvertising;
    }
    submit_phase_command(now_ms);
    schedule_for_phase(now_ms);
}

}  // namespace

void switch2_wake_initialize() {
    if (g_initialized) {
        return;
    }
    g_initialized = true;
#if SWITCH2_WAKE_CONFIGURED
    if (!configured_packet_valid()) {
        g_phase = Phase::kFailed;
        ++g_failures;
        return;
    }
    g_configured = true;
    gap_local_bd_addr(g_original_address);
    g_event_registration.callback = handle_hci_event;
    hci_add_event_handler(&g_event_registration);
    btstack_run_loop_set_timer_handler(&g_timer, task);
    g_phase = Phase::kIdle;
#endif
}


bool switch2_wake_request() {
    if (g_phase != Phase::kIdle) {
        return false;
    }
    ++g_accepted_requests;
    g_phase = Phase::kSetWakeAddress;
    schedule_task(0);
    return true;
}

void switch2_wake_diagnostics(Switch2WakeDiagnostics* out) {
    if (out == nullptr) {
        return;
    }
    *out = {
        g_configured,
        g_phase != Phase::kDisabled && g_phase != Phase::kIdle &&
            g_phase != Phase::kFailed,
        g_accepted_requests,
        g_completed_bursts,
        g_failures,
    };
}
