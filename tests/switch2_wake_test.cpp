#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <btstack.h>
#include "bluetooth_transport_config.h"

namespace {

struct SubmittedCommand {
    uint16_t opcode;
    uint8_t address[6];
    uint16_t interval_min;
    uint16_t interval_max;
    uint8_t advertising_type;
    uint8_t own_address_type;
    uint8_t channel_map;
    uint8_t filter_policy;
    uint8_t data[31];
    uint8_t data_length;
    uint8_t enabled;
};

uint32_t now_ms;
bd_addr_t original_address = {0xA8, 0x59, 0x5F, 0x9E, 0x37, 0x65};
btstack_packet_handler_t event_handler;
btstack_timer_source_t* installed_timer;
SubmittedCommand submitted[16]{};
size_t submitted_count;
bool command_available = true;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

}  // namespace

const hci_cmd_t hci_read_bd_addr = {0x1009, "read-address"};
const hci_cmd_t hci_le_set_advertising_parameters = {0x2006, "params"};
const hci_cmd_t hci_le_set_advertising_data = {0x2008, "data"};
const hci_cmd_t hci_le_set_advertise_enable = {0x200a, "enable"};

bool hci_can_send_command_packet_now() {
    return command_available;
}

uint8_t hci_send_cmd(const hci_cmd_t* command, ...) {
    require(command_available, "submitted without an HCI command credit");
    require(submitted_count < 16, "command capture overflow");
    command_available = false;
    SubmittedCommand& output = submitted[submitted_count++];
    output.opcode = command->opcode;
    va_list arguments;
    va_start(arguments, command);
    if (command->opcode == 0xfc01) {
        const uint8_t* address = va_arg(arguments, const uint8_t*);
        memcpy(output.address, address, sizeof(output.address));
    } else if (command->opcode == 0x2006) {
        output.interval_min = static_cast<uint16_t>(va_arg(arguments, int));
        output.interval_max = static_cast<uint16_t>(va_arg(arguments, int));
        output.advertising_type = static_cast<uint8_t>(va_arg(arguments, int));
        output.own_address_type = static_cast<uint8_t>(va_arg(arguments, int));
        (void)va_arg(arguments, int);
        (void)va_arg(arguments, const uint8_t*);
        output.channel_map = static_cast<uint8_t>(va_arg(arguments, int));
        output.filter_policy = static_cast<uint8_t>(va_arg(arguments, int));
    } else if (command->opcode == 0x2008) {
        output.data_length = static_cast<uint8_t>(va_arg(arguments, int));
        const uint8_t* data = va_arg(arguments, const uint8_t*);
        memcpy(output.data, data, output.data_length);
    } else if (command->opcode == 0x200a) {
        output.enabled = static_cast<uint8_t>(va_arg(arguments, int));
    }
    va_end(arguments);
    return ERROR_CODE_SUCCESS;
}

void hci_add_event_handler(
    btstack_packet_callback_registration_t* registration) {
    event_handler = registration->callback;
}

void gap_local_bd_addr(bd_addr_t output) {
    memcpy(output, original_address, sizeof(original_address));
}

uint32_t btstack_run_loop_get_time_ms() {
    return now_ms;
}

void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer,
    void (*handler)(btstack_timer_source_t*)) {
    timer->handler = handler;
    installed_timer = timer;
}

void btstack_run_loop_set_timer(
    btstack_timer_source_t* timer, uint32_t timeout_ms) {
    timer->timeout_ms = timeout_ms;
}

void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    ++timer->add_count;
}
int btstack_run_loop_remove_timer(btstack_timer_source_t*) {
    return 1;
}


#define SWITCH2_WAKE_CONFIGURED 1
#define SWITCH2_WAKE_SOURCE_ADDRESS_BYTES \
    {0x98, 0xE2, 0x55, 0x07, 0xDF, 0x00}
#define SWITCH2_WAKE_ADVERTISEMENT_DATA_BYTES \
    {0x02, 0x01, 0x06, 0x1B, 0xFF, 0x53, 0x05, 0x01, 0x00, 0x03, 0x7E, \
     0x05, 0x66, 0x20, 0x00, 0x01, 0x81, 0xBD, 0xD6, 0xF7, 0xEB, 0xF1, \
     0x48, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#include "input/switch2_wake.cpp"

namespace {

void complete(uint16_t opcode, uint8_t status = 0,
              const uint8_t* address = nullptr) {
    uint8_t event[12] = {
        HCI_EVENT_COMMAND_COMPLETE,
        static_cast<uint8_t>(address == nullptr ? 4 : 10),
        1,
        static_cast<uint8_t>(opcode), static_cast<uint8_t>(opcode >> 8),
        status,
    };
    if (address != nullptr) {
        for (size_t index = 0; index < 6; ++index) {
            event[6 + index] = address[5 - index];
        }
    }
    command_available = true;
    event_handler(
        HCI_EVENT_PACKET, 0, event,
        static_cast<uint16_t>(address == nullptr ? 6 : 12));
}

void run_task() {
    require(installed_timer != nullptr && installed_timer->handler != nullptr,
            "wake timer was not installed");
    installed_timer->handler(installed_timer);
}

void require_opcode(size_t index, uint16_t opcode) {
    require(index < submitted_count && submitted[index].opcode == opcode,
            "unexpected HCI command sequence");
}

void test_stable_identity_wake() {
    switch2_wake_initialize();
    Switch2WakeDiagnostics diagnostics{};
    switch2_wake_diagnostics(&diagnostics);
    require(diagnostics.configured && diagnostics.busy &&
                !switch2_wake_ready_for_connections() &&
                !switch2_wake_request(),
            "controller connections or wake escaped startup identity setup");

    run_task();
    require_opcode(0, 0xfc01);
    const uint8_t wake_address[] = {0x98, 0xE2, 0x55, 0x07, 0xDF, 0x00};
    require(memcmp(submitted[0].address, wake_address, 6) == 0,
            "stable wake address did not reach the radio command");
    complete(0xfc01);
    run_task();
    require_opcode(1, 0x1009);
    complete(0x1009, 0, wake_address);
    require(switch2_wake_ready_for_connections() && !g_timer_armed,
            "verified stable identity did not admit connections and go idle");

    switch2_wake_diagnostics(&diagnostics);
    require(!diagnostics.busy && switch2_wake_request() &&
                !switch2_wake_request(),
            "wake requests were not bounded while busy");
    run_task();
    require_opcode(2, 0x2006);
    require(submitted[2].interval_min == 0x20 &&
                submitted[2].interval_max == 0x20 &&
                submitted[2].advertising_type == 3 &&
                submitted[2].own_address_type == 0 &&
                submitted[2].channel_map == 7 &&
                submitted[2].filter_policy == 0,
            "known-working advertising parameters changed");
    complete(0x2006);

    run_task();
    require_opcode(3, 0x2008);
    require(submitted[3].data_length == 31 &&
                submitted[3].data[16] == 0x81,
            "captured wake payload was not submitted intact");
    complete(0x2008);

    run_task();
    require_opcode(4, 0x200a);
    require(submitted[4].enabled == 1,
            "wake advertising was not enabled");
    complete(0x200a);
    require(g_timer_armed && installed_timer->timeout_ms == 2000,
            "wake burst did not schedule one exact stop deadline");
    now_ms = 1999;
    run_task();
    require(submitted_count == 5,
            "wake burst stopped before two seconds");
    now_ms = 2000;
    run_task();
    require_opcode(5, 0x200a);
    require(submitted[5].enabled == 0,
            "wake advertising was not disabled");
    complete(0x200a);

    switch2_wake_diagnostics(&diagnostics);
    require(!diagnostics.busy && diagnostics.accepted_requests == 1 &&
                diagnostics.completed_bursts == 1 &&
                diagnostics.failures == 0 && !g_timer_armed,
            "completed wake did not return to a dormant idle state");
    require(submitted_count == 6,
            "wake burst changed the public identity after startup");
}

void test_failed_wake_keeps_stable_identity() {
    require(switch2_wake_request(),
            "idle module rejected a second wake request");
    run_task();
    require_opcode(6, 0x2006);
    complete(0x2006, 0x12);
    run_task();
    require(submitted_count == 7,
            "wake setup failure issued an address reset");

    Switch2WakeDiagnostics diagnostics{};
    switch2_wake_diagnostics(&diagnostics);
    require(!diagnostics.busy && diagnostics.failures == 1 &&
                switch2_wake_ready_for_connections(),
            "wake failure disrupted the stable controller identity");
}

void test_disabled_transport_wake() {
    switch2_wake_initialize();
    switch2_wake_initialize();
    Switch2WakeDiagnostics diagnostics{};
    switch2_wake_diagnostics(&diagnostics);
    require(!diagnostics.configured && !switch2_wake_ready_for_connections() &&
                !switch2_wake_request(),
            "Classic-only wake must be unavailable while preserving the paired host identity");
    run_task();
    require_opcode(0, 0xfc01);
    const uint8_t paired_address[] = SWITCH2_WAKE_SOURCE_ADDRESS_BYTES;
    require(memcmp(submitted[0].address, paired_address, sizeof(paired_address)) == 0,
            "Classic-only startup changed the address used by existing bonds");
    complete(0xfc01);
    run_task();
    require_opcode(1, 0x1009);
    complete(0x1009, 0, paired_address);
    require(switch2_wake_ready_for_connections() && !switch2_wake_request(),
            "Classic controller startup did not resume after identity verification");
    now_ms = 10000;
    run_task();
    switch2_wake_diagnostics(&diagnostics);
    require(!diagnostics.configured && !diagnostics.busy &&
                submitted_count == 2 && command_available &&
                diagnostics.accepted_requests == 0 &&
                diagnostics.completed_bursts == 0 && diagnostics.failures == 0 &&
                !switch2_wake_request(),
            "Classic-only wake submitted BLE advertising or remained busy");
}

}  // namespace

int main() {
    if (SWITCH_PICO_ENABLE_BLE) {
        test_stable_identity_wake();
        test_failed_wake_keeps_stable_identity();
    } else {
        test_disabled_transport_wake();
    }
    return 0;
}
