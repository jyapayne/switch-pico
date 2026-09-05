#pragma once

#include <stdbool.h>
#include <stdint.h>

using bd_addr_t = uint8_t[6];
using btstack_packet_handler_t = void (*)(uint8_t, uint16_t, uint8_t*, uint16_t);
struct btstack_packet_callback_registration_t {
    void* item;
    btstack_packet_handler_t callback;
};
struct btstack_timer_source_t {
    void (*handler)(btstack_timer_source_t*);
    uint32_t timeout_ms;
    uint32_t add_count;
};
struct hci_cmd_t {
    uint16_t opcode;
    const char* format;
};

enum {
    ERROR_CODE_SUCCESS = 0,
    HCI_EVENT_PACKET = 4,
    HCI_EVENT_COMMAND_COMPLETE = 0x0e,
    BD_ADDR_TYPE_LE_PUBLIC = 0,
};

extern const hci_cmd_t hci_read_bd_addr;
extern const hci_cmd_t hci_le_set_advertising_parameters;
extern const hci_cmd_t hci_le_set_advertising_data;
extern const hci_cmd_t hci_le_set_advertise_enable;

bool hci_can_send_command_packet_now();
uint8_t hci_send_cmd(const hci_cmd_t* command, ...);
void hci_add_event_handler(btstack_packet_callback_registration_t* registration);
void gap_local_bd_addr(bd_addr_t output);
uint32_t btstack_run_loop_get_time_ms();
void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer,
    void (*handler)(btstack_timer_source_t*));
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t timeout_ms);
void btstack_run_loop_add_timer(btstack_timer_source_t* timer);
int btstack_run_loop_remove_timer(btstack_timer_source_t* timer);

inline uint8_t hci_event_packet_get_type(const uint8_t* event) {
    return event[0];
}
inline uint16_t hci_event_command_complete_get_command_opcode(
    const uint8_t* event) {
    return static_cast<uint16_t>(event[3]) |
           static_cast<uint16_t>(event[4] << 8);
}
inline const uint8_t* hci_event_command_complete_get_return_parameters(
    const uint8_t* event) {
    return &event[5];
}
