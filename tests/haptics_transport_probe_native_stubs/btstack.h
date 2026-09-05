#pragma once

#include <stdint.h>

constexpr uint8_t HCI_EVENT_PACKET = 4;
constexpr uint8_t HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS = 0x13;
using hci_con_handle_t = uint16_t;
struct hci_connection_t {
    uint8_t num_packets_sent = 0;
};
struct btstack_packet_callback_registration_t {
    void* item = nullptr;
    void (*callback)(uint8_t, uint16_t, uint8_t*, uint16_t) = nullptr;
};
extern "C" {
void btstack_run_loop_base_poll_data_sources();
void btstack_run_loop_poll_data_sources_from_irq();
void hci_add_event_handler(btstack_packet_callback_registration_t* registration);
hci_connection_t* hci_connection_for_handle(hci_con_handle_t handle);
int hci_number_free_acl_slots_for_handle(hci_con_handle_t handle);
}
