#pragma once

#include <stdint.h>

struct btstack_timer_source_t {
    void (*process)(btstack_timer_source_t*) = nullptr;
    void* context = nullptr;
    uint64_t timeout_us = 0;
};

constexpr uint8_t ERROR_CODE_SUCCESS = 0;
enum gap_connection_type_t {
    GAP_CONNECTION_INVALID, GAP_CONNECTION_ACL, GAP_CONNECTION_LE, GAP_CONNECTION_SCO
};
gap_connection_type_t gap_get_connection_type(uint16_t handle);

void btstack_run_loop_set_timer_handler(
    btstack_timer_source_t* timer, void (*handler)(btstack_timer_source_t*));
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t timeout_ms);
void btstack_run_loop_add_timer(btstack_timer_source_t* timer);
int btstack_run_loop_remove_timer(btstack_timer_source_t* timer);
uint16_t l2cap_get_remote_mtu_for_local_cid(uint16_t cid);
bool l2cap_can_send_packet_now(uint16_t cid);
int hci_number_free_acl_slots_for_handle(uint16_t handle);
uint8_t l2cap_request_can_send_now_event(uint16_t cid);
uint8_t l2cap_send(uint16_t cid, const uint8_t* data, uint16_t size);
