#pragma once

#include <stdint.h>

enum btstack_data_source_callback_type_t {
    DATA_SOURCE_CALLBACK_POLL = 4,
};

struct btstack_data_source_t {
    void (*handler)(btstack_data_source_t*, btstack_data_source_callback_type_t) = nullptr;
    uint16_t callbacks = 0;
};

struct btstack_timer_source_t {
    void (*handler)(btstack_timer_source_t*) = nullptr;
    uint64_t due_us = 0;
};

void btstack_run_loop_set_data_source_handler(
    btstack_data_source_t* source,
    void (*handler)(btstack_data_source_t*, btstack_data_source_callback_type_t));
void btstack_run_loop_enable_data_source_callbacks(btstack_data_source_t* source,
                                                   uint16_t callbacks);
void btstack_run_loop_add_data_source(btstack_data_source_t* source);
void btstack_run_loop_poll_data_sources_from_irq();
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer,
                                         void (*handler)(btstack_timer_source_t*));
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t timeout_ms);
void btstack_run_loop_add_timer(btstack_timer_source_t* timer);
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer);
bool l2cap_can_send_packet_now(uint16_t cid);
int hci_number_free_acl_slots_for_handle(uint16_t handle);
uint8_t l2cap_request_can_send_now_event(uint16_t cid);
