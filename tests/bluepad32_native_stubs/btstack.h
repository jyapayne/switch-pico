#pragma once

#include "btstack_run_loop.h"
#include "uni.h"

uint16_t l2cap_get_remote_mtu_for_local_cid(uint16_t cid);
uint8_t l2cap_request_can_send_now_event(uint16_t cid);
uint8_t l2cap_send(uint16_t cid, const uint8_t* data, uint16_t size);
bool l2cap_can_send_packet_now(uint16_t cid);
int hci_number_free_acl_slots_for_handle(uint16_t handle);
