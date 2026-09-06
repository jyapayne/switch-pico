#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t bd_addr_t[6];
typedef uint16_t hci_con_handle_t;
typedef enum { GAP_CONNECTION_INVALID, GAP_CONNECTION_ACL, GAP_CONNECTION_SCO, GAP_CONNECTION_LE } gap_connection_type_t;

// Scheduling metadata lives in the test scheduler, not in parser memory.
// Keeping only the callback/context also avoids host pointer inflation of the
// firmware's fixed 256-byte parser allocation. Production ABI is built by CI.
typedef struct btstack_timer_source {
    void (*process)(struct btstack_timer_source* timer);
    void* context;
} btstack_timer_source_t;

#define ERROR_CODE_SUCCESS 0
#define ERROR_CODE_COMMAND_DISALLOWED 0x0c
#define BTSTACK_ACL_BUFFERS_FULL 0x57
#define HID_MESSAGE_TYPE_DATA 0x0a
#define HID_REPORT_TYPE_OUTPUT 0x02
#define btstack_min(a, b) ((a) < (b) ? (a) : (b))

void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t ms);
void btstack_run_loop_add_timer(btstack_timer_source_t* timer);
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer);
void btstack_run_loop_set_timer_context(btstack_timer_source_t* timer, void* context);
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer, void (*handler)(btstack_timer_source_t*));
void* btstack_run_loop_get_timer_context(btstack_timer_source_t* timer);
int l2cap_send(uint16_t cid, uint8_t* data, uint16_t len);
int l2cap_can_send_packet_now(uint16_t cid);
uint8_t l2cap_request_can_send_now_event(uint16_t cid);
gap_connection_type_t gap_get_connection_type(hci_con_handle_t handle);
void printf_hexdump(const void* data, int len);
const char* bd_addr_to_str(const bd_addr_t addr);
static inline int bd_addr_cmp(const bd_addr_t a, const bd_addr_t b) { return memcmp(a, b, 6); }
static inline void bd_addr_copy(bd_addr_t dst, const bd_addr_t src) { memcpy(dst, src, 6); }
