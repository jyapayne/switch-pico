#pragma once

#include <btstack.h>

struct uni_hid_device_s;
typedef struct uni_hid_device_s uni_hid_device_t;

struct uni_report_parser_t {
    void (*play_dual_rumble)(uni_hid_device_t*, uint16_t, uint16_t,
                             uint8_t, uint8_t) = nullptr;
};

enum uni_bt_conn_protocol_t {
    UNI_BT_CONN_PROTOCOL_NONE,
    UNI_BT_CONN_PROTOCOL_BR_EDR,
    UNI_BT_CONN_PROTOCOL_BLE,
};

struct uni_bt_conn_t {
    uint16_t handle = 0;
    uint16_t control_cid = 0;
    uint16_t interrupt_cid = 0;
    bool connected = false;
    uni_bt_conn_protocol_t protocol = UNI_BT_CONN_PROTOCOL_NONE;
};

struct uni_circular_buffer_t {
    unsigned queued = 0;
};
uint8_t uni_circular_buffer_is_empty(const uni_circular_buffer_t* buffer);

struct uni_hid_device_s {
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uni_report_parser_t report_parser;
    uni_bt_conn_t conn;
    uni_circular_buffer_t outgoing_buffer;

    // Fake parser/link state. The module only sees the real fields above.
    uint16_t remote_mtu = 143;
    gap_connection_type_t connection_type = GAP_CONNECTION_ACL;
    bool parser_rumble_active = false;
    bool parser_rumble_delayed = false;
    btstack_timer_source_t parser_timer;
    bool notification_pending = false;
    bool credit = true;
    uint16_t native_ready_cid = 0;
};
