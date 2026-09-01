#pragma once

#include <stdint.h>

typedef uint8_t bd_addr_t[6];
typedef uint8_t link_key_t[16];
typedef uint8_t sm_key_t[16];
typedef int link_key_type_t;

enum bd_addr_type_t {
    BD_ADDR_TYPE_LE_PUBLIC = 0,
    BD_ADDR_TYPE_LE_RANDOM = 1,
    BD_ADDR_TYPE_LE_PUBLIC_IDENTITY = 2,
    BD_ADDR_TYPE_LE_RANDOM_IDENTITY = 3,
    BD_ADDR_TYPE_UNKNOWN = 0xfe,
};

enum hci_link_type_t {
    HCI_LINK_TYPE_SCO = 0,
    HCI_LINK_TYPE_ACL = 1,
};

struct btstack_link_key_iterator_t {
    int index;
};

enum {
    ERROR_CODE_SUCCESS = 0,
    HCI_EVENT_PACKET = 4,
    HCI_EVENT_USER_CONFIRMATION_REQUEST = 0x33,
    HCI_EVENT_USER_PASSKEY_REQUEST = 0x34,
    SM_STK_GENERATION_METHOD_JUST_WORKS = 0x01,
    SM_STK_GENERATION_METHOD_OOB = 0x02,
    SM_STK_GENERATION_METHOD_PASSKEY = 0x04,
    SM_STK_GENERATION_METHOD_NUMERIC_COMPARISON = 0x08,
};
typedef int uni_property_idx_t;
typedef int uni_platform_oob_event_t;
struct uni_property_t {};

enum uni_error_t {
    UNI_ERROR_SUCCESS = 0,
    UNI_ERROR_IGNORE_DEVICE = 1,
    UNI_ERROR_INVALID_CONTROLLER = 2,
    UNI_ERROR_NO_SLOTS = 3,
};

enum {
    UNI_CONTROLLER_CLASS_GAMEPAD = 1,
    DPAD_UP = 1 << 0,
    DPAD_DOWN = 1 << 1,
    DPAD_LEFT = 1 << 2,
    DPAD_RIGHT = 1 << 3,
    BUTTON_A = 1 << 0,
    BUTTON_B = 1 << 1,
    BUTTON_X = 1 << 2,
    BUTTON_Y = 1 << 3,
    BUTTON_SHOULDER_L = 1 << 4,
    BUTTON_SHOULDER_R = 1 << 5,
    BUTTON_TRIGGER_L = 1 << 6,
    BUTTON_TRIGGER_R = 1 << 7,
    BUTTON_THUMB_L = 1 << 8,
    BUTTON_THUMB_R = 1 << 9,
    MISC_BUTTON_SYSTEM = 1 << 0,
    MISC_BUTTON_SELECT = 1 << 1,
    MISC_BUTTON_START = 1 << 2,
    MISC_BUTTON_CAPTURE = 1 << 3,
};

struct uni_gamepad_t {
    uint32_t dpad;
    uint32_t buttons;
    uint32_t misc_buttons;
    int32_t axis_x;
    int32_t axis_y;
    int32_t axis_rx;
    int32_t axis_ry;
    int32_t brake;
    int32_t throttle;
    int32_t accel[3];
    int32_t gyro[3];
};

struct uni_controller_t {
    int klass;
    uni_gamepad_t gamepad;
};

struct uni_hid_device_t;
typedef void (*btstack_packet_handler_t)(uint8_t, uint16_t, uint8_t*,
                                         uint16_t);
struct btstack_packet_callback_registration_t {
    void* item;
    btstack_packet_handler_t callback;
};
typedef void (*uni_play_dual_rumble_t)(uni_hid_device_t*, uint16_t,
                                        uint16_t, uint8_t, uint8_t);
typedef void (*uni_set_player_leds_t)(uni_hid_device_t*, uint8_t);
typedef void (*uni_set_lightbar_color_t)(uni_hid_device_t*, uint8_t, uint8_t,
                                         uint8_t);

struct uni_report_parser_t {
    uni_set_player_leds_t set_player_leds;
    uni_set_lightbar_color_t set_lightbar_color;
    uni_play_dual_rumble_t play_dual_rumble;
};

enum uni_bt_conn_protocol_t {
    UNI_BT_CONN_PROTOCOL_NONE,
    UNI_BT_CONN_PROTOCOL_BR_EDR,
    UNI_BT_CONN_PROTOCOL_BLE,
};


struct uni_bt_conn_t {
    bd_addr_t btaddr;
    uni_bt_conn_protocol_t protocol;
};

struct uni_hid_device_t {
    uni_bt_conn_t conn;
    int idx;
    bool gamepad;
    uni_report_parser_t report_parser;
    int rumble_calls;
    uint8_t last_high;
    uint8_t last_low;
    uint16_t last_rumble_duration_ms;
    int lightbar_calls;
    uint8_t lightbar_red;
    uint8_t lightbar_green;
    uint8_t lightbar_blue;
    int player_led_calls;
    uint8_t player_leds;
};

struct uni_platform {
    const char* name;
    void (*init)(int, const char**);
    void (*on_init_complete)();
    uni_error_t (*on_device_discovered)(bd_addr_t, const char*, uint16_t,
                                        uint8_t);
    void (*on_device_connected)(uni_hid_device_t*);
    void (*on_device_disconnected)(uni_hid_device_t*);
    uni_error_t (*on_device_ready)(uni_hid_device_t*);
    void* on_device_oob_event;
    void (*on_controller_data)(uni_hid_device_t*, uni_controller_t*);
    const uni_property_t* (*get_property)(uni_property_idx_t);
    void (*on_oob_event)(uni_platform_oob_event_t, void*);
    void* on_device_dump;
    void* on_gamepad_seat;
};

bool uni_hid_device_is_gamepad(const uni_hid_device_t* device);
int uni_hid_device_get_idx_for_instance(const uni_hid_device_t* device);
void uni_hid_device_disconnect(uni_hid_device_t* device);
void uni_bt_allow_incoming_connections(bool enabled);
void uni_bt_start_scanning_and_autoconnect_unsafe();
void uni_bt_stop_scanning_unsafe();
void uni_bt_bredr_scan_start();
void uni_bt_bredr_scan_stop();
void uni_bt_le_scan_start();
void uni_bt_le_scan_stop();
void uni_bt_del_keys_unsafe();
int gap_link_key_iterator_init(btstack_link_key_iterator_t* iterator);
int gap_link_key_iterator_get_next(
    btstack_link_key_iterator_t* iterator, bd_addr_t address,
    link_key_t link_key, link_key_type_t* type);
void gap_link_key_iterator_done(btstack_link_key_iterator_t* iterator);
int le_device_db_max_count();
void le_device_db_info(
    int index, int* address_type, bd_addr_t address, sm_key_t irk);
void gap_set_bondable_mode(int enabled);
void gap_ssp_set_auto_accept(int auto_accept);
void sm_set_accepted_stk_generation_methods(
    uint8_t accepted_stk_generation_methods);
int gap_ssp_confirmation_response(const bd_addr_t address);
int gap_ssp_confirmation_negative(const bd_addr_t address);
int gap_ssp_passkey_response(const bd_addr_t address, uint32_t passkey);
int gap_ssp_passkey_negative(const bd_addr_t address);
void hci_add_event_handler(
    btstack_packet_callback_registration_t* callback_handler);
uint8_t hci_event_packet_get_type(const uint8_t* packet);
void hci_event_user_confirmation_request_get_bd_addr(
    const uint8_t* packet, bd_addr_t address);
void hci_event_user_passkey_request_get_bd_addr(
    const uint8_t* packet, bd_addr_t address);
void uni_platform_set_custom(uni_platform* platform);
int uni_init(int argc, const char** argv);
