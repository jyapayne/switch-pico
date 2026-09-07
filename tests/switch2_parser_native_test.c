#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "protocol_fixture.h"
#include "parser/uni_hid_parser_switch2.h"
#include "parser/uni_switch2_pairing.h"
#include "sdkconfig.h"

#define PEERS CONFIG_BLUEPAD32_MAX_DEVICES
#define SERVICE_START 0x100
#define INPUT_HANDLE 0x104
#define RESPONSE_HANDLE 0x114
#define COMMAND_HANDLE 0x124
#define RUMBLE_HANDLE 0x134

static struct fixture_peer peers[PEERS];
static btstack_timer_source_t* timers[16];
static unsigned timer_count, connected, ready, disconnected, emitted, remembered, listeners;
static unsigned connected_events, disconnected_events;
static unsigned scan_stops, discovery_calls;
static uint16_t expected_discovery_pid;
static uint8_t expected_discovery_address_type;
static uint32_t now_ms;
static bool pairing_allowed, trusted, storage_ok, request_writes, admit, reject_connected;
static uint8_t next_write_error;
static uint8_t next_connect_error;
static bool scan_enabled, scan_running;
static const bd_addr_t host_address = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65};
static const bd_addr_t controller_address = {0xc0, 0x22, 0x33, 0x44, 0x55, 0x66};
static const uint8_t service_uuid[16] = {0xab,0x7d,0xe9,0xbe,0x89,0xfe,0x49,0xad,0x82,0x8f,0x11,0x8f,0x09,0xdf,0x7f,0xd0};
static const uint8_t input_uuid[16] = {0xab,0x7d,0xe9,0xbe,0x89,0xfe,0x49,0xad,0x82,0x8f,0x11,0x8f,0x09,0xdf,0x7f,0xd2};
static const uint8_t response_uuid[16] = {0xc7,0x65,0xa9,0x61,0xd9,0xd8,0x4d,0x36,0xa2,0x0a,0x53,0x15,0xb1,0x11,0x83,0x6a};
static const uint8_t command_uuid[16] = {0x64,0x9d,0x4a,0xc9,0x8e,0xb7,0x4e,0x6c,0xaf,0x44,0x1e,0xa5,0x4f,0xe5,0xf0,0x05};
static const uint8_t rumble_uuids[3][16] = {
    {0xcc,0x48,0x3f,0x51,0x92,0x58,0x42,0x7d,0xa9,0x39,0x63,0x0c,0x31,0xf7,0x2b,0x05},
    {0x28,0x93,0x26,0xcb,0xa4,0x71,0x48,0x5d,0xa8,0xf4,0x24,0x0c,0x14,0xf1,0x82,0x41},
    {0xfa,0x19,0xb0,0xfb,0xcd,0x1f,0x46,0xa7,0x84,0xa1,0xbb,0xb0,0x9e,0x00,0xc1,0x49},
};

static struct fixture_peer* peer_for_handle(hci_con_handle_t handle) {
    for (unsigned i = 0; i < PEERS; ++i)
        if (peers[i].used && peers[i].device.conn.handle == handle)
            return &peers[i];
    return NULL;
}

static void advance(uint32_t milliseconds) {
    uint32_t target = now_ms + milliseconds;
    for (;;) {
        int earliest = -1;
        for (unsigned i = 0; i < timer_count; ++i) {
            if ((int32_t)(timers[i]->timeout - target) <= 0 &&
                (earliest < 0 || (int32_t)(timers[i]->timeout - timers[earliest]->timeout) < 0))
                earliest = (int)i;
        }
        if (earliest < 0)
            break;
        btstack_timer_source_t* timer = timers[earliest];
        now_ms = timer->timeout;
        btstack_run_loop_remove_timer(timer);
        timer->process(timer);
    }
    now_ms = target;
}

void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t ms) { timer->timeout = now_ms + ms; }
void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < timer_count; ++i)
        assert(timers[i] != timer);
    assert(timer_count < 16);
    timers[timer_count++] = timer;
}
int btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < timer_count; ++i) {
        if (timers[i] == timer) {
            timers[i] = timers[--timer_count];
            return true;
        }
    }
    return false;
}
void btstack_run_loop_set_timer_context(btstack_timer_source_t* timer, void* context) { timer->context = context; }
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer, void (*handler)(btstack_timer_source_t*)) { timer->process = handler; }
void* btstack_run_loop_get_timer_context(btstack_timer_source_t* timer) { return timer->context; }
uint32_t btstack_run_loop_get_time_ms(void) { return now_ms; }
void uni_log(const char* format, ...) { (void)format; }
bool switch_pico_switch2_pairing_allowed(void) { return pairing_allowed; }
bool uni_switch2_pairing_known(uint8_t type, const uint8_t address[6]) { (void)type; (void)address; return trusted; }
bool uni_switch2_pairing_remember(uint8_t type, const uint8_t address[6]) {
    assert(type <= 1 && memcmp(address, controller_address, 6) == 0);
    ++remembered;
    return storage_ok;
}
void gap_local_bd_addr(bd_addr_t address) { memcpy(address, host_address, 6); }
void gap_stop_scan(void) { ++scan_stops; scan_running = false; }
void uni_bt_le_resume_scanning_if_enabled(void) { if (scan_enabled) scan_running = true; }
uint8_t gap_connect(const bd_addr_t address, bd_addr_type_t type) {
    (void)address; (void)type; ++connected;
    uint8_t status = next_connect_error;
    next_connect_error = 0;
    return status;
}
int gap_update_connection_parameters(hci_con_handle_t handle, uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout) {
    (void)handle; (void)min; (void)max; (void)latency; (void)timeout; return 0;
}
gap_connection_type_t gap_get_connection_type(hci_con_handle_t handle) {
    for (unsigned i = 0; i < PEERS; ++i)
        if (peers[i].link_alive && peers[i].device.conn.handle == handle)
            return GAP_CONNECTION_LE;
    return GAP_CONNECTION_INVALID;
}
uni_hid_device_t* uni_hid_device_create(bd_addr_t address) {
    for (unsigned i = 0; i < PEERS; ++i) {
        if (!peers[i].used) {
            memset(&peers[i], 0, sizeof(peers[i]));
            peers[i].used = true;
            peers[i].device.conn.handle = UNI_BT_CONN_HANDLE_INVALID;
            memcpy(peers[i].device.conn.btaddr, address, 6);
            return &peers[i].device;
        }
    }
    return NULL;
}
uni_hid_device_t* uni_hid_device_get_instance_for_address(bd_addr_t address) {
    for (unsigned i = 0; i < PEERS; ++i)
        if (peers[i].used && memcmp(peers[i].device.conn.btaddr, address, 6) == 0)
            return &peers[i].device;
    return NULL;
}
uni_hid_device_t* uni_hid_device_get_instance_for_connection_handle(hci_con_handle_t handle) {
    struct fixture_peer* peer = peer_for_handle(handle);
    return peer ? &peer->device : NULL;
}
uni_error_t uni_hid_device_on_device_discovered(bd_addr_t address, const char* name, uint16_t cod, uint8_t rssi) {
    ++discovery_calls;
    uni_hid_device_t* d = uni_hid_device_get_instance_for_address(address);
    uint8_t type = 0xff;
    assert(d && uni_hid_parser_switch2_is_ble_device(d));
    assert(uni_hid_parser_switch2_identity_address_type(d, &type));
    (void)name;
    assert(d->cod == cod && d->conn.rssi == rssi);
    if (expected_discovery_pid) {
        assert(d->product_id == expected_discovery_pid);
        assert(type == expected_discovery_address_type);
    }
    return admit ? UNI_ERROR_SUCCESS : (uni_error_t)1;
}
void uni_hid_device_set_vendor_id(uni_hid_device_t* d, uint16_t value) { d->vendor_id = value; }
void uni_hid_device_set_product_id(uni_hid_device_t* d, uint16_t value) { d->product_id = value; }
void uni_hid_device_set_cod(uni_hid_device_t* d, uint32_t value) { d->cod = value; }
void uni_hid_device_set_name(uni_hid_device_t* d, const char* value) { (void)d; (void)value; }
void uni_hid_device_guess_controller_type_from_pid_vid(uni_hid_device_t* d) { (void)d; }
void uni_bt_conn_set_protocol(uni_bt_conn_t* conn, uni_bt_conn_protocol_t value) { conn->protocol = value; }
void uni_bt_conn_set_state(uni_bt_conn_t* conn, uni_bt_conn_state_t value) { conn->state = value; }
void uni_hid_device_connect(uni_hid_device_t* d) {
    assert(!d->conn.connected);
    d->conn.connected = true;
    ++connected_events;
    if (reject_connected) {
        uni_hid_device_disconnect(d);
        uni_hid_device_delete(d);
    }
}
void uni_hid_device_set_ready(uni_hid_device_t* d) {
    d->conn.state = UNI_BT_CONN_STATE_DEVICE_PENDING_READY;
    uni_hid_parser_switch2_setup(d);
}
bool uni_hid_device_set_ready_complete(uni_hid_device_t* d) {
    assert(d->conn.connected && connected_events == 1);
    ++ready;
    d->conn.state = UNI_BT_CONN_STATE_DEVICE_READY;
    return true;
}
void uni_hid_device_disconnect(uni_hid_device_t* d) {
    ++disconnected;
    if (d->conn.connected)
        ++disconnected_events;
    d->conn.connected = false;
    uni_hid_parser_switch2_teardown(d);
    struct fixture_peer* peer = peer_for_handle(d->conn.handle);
    if (peer)
        peer->link_alive = false;
}
void uni_hid_device_delete(uni_hid_device_t* d) {
    uni_hid_parser_switch2_teardown(d);
    for (unsigned i = 0; i < PEERS; ++i)
        if (&peers[i].device == d)
            peers[i].used = false;
}
void uni_hid_device_process_controller(uni_hid_device_t* d) { (void)d; ++emitted; }

static uint8_t begin_query(btstack_packet_handler_t callback, hci_con_handle_t handle, enum fixture_query query) {
    struct fixture_peer* peer = peer_for_handle(handle);
    assert(peer && peer->query == QUERY_NONE);
    peer->callback = callback;
    peer->query = query;
    return 0;
}
uint8_t gatt_client_discover_primary_services_by_uuid128(btstack_packet_handler_t callback, hci_con_handle_t handle, const uint8_t* uuid) {
    assert(memcmp(uuid, service_uuid, 16) == 0);
    return begin_query(callback, handle, QUERY_SERVICE);
}
uint8_t gatt_client_discover_characteristics_for_service(btstack_packet_handler_t callback, hci_con_handle_t handle, gatt_client_service_t* service) {
    assert(service->start_group_handle == SERVICE_START);
    return begin_query(callback, handle, QUERY_CHARACTERISTICS);
}
uint8_t gatt_client_discover_characteristic_descriptors(btstack_packet_handler_t callback, hci_con_handle_t handle, gatt_client_characteristic_t* ch) {
    peer_for_handle(handle)->descriptor_value = ch->value_handle;
    return begin_query(callback, handle, QUERY_DESCRIPTORS);
}
uint8_t gatt_client_write_characteristic_descriptor_using_descriptor_handle(btstack_packet_handler_t callback, hci_con_handle_t handle,
                                                                          uint16_t descriptor, uint16_t length, uint8_t* value) {
    struct fixture_peer* peer = peer_for_handle(handle);
    peer->cccd_handle = descriptor;
    peer->pending_write = value;
    peer->pending_length = length;
    return begin_query(callback, handle, QUERY_CCCD);
}
void gatt_client_listen_for_characteristic_value_updates(gatt_client_notification_t* registration, btstack_packet_handler_t callback,
                                                        hci_con_handle_t handle, gatt_client_characteristic_t* ch) {
    registration->callback = callback;
    registration->con_handle = handle;
    registration->attribute_handle = ch->value_handle;
    ++listeners;
}
void gatt_client_stop_listening_for_characteristic_value_updates(gatt_client_notification_t* registration) { (void)registration; assert(listeners); --listeners; }
static uint8_t capture_write(hci_con_handle_t handle, uint16_t value_handle, uint16_t length, uint8_t* value) {
    if (next_write_error) {
        uint8_t error = next_write_error;
        next_write_error = 0;
        return error;
    }
    struct fixture_peer* peer = peer_for_handle(handle);
    assert(peer);
    if (value_handle == COMMAND_HANDLE) {
        assert(length <= sizeof(peer->command));
        memcpy(peer->command, value, length);
        peer->command_length = length;
        ++peer->commands;
    } else {
        assert(value_handle == RUMBLE_HANDLE && length <= sizeof(peer->rumble));
        memcpy(peer->rumble, value, length);
        peer->rumble_length = length;
        unsigned slot = peer->rumbles % FIXTURE_RUMBLE_HISTORY;
        memcpy(peer->rumble_history[slot], value, length);
        peer->rumble_times[slot] = now_ms;
        ++peer->rumbles;
    }
    return 0;
}
uint8_t gatt_client_write_value_of_characteristic_without_response(hci_con_handle_t handle, uint16_t value_handle, uint16_t length, uint8_t* value) {
    return capture_write(handle, value_handle, length, value);
}
uint8_t gatt_client_write_value_of_characteristic(btstack_packet_handler_t callback, hci_con_handle_t handle,
                                                uint16_t value_handle, uint16_t length, uint8_t* value) {
    uint8_t status = capture_write(handle, value_handle, length, value);
    if (status)
        return status;
    struct fixture_peer* peer = peer_for_handle(handle);
    peer->pending_write = value;
    peer->pending_length = length;
    memcpy(peer->pending_snapshot, value, length);
    return begin_query(callback, handle, QUERY_WRITE);
}

// Serialization here models BTstack-generated events; production accessors and
// types come directly from the SDK rather than a parallel mock btstack.h ABI.
void gatt_client_deserialize_service(const uint8_t* data, int offset, gatt_client_service_t* service) {
    service->start_group_handle = little_endian_read_16(data, offset);
    service->end_group_handle = little_endian_read_16(data, offset + 2);
    reverse_128(data + offset + 4, service->uuid128);
    service->uuid16 = 0;
}
void gatt_client_deserialize_characteristic(const uint8_t* data, int offset, gatt_client_characteristic_t* ch) {
    ch->start_handle = little_endian_read_16(data, offset);
    ch->value_handle = little_endian_read_16(data, offset + 2);
    ch->end_handle = little_endian_read_16(data, offset + 4);
    ch->properties = little_endian_read_16(data, offset + 6);
    reverse_128(data + offset + 8, ch->uuid128);
    ch->uuid16 = 0;
}
void gatt_client_deserialize_characteristic_descriptor(const uint8_t* data, int offset, gatt_client_characteristic_descriptor_t* descriptor) {
    descriptor->handle = little_endian_read_16(data, offset);
    reverse_128(data + offset + 2, descriptor->uuid128);
    descriptor->uuid16 = (uint16_t)big_endian_read_32(descriptor->uuid128, 0);
}

static void event(struct fixture_peer* peer, uint8_t* data, uint16_t size) {
    data[1] = (uint8_t)(size - 2);
    little_endian_store_16(data, 2, peer->device.conn.handle);
    peer->callback(HCI_EVENT_PACKET, 0, data, size);
}
static void query_done(struct fixture_peer* peer, uint8_t status) {
    uint8_t data[9] = {GATT_EVENT_QUERY_COMPLETE};
    if (peer->query == QUERY_CCCD)
        assert(peer->pending_length == 2 && peer->pending_write[0] == 1 && peer->pending_write[1] == 0);
    if (peer->query == QUERY_WRITE)
        assert(memcmp(peer->pending_write, peer->pending_snapshot, peer->pending_length) == 0);
    peer->query = QUERY_NONE;
    data[8] = status;
    event(peer, data, sizeof(data));
}
static void characteristic(struct fixture_peer* peer, uint16_t handle, const uint8_t* uuid, uint16_t properties) {
    uint8_t data[32] = {GATT_EVENT_CHARACTERISTIC_QUERY_RESULT};
    little_endian_store_16(data, 8, handle - 1);
    little_endian_store_16(data, 10, handle);
    little_endian_store_16(data, 12, handle + 3);
    little_endian_store_16(data, 14, properties);
    reverse_128(uuid, data + 16);
    event(peer, data, sizeof(data));
}
static void descriptor(struct fixture_peer* peer, uint16_t handle) {
    const uint8_t cccd_uuid[16] = {0,0,0x29,0x02,0,0,0x10,0,0x80,0,0,0x80,0x5f,0x9b,0x34,0xfb};
    uint8_t data[26] = {GATT_EVENT_ALL_CHARACTERISTIC_DESCRIPTORS_QUERY_RESULT};
    little_endian_store_16(data, 8, handle);
    reverse_128(cccd_uuid, data + 10);
    event(peer, data, sizeof(data));
}
static void discover(struct fixture_peer* peer) {
    uint8_t service[28] = {GATT_EVENT_SERVICE_QUERY_RESULT};
    little_endian_store_16(service, 8, SERVICE_START);
    little_endian_store_16(service, 10, SERVICE_START + 0x60);
    reverse_128(service_uuid, service + 12);
    event(peer, service, sizeof(service));
    query_done(peer, 0);
    characteristic(peer, INPUT_HANDLE, input_uuid, ATT_PROPERTY_NOTIFY);
    characteristic(peer, RESPONSE_HANDLE, response_uuid, ATT_PROPERTY_NOTIFY);
    characteristic(peer, COMMAND_HANDLE, command_uuid, request_writes ? ATT_PROPERTY_WRITE : ATT_PROPERTY_WRITE_WITHOUT_RESPONSE);
    unsigned kind = peer->device.product_id == UNI_SW2_PRO_PID ? 0 : peer->device.product_id == UNI_SW2_JOYCON_L_PID ? 1 : 2;
    characteristic(peer, RUMBLE_HANDLE, rumble_uuids[kind], request_writes ? ATT_PROPERTY_WRITE : ATT_PROPERTY_WRITE_WITHOUT_RESPONSE);
    query_done(peer, 0);
}
static void subscribe_response(struct fixture_peer* peer) {
    // Deliberately not value_handle+1: parser must discover, not guess CCCDs.
    descriptor(peer, RESPONSE_HANDLE + 2);
    query_done(peer, 0);
    descriptor(peer, INPUT_HANDLE + 2);
    query_done(peer, 0);
    assert(peer->cccd_handle == RESPONSE_HANDLE + 2);
    query_done(peer, 0);
}
static void notify(struct fixture_peer* peer, uint16_t handle, const uint8_t* value, uint16_t length) {
    uint8_t data[112] = {GATT_EVENT_NOTIFICATION};
    assert(length <= sizeof(data) - 12);
    little_endian_store_16(data, 8, handle);
    little_endian_store_16(data, 10, length);
    memcpy(data + 12, value, length);
    event(peer, data, 12 + length);
}
static void packed(uint8_t* data, uint16_t x, uint16_t y) {
    data[0] = (uint8_t)x;
    data[1] = (uint8_t)((x >> 8) | (y << 4));
    data[2] = (uint8_t)(y >> 4);
}
static unsigned response_data(struct fixture_peer* peer, uint8_t* out, bool erased) {
    memset(out, 0, 96);
    out[0] = peer->command[0];
    out[1] = out[2] = 1;
    out[3] = peer->command[3];
    out[5] = 0x78;
    if (out[0] == 2) {
        uint8_t length = peer->command[8];
        uint32_t address = little_endian_read_32(peer->command, 12);
        out[8] = length;
        little_endian_store_32(out, 12, address);
        if (address == 0x13000) {
            little_endian_store_16(out, 16 + 18, UNI_SW2_NINTENDO_VID);
            little_endian_store_16(out, 16 + 20, peer->device.product_id);
        } else if (address != 0x13044) {
            if (erased)
                memset(out + 16, 0xff, length);
            else {
                packed(out + 16, 1900, 2100);
                packed(out + 19, 1200, 1400);
                packed(out + 22, 1500, 1700);
            }
        }
        return 16 + length;
    }
    if (out[0] == 0x15) {
        out[8] = 1;
        return out[3] == 1 ? 17 : out[3] == 3 ? 9 : 25;
    }
    return 8;
}
static void acknowledge(struct fixture_peer* peer, bool erased) {
    uint8_t response[96];
    unsigned length = response_data(peer, response, erased);
    if (peer->query == QUERY_WRITE)
        query_done(peer, 0);
    notify(peer, RESPONSE_HANDLE, response, length);
}
static void finish_setup(struct fixture_peer* peer) {
    for (unsigned limit = 0; limit < 24 && !ready && !disconnected; ++limit) {
        if (peer->query == QUERY_CCCD) {
            assert(peer->cccd_handle == INPUT_HANDLE + 2);
            query_done(peer, 0);
        } else {
            acknowledge(peer, false);
        }
    }
    assert(ready == 1 && disconnected == 0);
}
static size_t advertisement(uint8_t* packet, uint16_t pid, bool fresh) {
    memset(packet, 0, 64);
    packet[0] = GAP_EVENT_ADVERTISING_REPORT;
    packet[2] = 0;
    packet[3] = BD_ADDR_TYPE_LE_PUBLIC;
    reverse_bytes(controller_address, packet + 4, 6);
    packet[10] = (uint8_t)-35;
    packet[11] = 20;
    packet[12] = 19;
    packet[13] = 0xff;
    uint8_t* mfg = packet + 14;
    little_endian_store_16(mfg, 0, 0x0553);
    little_endian_store_16(mfg, 5, UNI_SW2_NINTENDO_VID);
    little_endian_store_16(mfg, 7, pid);
    if (!fresh)
        reverse_bytes(host_address, mfg + 12, 6);
    packet[1] = 30;
    return 32;
}
static struct fixture_peer* connect_peer(uint16_t pid, bool fresh) {
    uint8_t packet[64];
    size_t size = advertisement(packet, pid, fresh);
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(connected == 1);
    struct fixture_peer* peer = &peers[0];
    peer->device.conn.handle = 0; // Handle zero is valid, including on retirement.
    peer->link_alive = true;
    uni_hid_parser_switch2_on_le_connected(&peer->device);
    return peer;
}
static void reset(void) {
    for (unsigned i = 0; i < PEERS; ++i) {
        if (peers[i].used)
            uni_hid_parser_switch2_teardown(&peers[i].device);
    }
    assert(timer_count == 0 && listeners == 0);
    memset(peers, 0, sizeof(peers));
    connected = ready = disconnected = emitted = remembered = 0;
    connected_events = disconnected_events = 0;
    scan_stops = discovery_calls = 0;
    expected_discovery_pid = 0;
    expected_discovery_address_type = BD_ADDR_TYPE_LE_PUBLIC;
    now_ms = 0;
    next_write_error = 0;
    next_connect_error = 0;
    scan_enabled = scan_running = true;
    pairing_allowed = trusted = storage_ok = admit = true;
    request_writes = false;
    reject_connected = false;
}

static void test_connected_callback_rejection(void) {
    reset();
    reject_connected = true;
    struct fixture_peer* peer = connect_peer(UNI_SW2_PRO_PID, false);
    assert(connected_events == 1 && disconnected_events == 1 && ready == 0);
    assert(peer->query == QUERY_NONE && timer_count == 0 && listeners == 0);
    advance(3000);
    assert(ready == 0 && peer->commands == 0);
}

static void test_advertisement_bounds_and_admission(void) {
    reset();
    uint8_t packet[64];
    size_t size = advertisement(packet, UNI_SW2_PRO_PID, true);
    for (size_t n = 0; n < size; ++n)
        assert(!uni_bt_le_switch2_handle_advertisement(packet, (uint16_t)n));
    packet[12] = 20;
    assert(!uni_bt_le_switch2_handle_advertisement(packet, size));
    packet[12] = 19;
    pairing_allowed = false;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 0);
    advertisement(packet, UNI_SW2_PRO_PID, false);
    trusted = false;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 0);
    trusted = true;
    memcpy(packet + 26, host_address, 6); // Wrong byte order must not reconnect.
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 0);
    advertisement(packet, UNI_SW2_PRO_PID, false);
    packet[3] = BD_ADDR_TYPE_LE_RANDOM;
    packet[9] = 0x40; // Resolving private address, not static identity.
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 0);
    assert(discovery_calls == 0 && scan_stops == 0);
    advertisement(packet, UNI_SW2_PRO_PID, false);
    admit = false;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 0);
    admit = true;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size) && connected == 1);
    uint8_t type = 0xff;
    assert(uni_hid_parser_switch2_identity_address_type(&peers[0].device, &type) && type == BD_ADDR_TYPE_LE_PUBLIC);
}

static void test_discovery_metadata_and_rejection_isolation(void) {
    reset();
    expected_discovery_pid = UNI_SW2_JOYCON_L_PID;
    struct fixture_peer* mate = connect_peer(UNI_SW2_JOYCON_L_PID, false);
    discover(mate);
    subscribe_response(mate);
    finish_setup(mate);
    assert(ready == 1 && discovery_calls == 1);
    const unsigned mate_timers = timer_count;
    const unsigned mate_listeners = listeners;

    uint8_t packet[64];
    size_t size = advertisement(packet, UNI_SW2_JOYCON_R_PID, false);
    packet[3] = BD_ADDR_TYPE_LE_RANDOM;
    packet[4] ^= 1; // A different static identity from the active mate.
    bd_addr_t candidate_address;
    gap_event_advertising_report_get_address(packet, candidate_address);
    expected_discovery_pid = UNI_SW2_JOYCON_R_PID;
    expected_discovery_address_type = BD_ADDR_TYPE_LE_RANDOM;
    admit = false;
    // More rejections than slots must not exhaust either device or parser storage.
    for (unsigned i = 0; i <= CONFIG_BLUEPAD32_MAX_DEVICES; ++i) {
        assert(uni_bt_le_switch2_handle_advertisement(packet, size));
        assert(discovery_calls == i + 2);
        assert(!uni_hid_device_get_instance_for_address(candidate_address));
        for (unsigned slot = 1; slot < PEERS; ++slot) {
            uint8_t type;
            assert(!peers[slot].used);
            assert(!uni_hid_parser_switch2_identity_address_type(&peers[slot].device, &type));
        }
        assert(connected == 1 && scan_stops == 1 && disconnected == 0);
        assert(timer_count == mate_timers && listeners == mate_listeners);
        assert(mate->used && mate->link_alive && mate->device.conn.state == UNI_BT_CONN_STATE_DEVICE_READY);
    }

    // A rejected candidate must remain discoverable once policy permits it.
    admit = true;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(uni_hid_device_get_instance_for_address(candidate_address));
    assert(connected == 2 && scan_stops == 2 && disconnected == 0);
    unsigned admitted_calls = discovery_calls;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(discovery_calls == admitted_calls && connected == 2 && scan_stops == 2);
    assert(mate->used && mate->link_alive);

    reset();
    size = advertisement(packet, UNI_SW2_JOYCON_R_PID, false);
    admit = false;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    advance(3000);
    assert(discovery_calls == 1 && connected == 0 && scan_stops == 0 && disconnected == 0);
    assert(timer_count == 0 && listeners == 0);
    for (unsigned slot = 0; slot < PEERS; ++slot)
        assert(!peers[slot].used);
}

static void test_missing_descriptor_and_setup_timeout(void) {
    reset();
    struct fixture_peer* peer = connect_peer(UNI_SW2_PRO_PID, false);
    discover(peer);
    descriptor(peer, RESPONSE_HANDLE); // CCCD outside characteristic's descriptor range.
    assert(disconnected == 1 && ready == 0 && timer_count == 0 && listeners == 0);
    reset();
    peer = connect_peer(UNI_SW2_PRO_PID, false);
    discover(peer);
    subscribe_response(peer);
    uint8_t response[96];
    unsigned length = response_data(peer, response, false);
    response[3] ^= 1; // Matching command but wrong subcommand.
    notify(peer, RESPONSE_HANDLE, response, length);
    response[3] ^= 1;
    response[12] ^= 1; // Matching command/subcommand but stale flash address.
    notify(peer, RESPONSE_HANDLE, response, length);
    uint8_t truncated[12] = {GATT_EVENT_NOTIFICATION};
    little_endian_store_16(truncated, 8, RESPONSE_HANDLE);
    little_endian_store_16(truncated, 10, 0xffff);
    event(peer, truncated, sizeof(truncated));
    assert(peer->commands == 1 && ready == 0);
    advance(2000);
    assert(disconnected == 1 && ready == 0 && timer_count == 0 && listeners == 0);
    advance(5000);
    assert(peer->commands == 1);
}

static void test_pairing_gate_and_write_ack_order(void) {
    reset();
    request_writes = true;
    struct fixture_peer* peer = connect_peer(UNI_SW2_PRO_PID, true);
    discover(peer);
    subscribe_response(peer);
    uint8_t response[96];
    unsigned length = response_data(peer, response, false);
    notify(peer, RESPONSE_HANDLE, response, length); // App ACK arrives before ATT write result.
    assert(peer->commands == 1 && !remembered && !ready);
    query_done(peer, 0);
    assert(peer->command[0] == 0x15 && peer->command[3] == 1);
    for (unsigned i = 0; i < 6; ++i)
        assert(peer->command[10 + i] == host_address[5 - i] && peer->command[16 + i] == host_address[5 - i]);
    pairing_allowed = false;
    acknowledge(peer, false);
    assert(disconnected == 1 && remembered == 0 && ready == 0);
    assert(connected_events == 1 && disconnected_events == 1);
    reset();
    peer = connect_peer(UNI_SW2_PRO_PID, true);
    discover(peer);
    subscribe_response(peer);
    storage_ok = false;
    for (unsigned i = 0; i < 5; ++i)
        acknowledge(peer, false); // Info and four app pairing commands.
    assert(remembered == 1 && disconnected == 1 && ready == 0 && timer_count == 0);
    reset();
    peer = connect_peer(UNI_SW2_PRO_PID, false);
    discover(peer);
    subscribe_response(peer);
    length = response_data(peer, response, false);
    response[5] = 0x81;
    notify(peer, RESPONSE_HANDLE, response, length);
    assert(disconnected == 1 && ready == 0);
    reset();
    request_writes = true;
    peer = connect_peer(UNI_SW2_PRO_PID, false);
    discover(peer);
    subscribe_response(peer);
    length = response_data(peer, response, false);
    notify(peer, RESPONSE_HANDLE, response, length);
    query_done(peer, ATT_ERROR_INSUFFICIENT_AUTHENTICATION);
    assert(disconnected == 1 && ready == 0);
}

static void test_calibration_physical_inputs_and_sensor_units(void) {
    reset();
    struct fixture_peer* peer = connect_peer(UNI_SW2_JOYCON_R_PID, false);
    discover(peer);
    subscribe_response(peer);
    acknowledge(peer, false); // Info.
    acknowledge(peer, true); // Missing user calibration falls back to factory.
    assert(little_endian_read_32(peer->command, 12) == 0x130a8);
    finish_setup(peer);
    uint8_t report[63] = {0};
    little_endian_store_32(report, 4, 0x4000 | 0x20 | 0x10 | 0x04);
    packed(report + 10, 4095, 4095); // Nonexistent left stick must stay neutral.
    packed(report + 13, 3100, 400);  // Calibrated +X/-Y full travel.
    notify(peer, INPUT_HANDLE, report, sizeof(report));
    uni_gamepad_t* gp = &peer->device.controller.gamepad;
    assert(gp->axis_x == 0 && gp->axis_y == 0 && gp->axis_rx == 511 && gp->axis_ry == 511);
    assert(gp->buttons == BUTTON_A && gp->misc_buttons == 0);
    assert(uni_hid_parser_switch2_extra_buttons(&peer->device) == (UNI_SW2_BUTTON_C | UNI_SW2_BUTTON_RIGHT_SL | UNI_SW2_BUTTON_RIGHT_SR));
    unsigned previous = emitted;
    notify(peer, INPUT_HANDLE, report, 62);
    assert(emitted == previous && gp->axis_rx == 511);
    little_endian_store_16(report, 48, 4096);
    little_endian_store_16(report, 50, 8192);
    little_endian_store_16(report, 52, (uint16_t)-4096);
    little_endian_store_16(report, 54, 32767);
    little_endian_store_16(report, 56, 0);
    little_endian_store_16(report, 58, 0);
    for (unsigned sample = 0; sample < 46; ++sample) {
        little_endian_store_32(report, 42, 0xffff0000u + sample * 10000u);
        notify(peer, INPUT_HANDLE, report, sizeof(report));
        advance(10);
    }
    assert(gp->accel[0] == 8192 && gp->accel[1] == -8192 && gp->accel[2] == -16384);
    assert(gp->gyro[0] >= 2041740 && gp->gyro[0] <= 2041750);
    notify(peer, INPUT_HANDLE, report, sizeof(report)); // Repeated sensor sample must not become zero.
    assert(gp->gyro[0] >= 2041740);
    reset();
    peer = connect_peer(UNI_SW2_JOYCON_L_PID, false);
    discover(peer);
    subscribe_response(peer);
    finish_setup(peer);
    memset(report, 0, sizeof(report));
    little_endian_store_32(report, 4, 0x300000);
    packed(report + 10, 1900, 2100);
    notify(peer, INPUT_HANDLE, report, sizeof(report));
    assert(peer->device.controller.gamepad.misc_buttons == 0); // Rail buttons are NOT Home/Capture.
    assert(uni_hid_parser_switch2_extra_buttons(&peer->device) == (UNI_SW2_BUTTON_LEFT_SL | UNI_SW2_BUTTON_LEFT_SR));
}

static uint64_t rumble_frame(const struct fixture_peer* peer) {
    uint64_t value = 0;
    for (unsigned i = 0; i < 5; ++i)
        value |= (uint64_t)peer->rumble[2 + i] << (8 * i);
    return value;
}
static bool rumble_active(const struct fixture_peer* peer) {
    uint64_t frame = rumble_frame(peer);
    return ((frame >> 10) & 1023) != 0 || ((frame >> 30) & 1023) != 0;
}
static void test_rumble_delay_expiry_retry_and_teardown(void) {
    reset();
    struct fixture_peer* peer = connect_peer(UNI_SW2_PRO_PID, true);
    discover(peer);
    subscribe_response(peer);
    finish_setup(peer);
    assert(remembered == 1);
    uni_hid_parser_switch2_set_player_leds(&peer->device, 0x0a);
    assert(peer->command[8] == 0x0a);
    acknowledge(peer, false);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 20, 50, 40, 80);
    advance(19);
    assert(peer->rumble_length == 33 && !rumble_active(peer));
    advance(1);
    assert(rumble_active(peer));
    assert(((rumble_frame(peer) >> 10) & 1023) == 320 && ((rumble_frame(peer) >> 30) & 1023) == 160);
    uint8_t successful_id = peer->rumble[1];
    next_write_error = BTSTACK_ACL_BUFFERS_FULL;
    advance(13);
    assert(peer->rumble[1] == successful_id);
    advance(13);
    assert(peer->rumble[1] == (uint8_t)(0x50 | ((successful_id + 1) & 15)) && rumble_active(peer));
    advance(24);
    assert(!rumble_active(peer));
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, UINT16_MAX, 12, 34);
    advance(70000); // Held local feedback does not expire at65.535 seconds.
    assert(((rumble_frame(peer) >> 10) & 1023) == 136 && ((rumble_frame(peer) >> 30) & 1023) == 48);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 100, 255, 255);
    advance(1);
    assert(rumble_active(peer));
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 0, 0, 0);
    advance(1);
    assert(!rumble_active(peer));
    unsigned writes = peer->rumbles;
    uni_hid_device_disconnect(&peer->device);
    assert(disconnected_events == 1);
    uni_hid_parser_switch2_teardown(&peer->device);
    assert(timer_count == 0 && listeners == 0);
    advance(5000);
    assert(peer->rumbles == writes);
    assert(!uni_hid_parser_switch2_identity_address_type(&peer->device, &(uint8_t){0}));
}

static void test_write_request_buffers_and_failed_completion(void) {
    reset();
    request_writes = true;
    struct fixture_peer* peer = connect_peer(UNI_SW2_JOYCON_L_PID, false);
    discover(peer);
    subscribe_response(peer);
    finish_setup(peer);
    advance(1);
    assert(peer->rumble_length == 17 && peer->query == QUERY_WRITE);
    uint8_t pending[33];
    uint16_t length = peer->pending_length;
    const uint8_t* pending_buffer = peer->pending_write;
    memcpy(pending, pending_buffer, length);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, UINT16_MAX, 20, 30);
    uni_hid_parser_switch2_set_player_leds(&peer->device, 0x05);
    advance(13);
    assert(memcmp(pending, pending_buffer, length) == 0); // ATT still borrows this buffer.
    query_done(peer, 0);
    assert(peer->command[0] == 9 && peer->command[8] == 0x05);
    acknowledge(peer, false);
    advance(13);
    assert(rumble_active(peer) && peer->rumble[1] == 0x51);
    query_done(peer, ATT_ERROR_UNLIKELY_ERROR);
    assert(disconnected_events == 1 && timer_count == 0 && listeners == 0);
    unsigned writes = peer->rumbles;
    advance(3000);
    assert(peer->rumbles == writes);
}

static struct fixture_peer* ready_peer(uint16_t pid, bool requests) {
    reset();
    request_writes = requests;
    struct fixture_peer* peer = connect_peer(pid, false);
    discover(peer);
    subscribe_response(peer);
    finish_setup(peer);
    return peer;
}

static uni_switch2_haptics_frame_t native_frame(unsigned left, unsigned right, unsigned seed) {
    uni_switch2_haptics_frame_t frame = {0};
    frame.sides[0].count = left;
    frame.sides[1].count = right;
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned i = 0; i < frame.sides[side].count; ++i) {
            uni_switch2_haptics_encode_sample(frame.sides[side].samples[i],
                10 + seed + 5 * i + side, 80 + seed + i + side,
                1000 + seed * 100 + i * 200 + side * 3000,
                5000 + seed * 100 + i * 700 + side * 500);
        }
    }
    return frame;
}

static uni_switch2_haptics_side_t final_hold(const uni_switch2_haptics_side_t* source) {
    uni_switch2_haptics_side_t side = {.count = 1};
    memcpy(side.samples[0], source->samples[source->count - 1], 5);
    return side;
}

static void assert_block(const struct fixture_peer* peer, unsigned side,
                         const uni_switch2_haptics_side_t* expected) {
    assert(peer->rumble_length >= 17 + side * 16);
    const uint8_t* block = peer->rumble + 1 + side * 16;
    assert((block[0] & 0xf0) == (0x40 | (expected->count << 4)));
    assert(memcmp(block + 1, expected->samples, 5 * expected->count) == 0);
    for (unsigned i = 1 + 5 * expected->count; i < 16; ++i)
        assert(block[i] == 0);
}

static void assert_silent(const struct fixture_peer* peer, unsigned side) {
    const uint8_t* block = peer->rumble + 1 + side * 16;
    assert((block[0] & 0xf0) == 0x50);
    assert(!(block[2] & 0xfc) && !(block[3] & 0x0f) && !(block[4] & 0xc0) && !block[5]);
    for (unsigned i = 6; i < 16; ++i)
        assert(block[i] == 0);
}

static void test_native_fifo_stereo_counts_and_retry(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uni_switch2_haptics_frame_t a = native_frame(3, 2, 1);
    uni_switch2_haptics_frame_t b = native_frame(2, 0, 2);
    uni_switch2_haptics_frame_t c = native_frame(1, 3, 3);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &b, now_ms));
    next_write_error = BTSTACK_ACL_BUFFERS_FULL;
    advance(1);
    assert(peer->rumbles == 0);
    advance(13);
    assert(peer->rumbles == 1 && peer->rumble_length == 33);
    assert(peer->rumble[1] == 0x70 && peer->rumble[17] == 0x60);
    assert_block(peer, 0, &a.sides[0]);
    assert_block(peer, 1, &a.sides[1]);
    // C arrives while A plays, with enough source lifetime for all three frames.
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &c, now_ms));
    advance(15);
    assert(peer->rumbles == 1); // Three samples cannot be interrupted at13ms.
    advance(1);
    assert(peer->rumbles == 2 && peer->rumble[1] == 0x61);
    assert_block(peer, 0, &b.sides[0]);
    uni_switch2_haptics_side_t right = final_hold(&a.sides[1]);
    assert_block(peer, 1, &right);
    advance(10);
    assert(peer->rumbles == 2);
    advance(1);
    assert(peer->rumbles == 3 && peer->rumble[1] == 0x52 && peer->rumble[17] == 0x72);
    assert_block(peer, 0, &c.sides[0]);
    assert_block(peer, 1, &c.sides[1]);
    assert(peer->rumble_times[1] - peer->rumble_times[0] == 16);
    assert(peer->rumble_times[2] - peer->rumble_times[1] == 11);
    assert(uni_hid_parser_switch2_haptics_dropped() == drops);
    advance(23); // C expires at receipt14 +50, not transmission41 +50.
    assert_silent(peer, 0);
    assert_silent(peer, 1);
}

static void test_native_hold_and_absent_side_watchdogs(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uni_switch2_haptics_frame_t a = native_frame(3, 2, 4);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    assert_block(peer, 0, &a.sides[0]);
    advance(15);
    assert(peer->rumbles == 1);
    advance(1);
    uni_switch2_haptics_side_t left = final_hold(&a.sides[0]);
    uni_switch2_haptics_side_t right = final_hold(&a.sides[1]);
    assert(peer->rumbles == 2);
    assert_block(peer, 0, &left);
    assert_block(peer, 1, &right);
    advance(3); // t20; only the left actuator receives a fresh source update.
    uni_switch2_haptics_frame_t b = native_frame(1, 0, 5);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &b, now_ms));
    advance(3); // Last count1 hold guarded until t23.
    assert_block(peer, 0, &b.sides[0]);
    assert_block(peer, 1, &right);
    advance(27);
    assert_block(peer, 0, &b.sides[0]);
    assert_silent(peer, 1); // Absent side must not gain a new50ms watchdog.
    advance(20);
    assert_silent(peer, 0);
    assert_silent(peer, 1);
}

static void test_feedback_advances_host_and_resumes_only_final_samples(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    uni_switch2_haptics_frame_t a = native_frame(2, 3, 6);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 40, 20, 30);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    assert(((rumble_frame(peer) >> 10) & 1023) == 120);
    assert((peer->rumble[1] & 0xf0) == 0x50);
    advance(9);
    uni_switch2_haptics_frame_t b = native_frame(3, 0, 7);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &b, now_ms));
    advance(1);
    assert(((rumble_frame(peer) >> 10) & 1023) == 120);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 0, 0, 0);
    advance(1);
    uni_switch2_haptics_side_t left = final_hold(&b.sides[0]);
    uni_switch2_haptics_side_t right = final_hold(&a.sides[1]);
    assert_block(peer, 0, &left);
    assert_block(peer, 1, &right);
    for (unsigned i = 0; i < peer->rumbles; ++i) {
        assert((peer->rumble_history[i][1] & 0xf0) == 0x50);
        assert((peer->rumble_history[i][17] & 0xf0) == 0x50);
    }
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 100, 50, 60);
    advance(48); // Both original host watchdogs elapse under feedback.
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 0, 0, 0);
    advance(1);
    assert_silent(peer, 0);
    assert_silent(peer, 1);
    assert(uni_hid_parser_switch2_haptics_dropped() == drops);
}

static void test_full_fifo_stop_barrier_and_async_completion(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_JOYCON_R_PID, true);
    uni_switch2_haptics_frame_t a = native_frame(3, 0, 8);
    uni_switch2_haptics_frame_t b = native_frame(2, 0, 9);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    assert(peer->query == QUERY_WRITE && peer->rumble_length == 17);
    const uint8_t* borrowed = peer->pending_write;
    uint8_t snapshot[17];
    memcpy(snapshot, borrowed, sizeof(snapshot));
    for (unsigned i = 1; i < 16; ++i)
        assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    assert(!uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    assert(uni_hid_parser_switch2_haptics_dropped() == drops);
    uni_switch2_haptics_frame_t stop;
    uni_switch2_haptics_silence(&stop);
    stop.sides[1].count = 0; // Physical Joy-Con stop is not a logical stereo stop.
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &stop, now_ms - 100));
    assert(uni_hid_parser_switch2_haptics_dropped() == drops + 16);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &b, now_ms));
    advance(1);
    assert(memcmp(snapshot, borrowed, sizeof(snapshot)) == 0);
    query_done(peer, 0); // Completion of old epoch must not consume b/the stop.
    advance(1);
    assert(peer->rumbles == 2 && peer->rumble[1] == 0x51);
    assert_silent(peer, 0);
    query_done(peer, 0);
    advance(6);
    assert(peer->rumbles == 3 && peer->rumble[1] == 0x62);
    assert_block(peer, 0, &b.sides[0]);
    query_done(peer, 0);
    advance(11);
    uni_switch2_haptics_side_t hold = final_hold(&b.sides[0]);
    assert_block(peer, 0, &hold);
    query_done(peer, 0);
}

static void test_async_expiry_and_topology_reset_do_not_revive_state(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, true);
    uni_switch2_haptics_frame_t a = native_frame(3, 2, 10);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    advance(49); // Write request is still borrowing the original batch.
    assert(peer->rumbles == 1 && uni_hid_parser_switch2_haptics_dropped() == drops + 1);
    query_done(peer, 0);
    advance(1);
    assert_silent(peer, 0);
    assert_silent(peer, 1);
    query_done(peer, 0);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, UINT16_MAX, 40, 50);
    advance(6);
    assert(rumble_active(peer) && peer->query == QUERY_WRITE);
    uni_switch2_haptics_frame_t b = native_frame(2, 1, 11);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    uni_hid_parser_switch2_reset_haptics(&peer->device);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &b, now_ms));
    advance(1);
    query_done(peer, 0); // Old local overlay must not clear the reset neutral.
    advance(1);
    assert_silent(peer, 0);
    assert_silent(peer, 1);
    query_done(peer, 0);
    advance(6);
    assert_block(peer, 0, &b.sides[0]);
    assert_block(peer, 1, &b.sides[1]);
    query_done(peer, 0);
}

static void test_host_stop_does_not_cancel_local_feedback(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, UINT16_MAX, 100, 200);
    uni_switch2_haptics_frame_t a = native_frame(3, 3, 12);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    assert(uni_hid_parser_switch2_queue_rumble(&peer->device, 0, 0, 0, now_ms));
    advance(1);
    assert(((rumble_frame(peer) >> 10) & 1023) == 800);
    uni_hid_parser_switch2_play_dual_rumble(&peer->device, 0, 0, 0, 0);
    advance(1);
    assert_silent(peer, 0);
    assert_silent(peer, 1);
}

static void test_conventional_host_lifetimes_and_invalid_native_input(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_JOYCON_L_PID, false);
    assert(uni_hid_parser_switch2_queue_rumble(&peer->device, 255, 200, UINT16_MAX, now_ms));
    advance(70000);
    assert((rumble_frame(peer) & 1023) == 0xe1);
    assert(((rumble_frame(peer) >> 20) & 1023) == 0x1e1);
    assert(((rumble_frame(peer) >> 10) & 1023) == 800);
    assert(((rumble_frame(peer) >> 30) & 1023) == 1020);
    assert((peer->rumble[1] & 0xf0) == 0x50);
    for (unsigned i = 7; i < 17; ++i)
        assert(peer->rumble[i] == 0);
    assert(uni_hid_parser_switch2_queue_rumble(&peer->device, 1, 2, 20, now_ms - 10));
    advance(9);
    assert(((rumble_frame(peer) >> 10) & 1023) == 8);
    advance(1);
    assert_silent(peer, 0);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    uni_switch2_haptics_frame_t invalid = native_frame(1, 0, 13);
    invalid.sides[0].count = 4;
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &invalid, now_ms));
    uni_switch2_haptics_frame_t stale = native_frame(1, 0, 14);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &stale, now_ms - 50));
    assert(uni_hid_parser_switch2_queue_rumble(&peer->device, 10, 20, 100, now_ms - 50));
    assert(uni_hid_parser_switch2_haptics_dropped() == drops + 3);
    advance(13);
    assert_silent(peer, 0);
}

static void test_native_watchdog_clock_wrap(void) {
    reset();
    now_ms = UINT32_MAX - 20;
    struct fixture_peer* peer = connect_peer(UNI_SW2_JOYCON_L_PID, false);
    discover(peer);
    subscribe_response(peer);
    finish_setup(peer);
    uni_switch2_haptics_frame_t frame = native_frame(1, 0, 15);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    advance(49);
    assert_block(peer, 0, &frame.sides[0]);
    advance(1);
    assert_silent(peer, 0);
}

static void test_identical_hold_coalescing_refreshes_borrowed_head(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_JOYCON_L_PID, true);
    uni_switch2_haptics_frame_t frame = native_frame(1, 0, 16);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    advance(1);
    const uint8_t* borrowed = peer->pending_write;
    uint8_t snapshot[17];
    memcpy(snapshot, borrowed, sizeof(snapshot));
    advance(39);
    for (unsigned i = 0; i < 32; ++i)
        assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    assert(memcmp(snapshot, borrowed, sizeof(snapshot)) == 0);
    advance(10); // The original t0 hold would expire now without source refresh.
    assert(uni_hid_parser_switch2_haptics_dropped() == drops);
    query_done(peer, 0);
    advance(39);
    assert_block(peer, 0, &frame.sides[0]);
    query_done(peer, 0);
    advance(1); // Refreshed deadline remains t40+50, not ATT completion+50.
    assert_silent(peer, 0);
    assert(uni_hid_parser_switch2_haptics_dropped() == drops);
    query_done(peer, 0);
}

static void test_hold_coalescing_requires_identical_samples_and_side_masks(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, true);
    uni_switch2_haptics_frame_t frame;
    for (unsigned i = 0; i < 16; ++i) {
        frame = native_frame(1, 1, i + 1);
        assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    }
    // An identical tail hold can refresh even at capacity. A partial-side update
    // is a different command, as is a changed sample; neither may erase history.
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    frame.sides[1].count = 0;
    assert(!uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    frame = native_frame(1, 1, 17);
    assert(!uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    advance(1);
    uni_switch2_haptics_frame_t first = native_frame(1, 1, 1);
    assert_block(peer, 0, &first.sides[0]);
    assert_block(peer, 1, &first.sides[1]);
}

static void test_expired_history_does_not_starve_fresh_sequences(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uni_switch2_haptics_frame_t a = native_frame(3, 3, 20);
    uni_switch2_haptics_frame_t stale = native_frame(3, 3, 21);
    uni_switch2_haptics_frame_t fresh = native_frame(3, 3, 22);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms));
    advance(1);
    unsigned sent = peer->rumbles;
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &stale, now_ms - 40));
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &fresh, now_ms));
    advance(10);
    assert(peer->rumbles == sent); // Unplayed stale history must not interrupt A.
    advance(6);
    assert_block(peer, 0, &fresh.sides[0]);
    assert_block(peer, 1, &fresh.sides[1]);

    peer = ready_peer(UNI_SW2_PRO_PID, false);
    a = native_frame(1, 1, 23);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &a, now_ms - 43));
    advance(1);
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &fresh, now_ms));
    advance(6); // A's source watchdog expires as its playback guard ends.
    assert_block(peer, 0, &fresh.sides[0]); // No unnecessary silent/HOLD packet.

    peer = ready_peer(UNI_SW2_PRO_PID, false);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &fresh, now_ms - 40));
    advance(1);
    assert_silent(peer, 0); // Nine ms cannot contain a complete three-frame batch.
    assert(uni_hid_parser_switch2_haptics_dropped() == drops + 1);
}

static void test_gatt_busy_retries_without_disconnect_or_sequence_loss(void) {
    struct fixture_peer* peer = ready_peer(UNI_SW2_PRO_PID, false);
    uni_switch2_haptics_frame_t frame = native_frame(3, 2, 24);
    uint32_t drops = uni_hid_parser_switch2_haptics_dropped();
    next_write_error = GATT_CLIENT_BUSY;
    assert(uni_hid_parser_switch2_queue_haptics(&peer->device, &frame, now_ms));
    advance(1);
    assert(!disconnected && peer->rumbles == 0);
    advance(13);
    assert(!disconnected && peer->rumbles == 1);
    assert((peer->rumble[1] & 15) == 0);
    assert_block(peer, 0, &frame.sides[0]);
    assert_block(peer, 1, &frame.sides[1]);

    // The SDK uses the same busy status on command writes such as player LEDs.
    next_write_error = GATT_CLIENT_BUSY;
    uni_hid_parser_switch2_set_player_leds(&peer->device, 3);
    assert(!disconnected);
    advance(13);
    assert(peer->command[0] == 0x09 && peer->command[8] == 3);
    acknowledge(peer, false);
    assert(!disconnected && uni_hid_parser_switch2_haptics_dropped() == drops);
}

static void test_immediate_connection_failure_preserves_reconnect_discovery(void) {
    reset();
    uint8_t packet[64];
    size_t size = advertisement(packet, UNI_SW2_JOYCON_R_PID, false);
    next_connect_error = ERROR_CODE_COMMAND_DISALLOWED;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(scan_running && !peers[0].used && !timer_count && !listeners);
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(peers[0].used && connected == 2); // The same remembered peer can retry.

    reset();
    scan_enabled = scan_running = false;
    next_connect_error = ERROR_CODE_COMMAND_DISALLOWED;
    assert(uni_bt_le_switch2_handle_advertisement(packet, size));
    assert(!scan_running && !peers[0].used); // Explicit stop must not be overridden.
}

int main(void) {
    test_connected_callback_rejection();
    test_advertisement_bounds_and_admission();
    test_discovery_metadata_and_rejection_isolation();
    test_immediate_connection_failure_preserves_reconnect_discovery();
    test_missing_descriptor_and_setup_timeout();
    test_pairing_gate_and_write_ack_order();
    test_calibration_physical_inputs_and_sensor_units();
    test_rumble_delay_expiry_retry_and_teardown();
    test_write_request_buffers_and_failed_completion();
    test_native_fifo_stereo_counts_and_retry();
    test_native_hold_and_absent_side_watchdogs();
    test_feedback_advances_host_and_resumes_only_final_samples();
    test_full_fifo_stop_barrier_and_async_completion();
    test_async_expiry_and_topology_reset_do_not_revive_state();
    test_host_stop_does_not_cancel_local_feedback();
    test_conventional_host_lifetimes_and_invalid_native_input();
    test_native_watchdog_clock_wrap();
    test_identical_hold_coalescing_refreshes_borrowed_head();
    test_hold_coalescing_requires_identical_samples_and_side_masks();
    test_expired_history_does_not_starve_fresh_sequences();
    test_gatt_busy_retries_without_disconnect_or_sequence_loss();
    reset();
    puts("Switch2 protocol boundaries, setup failure, pairing, calibration, physical input, motion and rumble passed");
    return 0;
}
