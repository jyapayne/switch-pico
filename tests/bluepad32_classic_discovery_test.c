#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include <btstack.h>
#include "bt/uni_bt.h"
#include "bt/uni_bt_bredr.h"
#include "bt/uni_bt_sdp.h"
#include "uni_hid_device.h"

static const bd_addr_t paired = {1, 2, 3, 4, 5, 6};
static const bd_addr_t unknown = {7, 8, 9, 10, 11, 12};
static bool bondable;
static bool allocated;
static unsigned connection_attempts;
static bd_addr_t attempted_address;
static uni_hid_device_t device;

int gap_get_bondable_mode(void) { return bondable; }
bool gap_get_link_key_for_bd_addr(bd_addr_t address, link_key_t key, link_key_type_t* type) {
    if (memcmp(address, paired, sizeof(bd_addr_t)) != 0) return false;
    memset(key, 0x55, sizeof(link_key_t));
    *type = COMBINATION_KEY;
    return true;
}

void uni_log(const char* format, ...) { (void)format; }
void btstack_assert_failed(const char* file, uint16_t line) {
    (void)file;
    (void)line;
    assert(false);
}

uni_error_t uni_hid_device_on_device_discovered(bd_addr_t address, const char* name,
                                               uint16_t cod, uint8_t rssi) {
    (void)address; (void)name; (void)cod; (void)rssi;
    return UNI_ERROR_SUCCESS; // The platform has room and permits discovery.
}
uni_hid_device_t* uni_hid_device_get_instance_for_address(bd_addr_t address) {
    return allocated && memcmp(address, device.conn.btaddr, sizeof(bd_addr_t)) == 0 ? &device : NULL;
}
uni_hid_device_t* uni_hid_device_create(bd_addr_t address) {
    assert(!allocated);
    allocated = true;
    memset(&device, 0, sizeof(device));
    memcpy(device.conn.btaddr, address, sizeof(bd_addr_t));
    return &device;
}
void uni_hid_device_set_cod(uni_hid_device_t* d, uint32_t cod) { d->cod = cod; }
void uni_hid_device_set_name(uni_hid_device_t* d, const char* name) {
    assert(strlen(name) < sizeof(d->name));
    strcpy(d->name, name);
}
bool uni_hid_device_has_name(const uni_hid_device_t* d) { return d->name[0] != 0; }
bool uni_hid_device_is_incoming(const uni_hid_device_t* d) { return d->conn.incoming; }
bool uni_hid_device_guess_controller_type_from_name(uni_hid_device_t* d, const char* name) {
    (void)d; (void)name;
    return false;
}
void uni_hid_device_set_ready(uni_hid_device_t* d) { (void)d; assert(false); }
void uni_bt_sdp_query_start(uni_hid_device_t* d) { (void)d; assert(false); }
void uni_bt_sdp_query_start_hid_descriptor(uni_hid_device_t* d) { (void)d; assert(false); }
int gap_remote_name_request(const bd_addr_t address, uint8_t mode, uint16_t offset) {
    (void)address; (void)mode; (void)offset;
    assert(false); // The real inquiry event below already contains a name.
    return ERROR_CODE_COMMAND_DISALLOWED;
}
void uni_bt_packet_handler(uint8_t type, uint16_t channel, uint8_t* packet, uint16_t size) {
    (void)type; (void)channel; (void)packet; (void)size;
    assert(false);
}
uint8_t l2cap_create_channel(btstack_packet_handler_t handler, bd_addr_t address,
                             uint16_t psm, uint16_t mtu, uint16_t* cid) {
    (void)handler; (void)mtu;
    assert(psm == BLUETOOTH_PSM_HID_CONTROL);
    memcpy(attempted_address, address, sizeof(bd_addr_t));
    ++connection_attempts;
    *cid = 0x40;
    return ERROR_CODE_SUCCESS;
}

static void discover(const bd_addr_t address) {
    // Real GAP_EVENT_INQUIRY_RESULT layout, decoded by BTstack accessors.
    static const char name[] = "Test gamepad";
    uint8_t event[27 + sizeof(name) - 1] = {GAP_EVENT_INQUIRY_RESULT, sizeof(event) - 2};
    for (unsigned i = 0; i < sizeof(bd_addr_t); ++i) event[2 + i] = address[5 - i];
    event[8] = 1;
    event[9] = 0x08; event[10] = 0x25; // Peripheral/gamepad class.
    event[14] = 1; event[15] = 220;
    event[25] = 1; event[26] = sizeof(name) - 1;
    memcpy(event + 27, name, sizeof(name) - 1);
    uni_bt_bredr_on_gap_inquiry_result(0, event, sizeof(event));
}

int main(void) {
    bondable = false;
    discover(unknown);
    assert(connection_attempts == 0 && !allocated);

    discover(paired);
    assert(connection_attempts == 1 && memcmp(attempted_address, paired, sizeof(bd_addr_t)) == 0);

    allocated = false;
    bondable = true;
    discover(unknown);
    assert(connection_attempts == 2 && memcmp(attempted_address, unknown, sizeof(bd_addr_t)) == 0);

    // Closing the pairing window must reject the same still-unpaired peer.
    allocated = false;
    bondable = false;
    discover(unknown);
    assert(connection_attempts == 2 && !allocated);
    return 0;
}
