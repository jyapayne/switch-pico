#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "parser/uni_hid_parser_ds5.h"
#include "parser/uni_hid_parser_native_motion.h"
#include "uni_hid_device.h"
#include "uni_utils.h"

// Real staged DS5 parser; substitute only the radio, virtual mouse and clock.
static uni_hid_device_t device;
static unsigned ready_count;
static uint8_t requested_feature;
static uint8_t output[79];
static unsigned output_count;
static uint32_t now_ms;
static bool transport_available = true;
static int send_status = ERROR_CODE_SUCCESS;
static unsigned disconnect_count;
static struct {
    btstack_timer_source_t* timer;
    uint32_t deadline;
    bool active;
} timers[2];

void uni_log(const char* fmt, ...) { (void)fmt; }
void uni_hid_device_send_ctrl_report(uni_hid_device_t* d, const uint8_t* bytes, uint16_t len) {
    assert(d == &device && len == 2 && bytes[0] == 0x43);
    requested_feature = bytes[1];
}
void uni_hid_device_send_intr_report(uni_hid_device_t* d, const uint8_t* bytes, uint16_t len) {
    assert(d == &device && len == sizeof(output));
    memcpy(output, bytes, len);
    ++output_count;
}
uint32_t btstack_run_loop_get_time_ms(void) { return now_ms; }
int l2cap_can_send_packet_now(uint16_t cid) {
    assert(cid == device.conn.interrupt_cid);
    return transport_available;
}
int l2cap_send(uint16_t cid, uint8_t* bytes, uint16_t len) {
    assert(cid == device.conn.interrupt_cid);
    if (send_status != ERROR_CODE_SUCCESS) return send_status;
    uni_hid_device_send_intr_report(&device, bytes, len);
    return ERROR_CODE_SUCCESS;
}
void uni_hid_device_disconnect(uni_hid_device_t* d) {
    assert(d == &device);
    ++disconnect_count;
    uni_hid_parser_ds5_bridge_teardown(d);
}
bool uni_hid_device_set_ready_complete(uni_hid_device_t* d) {
    assert(d == &device);
    ++ready_count;
    return true;
}
uni_hid_device_t* uni_hid_device_create_virtual(uni_hid_device_t* d) { (void)d; return NULL; }
void uni_hid_device_set_cod(uni_hid_device_t* d, uint32_t cod) { (void)d; (void)cod; }
void uni_hid_device_connect(uni_hid_device_t* d) { (void)d; }
void uni_hid_device_process_controller(uni_hid_device_t* d) { (void)d; }
// The common accessor must not select a different family in this fixture.
void uni_hid_parser_wii_setup(uni_hid_device_t* d) { (void)d; assert(false); }
bool uni_hid_parser_wii_accel_snapshot(uni_hid_device_t* d, int32_t v[3], uint32_t* seq) {
    (void)d; (void)v; (void)seq; assert(false); return false;
}
bool uni_hid_parser_wii_gyro_snapshot(uni_hid_device_t* d, int32_t v[3], uint32_t* seq) {
    (void)d; (void)v; (void)seq; assert(false); return false;
}
uint8_t uni_hid_parser_hat_to_dpad(uint8_t hat) {
    const uint8_t values[8] = {DPAD_UP, DPAD_UP | DPAD_RIGHT, DPAD_RIGHT,
        DPAD_RIGHT | DPAD_DOWN, DPAD_DOWN, DPAD_DOWN | DPAD_LEFT,
        DPAD_LEFT, DPAD_LEFT | DPAD_UP};
    return hat < 8 ? values[hat] : 0;
}

void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t ms) {
    for (unsigned i = 0; i < 2; ++i) {
        if (timers[i].timer != timer && timers[i].timer != NULL) continue;
        timers[i].timer = timer;
        timers[i].deadline = now_ms + ms;
        return;
    }
    assert(false);
}
void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < 2; ++i)
        if (timers[i].timer == timer) { timers[i].active = true; return; }
    assert(false);
}
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < 2; ++i) {
        if (timers[i].timer != timer) continue;
        const bool was_active = timers[i].active;
        timers[i].active = false;
        return was_active;
    }
    return false;
}
static void advance(uint32_t time) {
    now_ms = time;
    for (unsigned i = 0; i < 2; ++i) {
        if (!timers[i].active || (int32_t)(now_ms - timers[i].deadline) < 0) continue;
        timers[i].active = false;
        timers[i].timer->process(timers[i].timer);
    }
}
static void put16(uint8_t* bytes, int16_t value) {
    bytes[0] = (uint16_t)value;
    bytes[1] = (uint16_t)value >> 8;
}
static void put32(uint8_t* bytes, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) bytes[i] = value >> (8 * i);
}
static void seal(uint8_t* bytes, size_t size, uint8_t transaction) {
    uint32_t crc = uni_crc32_le(UINT32_MAX, &transaction, 1);
    crc = ~uni_crc32_le(crc, bytes, size - 4);
    put32(bytes + size - 4, crc);
}
static void feature(uint8_t* bytes, uint16_t len) {
    seal(bytes, len, 0xa3);
    uni_hid_parser_ds5_parse_feature_report(&device, bytes, len);
}
static void calibration(uint8_t bytes[41], bool fallback) {
    memset(bytes, 0, 41);
    bytes[0] = 5;
    const int16_t bias[3] = {10, -20, 30};
    for (unsigned axis = 0; axis < 3; ++axis) {
        put16(bytes + 1 + axis * 2, bias[axis]);
        put16(bytes + 7 + axis * 4, bias[axis] + 100);
        put16(bytes + 9 + axis * 4, bias[axis] - 100);
        put16(bytes + 23 + axis * 4, 8192);
        put16(bytes + 25 + axis * 4, -8192);
    }
    put16(bytes + 19, 100);
    put16(bytes + 21, 100);
    if (fallback) {
        put16(bytes + 23, 0);
        put16(bytes + 25, 0);
    }
}
static void input(uint8_t bytes[78], uint32_t timestamp) {
    memset(bytes, 0, 78);
    bytes[0] = 0x31;
    bytes[2] = bytes[3] = bytes[4] = bytes[5] = 127;
    bytes[9] = 0x28; // Cross + neutral hat.
    bytes[11] = 0x02; // Touchpad click, not mute.
    put16(bytes + 17, 11);
    put16(bytes + 19, -20);
    put16(bytes + 21, 30);
    put16(bytes + 25, 8192);
    put32(bytes + 29, timestamp);
    seal(bytes, 78, 0xa1);
}
static uni_native_motion_snapshot_t feed(uint8_t* bytes, uint16_t len, bool admitted) {
    uni_hid_parser_ds5_init_report(&device);
    uni_hid_parser_ds5_parse_input_report(&device, bytes, len);
    uni_native_motion_snapshot_t snapshot;
    assert(uni_hid_parser_native_motion_snapshot(&device, &snapshot));
    assert(snapshot.report_tracked && snapshot.report_valid == admitted);
    return snapshot;
}

int main(void) {
    device.controller_type = CONTROLLER_TYPE_PS5Controller;
    device.product_id = 0x0df2; // Edge takes the real PS5 path.
    device.report_parser.setup = uni_hid_parser_ds5_setup;
    device.conn.interrupt_cid = 0x40;
    uni_hid_parser_ds5_setup(&device);
    assert(requested_feature == 9);
    uint8_t pairing[20] = {9};
    feature(pairing, sizeof(pairing));
    assert(requested_feature == 0x20);
    uint8_t firmware[64] = {0x20};
    feature(firmware, sizeof(firmware));
    assert(requested_feature == 5);
    uint8_t calib[41];
    calibration(calib, false);
    seal(calib, sizeof(calib), 0xa3);
    uni_hid_parser_ds5_parse_feature_report(&device, calib, 40);
    assert(ready_count == 0); // A partial feature cannot initialize calibration.
    calib[25] ^= 1;
    uni_hid_parser_ds5_parse_feature_report(&device, calib, sizeof(calib));
    assert(ready_count == 0); // Nor can a full feature with a corrupt CRC.
    calibration(calib, true);
    feature(calib, sizeof(calib));
    assert(ready_count == 1);
    uint8_t report[78];
    input(report, 100);
    uni_native_motion_snapshot_t snapshot = feed(report, sizeof(report), true);
    assert(!snapshot.accel_valid && !snapshot.gyro_valid);
    assert(device.controller.gamepad.buttons & BUTTON_A); // Controls survive fallback.
    calibration(calib, false);
    feature(calib, sizeof(calib));
    snapshot = feed(report, sizeof(report), true);
    assert(!snapshot.accel_valid && !snapshot.gyro_valid); // Calibration alone is not fresh motion.
    input(report, 101);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_valid && snapshot.gyro_valid);
    assert(device.controller.gamepad.gyro[0] == 1024 && device.controller.gamepad.gyro[1] == 0);
    assert(device.controller.gamepad.accel[1] == 8192);
    assert(device.controller.gamepad.misc_buttons & MISC_BUTTON_CAPTURE);
    uni_native_motion_snapshot_t common;
    assert(uni_hid_parser_native_motion_snapshot(&device, &common));
    assert(common.accel_valid && common.gyro_valid &&
           common.accel_q13[1] == 8192 && common.gyro_q10[0] == 1024);
    const uint32_t common_sequence = common.accel_sequence;
    const uint32_t report_sequence = snapshot.report_sequence;
    uni_native_motion_snapshot_t polled;
    assert(uni_hid_parser_native_motion_snapshot(&device, &polled) && polled.report_sequence == report_sequence);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_sequence == common_sequence && !snapshot.accel_valid && !snapshot.gyro_valid);
    input(report, 99);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_sequence == common_sequence && !snapshot.accel_valid && !snapshot.gyro_valid);
    input(report, 102);
    feed(report, 77, false);
    report[9] ^= 0x20;
    feed(report, sizeof(report), false);
    feed(NULL, 0, false);
    input(report, 102);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_valid && snapshot.gyro_valid && snapshot.accel_sequence != common_sequence);

    // A real uint32 sensor-clock wrap is forward progress, not a duplicate.
    uni_hid_parser_ds5_setup(&device);
    calibration(calib, false);
    feature(calib, sizeof(calib));
    input(report, UINT32_MAX - 15);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_valid && snapshot.gyro_valid);
    assert(uni_hid_parser_native_motion_snapshot(&device, &common));
    assert(common.accel_valid && common.accel_sequence != common_sequence);
    input(report, 16);
    snapshot = feed(report, sizeof(report), true);
    assert(snapshot.accel_valid && snapshot.gyro_valid && snapshot.accel_sequence != common.accel_sequence);

    const unsigned before_busy = output_count;
    transport_available = false;
    assert(!uni_hid_parser_ds5_bridge_rumble(&device, 60, 31, 217) && output_count == before_busy);
    transport_available = true;
    send_status = BTSTACK_ACL_BUFFERS_FULL;
    assert(!uni_hid_parser_ds5_bridge_rumble(&device, 60, 31, 217) && output_count == before_busy);
    send_status = ERROR_CODE_SUCCESS;
    assert(uni_hid_parser_ds5_bridge_rumble(&device, 60, 31, 217));
    assert(output[6] == 31 && output[7] == 217); // Wire motor right/left, not callback echoes.
    advance(59);
    assert(output[6] == 31 && output[7] == 217);
    advance(60);
    assert(output[6] == 0 && output[7] == 0); // Real parser's finite duration timer stops both.
    uni_hid_parser_ds5_play_dual_rumble(&device, 100, 1000, 90, 0);
    const unsigned sent = output_count;
    uni_hid_parser_ds5_bridge_teardown(&device);
    advance(2000);
    assert(output_count == sent);
    assert(!uni_hid_parser_native_motion_snapshot(&device, &common));
    assert(!common.report_valid && !common.accel_valid && !common.gyro_valid);
    uni_hid_parser_ds5_play_dual_rumble(&device, 0, 1000, 0, 90);
    const unsigned active_sent = output_count;
    uni_hid_parser_ds5_bridge_teardown(&device);
    advance(4000);
    assert(output_count == active_sent); // No timer callback into reused parser memory.
    assert(uni_hid_parser_ds5_bridge_rumble(&device, 60, 31, 217));
    transport_available = false;
    advance(4060);
    assert(output[6] == 31 && output[7] == 217 && disconnect_count == 0);
    transport_available = true;
    advance(4065);
    assert(output[6] == 0 && output[7] == 0 && disconnect_count == 0);
    assert(uni_hid_parser_ds5_bridge_rumble(&device, 60, 31, 217));
    transport_available = false;
    advance(4125);
    advance(6125);
    assert(disconnect_count == 1); // OFF cannot stall forever on a live link.
    const unsigned after_disconnect = output_count;
    advance(9000);
    assert(output_count == after_disconnect);
    puts("DualSense calibrated admission and bounded driver lifetime passed");
    return 0;
}
