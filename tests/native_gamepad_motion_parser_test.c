#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "parser/uni_hid_parser_ds4.h"
#include "parser/uni_hid_parser_psmove.h"
#include "parser/uni_hid_parser_switch.h"
#include "parser/uni_hid_parser_native_motion.h"
#include "uni_hid_device.h"
#include "uni_utils.h"

// Real staged parsers and normalization. Only radio, clock and platform edges
// are substituted; fixtures enter public setup/feature/input APIs.
static uint8_t command[128];
static unsigned command_len, ready_count;
void uni_log(const char* fmt, ...) { (void)fmt; }
void printf_hexdump(const void* data, int len) { (void)data; (void)len; }
void uni_hid_device_send_ctrl_report(uni_hid_device_t* d, const uint8_t* bytes, uint16_t len) {
    (void)d; assert(len <= sizeof(command)); memcpy(command, bytes, len); command_len = len;
}
void uni_hid_device_send_intr_report(uni_hid_device_t* d, const uint8_t* bytes, uint16_t len) {
    uni_hid_device_send_ctrl_report(d, bytes, len);
}
int l2cap_can_send_packet_now(uint16_t cid) { (void)cid; return 1; }
int l2cap_send(uint16_t cid, uint8_t* bytes, uint16_t len) {
    (void)cid; uni_hid_device_send_intr_report(NULL, bytes, len); return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_request_can_send_now_event(uint16_t cid) { (void)cid; return 0; }
uint32_t btstack_run_loop_get_time_ms(void) { return 0; }
void btstack_run_loop_set_timer(btstack_timer_source_t* t, uint32_t ms) { (void)t; (void)ms; }
void btstack_run_loop_add_timer(btstack_timer_source_t* t) { (void)t; }
bool btstack_run_loop_remove_timer(btstack_timer_source_t* t) { (void)t; return false; }
void btstack_run_loop_set_timer_context(btstack_timer_source_t* t, void* p) { t->context = p; }
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* t, void (*fn)(btstack_timer_source_t*)) { t->process = fn; }
void* btstack_run_loop_get_timer_context(btstack_timer_source_t* t) { return t->context; }
bool uni_hid_device_set_ready_complete(uni_hid_device_t* d) { (void)d; ++ready_count; return true; }
uni_hid_device_t* uni_hid_device_create_virtual(uni_hid_device_t* d) { (void)d; return NULL; }
void uni_hid_device_set_cod(uni_hid_device_t* d, uint32_t cod) { (void)d; (void)cod; }
void uni_hid_device_connect(uni_hid_device_t* d) { (void)d; }
void uni_hid_device_process_controller(uni_hid_device_t* d) { (void)d; }
void uni_hid_device_set_product_id(uni_hid_device_t* d, uint16_t pid) { d->product_id = pid; }
void uni_hid_device_set_vendor_id(uni_hid_device_t* d, uint16_t vid) { d->vendor_id = vid; }
uint8_t uni_hid_parser_hat_to_dpad(uint8_t hat) { return hat == 0 ? DPAD_UP : 0; }
void uni_hid_parser_wii_setup(uni_hid_device_t* d) { (void)d; assert(false); }
bool uni_hid_parser_wii_accel_snapshot(uni_hid_device_t* d, int32_t v[3], uint32_t* s) {
    (void)d; (void)v; (void)s; assert(false); return false;
}
bool uni_hid_parser_wii_gyro_snapshot(uni_hid_device_t* d, int32_t v[3], uint32_t* s) {
    (void)d; (void)v; (void)s; assert(false); return false;
}
static void put16(uint8_t* bytes, int value) {
    bytes[0] = (uint16_t)value; bytes[1] = (uint16_t)value >> 8;
}
static uni_native_motion_snapshot_t snapshot(uni_hid_device_t* d) {
    uni_native_motion_snapshot_t result;
    assert(uni_hid_parser_native_motion_snapshot(d, &result));
    return result;
}
static void ds4_seal(uint8_t bytes[78]) {
    const uint8_t transaction = 0xa1;
    uint32_t crc = ~uni_crc32_le(uni_crc32_le(UINT32_MAX, &transaction, 1), bytes, 74);
    for (unsigned i = 0; i < 4; ++i) bytes[74 + i] = crc >> (8 * i);
}
static void ds4_input(uni_hid_device_t* d, uint8_t bytes[78], uint16_t tick) {
    put16(bytes + 12, tick);
    ds4_seal(bytes);
    uni_hid_parser_ds4_init_report(d);
    uni_hid_parser_ds4_parse_input_report(d, bytes, 78);
}
static void ds4_provenance(void) {
    uni_hid_device_t d = {0};
    d.report_parser.setup = uni_hid_parser_ds4_setup;
    uni_hid_parser_ds4_setup(&d);
    uint8_t bytes[78] = {0x11};
    bytes[7] = 0x28; // Cross, neutral hat.
    put16(bytes + 19, 8192); // Gyro Z.
    put16(bytes + 23, 8192); // Acceleration Y.
    ds4_input(&d, bytes, 65530);
    uni_native_motion_snapshot_t first = snapshot(&d);
    assert(first.report_valid && !first.accel_valid && !first.gyro_valid);
    assert(d.controller.gamepad.buttons & BUTTON_A);
    uint8_t calibration[37] = {2};
    for (unsigned i = 0; i < 3; ++i) {
        put16(calibration + 7 + 2 * i, 100);
        put16(calibration + 13 + 2 * i, -100);
        put16(calibration + 23 + 4 * i, 8192);
        put16(calibration + 25 + 4 * i, -8192);
    }
    put16(calibration + 19, 100); put16(calibration + 21, 100);
    uni_hid_parser_ds4_parse_feature_report(&d, calibration, 36);
    ds4_input(&d, bytes, 65531);
    assert(!snapshot(&d).accel_valid);
    uni_hid_parser_ds4_parse_feature_report(&d, calibration, 37);
    ds4_input(&d, bytes, 65531);
    assert(!snapshot(&d).accel_valid); // Calibration cannot rejuvenate a cached tick.
    ds4_input(&d, bytes, 2); // Wrapped sensor clock is forward progress.
    first = snapshot(&d);
    assert(first.accel_valid && first.gyro_valid && first.accel_q13[1] == 8192);
    ds4_input(&d, bytes, 2);
    assert(snapshot(&d).accel_sequence == first.accel_sequence);
    ds4_input(&d, bytes, 1);
    assert(snapshot(&d).gyro_sequence == first.gyro_sequence);
    bytes[7] ^= 0x20; // Corrupt input must not be admitted as button release.
    uni_hid_parser_ds4_parse_input_report(&d, bytes, 78);
    assert(!snapshot(&d).report_valid);
    assert(snapshot(&d).accel_sequence == first.accel_sequence);
    uint8_t buttons[10] = {1};
    uni_hid_parser_ds4_init_report(&d);
    uni_hid_parser_ds4_parse_input_report(&d, buttons, sizeof(buttons));
    assert(snapshot(&d).report_valid && snapshot(&d).accel_sequence == first.accel_sequence);
    // Valid accel calibration with degenerate gyro extrema is accel-only.
    memset(calibration + 7, 0, 12);
    uni_hid_parser_ds4_parse_feature_report(&d, calibration, 37);
    ds4_input(&d, bytes, 3);
    assert(snapshot(&d).accel_valid && !snapshot(&d).gyro_valid);
    uni_hid_parser_native_motion_forget(&d);
    uni_hid_parser_ds4_setup(&d); // Same pointer, new parser lifetime.
    assert(!snapshot(&d).report_valid && !snapshot(&d).accel_valid);
    uni_hid_parser_ds4_parse_feature_report(&d, calibration, 37);
    ds4_input(&d, bytes, 3);
    assert(snapshot(&d).accel_sequence != first.accel_sequence);
    uni_hid_parser_native_motion_forget(&d);
}

static void move_calibration(uni_hid_device_t* d, bool zcm2, bool valid) {
    uint8_t blob[143] = {0};
    const uint8_t lo1[] = {0x0a, 0x24, 0x14}, hi1[] = {0x16, 0x1e, 0x08};
    const uint8_t lo2[] = {0x08, 0x16, 0x24}, hi2[] = {0x02, 0x10, 0x1e};
    const uint8_t bias1[] = {0x2a, 0x2c, 0x2e}, high1[] = {0x46, 0x50, 0x5a};
    const uint8_t bias2[] = {0x26, 0x28, 0x2a}, high2[] = {0x30, 0x38, 0x40}, low2[] = {0x42, 0x4a, 0x52};
    for (unsigned i = 0; i < 3; ++i) {
        int center = zcm2 ? 0 : 0x8000;
        put16(blob + (zcm2 ? lo2[i] : lo1[i]), center - (valid ? 1000 : 0));
        put16(blob + (zcm2 ? hi2[i] : hi1[i]), center + (valid ? 1000 : 0));
        put16(blob + (zcm2 ? bias2[i] : bias1[i]), center);
        put16(blob + (zcm2 ? high2[i] : high1[i]), center + 1000);
        if (zcm2) put16(blob + low2[i], -1000);
    }
    blob[0] = 0x10;
    uni_hid_parser_psmove_parse_feature_report(d, blob, 49);
    uint8_t continuation[49] = {0x10, zcm2 ? 0x81 : 1};
    memcpy(continuation + 2, blob + 49, 47);
    uni_hid_parser_psmove_parse_feature_report(d, continuation, 49);
    if (!zcm2) {
        continuation[1] = 0x82;
        memcpy(continuation + 2, blob + 96, 47);
        uni_hid_parser_psmove_parse_feature_report(d, continuation, 49);
    }
}
static void move_provenance(bool zcm2) {
    uni_hid_device_t d = {0};
    d.report_parser.setup = uni_hid_parser_psmove_setup;
    d.product_id = zcm2 ? 0x0c5e : 0x03d5;
    uni_hid_parser_psmove_setup(&d);
    uint8_t report[49] = {1};
    report[2] = 0x40; // Cross.
    report[43] = 1;
    for (unsigned i = 0; i < 12; ++i)
        put16(report + 13 + 2 * i, (zcm2 ? 0 : 0x8000) + 1000);
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    assert(snapshot(&d).report_valid && !snapshot(&d).accel_valid);
    move_calibration(&d, zcm2, false);
    report[43] = 2;
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    assert(!snapshot(&d).accel_valid && !snapshot(&d).gyro_valid);
    move_calibration(&d, zcm2, true);
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    assert(!snapshot(&d).accel_valid);
    report[43] = 3;
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    uni_native_motion_snapshot_t first = snapshot(&d);
    assert(first.accel_valid && first.gyro_valid && first.accel_q13[0] == 8192);
    assert(first.gyro_q10[0] == (zcm2 ? 540 : 480) * 1024);
    uni_hid_parser_psmove_init_report(&d);
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    assert(snapshot(&d).gyro_sequence == first.gyro_sequence);
    report[43] = 2;
    uni_hid_parser_psmove_parse_input_report(&d, report, sizeof(report));
    assert(snapshot(&d).accel_sequence == first.accel_sequence);
    uni_hid_parser_psmove_parse_input_report(&d, report, 43);
    assert(!snapshot(&d).report_valid && snapshot(&d).gyro_sequence == first.gyro_sequence);
    uni_hid_parser_native_motion_forget(&d);
}

static void switch_setup(uni_hid_device_t* d, uint8_t type, bool calibrated) {
    memset(d, 0, sizeof(*d));
    d->report_parser.setup = uni_hid_parser_switch_setup;
    d->conn.interrupt_cid = 0x40;
    unsigned ready_before = ready_count;
    uni_hid_parser_switch_setup(d);
    for (unsigned step = 0; ready_count == ready_before && step < 12; ++step) {
        assert(command_len >= 12);
        uint8_t reply[49] = {0x21};
        reply[13] = 0x80;
        reply[14] = command[11];
        if (reply[14] == 2) reply[17] = type;
        if (reply[14] == 0x10) {
            memcpy(reply + 15, command + 12, 5);
            // Centered sticks, nonzero spans; no user calibration magic.
            if (little_endian_read_32(reply, 15) == 0x603d || little_endian_read_32(reply, 15) == 0x6046)
                memset(reply + 20, 0x80, reply[19]);
            if (little_endian_read_32(reply, 15) == 0x6020) {
                for (unsigned i = 0; i < 3; ++i) {
                    put16(reply + 26 + 2 * i, calibrated ? 16384 : 0);
                    put16(reply + 38 + 2 * i, calibrated ? 13371 : 0);
                }
                // Truncation with a convincing declared length cannot certify sensors.
                uni_hid_parser_switch_parse_input_report(d, reply, 22);
                assert(!snapshot(d).accel_valid);
            }
        }
        uni_hid_parser_switch_parse_input_report(d, reply, sizeof(reply));
    }
    assert(ready_count != ready_before);
}
static void switch_provenance(uint8_t type, bool calibrated) {
    uni_hid_device_t d;
    switch_setup(&d, type, calibrated);
    uint8_t report[49] = {0x30, 254};
    memset(report + 6, 0x80, 6);
    put16(report + 37, 4096); // Latest accelerometer sample, native X.
    put16(report + 43, 1000); // Latest gyro sample, native X.
    uni_hid_parser_switch_parse_input_report(&d, report, sizeof(report));
    uni_native_motion_snapshot_t first = snapshot(&d);
    assert(first.report_tracked && first.report_valid);
    if (type == 0x0b || !calibrated) {
        assert(!first.accel_valid && !first.gyro_valid);
    } else {
        assert(first.accel_valid && first.gyro_valid && first.accel_q13[2] == -8192);
        uni_hid_parser_switch_parse_input_report(&d, report, sizeof(report));
        assert(snapshot(&d).accel_sequence == first.accel_sequence);
        report[1] = 1;
        uni_hid_parser_switch_parse_input_report(&d, report, sizeof(report));
        assert(snapshot(&d).gyro_sequence != first.gyro_sequence);
        first = snapshot(&d);
        report[1] = 0;
        uni_hid_parser_switch_parse_input_report(&d, report, sizeof(report));
        assert(snapshot(&d).gyro_sequence == first.gyro_sequence);
    }
    uni_hid_parser_switch_parse_input_report(&d, report, 48);
    assert(!snapshot(&d).report_valid && snapshot(&d).accel_sequence == first.accel_sequence);
    uint8_t buttons[12] = {0x3f};
    uni_hid_parser_switch_parse_input_report(&d, buttons, sizeof(buttons));
    assert(snapshot(&d).report_valid && snapshot(&d).accel_sequence == first.accel_sequence);
    uni_hid_parser_switch_teardown(&d);
    assert(!uni_hid_parser_native_motion_snapshot(&d, &first));
}
int main(void) {
    ds4_provenance();
    move_provenance(false);
    move_provenance(true);
    switch_provenance(3, true);
    switch_provenance(1, true);
    switch_provenance(2, true);
    switch_provenance(3, false);
    switch_provenance(0x0b, false);
    uni_hid_device_t sensorless = {0};
    uni_native_motion_snapshot_t absent;
    memset(&absent, 0xff, sizeof(absent));
    assert(!uni_hid_parser_native_motion_snapshot(&sensorless, &absent));
    assert(!absent.report_tracked && !absent.accel_valid && !absent.gyro_valid && absent.gyro_sequence == 0);
    puts("Native DS4, Move, Switch motion provenance and sensorless absence passed");
    return 0;
}
