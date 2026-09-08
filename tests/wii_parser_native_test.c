#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "parser/uni_hid_parser_wii.h"
#include "uni_hid_device.h"

// The real staged parser and gamepad definitions are linked. Only transport and
// the ready notification are substituted; no parser-private state is inspected.
// Wire fixtures follow WiiBrew's Wiimote and Wii Motion Plus protocol pages.
typedef struct {
    uint16_t len;
    uint8_t bytes[32];
} transaction_t;

typedef enum { EXT_NONE, EXT_NUNCHUK, EXT_WII_U_PRO } extension_t;

typedef struct {
    uni_hid_device_t device;
    transaction_t sent[128];
    unsigned sent_count;
    unsigned cursor;
    unsigned ready_count;
    extension_t extension;
    bool motionplus;
    bool mp_initialized;
    bool mp_active;
    bool status_before_activation_ack;
    bool status_after_activation_ack;
    bool deactivation_status_before_ack;
    bool deactivation_status_after_ack;
    unsigned initializing_extension_reads;
    unsigned transient_extension_read_errors;
    uint8_t extension_identity[6];
    bool fail_accel_reads;
    uint8_t connection_buttons;
    uint8_t activation_mode;
    uint8_t report_mode;
    uint32_t fail_read_address;
    uint32_t fail_write_address;
    uint8_t accel_calibration[20];
    uint8_t mp_calibration[32];
    uint8_t nunchuk_calibration[16];
} fixture_t;

static fixture_t f;

void uni_hid_device_send_intr_report(uni_hid_device_t* d, const uint8_t* report, uint16_t len) {
    assert(d == &f.device);
    assert(f.sent_count < 128 && len >= 2 && len <= sizeof(f.sent[0].bytes));
    assert(report[0] == 0xa2);
    transaction_t* transaction = &f.sent[f.sent_count++];
    transaction->len = len;
    memcpy(transaction->bytes, report, len);
}

bool uni_hid_device_set_ready_complete(uni_hid_device_t* d) {
    assert(d == &f.device);
    ++f.ready_count;
    d->conn.state = UNI_BT_CONN_STATE_DEVICE_READY;
    return true;
}

void uni_log(const char* fmt, ...) { (void)fmt; }
void printf_hexdump(const void* data, int len) { (void)data; (void)len; }

static void put_be16(uint8_t* output, uint16_t value) {
    output[0] = value >> 8;
    output[1] = value;
}

static void put_le16(uint8_t* output, uint16_t value) {
    output[0] = value;
    output[1] = value >> 8;
}

static void pack_accel_calibration(uint8_t* output, const uint16_t values[3]) {
    output[3] = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        output[axis] = values[axis] >> 2;
        output[3] |= (values[axis] & 3) << (4 - 2 * axis);
    }
}

static void update_mp_checksum(void) {
    // CRC32 covers fast[0..13] followed by slow[0..13], not the CRC slots.
    uint32_t crc = UINT32_MAX;
    for (unsigned block = 0; block < 2; ++block) {
        for (unsigned byte = 0; byte < 14; ++byte) {
            crc ^= f.mp_calibration[block * 16 + byte];
            for (unsigned bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
        }
    }
    crc ^= UINT32_MAX;
    put_be16(&f.mp_calibration[14], crc >> 16);
    put_be16(&f.mp_calibration[30], crc);
}

static void set_nunchuk_stick_calibration(const uint8_t values[6]) {
    memcpy(f.nunchuk_calibration + 8, values, 6);
    uint8_t checksum = 0x55;
    for (unsigned i = 0; i < 14; ++i)
        checksum += f.nunchuk_calibration[i];
    f.nunchuk_calibration[14] = checksum;
    f.nunchuk_calibration[15] = checksum + 0x55;
}

static void reset_fixture(uint16_t product_id, bool motionplus, extension_t extension) {
    memset(&f, 0, sizeof(f));
    f.device.vendor_id = 0x057e;
    f.device.product_id = product_id;
    f.device.conn.connected = true;
    f.device.conn.interrupt_cid = 0x40;
    f.extension = extension;
    f.motionplus = motionplus;
    f.fail_read_address = UINT32_MAX;
    f.fail_write_address = UINT32_MAX;
    const uint8_t nunchuk_identity[6] = {0, 0, 0xa4, 0x20, 0, 0};
    memcpy(f.extension_identity, nunchuk_identity, sizeof(nunchuk_identity));
    set_nunchuk_stick_calibration((const uint8_t[]){224, 32, 128, 224, 32, 128});

    // Non-default centers, unequal spans, and nonzero packed low bits catch
    // nominal 0x200/100-count calibration and high-byte-only decoding.
    const uint16_t zero[3] = {510, 506, 514};
    const uint16_t one_g[3] = {614, 634, 594};
    pack_accel_calibration(f.accel_calibration, zero);
    pack_accel_calibration(&f.accel_calibration[4], one_g);
    uint8_t checksum = 0x55;
    for (unsigned byte = 0; byte < 9; ++byte)
        checksum += f.accel_calibration[byte];
    f.accel_calibration[9] = checksum;
    memcpy(&f.accel_calibration[10], f.accel_calibration, 10);

    // MP words have 16-bit precision; reports have 14 bits. Factory yaw and
    // pitch spans are negative, whereas roll is positive. Fast and slow also
    // deliberately have different centers and sensitivities on every axis.
    const uint16_t zero_fast[3] = {32000, 31600, 31200};
    const uint16_t scale_fast[3] = {22400, 36400, 28000};
    const uint16_t zero_slow[3] = {32400, 32000, 31600};
    const uint16_t scale_slow[3] = {21600, 37400, 28000};
    for (unsigned axis = 0; axis < 3; ++axis) {
        put_be16(&f.mp_calibration[2 * axis], zero_fast[axis]);
        put_be16(&f.mp_calibration[6 + 2 * axis], scale_fast[axis]);
        put_be16(&f.mp_calibration[16 + 2 * axis], zero_slow[axis]);
        put_be16(&f.mp_calibration[22 + 2 * axis], scale_slow[axis]);
    }
    f.mp_calibration[12] = 200;  // Fast calibration is at 1200 degrees/s.
    f.mp_calibration[28] = 45;   // Slow calibration is at 270 degrees/s.
    f.mp_calibration[13] = 0x31;
    f.mp_calibration[29] = 0x72;
    update_mp_checksum();
}

static void feed(const uint8_t* report, uint16_t len) {
    uni_hid_parser_wii_init_report(&f.device);
    uni_hid_parser_wii_parse_input_report(&f.device, report, len);
}

static void send_status(bool extension_present) {
    const uint8_t report[] = {0x20, f.connection_buttons, 0, extension_present ? 2 : 0, 0, 0, 0xc0};
    feed(report, sizeof(report));
}

static void send_ack(uint8_t command, uint8_t error) {
    const uint8_t report[] = {0x22, 0, 0, command, error};
    feed(report, sizeof(report));
}

static uint32_t transaction_address(const transaction_t* transaction) {
    assert(transaction->len >= 8);
    const uint8_t* bytes = transaction->bytes;
    return ((uint32_t)bytes[3] << 16) | ((uint32_t)bytes[4] << 8) | bytes[5];
}

static void send_read_reply(uint16_t address, const uint8_t* data, uint8_t size, uint8_t error) {
    assert(size >= 1 && size <= 16);
    uint8_t report[22] = {0x21, 0, 0, (uint8_t)(((size - 1) << 4) | error)};
    put_be16(&report[4], address);
    if (!error)
        memcpy(&report[6], data, size);
    feed(report, sizeof(report));
}

static bool read_wire_byte(uint8_t space, uint32_t address, uint8_t* value) {
    if (space == 0) {
        if (address >= 0x16 && address < 0x2a && !f.fail_accel_reads) {
            *value = f.accel_calibration[address - 0x16];
            return true;
        }
        return false;
    }
    assert(space == 4);
    uint32_t bank = address >> 16;
    uint16_t offset = address;
    if ((bank == 0xa6 && f.motionplus && !f.mp_active) ||
        (bank == 0xa4 && f.mp_active)) {
        if (offset >= 0x20 && offset < 0x40) {
            *value = f.mp_calibration[offset - 0x20];
            return true;
        }
        if (offset >= 0xfa && offset <= 0xff) {
            // RVL-CNT-01-TR hardware returns an A4 signature even when read
            // through inactive A600FA; the ID is not an echo of the read bank.
            const bool integrated = f.device.product_id == 0x0330;
            const uint8_t identity[] = {integrated ? 1 : 0, 0, integrated ? 0xa4 : (uint8_t)bank,
                                       0x20, f.mp_active ? f.activation_mode : 0, 5};
            *value = identity[offset - 0xfa];
            return true;
        }
    }
    if (bank == 0xa4 && !f.mp_active && f.extension == EXT_NUNCHUK && offset >= 0x20 && offset < 0x30) {
        *value = f.nunchuk_calibration[offset - 0x20];
        return true;
    }
    if (bank == 0xa4 && !f.mp_active && f.extension != EXT_NONE && offset >= 0xfa && offset <= 0xff) {
        uint8_t identity[6];
        memcpy(identity, f.extension_identity, sizeof(identity));
        if (f.extension == EXT_WII_U_PRO) {
            identity[4] = 1;
            identity[5] = 0x20;
        }
        *value = identity[offset - 0xfa];
        return true;
    }
    return false;
}

static void answer_transaction(const transaction_t* transaction) {
    const uint8_t* bytes = transaction->bytes;
    switch (bytes[1]) {
        case 0x11:  // LEDs do not acknowledge unless explicitly requested.
            if (bytes[2] & 2)
                send_ack(0x11, 0);
            break;
        case 0x12:
            assert(transaction->len == 4);
            f.report_mode = bytes[3];
            if (bytes[2] & 2)
                send_ack(0x12, 0);
            break;
        case 0x15:
            send_status(f.extension != EXT_NONE || f.mp_active);
            break;
        case 0x16: {
            uint32_t address = transaction_address(transaction);
            assert((bytes[2] & 0xfe) == 4);
            assert(bytes[6] == 1 && transaction->len >= 8);
            uint8_t error = 0;
            bool deactivated = false;
            bool activated = false;
            if (address == f.fail_write_address || ((address >> 16) == 0xa6 && !f.motionplus)) {
                error = 7;
            } else if (address == 0xa600f0) {
                assert(bytes[7] == 0x55);
                f.mp_initialized = true;
            } else if (address == 0xa600fe) {
                // Initialization before activation and the passthrough selector
                // are protocol requirements, not a pinned parser FSM sequence.
                assert(f.mp_initialized);
                assert(bytes[7] == 4 || bytes[7] == 5);
                f.mp_active = true;
                f.activation_mode = bytes[7];
                activated = true;
                if (f.status_before_activation_ack)
                    send_status(true);
            } else if (address == 0xa400f0) {
                assert(bytes[7] == 0x55);
                deactivated = f.mp_active;
                f.mp_active = false;
                if (deactivated && f.deactivation_status_before_ack) {
                    send_status(false);
                    send_status(true);
                }
            } else {
                assert((address == 0xa400fb || address == 0xa600fb) && bytes[7] == 0);
            }
            send_ack(0x16, error);
            if (deactivated && f.deactivation_status_after_ack) {
                send_status(false);
                send_status(true);
            }
            if (activated && f.status_after_activation_ack)
                send_status(true);
            break;
        }
        case 0x17: {
            uint32_t address = transaction_address(transaction);
            uint16_t remaining = ((uint16_t)bytes[6] << 8) | bytes[7];
            assert(remaining && remaining <= 32);
            if (address == 0xa400fa && !f.mp_active && f.extension != EXT_NONE) {
                if (f.transient_extension_read_errors) {
                    --f.transient_extension_read_errors;
                    send_read_reply(address, NULL, 1, 7);
                    break;
                }
                if (f.initializing_extension_reads) {
                    --f.initializing_extension_reads;
                    const uint8_t initializing[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
                    send_read_reply(address, initializing, sizeof(initializing), 0);
                    break;
                }
            }
            if (address == f.fail_read_address) {
                send_read_reply(address, NULL, 1, 7);
                break;
            }
            while (remaining) {
                uint8_t data[16];
                uint8_t size = remaining > 16 ? 16 : remaining;
                bool available = true;
                for (unsigned i = 0; i < size; ++i)
                    available &= read_wire_byte(bytes[2] & 0xfe, address + i, &data[i]);
                send_read_reply(address, data, size, available ? 0 : 7);
                if (!available)
                    break;
                address += size;
                remaining -= size;
            }
            break;
        }
        default:
            assert(!"unexpected Wii output report");
    }
}

static void finish_setup(void) {
    // Output may be emitted recursively by each reply. Bound the wire dialogue
    // so retry loops fail instead of hanging pytest.
    for (unsigned step = 0; step < 128 && f.cursor < f.sent_count; ++step) {
        transaction_t transaction = f.sent[f.cursor++];
        answer_transaction(&transaction);
    }
    assert(f.cursor == f.sent_count);
    assert(f.ready_count == 1);
    assert(f.report_mode != 0);
}

static void connect_device(void) {
    uni_hid_parser_wii_setup(&f.device);
    finish_setup();
}

static transaction_t stop_at_read(uint32_t address) {
    for (unsigned step = 0; step < 128 && f.cursor < f.sent_count; ++step) {
        transaction_t transaction = f.sent[f.cursor++];
        if (transaction.bytes[1] == 0x17 && transaction_address(&transaction) == address)
            return transaction;
        answer_transaction(&transaction);
    }
    assert(!"required Wii memory request was not sent");
    return (transaction_t){0};
}

static void put_accel_report(uint8_t* report, uint16_t x, uint16_t y, uint16_t z) {
    assert(!(y & 1) && !(z & 1));  // Y/Z wire samples have no least-significant bit.
    report[1] |= (x & 3) << 5;
    report[2] |= ((y & 2) << 4) | ((z & 2) << 5);
    report[3] = x >> 2;
    report[4] = y >> 2;
    report[5] = z >> 2;
}

static void put_gyro_report(uint8_t* extension, uint16_t yaw, uint16_t roll, uint16_t pitch,
                            bool yaw_slow, bool roll_slow, bool pitch_slow) {
    extension[0] = yaw;
    extension[1] = roll;
    extension[2] = pitch;
    extension[3] = ((yaw >> 8) << 2) | (yaw_slow ? 2 : 0) | (pitch_slow ? 1 : 0);
    extension[4] = ((roll >> 8) << 2) | (roll_slow ? 2 : 0) | (f.extension != EXT_NONE ? 1 : 0);
    extension[5] = ((pitch >> 8) << 2) | 2;
}

static void send_motion(uint16_t yaw, uint16_t roll, uint16_t pitch,
                        bool yaw_slow, bool roll_slow, bool pitch_slow) {
    uint8_t report[22] = {0x35, 1, 0x0a};  // Left + A + 1, no connection-time A needed.
    put_accel_report(report, 614, 442, 534);
    put_gyro_report(&report[6], yaw, roll, pitch, yaw_slow, roll_slow, pitch_slow);
    feed(report, sizeof(report));
}

static void expect_vector(const int32_t vector[3], int32_t x, int32_t y, int32_t z) {
    assert(vector[0] == x);
    assert(vector[1] == y);
    assert(vector[2] == z);
}

static void expect_accel(void) {
    // Native (+1g, -0.5g, +0.25g) maps to SDL (-X, +Z, +Y).
    expect_vector(f.device.controller.gamepad.accel, -8192, 2048, -4096);
}

static void send_core_and_accel(void) {
    uint8_t report[22] = {f.report_mode, 1, 0x0a};
    assert(f.report_mode == 0x30 || f.report_mode == 0x31 || f.report_mode == 0x35);
    uint16_t len = f.report_mode == 0x35 ? 22 : f.report_mode == 0x31 ? 6 : 3;
    if (len >= 6)
        put_accel_report(report, 614, 442, 534);
    feed(report, len);
    assert(f.device.controller.gamepad.dpad == DPAD_DOWN);
    assert(f.device.controller.gamepad.buttons == (BUTTON_X | BUTTON_A));
}

static void send_nunchuk_controls(bool passthrough) {
    uint8_t report[22] = {0x35};
    put_accel_report(report, 614, 442, 534);
    const uint8_t extension[6] = {160, 192, 128, 128, passthrough ? 129 : 128, passthrough ? 4 : 1};
    memcpy(report + 6, extension, sizeof(extension));
    feed(report, sizeof(report));
    assert(f.device.controller.gamepad.buttons == BUTTON_X);
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.axis_y == -341);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK_ACCEL);
    expect_accel();
}

static void integrated_motionplus_calibration_and_slow_bits(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    connect_device();
    assert(f.mp_active && f.activation_mode == 4 && f.report_mode == 0x35);

    // Each axis is the only slow axis in one packet. A shared slow flag or
    // swapped e[3]/e[4] bits cannot accidentally pass all three vectors.
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    expect_accel();
    assert(f.device.controller.gamepad.dpad == DPAD_DOWN);
    assert(f.device.controller.gamepad.buttons == (BUTTON_X | BUTTON_A));

    send_motion(8300, 7660, 7560, true, false, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, 20 * 1024, -240 * 1024);
    send_motion(7760, 7660, 8100, false, false, true);
    expect_vector(f.device.controller.gamepad.gyro, -60 * 1024, -120 * 1024, -240 * 1024);
}

static void external_motionplus_without_extension_status(void) {
    reset_fixture(0x0306, true, EXT_NONE);
    f.status_before_activation_ack = true;
    connect_device();
    assert(f.mp_active && f.activation_mode == 4 && f.report_mode == 0x35);
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    expect_accel();
}

static void absent_motionplus_keeps_calibrated_remote(void) {
    reset_fixture(0x0306, false, EXT_NONE);
    connect_device();
    assert(!f.mp_active && f.report_mode == 0x31);
    send_core_and_accel();
    expect_accel();
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
}

static void setup_read_and_write_errors_leave_buttons_ready(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    f.fail_read_address = 0xa60030;
    connect_device();
    send_core_and_accel();
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);

    reset_fixture(0x0330, true, EXT_NONE);
    f.fail_write_address = 0xa600fe;
    connect_device();
    assert(!f.mp_active);
    send_core_and_accel();
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);

    reset_fixture(0x0306, false, EXT_NUNCHUK);
    f.fail_write_address = 0xa400f0;
    connect_device();
    // A failed reset ACK is not an extension identity. If the subsequent read
    // succeeds, preserve the real Nunchuk rather than forcing standalone mode.
    send_nunchuk_controls(false);
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
}

static void failed_calibration_never_fabricates_motion(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    f.accel_calibration[9] ^= 1;
    f.accel_calibration[19] ^= 1;
    connect_device();
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.accel, 0, 0, 0);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    assert(f.device.controller.gamepad.buttons == (BUTTON_X | BUTTON_A));

    reset_fixture(0x0330, true, EXT_NONE);
    f.mp_calibration[30] ^= 1;
    connect_device();
    if (f.report_mode == 0x35)
        send_motion(7760, 8200, 7560, false, true, false);
    else
        send_core_and_accel();
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
    expect_accel();
    assert(f.device.controller.gamepad.buttons == (BUTTON_X | BUTTON_A));

    // A correct CRC alone is insufficient: zero sensitivity must be rejected.
    reset_fixture(0x0330, true, EXT_NONE);
    memcpy(&f.mp_calibration[6], f.mp_calibration, 2);
    update_mp_checksum();
    connect_device();
    if (f.report_mode == 0x35)
        send_motion(7760, 8200, 7560, false, true, false);
    else
        send_core_and_accel();
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
    expect_accel();
}

static void accel_backup_and_memory_error_recovery(void) {
    reset_fixture(0x0306, false, EXT_NONE);
    f.accel_calibration[9] ^= 1;
    connect_device();
    send_core_and_accel();
    expect_accel();

    reset_fixture(0x0306, false, EXT_NONE);
    f.fail_accel_reads = true;
    connect_device();
    send_core_and_accel();
    expect_vector(f.device.controller.gamepad.accel, 0, 0, 0);
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
}

static void malformed_and_mismatched_setup_replies_do_not_advance(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    uni_hid_parser_wii_setup(&f.device);
    transaction_t request = stop_at_read(0xa60020);
    unsigned count = f.sent_count;
    const uint8_t empty[] = {0x21};
    const uint8_t short_header[] = {0x21, 0, 0, 0xf0, 0};
    const uint8_t short_data[] = {0x21, 0, 0, 0xf0, 0, 0x20, 0};
    const uint8_t short_ack[] = {0x22, 0, 0, 0x16};
    feed(empty, 0);
    feed(empty, sizeof(empty));
    feed(short_header, sizeof(short_header));
    feed(short_data, sizeof(short_data));
    feed(short_ack, sizeof(short_ack));
    send_ack(0x12, 0);
    send_ack(0x16, 0);  // No write is outstanding.
    send_read_reply(0x30, &f.mp_calibration[16], 16, 0);
    send_read_reply(0x20, f.mp_calibration, 15, 0);
    assert(f.sent_count == count && f.ready_count == 0);

    // Accept either two 16-byte reads or one 32-byte read; both produce the
    // same two wire replies, since a 0x21 response carries at most 16 bytes.
    unsigned requested = ((unsigned)request.bytes[6] << 8) | request.bytes[7];
    assert(requested == 16 || requested == 32);
    send_read_reply(0x20, f.mp_calibration, 16, 0);
    if (requested == 16)
        (void)stop_at_read(0xa60030);
    count = f.sent_count;
    // Duplicate first half cannot satisfy the outstanding second-half read.
    send_read_reply(0x20, f.mp_calibration, 16, 0);
    assert(f.sent_count == count && f.ready_count == 0);
    send_read_reply(0x30, &f.mp_calibration[16], 16, 0);
    finish_setup();
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
}

static void short_motion_reports_do_not_publish_partial_samples(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    connect_device();
    uint8_t report[22];
    memset(report, 0xff, sizeof(report));
    report[0] = 0x35;
    unsigned count = f.sent_count;
    // The sixth extension byte contains pitch MSBs and the MP discriminator.
    // No prefix missing that byte constitutes a valid combined sample.
    for (uint16_t len = 1; len < 12; ++len) {
        feed(report, len);
        expect_vector(f.device.controller.gamepad.accel, 0, 0, 0);
        expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
    }
    report[0] = 0x31;
    feed(report, 5);
    expect_vector(f.device.controller.gamepad.accel, 0, 0, 0);
    report[0] = 0x20;
    feed(report, 6);
    assert(f.sent_count == count && f.ready_count == 1);
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    expect_accel();
}

static void nunchuk_passthrough_retains_controls_between_samples(void) {
    reset_fixture(0x0330, true, EXT_NUNCHUK);
    connect_device();
    assert(f.mp_active && f.activation_mode == 5 && f.report_mode == 0x35);
    uint8_t nunchuk[22] = {0x35, 1, 0x0a};
    put_accel_report(nunchuk, 614, 442, 534);
    const uint8_t extension[6] = {160, 96, 128, 128, 129, 4};  // C held, Z released.
    memcpy(&nunchuk[6], extension, sizeof(extension));
    feed(nunchuk, sizeof(nunchuk));
    assert(f.device.controller.gamepad.dpad == DPAD_LEFT);
    assert(f.device.controller.gamepad.buttons == (BUTTON_B | BUTTON_SHOULDER_L | BUTTON_X));
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.axis_y == 170);

    uint8_t gyro[22] = {0x35};  // Release core controls but keep the Nunchuk held.
    put_accel_report(gyro, 614, 442, 534);
    put_gyro_report(&gyro[6], 7760, 8200, 7560, false, true, false);
    feed(gyro, sizeof(gyro));
    assert(f.device.controller.gamepad.buttons == BUTTON_X);
    assert(f.device.controller.gamepad.dpad == 0);
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.axis_y == 170);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    expect_accel();

    nunchuk[2] &= 0x60;  // Preserve accel low bits, release core buttons.
    nunchuk[1] &= 0x60;
    nunchuk[11] = 8;  // C released, Z held: moved bits 3/2, not plain bits 1/0.
    feed(nunchuk, sizeof(nunchuk));
    assert(f.device.controller.gamepad.buttons == BUTTON_Y);
    feed(gyro, sizeof(gyro));
    assert(f.device.controller.gamepad.buttons == BUTTON_Y);
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.axis_y == 170);

    nunchuk[6] = nunchuk[7] = 128;
    nunchuk[11] = 12;  // Both released; stale cached presses must now disappear.
    feed(nunchuk, sizeof(nunchuk));
    feed(gyro, sizeof(gyro));
    assert(f.device.controller.gamepad.buttons == 0);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
    assert(f.device.controller.gamepad.axis_x == 0 && f.device.controller.gamepad.axis_y == 0);
}

static void legacy_plain_nunchuk_and_wii_u_pro(void) {
    reset_fixture(0x0306, false, EXT_NUNCHUK);
    connect_device();
    assert(!f.mp_active && (f.report_mode == 0x32 || f.report_mode == 0x35));
    uint8_t nunchuk[22] = {f.report_mode, 1, 0x0a};
    unsigned offset = f.report_mode == 0x35 ? 6 : 3;
    if (offset == 6)
        put_accel_report(nunchuk, 614, 442, 534);
    const uint8_t extension[6] = {160, 192, 128, 128, 128, 1};  // Plain C held, Z released.
    memcpy(&nunchuk[offset], extension, sizeof(extension));
    feed(nunchuk, f.report_mode == 0x35 ? 22 : 11);
    assert(f.device.controller.gamepad.dpad == DPAD_LEFT);
    assert(f.device.controller.gamepad.buttons == (BUTTON_B | BUTTON_SHOULDER_L | BUTTON_X));
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.axis_y == -341);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);

    reset_fixture(0x0330, false, EXT_WII_U_PRO);
    connect_device();
    assert(!f.mp_active && f.report_mode == 0x34);
    uint8_t pro[22] = {0x34};
    put_le16(&pro[3], 0x0a80);
    put_le16(&pro[5], 0x0580);
    put_le16(&pro[7], 0x06c0);
    put_le16(&pro[9], 0x0940);
    pro[11] = 0x7b;  // Right and + held.
    pro[12] = 0xbf;  // Native B held.
    pro[13] = 0xff;
    feed(pro, sizeof(pro));
    assert(f.device.controller.gamepad.axis_x == 256 && f.device.controller.gamepad.axis_y == 128);
    assert(f.device.controller.gamepad.axis_rx == -256 && f.device.controller.gamepad.axis_ry == -128);
    assert(f.device.controller.gamepad.dpad == DPAD_RIGHT);
    assert(f.device.controller.gamepad.buttons == BUTTON_A);
    assert(f.device.controller.gamepad.misc_buttons == MISC_BUTTON_START);
    expect_vector(f.device.controller.gamepad.accel, 0, 0, 0);
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
}

static void plus_selects_vertical_without_disabling_motion(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    f.connection_buttons = 0x10;
    connect_device();
    assert(f.mp_active && f.report_mode == 0x35);
    send_motion(7760, 8200, 7560, false, true, false);
    assert(f.device.controller.gamepad.dpad == DPAD_LEFT);
    assert(f.device.controller.gamepad.buttons == (BUTTON_B | BUTTON_X));
    expect_accel();
}

static void unsolicited_status_restores_stream_without_duplicate_ready(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    connect_device();
    unsigned start = f.sent_count;
    send_status(true);
    finish_setup();
    bool restored = false;
    for (unsigned i = start; i < f.sent_count; ++i) {
        if (f.sent[i].bytes[1] == 0x12 && f.sent[i].bytes[3] == 0x35)
            restored = true;
        assert(f.sent[i].bytes[1] != 0x16);  // Reinitialization deactivates MP.
    }
    assert(restored && f.ready_count == 1 && f.mp_active);
    start = f.sent_count;
    send_ack(0x16, 0);
    send_read_reply(0xfa, (const uint8_t[]){0, 0, 0xa6, 0x20, 0, 5}, 6, 0);
    assert(f.sent_count == start && f.ready_count == 1);
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
}

static void initializing_and_vendor_nunchuk_ids_at_connection(void) {
    reset_fixture(0x0330, true, EXT_NUNCHUK);
    // Some third-party revisions do not have the canonical 0000 prefix.
    f.extension_identity[0] = 1;
    f.initializing_extension_reads = 2;
    f.transient_extension_read_errors = 1;
    f.status_after_activation_ack = true;
    connect_device();
    assert(f.mp_active && f.activation_mode == 5);
    send_nunchuk_controls(true);
    send_motion(7760, 8200, 7560, false, true, false);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    assert(f.device.controller.gamepad.axis_x == 170 && f.device.controller.gamepad.buttons & BUTTON_X);

    // An all-zero suffix is not proof of absence: SDL and upstream recognize
    // the zero-filled ID documented for a replugged wireless BladeFX adapter.
    reset_fixture(0x0306, false, EXT_NUNCHUK);
    memset(f.extension_identity, 0, sizeof(f.extension_identity));
    connect_device();
    send_nunchuk_controls(false);
}

static void motionplus_nunchuk_hotplug_and_detach(void) {
    reset_fixture(0x0330, true, EXT_NONE);
    connect_device();
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL);
    f.deactivation_status_before_ack = true;
    f.status_after_activation_ack = true;
    f.extension = EXT_NUNCHUK;
    // Downstream hotplug is carried in MP data, not the ordinary status bit.
    send_motion(7760, 8200, 7560, false, true, false);
    finish_setup();
    assert(f.mp_active && f.activation_mode == 5 && f.ready_count == 1);
    send_nunchuk_controls(true);

    // Unsolicited status starts a mapped-ID check. A detach during that pending
    // read must be deferred, not overwrite its address-less transaction state.
    send_status(false);
    transaction_t verify = stop_at_read(0xa400fa);
    f.extension = EXT_NONE;
    uint8_t gyro[22] = {0x35};
    put_accel_report(gyro, 614, 442, 534);
    put_gyro_report(gyro + 6, 7760, 8200, 7560, false, true, false);
    feed(gyro, sizeof(gyro));
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
    assert(f.device.controller.gamepad.axis_x == 0 && f.device.controller.gamepad.axis_y == 0);
    assert(f.device.controller.gamepad.buttons == 0);
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    answer_transaction(&verify);
    finish_setup();
    assert(f.mp_active && f.activation_mode == 4 && f.ready_count == 1);
    send_motion(7760, 8200, 7560, false, true, false);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
    assert(f.device.controller.gamepad.axis_x == 0 && f.device.controller.gamepad.axis_y == 0);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);
    expect_accel();
}

static void external_motionplus_hotplug_retains_selected_orientation(void) {
    reset_fixture(0x0306, true, EXT_NONE);
    f.connection_buttons = 0x10;
    connect_device();
    f.deactivation_status_after_ack = true;
    f.status_before_activation_ack = true;
    f.extension = EXT_NUNCHUK;
    send_motion(7760, 8200, 7560, false, true, false);
    finish_setup();
    send_nunchuk_controls(true);
    f.extension = EXT_NONE;
    send_motion(7760, 8200, 7560, false, true, false);
    finish_setup();
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL);
    assert(f.mp_active && f.activation_mode == 4 && f.ready_count == 1);
    send_motion(7760, 8200, 7560, false, true, false);
    assert(f.device.controller.gamepad.dpad == DPAD_LEFT);
    expect_vector(f.device.controller.gamepad.gyro, 360 * 1024, -120 * 1024, 40 * 1024);

    // Removing the external MP itself clears its stale angular velocity, while
    // the Remote's independently calibrated accelerometer remains available.
    f.motionplus = false;
    f.mp_active = false;
    send_status(false);
    finish_setup();
    uint8_t remote[6] = {0x31, 1, 0x0a};
    put_accel_report(remote, 614, 442, 534);
    feed(remote, sizeof(remote));
    assert(f.device.controller.gamepad.dpad == DPAD_LEFT);
    assert(f.device.controller.gamepad.buttons == (BUTTON_B | BUTTON_X));
    expect_vector(f.device.controller.gamepad.gyro, 0, 0, 0);
    expect_accel();
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL);
}

static void plain_nunchuk_hotplug_and_setup_race(void) {
    reset_fixture(0x0306, false, EXT_NONE);
    uni_hid_parser_wii_setup(&f.device);
    transaction_t accel = stop_at_read(0x16);
    // An attachment status while EEPROM is being read must not lose that read.
    f.extension = EXT_NUNCHUK;
    send_status(true);
    answer_transaction(&accel);
    finish_setup();
    send_nunchuk_controls(false);
    f.extension = EXT_NONE;
    send_status(false);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
    assert(f.device.controller.gamepad.axis_x == 0 && f.device.controller.gamepad.axis_y == 0);
    assert(f.device.controller.gamepad.buttons == 0);
    finish_setup();
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL);
    send_core_and_accel();
    expect_accel();

    uni_hid_parser_wii_set_mode(&f.device, WII_MODE_VERTICAL);
    finish_setup();
    f.extension = EXT_NUNCHUK;
    send_status(true);
    finish_setup();
    send_nunchuk_controls(false);
    f.extension = EXT_NONE;
    send_status(false);
    finish_setup();
    assert(f.device.controller_subtype == CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL);
    assert(f.ready_count == 1);
}

static void expect_nunchuk_stick(uint8_t x, uint8_t y, int expected_x, int expected_y) {
    uint8_t report[22] = {0x35};
    put_accel_report(report, 614, 442, 534);
    report[6] = x;
    report[7] = y;
    report[10] = 1;  // Extension connected.
    report[11] = f.mp_active ? 12 : 3;  // C/Z released in the selected format.
    feed(report, sizeof(report));
    assert(f.device.controller.gamepad.axis_x == expected_x);
    assert(f.device.controller.gamepad.axis_y == expected_y);
    assert(f.device.controller.gamepad.axis_rx == 0 && f.device.controller.gamepad.axis_ry == 0);
}

static void calibrated_nunchuk_left_stick_endpoints_and_replacement(void) {
    reset_fixture(0x0330, true, EXT_NUNCHUK);
    set_nunchuk_stick_calibration((const uint8_t[]){220, 40, 124, 210, 30, 126});
    connect_device();
    expect_nunchuk_stick(124, 126, 0, 0);
    expect_nunchuk_stick(220, 126, 511, 0);
    expect_nunchuk_stick(40, 126, -512, 0);
    expect_nunchuk_stick(124, 210, 0, -512);
    expect_nunchuk_stick(124, 30, 0, 511);
    expect_nunchuk_stick(172, 168, 256, -256);
    expect_nunchuk_stick(82, 78, -256, 256);
    expect_nunchuk_stick(255, 0, 511, 511);
    expect_nunchuk_stick(0, 255, -512, -512);
    send_motion(7760, 8200, 7560, false, true, false);
    assert(f.device.controller.gamepad.axis_x == -512 && f.device.controller.gamepad.axis_y == -512);
    f.extension = EXT_NONE;
    send_motion(7760, 8200, 7560, false, true, false);
    finish_setup();
    assert(f.device.controller.gamepad.axis_x == 0 && f.device.controller.gamepad.axis_y == 0);
    // A different Nunchuk must not inherit the previous extension's center/range.
    set_nunchuk_stick_calibration((const uint8_t[]){230, 50, 130, 220, 40, 120});
    f.extension = EXT_NUNCHUK;
    send_motion(7760, 8200, 7560, false, true, false);
    finish_setup();
    expect_nunchuk_stick(130, 120, 0, 0);
    expect_nunchuk_stick(230, 40, 511, 511);
}

static void unavailable_nunchuk_calibration_keeps_safe_nominal_stick(void) {
    reset_fixture(0x0306, false, EXT_NUNCHUK);
    f.fail_read_address = 0xa40020;
    connect_device();
    expect_nunchuk_stick(128, 128, 0, 0);
    expect_nunchuk_stick(224, 32, 511, 511);
    reset_fixture(0x0330, true, EXT_NUNCHUK);
    set_nunchuk_stick_calibration((const uint8_t[]){128, 32, 128, 224, 32, 128});
    connect_device();  // Correct checksum but zero positive span.
    expect_nunchuk_stick(160, 96, 170, 170);
    reset_fixture(0x0330, true, EXT_NUNCHUK);
    f.nunchuk_calibration[15] ^= 1;
    connect_device();
    expect_nunchuk_stick(128, 128, 0, 0);
    expect_nunchuk_stick(224, 32, 511, 511);
}

static void run_case(const char* name, void (*test)(void)) {
    printf("Wii parser: %s\n", name);
    fflush(stdout);
    test();
}

int main(void) {
    run_case("calibrated Nunchuk left-stick endpoints and replacement", calibrated_nunchuk_left_stick_endpoints_and_replacement);
    run_case("unavailable Nunchuk calibration uses safe nominal travel", unavailable_nunchuk_calibration_keeps_safe_nominal_stick);
    run_case("integrated MotionPlus calibration and per-axis slow bits", integrated_motionplus_calibration_and_slow_bits);
    run_case("external MotionPlus with absent extension status bit", external_motionplus_without_extension_status);
    run_case("absent MotionPlus retains calibrated remote", absent_motionplus_keeps_calibrated_remote);
    run_case("setup read/write errors leave buttons ready", setup_read_and_write_errors_leave_buttons_ready);
    run_case("invalid calibration suppresses only unavailable motion", failed_calibration_never_fabricates_motion);
    run_case("accelerometer backup and EEPROM errors", accel_backup_and_memory_error_recovery);
    run_case("malformed and mismatched setup replies", malformed_and_mismatched_setup_replies_do_not_advance);
    run_case("short reports reject partial motion samples", short_motion_reports_do_not_publish_partial_samples);
    run_case("Nunchuk passthrough retains and releases held controls", nunchuk_passthrough_retains_controls_between_samples);
    run_case("legacy plain Nunchuk and Wii U Pro mappings", legacy_plain_nunchuk_and_wii_u_pro);
    run_case("connection-time plus selects vertical motion mode", plus_selects_vertical_without_disabling_motion);
    run_case("unsolicited status restores reporting exactly once", unsolicited_status_restores_stream_without_duplicate_ready);
    run_case("initializing and vendor Nunchuk identities", initializing_and_vendor_nunchuk_ids_at_connection);
    run_case("integrated MotionPlus downstream hotplug and detach", motionplus_nunchuk_hotplug_and_detach);
    run_case("external MotionPlus preserves selected standalone orientation", external_motionplus_hotplug_retains_selected_orientation);
    run_case("plain Nunchuk hotplug and setup-time status race", plain_nunchuk_hotplug_and_setup_race);
    puts("Wii parser native contracts passed");
    return 0;
}
