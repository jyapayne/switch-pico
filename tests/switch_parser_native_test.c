#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bt/uni_bt_service.h"
#include "parser/uni_hid_parser_switch.h"
#include "parser/uni_hid_parser_switch2.h"
#include "parser/uni_hid_parser_wii.h"
#include "platform/uni_platform.h"
#include "uni_hid_device.h"

// Link the actual parser, generic send queue, connection and circular buffer.
// Only radio, platform notifications and run-loop scheduling are substituted.
static const uint8_t neutral[8] = {0, 1, 0x40, 0x40, 0, 1, 0x40, 0x40};
static const uint8_t first_word[8] = {0, 0x81, 0x40, 0x60, 0, 1, 0x40, 0x40};
// Three compressed substeps, distinct from the absolute baseline above.
static const uint8_t compressed[8] = {0x18, 0x63, 0x8c, 0xf1, 0, 1, 0x40, 0x40};
static bool credit = true;
static bool fail_submission;
static unsigned sent_count;
static unsigned requests;
static struct { uint16_t cid, len; uint8_t bytes[128]; } sent[256];
static uint32_t now_ms;
static struct { btstack_timer_source_t* timer; uint32_t deadline; bool active; } timers[32];

bool uni_hid_parser_switch2_is_ble_device(const uni_hid_device_t* d) {
    (void)d;
    return false;  // This fixture exercises only Classic Switch devices.
}
void uni_hid_parser_switch2_teardown(uni_hid_device_t* d) {
    (void)d;
    assert(!"Switch 2 teardown reached a Classic Switch fixture");
}
void uni_hid_parser_wii_setup(uni_hid_device_t* d) {
    (void)d;
    assert(!"Wii setup reached a Classic Switch fixture");
}
void uni_hid_parser_wii_teardown(uni_hid_device_t* d) {
    (void)d;
    assert(!"Wii teardown reached a Classic Switch fixture");
}

static unsigned timer_index(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < 32; ++i) {
        if (timers[i].timer == timer) return i;
        if (!timers[i].timer) { timers[i].timer = timer; return i; }
    }
    assert(!"timer capacity exceeded");
    return 0;
}
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t ms) {
    timers[timer_index(timer)].deadline = now_ms + ms;
}
void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    timers[timer_index(timer)].active = true;
}
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    for (unsigned i = 0; i < 32; ++i) {
        if (timers[i].timer == timer) {
            bool active = timers[i].active;
            timers[i].active = false;
            return active;
        }
    }
    return false;
}
void btstack_run_loop_set_timer_context(btstack_timer_source_t* timer, void* context) { timer->context = context; }
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer, void (*handler)(btstack_timer_source_t*)) { timer->process = handler; }
void* btstack_run_loop_get_timer_context(btstack_timer_source_t* timer) { return timer->context; }
static void advance(uint32_t ms) {
    const uint32_t end = now_ms + ms;
    for (unsigned callbacks = 0; callbacks < 256; ++callbacks) {
        unsigned next = 32;
        for (unsigned i = 0; i < 32; ++i)
            if (timers[i].active && timers[i].deadline <= end &&
                (next == 32 || timers[i].deadline < timers[next].deadline)) next = i;
        if (next == 32) { now_ms = end; return; }
        now_ms = timers[next].deadline;
        timers[next].active = false;
        timers[next].timer->process(timers[next].timer);
    }
    assert(!"unbounded timer callback loop");
}
int l2cap_can_send_packet_now(uint16_t cid) { (void)cid; return credit; }
int l2cap_send(uint16_t cid, uint8_t* data, uint16_t len) {
    if (!credit || fail_submission) return BTSTACK_ACL_BUFFERS_FULL;
    assert(sent_count < 256 && len <= 128);
    sent[sent_count].cid = cid;
    sent[sent_count].len = len;
    memcpy(sent[sent_count++].bytes, data, len);
    return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_request_can_send_now_event(uint16_t cid) { (void)cid; ++requests; return 0; }
gap_connection_type_t gap_get_connection_type(hci_con_handle_t handle) { (void)handle; return GAP_CONNECTION_ACL; }
void printf_hexdump(const void* data, int len) { (void)data; (void)len; }
const char* bd_addr_to_str(const bd_addr_t addr) { (void)addr; return "native-test"; }
void uni_log(const char* fmt, ...) { (void)fmt; }
void uni_bt_bredr_disconnect(uni_hid_device_t* d) { (void)d; }
void uni_bt_le_disconnect(uni_hid_device_t* d) { (void)d; }
void uni_bt_service_on_device_ready(const uni_hid_device_t* d) { (void)d; }
void uni_bt_service_on_device_connected(const uni_hid_device_t* d) { (void)d; }
void uni_bt_service_on_device_disconnected(const uni_hid_device_t* d) { (void)d; }
uint8_t uni_hid_parser_hat_to_dpad(uint8_t hat) { (void)hat; return 0; }
static uni_error_t ready(uni_hid_device_t* d) { (void)d; return UNI_ERROR_SUCCESS; }
static void connected(uni_hid_device_t* d) { (void)d; }
static struct uni_platform platform = {
    .on_device_ready = ready,
    .on_device_connected = connected,
    .on_device_disconnected = connected,
};
struct uni_platform* uni_get_platform(void) { return &platform; }

static void reset(void) {
    memset(timers, 0, sizeof(timers));
    sent_count = requests = now_ms = 0;
    credit = true;
    fail_submission = false;
}
static void reply(uni_hid_device_t* d, uint8_t cmd, uint8_t type, uint8_t ack, uint16_t len) {
    uint8_t report[49] = {0x21};
    report[13] = ack;
    report[14] = cmd;
    if (cmd == 2) {
        report[15] = 5;
        report[16] = 7;
        report[17] = type;
    }
    // Calibration replies intentionally have zero length, leaving the parser's
    // normal fallback calibration intact; these tests exercise only output.
    uni_hid_parser_switch_parse_input_report(d, report, len);
}
static void begin_device(uni_hid_device_t* d, uint16_t cid) {
    uni_hid_device_init(d);
    d->conn.connected = true;
    d->conn.interrupt_cid = cid;
    d->conn.handle = cid;
    d->report_parser.setup = uni_hid_parser_switch_setup;
    uni_hid_parser_switch_setup(d);
}
static void finish_device(uni_hid_device_t* d, uint8_t type, uint8_t ack, uint16_t info_len) {
    reply(d, 2, type, ack, info_len);
    for (unsigned step = 0; step < 10 && d->conn.state != UNI_BT_CONN_STATE_DEVICE_READY; ++step) {
        assert(sent_count && sent[sent_count - 1].len >= 12);
        reply(d, sent[sent_count - 1].bytes[11], type, 0x80, 49);
    }
    assert(d->conn.state == UNI_BT_CONN_STATE_DEVICE_READY);
}
static void init_device(uni_hid_device_t* d, uint16_t cid) {
    begin_device(d, cid);
    finish_device(d, 3, 0x80, 18);
}
static void expect_rumble(unsigned index, uint16_t cid, const uint8_t word[8]) {
    assert(index < sent_count && sent[index].cid == cid);
    assert(sent[index].len == 11);
    assert(sent[index].bytes[0] == 0xa2 && sent[index].bytes[1] == 0x10);
    assert(memcmp(&sent[index].bytes[3], word, 8) == 0);
}
static void expect_led(unsigned index, uint8_t leds, const uint8_t word[8]) {
    assert(index < sent_count && sent[index].len == 13);
    assert(sent[index].bytes[0] == 0xa2 && sent[index].bytes[1] == 1);
    assert(sent[index].bytes[11] == 0x30 && sent[index].bytes[12] == leds);
    assert(memcmp(&sent[index].bytes[3], word, 8) == 0);
}

static void identity_and_per_device_counter(void) {
    reset();
    uni_hid_device_t a, b;
    init_device(&a, 0x40);
    unsigned b_first = sent_count;
    init_device(&b, 0x41);
    assert(sent[0].bytes[2] == 0 && sent[b_first].bytes[2] == 0);
    uint8_t type = 0, hi = 0, lo = 0;
    assert(uni_hid_parser_switch_native_info(&a, &type, &hi, &lo));
    assert(type == 3 && hi == 5 && lo == 7);
    assert(uni_hid_parser_switch_native_acquire(&a));
    assert(uni_hid_parser_switch_native_acquire(&b));
    for (unsigned i = 0; i < 20; ++i) {
        unsigned index = sent_count;
        assert(uni_hid_parser_switch_native_send(&a, first_word));
        expect_rumble(index, 0x40, first_word);
        if (i == 3) uni_hid_parser_switch_set_player_leds(&a, 4);
        assert(uni_hid_parser_switch_native_send(&b, neutral));
    }
    unsigned led_index = sent_count;
    uni_hid_parser_switch_set_player_leds(&a, 2);
    uni_hid_parser_switch_set_player_leds(&b, 8);
    expect_led(led_index, 2, first_word);
    expect_led(led_index + 1, 8, neutral);
    unsigned expected[2] = {0, 0};
    for (unsigned i = 0; i < sent_count; ++i) {
        unsigned device = sent[i].cid - 0x40;
        assert(device < 2 && sent[i].bytes[2] == (expected[device]++ & 15));
    }
    uni_hid_device_disconnect(&a);
    assert(!uni_hid_parser_switch_native_info(&a, NULL, NULL, NULL));
    assert(!uni_hid_parser_switch_native_send(&a, neutral));
}

static void congestion_and_queued_leds(void) {
    reset();
    uni_hid_device_t d;
    init_device(&d, 0x40);
    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, first_word));
    unsigned baseline = sent_count;
    credit = false;
    assert(!uni_hid_parser_switch_native_send(&d, compressed));
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    fail_submission = true;
    assert(!uni_hid_parser_switch_native_send(&d, compressed));
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    fail_submission = false;
    uni_hid_parser_switch_set_player_leds(&d, 1);
    expect_led(baseline, 1, first_word);
    assert(sent[baseline].bytes[2] == ((sent[baseline - 1].bytes[2] + 1) & 15));

    credit = false;
    uni_hid_parser_switch_set_player_leds(&d, 2);
    uni_hid_parser_switch_set_player_leds(&d, 8);
    assert(!uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    assert(uni_hid_parser_switch_native_send(&d, compressed));
    baseline = sent_count;
    uni_hid_device_send_queued_reports(&d);
    uni_hid_device_send_queued_reports(&d);
    expect_led(baseline, 8, compressed);
    expect_led(baseline + 1, 8, compressed);
    assert(sent[baseline].bytes[2] == ((sent[baseline - 1].bytes[2] + 1) & 15));
    assert(sent[baseline + 1].bytes[2] == ((sent[baseline].bytes[2] + 1) & 15));
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
}

static void compatibility_and_ownership_timers(void) {
    reset();
    uni_hid_device_t d;
    init_device(&d, 0x40);
    unsigned baseline = sent_count;
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 125, 160, 200);
    uint8_t conventional[8];
    memcpy(conventional, &sent[baseline].bytes[3], 8);
    advance(39);
    assert(sent_count == baseline + 1);
    advance(1);
    expect_rumble(baseline + 1, 0x40, conventional);
    advance(40);
    expect_rumble(baseline + 2, 0x40, conventional);
    // Acquire retires both refresh and duration; they cannot stop native audio.
    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, first_word));
    baseline = sent_count;
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 1, 255, 255);
    uni_hid_parser_switch_play_dual_rumble(&d, 1, 1, 255, 255);
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 0, 0, 0);
    uint8_t competing[11] = {0xa2, 0x10, 0};
    memcpy(&competing[3], neutral, 8);
    uni_hid_device_send_intr_report(&d, competing, sizeof(competing));
    credit = false;
    uni_hid_device_send_intr_report(&d, competing, sizeof(competing));
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    advance(500);
    assert(sent_count == baseline);
    uni_hid_parser_switch_native_release(&d);
    expect_rumble(baseline, 0x40, neutral);
    assert(!uni_hid_parser_switch_native_send(&d, first_word));

    uni_hid_parser_switch_play_dual_rumble(&d, 100, 100, 33, 44);
    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, compressed));
    baseline = sent_count;
    advance(300);
    assert(sent_count == baseline);
    uni_hid_parser_switch_native_release(&d);
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 20, 33, 44);
    baseline = sent_count;
    advance(20);
    expect_rumble(baseline, 0x40, neutral);

    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, first_word));
    credit = false;
    uni_hid_parser_switch_native_release(&d);
    assert(!uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    baseline = sent_count;
    uni_hid_device_send_queued_reports(&d);
    expect_rumble(baseline, 0x40, neutral);
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));

    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, first_word));
    credit = false;
    uni_hid_parser_switch_native_release(&d);
    // Reacquiring must retire the delayed neutral, not stop the new owner.
    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    assert(uni_hid_parser_switch_native_send(&d, compressed));
    assert(uni_hid_parser_switch_native_send(&d, neutral));
    baseline = sent_count;
    credit = false;
    uni_hid_parser_switch_native_release(&d);
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    credit = true;
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 100, 33, 44);
    assert(sent_count == baseline + 1);
}

static void queue_retirement_disconnect_and_reuse(void) {
    reset();
    uni_hid_device_t d;
    init_device(&d, 0x40);
    // Walk the ring near its end before interleaving stale rumble and LEDs.
    for (unsigned i = 0; i < 30; ++i) {
        credit = false;
        uni_hid_parser_switch_set_player_leds(&d, 1);
        credit = true;
        uni_hid_device_send_queued_reports(&d);
    }
    credit = false;
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 200, 200, 200);
    uni_hid_parser_switch_set_player_leds(&d, 2);
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 0, 0, 0);
    uni_hid_parser_switch_set_player_leds(&d, 4);
    assert(uni_hid_parser_switch_native_acquire(&d));
    credit = true;
    assert(uni_hid_parser_switch_native_send(&d, compressed));
    unsigned baseline = sent_count;
    uni_hid_device_send_queued_reports(&d);
    uni_hid_device_send_queued_reports(&d);
    assert(uni_circular_buffer_is_empty(&d.outgoing_buffer));
    expect_led(baseline, 4, compressed);
    expect_led(baseline + 1, 4, compressed);

    uni_hid_parser_switch_native_release(&d);
    uni_hid_parser_switch_play_dual_rumble(&d, 100, 100, 200, 200);
    btstack_timer_source_t stale[32];
    unsigned stale_count = 0;
    for (unsigned i = 0; i < 32; ++i)
        if (timers[i].active) stale[stale_count++] = *timers[i].timer;
    uni_hid_device_disconnect(&d);
    uni_hid_device_delete(&d);
    init_device(&d, 0x42);
    assert(uni_hid_parser_switch_native_acquire(&d));
    assert(uni_hid_parser_switch_native_send(&d, first_word));
    baseline = sent_count;
    for (unsigned i = 0; i < stale_count; ++i) stale[i].process(&stale[i]);
    advance(300);
    assert(sent_count == baseline);
    // The other timers also retire on delete even without a preceding disconnect.
    uni_hid_parser_switch_native_release(&d);
    uni_hid_parser_switch_play_dual_rumble(&d, 0, 100, 30, 40);
    uni_hid_device_delete(&d);
    baseline = sent_count;
    advance(200);
    assert(sent_count == baseline);
}

static void identity_requires_real_reply(void) {
    reset();
    uni_hid_device_t d;
    begin_device(&d, 0x40);
    assert(!uni_hid_parser_switch_native_info(&d, NULL, NULL, NULL));
    finish_device(&d, 3, 0x80, 17);  // Truncated firmware/type tuple.
    assert(!uni_hid_parser_switch_native_acquire(&d));
    uni_hid_device_delete(&d);
    begin_device(&d, 0x40);
    finish_device(&d, 3, 0, 18);  // Negative acknowledgement is not evidence.
    assert(!uni_hid_parser_switch_native_info(&d, NULL, NULL, NULL));
    uni_hid_device_delete(&d);
    begin_device(&d, 0x40);
    finish_device(&d, 0x0b, 0x80, 18);
    uint8_t type;
    assert(uni_hid_parser_switch_native_info(&d, &type, NULL, NULL) && type == 0x0b);
    assert(!uni_hid_parser_switch_native_acquire(&d));
    uni_hid_device_delete(&d);
    for (uint8_t original_type = 1; original_type <= 2; ++original_type) {
        begin_device(&d, 0x40);
        finish_device(&d, original_type, 0x80, 18);
        assert(uni_hid_parser_switch_native_info(&d, &type, NULL, NULL));
        assert(type == original_type && uni_hid_parser_switch_native_acquire(&d));
        uni_hid_device_delete(&d);
    }
    assert(!uni_hid_parser_switch_native_acquire(NULL));
}

int main(void) {
    identity_and_per_device_counter();
    congestion_and_queued_leds();
    compatibility_and_ownership_timers();
    queue_retirement_disconnect_and_reuse();
    identity_requires_real_reply();
    puts("Switch parser native wire/queue/LED/ownership/timer checks passed");
    return 0;
}
