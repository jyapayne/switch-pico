#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cyw43_btbus.h"
#include "cyw43_config.h"
#include "cybt_shared_bus_driver.h"

static uint8_t ring[BTSDIO_FWBUF_SIZE];
static cybt_fw_membuf_index_t indices;
static unsigned lock_depth;
static unsigned enters;
static unsigned exits;
static unsigned wake_polls;
static unsigned delays;
static unsigned snapshots;
static unsigned reads;
static unsigned publications;
static unsigned notifications;
static unsigned fail_read;
static bool fail_wake;
static bool fail_snapshot;
static bool fail_publication;
static bool fail_notification;
static int awake_value;
static bool append_on_header;
static bool expect_panic;
static jmp_buf panic_jump;

void cyw43_thread_enter(void) {
    ++lock_depth;
    ++enters;
}

void cyw43_thread_exit(void) {
    assert(lock_depth != 0);
    --lock_depth;
    ++exits;
}

void cyw43_delay_ms(uint32_t milliseconds) {
    assert(lock_depth != 0);
    assert(milliseconds == 1);
    ++delays;
}

_Noreturn void panic(const char *message, ...) {
    if (!expect_panic) {
        fprintf(stderr, "unexpected panic: %s\n", message);
        abort();
    }
    assert(strcmp(message, "cyw43 buffer overflow") == 0);
    longjmp(panic_jump, 1);
}

int test_cyw43_btbus_init(cyw43_ll_t *self) {
    (void)self;
    assert(lock_depth != 0);
    memset(&indices, 0, sizeof(indices));
    return 0;
}

cybt_result_t cybt_set_bt_awake(int value) {
    assert(lock_depth != 0);
    assert(value == 1);
    return fail_wake ? CYBT_ERR_HCI_READ_FAILED : CYBT_SUCCESS;
}

int cybt_awake(void) {
    assert(lock_depth != 0);
    ++wake_polls;
    return awake_value;
}

cybt_result_t cybt_get_bt_buf_index(cybt_fw_membuf_index_t *out) {
    assert(lock_depth != 0);
    ++snapshots;
    if (fail_snapshot) {
        return CYBT_ERR_HCI_READ_FAILED;
    }
    *out = indices;
    return CYBT_SUCCESS;
}

static uint32_t put_packet(uint32_t offset, uint32_t payload_length, uint8_t type) {
    uint32_t wire_length = 4 + ((payload_length + 3) & ~3u);
    assert(wire_length <= BTSDIO_FWBUF_SIZE);
    for (uint32_t byte = 0; byte < wire_length; ++byte) {
        uint8_t value = (uint8_t)(byte - 4);
        if (byte < 3) {
            value = (uint8_t)(payload_length >> (byte * 8));
        } else if (byte == 3) {
            value = type;
        }
        ring[(offset + byte) & (BTSDIO_FWBUF_SIZE - 1)] = value;
    }
    return (offset + wire_length) & (BTSDIO_FWBUF_SIZE - 1);
}

cybt_result_t cybt_mem_read_idx(cybt_addr_idx_t index, uint32_t offset,
                                uint8_t *buffer, uint32_t length) {
    assert(lock_depth != 0);
    assert(index == B2H_BUF_ADDR_IDX);
    assert(length >= 4 && (length & 3) == 0);
    assert((offset & 3) == 0 && offset + length <= BTSDIO_FWBUF_SIZE);
    assert(((uintptr_t)buffer & 3) == 0);
    ++reads;
    if (reads == fail_read) {
        /* Model a failed SPI transfer that has already modified its destination. */
        buffer[0] = 0xEF;
        return CYBT_ERR_HCI_READ_FAILED;
    }
    memcpy(buffer, ring + offset, length);
    if (append_on_header && reads == 1) {
        indices.bt2host_in_val = put_packet(indices.bt2host_in_val, 4, 2);
    }
    return CYBT_SUCCESS;
}

cybt_result_t cybt_reg_write_idx(cybt_addr_idx_t index, uint32_t value) {
    assert(lock_depth != 0);
    assert(index == B2H_BUF_OUT_ADDR_IDX);
    assert(value < BTSDIO_FWBUF_SIZE && (value & 3) == 0);
    ++publications;
    if (fail_publication) {
        return CYBT_ERR_HCI_READ_FAILED;
    }
    indices.bt2host_out_val = value;
    return CYBT_SUCCESS;
}

cybt_result_t cybt_toggle_bt_intr(void) {
    assert(lock_depth != 0);
    assert(publications > notifications);
    ++notifications;
    return fail_notification ? CYBT_ERR_HCI_READ_FAILED : CYBT_SUCCESS;
}

static void reset(void) {
    lock_depth = 0;
    expect_panic = false;
    assert(cyw43_btbus_init(NULL) == 0);
    memset(ring, 0, sizeof(ring));
    enters = exits = wake_polls = delays = snapshots = reads = publications = notifications = 0;
    fail_read = 0;
    fail_wake = fail_snapshot = fail_publication = fail_notification = false;
    append_on_header = false;
    awake_value = 1;
}

static void check_unconsumed(uint32_t original_out) {
    assert(indices.bt2host_out_val == original_out);
    assert(notifications == 0);
    assert(lock_depth == 0 && enters == exits);
}

static void check_packet(const uint8_t *buffer, uint32_t length, uint8_t type) {
    assert(buffer[0] == (uint8_t)length);
    assert(buffer[1] == (uint8_t)(length >> 8));
    assert(buffer[2] == (uint8_t)(length >> 16));
    assert(buffer[3] == type);
    for (uint32_t byte = 0; byte < length; ++byte) {
        assert(buffer[4 + byte] == (uint8_t)byte);
    }
}

static void test_empty_and_partial(void) {
    _Alignas(4) uint8_t buffer[32];
    uint32_t size = 123;
    reset();
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 0);
    assert(snapshots == 1 && reads == 0 && publications == 0);
    check_unconsumed(0);

    uint32_t end = put_packet(0, 5, 4);
    for (uint32_t published = 1; published <= 3; ++published) {
        indices.bt2host_in_val = published;
        size = 123;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 0);
        assert(reads == 0 && publications == 0);
        check_unconsumed(0);
    }
    const uint32_t partial_lengths[] = {4, 8, 9, 11};
    for (unsigned i = 0; i < sizeof(partial_lengths) / sizeof(partial_lengths[0]); ++i) {
        indices.bt2host_in_val = partial_lengths[i];
        size = 123;
        unsigned before_reads = reads;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 0);
        assert(reads == before_reads + 1 && publications == 0);
        check_unconsumed(0);
    }
    indices.bt2host_in_val = end;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 9);
    check_packet(buffer, 5, 4);
    assert(indices.bt2host_out_val == end && publications == 1 && notifications == 1);
}

static void test_complete_wrapped_and_full(void) {
    _Alignas(4) uint8_t buffer[BTSDIO_FWBUF_SIZE];
    const uint32_t starts[] = {0, BTSDIO_FWBUF_SIZE - 4, BTSDIO_FWBUF_SIZE - 8};
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); ++i) {
        reset();
        indices.bt2host_out_val = starts[i];
        indices.bt2host_in_val = put_packet(starts[i], 9, 2);
        uint32_t size = 0;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 13);
        check_packet(buffer, 9, 2);
        assert(indices.bt2host_out_val == indices.bt2host_in_val);
        assert(snapshots == 1 && publications == 1 && notifications == 1);
        assert(lock_depth == 0 && enters == exits);
    }

    reset();
    indices.bt2host_in_val = put_packet(0, BTSDIO_FWBUF_SIZE - 8, 2);
    uint32_t size = 0;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0);
    assert(size == BTSDIO_FWBUF_SIZE - 4);
    check_packet(buffer, BTSDIO_FWBUF_SIZE - 8, 2);
    assert(indices.bt2host_out_val == indices.bt2host_in_val);

    reset();
    indices.bt2host_in_val = put_packet(0, 0, 0);
    assert(cyw43_btbus_read(buffer, 4, &size) == 0 && size == 4);
    assert(reads == 1 && publications == 1 && notifications == 1);
}

static void test_exact_capacity_and_snapshot_growth(void) {
    _Alignas(4) uint8_t buffer[32];
    uint32_t size = 0;
    reset();
    memset(buffer, 0xA5, sizeof(buffer));
    indices.bt2host_in_val = put_packet(0, 5, 4);
    assert(cyw43_btbus_read(buffer, 9, &size) == 0 && size == 9);
    check_packet(buffer, 5, 4);
    for (unsigned i = 9; i < sizeof(buffer); ++i) {
        assert(buffer[i] == 0xA5);
    }

    reset();
    indices.bt2host_in_val = put_packet(0, 5, 4);
    uint32_t first_end = indices.bt2host_in_val;
    append_on_header = true;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 9);
    check_packet(buffer, 5, 4);
    assert(indices.bt2host_out_val == first_end && snapshots == 1);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 8);
    check_packet(buffer, 4, 2);
    assert(indices.bt2host_out_val == indices.bt2host_in_val);
    assert(publications == 2 && notifications == 2);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 0);
    assert(publications == 2 && notifications == 2);
}

static void test_oversized_and_invalid(void) {
    _Alignas(4) uint8_t buffer[BTSDIO_FWBUF_SIZE * 2];
    uint32_t size = 123;
    reset();
    indices.bt2host_in_val = put_packet(0, 5, 4);
    assert(cyw43_btbus_read(buffer, 8, &size) != 0 && size == 0);
    check_unconsumed(0);
    assert(reads == 1 && publications == 0);
    ring[0] = ring[1] = ring[2] = 0xFF;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    check_unconsumed(0);

    reset();
    /* This would occupy the reserved word and make full indistinguishable from empty. */
    put_packet(0, BTSDIO_FWBUF_SIZE - 4, 2);
    indices.bt2host_in_val = 4;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    check_unconsumed(0);

    reset();
    assert(cyw43_btbus_read(NULL, sizeof(buffer), &size) != 0 && size == 0);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), NULL) != 0);
    assert(cyw43_btbus_read(buffer + 1, sizeof(buffer) - 1, &size) != 0);
    for (uint32_t capacity = 0; capacity < 4; ++capacity) {
        assert(cyw43_btbus_read(buffer, capacity, &size) != 0 && size == 0);
    }
    assert(enters == 0 && snapshots == 0 && publications == 0);
    check_unconsumed(0);

    for (unsigned field = 0; field < 4; ++field) {
        reset();
        switch (field) {
            case 0: indices.host2bt_in_val = BTSDIO_FWBUF_SIZE; break;
            case 1: indices.host2bt_out_val = BTSDIO_FWBUF_SIZE; break;
            case 2: indices.bt2host_in_val = BTSDIO_FWBUF_SIZE; break;
            case 3: indices.bt2host_out_val = BTSDIO_FWBUF_SIZE; break;
        }
        uint32_t original_out = indices.bt2host_out_val;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
        check_unconsumed(original_out);
        assert(reads == 0 && publications == 0);
    }
    reset();
    indices.bt2host_out_val = 1;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    check_unconsumed(1);
    reset();
    indices.bt2host_in_val = BTSDIO_FWBUF_SIZE - 1;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    check_unconsumed(0);
}

static void test_status_failures_and_retry(void) {
    _Alignas(4) uint8_t buffer[32];
    uint32_t size = 123;
    reset();
    fail_wake = true;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    assert(snapshots == 0 && wake_polls == 0);
    check_unconsumed(0);
    reset();
    awake_value = -1;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    assert(snapshots == 0 && wake_polls == 1);
    check_unconsumed(0);
    reset();
    awake_value = 0;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    assert(snapshots == 0 && wake_polls == 301 && delays == 301);
    check_unconsumed(0);
    reset();
    fail_snapshot = true;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    assert(reads == 0 && publications == 0);
    check_unconsumed(0);

    /* Fail header, first wrapped payload segment, then second payload segment. */
    for (unsigned failure = 1; failure <= 3; ++failure) {
        reset();
        indices.bt2host_out_val = BTSDIO_FWBUF_SIZE - 8;
        indices.bt2host_in_val = put_packet(indices.bt2host_out_val, 9, 2);
        fail_read = failure;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
        assert(publications == 0);
        check_unconsumed(BTSDIO_FWBUF_SIZE - 8);
        fail_read = 0;
        assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 13);
        check_packet(buffer, 9, 2);
        assert(indices.bt2host_out_val == indices.bt2host_in_val);
    }
    reset();
    indices.bt2host_in_val = put_packet(0, 5, 4);
    fail_read = 3; /* Final padded word when the caller provides exact capacity. */
    assert(cyw43_btbus_read(buffer, 9, &size) != 0 && size == 0);
    assert(publications == 0);
    check_unconsumed(0);

    reset();
    indices.bt2host_in_val = put_packet(0, 5, 4);
    fail_publication = true;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    check_unconsumed(0);
    assert(publications == 1);

    reset();
    indices.bt2host_in_val = put_packet(0, 5, 4);
    fail_notification = true;
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) != 0 && size == 0);
    /* Notification follows the irreversible commit; rollback would violate ownership. */
    assert(indices.bt2host_out_val == indices.bt2host_in_val);
    assert(publications == 1 && notifications == 1);
    assert(lock_depth == 0 && enters == exits);
}

static void test_overflow_and_controller_reset(void) {
    _Alignas(4) uint8_t buffer[32];
    uint32_t size = 0;
    reset();
    uint32_t first_end = put_packet(0, 5, 4);
    indices.bt2host_in_val = put_packet(first_end, 8, 2);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 9);
    assert(indices.bt2host_out_val == first_end);
    indices.bt2host_in_val -= 4; /* Published unread data disappeared. */
    expect_panic = true;
    if (setjmp(panic_jump) == 0) {
        cyw43_btbus_read(buffer, sizeof(buffer), &size);
        assert(false && "overflow must retain the SDK's fatal check");
    }
    expect_panic = false;
    assert(indices.bt2host_out_val == first_end);
    assert(publications == 1 && notifications == 1);
    lock_depth = 0; /* panic does not return on hardware. */

    reset();
    first_end = put_packet(0, 5, 4);
    indices.bt2host_in_val = put_packet(first_end, 8, 2);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 9);
    assert(cyw43_btbus_init(NULL) == 0);
    assert(cyw43_btbus_read(buffer, sizeof(buffer), &size) == 0 && size == 0);
    assert(indices.bt2host_out_val == 0 && lock_depth == 0);
}

int main(void) {
    test_empty_and_partial();
    test_complete_wrapped_and_full();
    test_exact_capacity_and_snapshot_growth();
    test_oversized_and_invalid();
    test_status_failures_and_retry();
    test_overflow_and_controller_reset();
    puts("cyw43 packet transport regression checks passed");
    return 0;
}
