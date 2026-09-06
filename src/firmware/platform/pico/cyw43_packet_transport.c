/*
 * Opt-in GNU --wrap=cyw43_btbus_read / --wrap=cyw43_btbus_init transport.
 * Requires the build-local cybt_shared_bus_driver.c status-propagation patch;
 * the stock driver's memory-read helpers discard SPI errors.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cyw43_btbus.h"
#include "cyw43_config.h"
#include "cybt_shared_bus_driver.h"

#define PACKET_HEADER_SIZE 4u
#define RING_MASK (BTSDIO_FWBUF_SIZE - 1u)
#define RING_CAPACITY (BTSDIO_FWBUF_SIZE - 4u)
#define WAKE_RETRIES 300u

/*
 * Preserve cybt_hci_read's overflow check, not its cached indices. This lower
 * bound is reset with every controller initialization, under the driver lock.
 * All actual indices and packet availability are fetched afresh on each call.
 */
static uint32_t minimum_available;

int __real_cyw43_btbus_init(cyw43_ll_t *self);

int __wrap_cyw43_btbus_init(cyw43_ll_t *self) {
    CYW43_THREAD_ENTER
    minimum_available = 0;
    int result = __real_cyw43_btbus_init(self);
    CYW43_THREAD_EXIT
    return result;
}

static cybt_result_t request_bus(void) {
    cybt_result_t result = cybt_set_bt_awake(true);
    if (result != CYBT_SUCCESS) {
        return result;
    }
    /* Match the SDK's bounded 301 polls and 1 ms wake-handshake delay. */
    for (uint32_t remaining = WAKE_RETRIES;; --remaining) {
        int awake = cybt_awake();
        if (awake == 1) {
            return CYBT_SUCCESS;
        }
        if (awake != 0) {
            return CYBT_ERR_HCI_READ_FAILED;
        }
        cyw43_delay_ms(1);
        if (remaining == 0) {
            return CYBT_ERR_TIMEOUT;
        }
    }
}

/* All offsets, lengths and destinations here are word aligned. */
static cybt_result_t read_ring(uint32_t offset, uint8_t *buffer, uint32_t length) {
    if (length == 0) {
        return CYBT_SUCCESS;
    }
    uint32_t first = BTSDIO_FWBUF_SIZE - offset;
    if (first > length) {
        first = length;
    }
    cybt_result_t result = cybt_mem_read_idx(B2H_BUF_ADDR_IDX, offset, buffer, first);
    if (result != CYBT_SUCCESS || first == length) {
        return result;
    }
    return cybt_mem_read_idx(B2H_BUF_ADDR_IDX, 0, buffer + first, length - first);
}

static cybt_result_t read_packet(uint8_t *buffer, uint32_t capacity, uint32_t *size) {
    cybt_fw_membuf_index_t indices;
    cybt_result_t result = cybt_get_bt_buf_index(&indices);
    if (result != CYBT_SUCCESS) {
        return result;
    }
    /* Keep the SDK's four-register corruption checks effective in release. */
    if (indices.host2bt_in_val >= BTSDIO_FWBUF_SIZE ||
        indices.host2bt_out_val >= BTSDIO_FWBUF_SIZE ||
        indices.bt2host_in_val >= BTSDIO_FWBUF_SIZE ||
        indices.bt2host_out_val >= BTSDIO_FWBUF_SIZE ||
        (indices.bt2host_out_val & 3u) != 0) {
        return CYBT_ERR_HCI_READ_FAILED;
    }
    uint32_t available = (indices.bt2host_in_val - indices.bt2host_out_val) & RING_MASK;
    if (available > RING_CAPACITY) {
        return CYBT_ERR_HCI_READ_FAILED;
    }
    if (available < minimum_available) {
        panic("cyw43 buffer overflow");
    }
    minimum_available = available;
    if (available < PACKET_HEADER_SIZE) {
        /* No partial-header consumption and no polling for more packet data. */
        return CYBT_SUCCESS;
    }

    result = read_ring(indices.bt2host_out_val, buffer, PACKET_HEADER_SIZE);
    if (result != CYBT_SUCCESS) {
        return result;
    }
    uint32_t payload_length = (uint32_t)buffer[0] |
                              ((uint32_t)buffer[1] << 8) |
                              ((uint32_t)buffer[2] << 16);
    if (payload_length > capacity - PACKET_HEADER_SIZE ||
        payload_length > RING_CAPACITY - PACKET_HEADER_SIZE) {
        return CYBT_ERR_BADARG;
    }
    uint32_t payload_wire_length = (payload_length + 3u) & ~3u;
    uint32_t wire_length = PACKET_HEADER_SIZE + payload_wire_length;
    if (wire_length > available) {
        /* The producer may finish later; retain both header and payload. */
        return CYBT_SUCCESS;
    }

    uint32_t payload_offset = (indices.bt2host_out_val + PACKET_HEADER_SIZE) & RING_MASK;
    if (wire_length <= capacity) {
        result = read_ring(payload_offset, buffer + PACKET_HEADER_SIZE, payload_wire_length);
    } else {
        /* A valid unpadded capacity must not permit a 1-3 byte buffer overrun. */
        uint32_t whole_length = payload_length & ~3u;
        result = read_ring(payload_offset, buffer + PACKET_HEADER_SIZE, whole_length);
        if (result == CYBT_SUCCESS) {
            uint32_t last_word;
            result = read_ring((payload_offset + whole_length) & RING_MASK,
                               (uint8_t *)&last_word, sizeof(last_word));
            if (result == CYBT_SUCCESS) {
                memcpy(buffer + PACKET_HEADER_SIZE + whole_length, &last_word,
                       payload_length - whole_length);
            }
        }
    }
    if (result != CYBT_SUCCESS) {
        return result;
    }

    /*
     * The producer publishes only readable bytes and may not overwrite unread
     * bytes (the same ring ownership rule as the SDK). One availability snapshot
     * therefore covers header, payload and padding until this commit point.
     */
    uint32_t next = (indices.bt2host_out_val + wire_length) & RING_MASK;
    result = cybt_reg_write_idx(B2H_BUF_OUT_ADDR_IDX, next);
    if (result != CYBT_SUCCESS) {
        return result;
    }
    minimum_available = available - wire_length;
    /*
     * DATA_VALID signals changed host ring state, not an RX IRQ acknowledgement:
     * cyw43_ll_bt_has_work separately clears SDIO_INT_STATUS/I_HMB_FC_CHANGE.
     * Empty/partial reads change no ring state and need no DATA_VALID toggle.
     * The SDK's notification helper always succeeds (its register write is void).
     * If that contract changes, a failure here cannot safely roll back the
     * consumer: the controller already owns the reclaimed bytes.
     */
    result = cybt_toggle_bt_intr();
    if (result != CYBT_SUCCESS) {
        return result;
    }
    *size = PACKET_HEADER_SIZE + payload_length;
    return CYBT_SUCCESS;
}

int __wrap_cyw43_btbus_read(uint8_t *buffer, uint32_t capacity, uint32_t *size) {
    if (size == NULL) {
        return -1;
    }
    *size = 0;
    if (buffer == NULL || capacity < PACKET_HEADER_SIZE || ((uintptr_t)buffer & 3u) != 0) {
        return -1;
    }

    CYW43_THREAD_ENTER
    cybt_result_t result = request_bus();
    if (result == CYBT_SUCCESS) {
        result = read_packet(buffer, capacity, size);
    }
    CYW43_THREAD_EXIT
    return result == CYBT_SUCCESS ? 0 : -1;
}
