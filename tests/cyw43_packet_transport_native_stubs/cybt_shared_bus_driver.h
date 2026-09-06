#pragma once

#include <stdint.h>

/* Subset of the Pico SDK's exposed shared-bus ABI used by this transport. */
#define BTSDIO_FWBUF_SIZE 0x1000

typedef enum {
    CYBT_SUCCESS = 0,
    CYBT_ERR_BADARG = 0xB1,
    CYBT_ERR_TIMEOUT = 0xB3,
    CYBT_ERR_HCI_READ_FAILED = 0xB9,
} cybt_result_t;

typedef enum {
    H2B_BUF_ADDR_IDX = 0x10,
    H2B_BUF_IN_ADDR_IDX,
    H2B_BUF_OUT_ADDR_IDX,
    B2H_BUF_ADDR_IDX,
    B2H_BUF_IN_ADDR_IDX,
    B2H_BUF_OUT_ADDR_IDX,
} cybt_addr_idx_t;

typedef struct {
    uint32_t host2bt_in_val;
    uint32_t host2bt_out_val;
    uint32_t bt2host_in_val;
    uint32_t bt2host_out_val;
} cybt_fw_membuf_index_t;

cybt_result_t cybt_set_bt_awake(int value);
int cybt_awake(void);
cybt_result_t cybt_get_bt_buf_index(cybt_fw_membuf_index_t *indices);
cybt_result_t cybt_mem_read_idx(cybt_addr_idx_t index, uint32_t offset,
                                uint8_t *buffer, uint32_t length);
cybt_result_t cybt_reg_write_idx(cybt_addr_idx_t index, uint32_t value);
cybt_result_t cybt_toggle_bt_intr(void);
