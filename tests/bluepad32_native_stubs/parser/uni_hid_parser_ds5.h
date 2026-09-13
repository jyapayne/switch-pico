#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <uni.h>

typedef struct {
    uint32_t report_sequence;
    uint32_t motion_sequence;
    bool motion_valid;
} uni_ds5_bridge_snapshot_t;

void uni_hid_parser_ds5_parse_input_report(uni_hid_device_t*, const uint8_t*, uint16_t);
bool uni_hid_parser_ds5_bridge_rumble(uni_hid_device_t*, uint16_t, uint8_t, uint8_t);
bool uni_hid_parser_ds5_bridge_snapshot(uni_hid_device_t*, uni_ds5_bridge_snapshot_t*);
