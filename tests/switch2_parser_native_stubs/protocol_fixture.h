#pragma once

#include <btstack.h>
#include "uni_hid_device.h"

// The parser uses the SDK's actual GATT types/accessors. Only the asynchronous
// radio, run loop, platform admission and persistence endpoints are simulated.
enum fixture_query { QUERY_NONE, QUERY_SERVICE, QUERY_CHARACTERISTICS, QUERY_DESCRIPTORS, QUERY_CCCD, QUERY_WRITE };
#define FIXTURE_RUMBLE_HISTORY 128
struct fixture_peer {
    uni_hid_device_t device;
    bool used, link_alive;
    enum fixture_query query;
    btstack_packet_handler_t callback;
    uint16_t descriptor_value, cccd_handle;
    uint8_t command[32];
    uint16_t command_length;
    const uint8_t* pending_write;
    uint16_t pending_length;
    uint8_t pending_snapshot[33];
    uint8_t rumble[33];
    uint16_t rumble_length;
    unsigned commands, rumbles;
    uint8_t rumble_history[FIXTURE_RUMBLE_HISTORY][33];
    uint32_t rumble_times[FIXTURE_RUMBLE_HISTORY];
};
