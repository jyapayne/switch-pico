#pragma once

#include <stdint.h>

struct uni_hid_device_s;
using uni_hid_device_t = uni_hid_device_s;
using uni_play_dual_rumble_t = void (*)(uni_hid_device_t*, uint16_t, uint16_t,
                                       uint8_t, uint8_t);

struct uni_report_parser_t {
    uni_play_dual_rumble_t play_dual_rumble = nullptr;
};
struct uni_circular_buffer_t {
    unsigned queued = 0;
};
inline bool uni_circular_buffer_is_empty(const uni_circular_buffer_t* buffer) {
    return buffer->queued == 0;
}


// Only the parser boundary is faked. Native sends either enter the byte sink
// synchronously or fail without retaining a packet; no hidden transmit queue.
struct uni_hid_device_s {
    uni_report_parser_t report_parser{};
    struct {
        uint16_t interrupt_cid = 0;
        uint16_t handle = 0;
        bool connected = true;
    } conn;
    uni_circular_buffer_t outgoing_buffer{};
    bool info_ready = true;
    bool acquire_allowed = true;
    bool native_owned = false;
    uint8_t controller_type = 3;
    unsigned acquisitions = 0;
    unsigned releases = 0;
};

