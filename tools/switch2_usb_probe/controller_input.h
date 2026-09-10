#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool active;
    uint32_t serial;
    uint8_t buttons[2];
    uint8_t stick[3];
    // Latest opaque native 08 byte 8.
    uint8_t native_status;
    // Cumulative signed relative totals within mouse_epoch, not per-poll
    // deltas. Cached polls repeat these totals without consuming motion.
    // Epoch changes on reconnect, including a teardown missed between polls.
    uint32_t mouse_epoch;
    int64_t mouse_total_x;
    int64_t mouse_total_y;
    // Latest opaque native 08 byte 13.
    uint8_t mouse_surface;
} probe_controller_input;

// Core 0, before stdio/peripheral initialization.
void probe_controller_input_clock_init(void);
// Core 0, after stdio and before protocol reset or USB startup.
void probe_controller_input_init(void);
// True means both cores are registered for flash coordination, not that the
// radio is ready or a controller is connected. Failure is latched: keep USB
// and flash-writing protocol operations disabled rather than retrying startup.
bool probe_controller_input_start(void);
// Core 0 after start(): polls the existing two-second BOOTSEL hold gesture.
// True means a Bluetooth pairing-window request was queued. Long holds NEVER
// clear pairings in this bridge, and this does not inject USB controller input.
bool probe_controller_input_pairing_task(void);
// Core 0 native 08 relay: disabled until explicitly enabled after flash-ready
// startup. Disable clears queued data, repeated enable preserves it. Resets
// must disable the stream; this never changes Bluetooth bonds or pairing.
// Only subsequent selected-source packets enter the separate 32-entry FIFO.
// Overflow drops queued history and retains only the arriving packet.
void probe_controller_input_set_native_stream(bool enabled);
// Copy a full opaque 63-byte payload (without report ID), oldest first. Returns
// its never-reused boot-lifetime serial; 0 leaves report untouched. Latest source
// or head >=500 ms old discards the FIFO; now_ms is the Pico boot-ms clock.
// Nondestructive until successful HID submission followed by commit.
uint32_t probe_controller_input_peek_native_report(uint32_t now_ms, uint8_t report[63]);
// Remove only the exact current head once. A stale/replaced token cannot pop a
// new stream's packet. Before flash-ready startup peek/commit return 0/false.
bool probe_controller_input_commit_native_report(uint32_t serial);
// Built-in vibration samples only; raw HD-rumble output is not forwarded.
// A nonzero token means queued, not acknowledged. Result: 0 pending, 1 real
// source ACK, -1 failed/stale. Reset cancels the request, never stored pairing.
bool probe_controller_input_play_sample(uint8_t sample_id, uint64_t* token);
int probe_controller_input_sample_result(uint64_t token, uint32_t now_ms);
void probe_controller_input_cancel_sample(void);
// Core 0 at 250 Hz; now_ms uses the Pico boot-millisecond clock. Only fresh
// native 08 buttons/stick and cumulative mouse totals are exposed. Inactive
// fields are zero except serial; the USB protocol must supply its calibrated
// stick center rather than forwarding inactive stick bytes.
void probe_controller_input_poll(uint32_t now_ms, probe_controller_input* out);

#ifdef __cplusplus
}
#endif
