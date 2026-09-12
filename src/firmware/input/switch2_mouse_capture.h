#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t SWITCH2_MOUSE_CAPTURE_CAPACITY = 9;
constexpr size_t SWITCH2_MOUSE_CAPTURE_REPORT_SIZE = 64;
constexpr size_t SWITCH2_MOUSE_CAPTURE_HEADER_SIZE = 16;
constexpr size_t SWITCH2_MOUSE_CAPTURE_ROW_SIZE = 20 + SWITCH2_MOUSE_CAPTURE_REPORT_SIZE;
constexpr size_t SWITCH2_MOUSE_CAPTURE_MAXIMUM_PAYLOAD_SIZE =
    SWITCH2_MOUSE_CAPTURE_HEADER_SIZE +
    SWITCH2_MOUSE_CAPTURE_CAPACITY * SWITCH2_MOUSE_CAPTURE_ROW_SIZE;

// Initialize once before starting the Bluetooth producer on its owning core.
// Repeated initialization never clears the boot-lifetime sequence or records.
void switch2_mouse_capture_init();

// Nondestructive, oldest-to-newest snapshot. All integers are little-endian.
// Header: total_records u32, count u8, eleven zero bytes.
// Row: serial u32, received_ms u32, PID u16, report_id u8, length u8,
//      address[6] in Bluepad32 order, two zero bytes, raw[64] (zero-padded).
// Returns zero before initialization or when the complete snapshot will not fit.
// total_records is taken under the same lock as the serialized rows.
size_t switch2_mouse_capture_snapshot(uint8_t* output, size_t capacity,
                                     uint32_t* total_records);

constexpr size_t SWITCH2_MOUSE_CAPTURE_NATIVE_INPUT_SIZE = 63;

#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
constexpr uint8_t SWITCH2_MOUSE_CAPTURE_SOURCE_COUNT = 2;
#else
constexpr uint8_t SWITCH2_MOUSE_CAPTURE_SOURCE_COUNT = 1;
#endif

struct Switch2MouseCaptureInput {
    uint32_t serial;
    uint32_t received_ms;
    bool active;
    // Unrotated native 07/08 payload bytes 2..3 and 5..7, without report ID.
    uint8_t buttons[2];
    uint8_t stick[3];
    // Latest opaque native 07/08 byte 8; preserve without interpretation.
    uint8_t native_status;
    // Cumulative relative motion, never consumed by a snapshot. A stream's
    // epoch is its first native packet's boot-lifetime serial; inactive is 0.
    uint32_t mouse_epoch;
    int64_t mouse_total_x;
    int64_t mouse_total_y;
    // Latest opaque native 07/08 byte 13; no interpreted surface semantics.
    uint8_t mouse_surface;
};

// Select one physical Joy-Con per instance by address and PID (2067 L, 2066 R).
// Invalid instance/address/PID or a source owned by another instance leaves the
// active selection unchanged. A physical source can never feed both instances.
// Selection starts empty; it never replays the serialized ring or clears it.
// Its latest native 07/08 input / teardown survives unrelated ring traffic.
// Each selection also disables and clears the native FIFO and pending sample.
void switch2_mouse_capture_select_input(uint8_t instance, const uint8_t address[6], uint16_t product_id);

// Copy a selected event newer than after_serial. A teardown is an event
// with active=false and zeroed fields. A new, still-empty selection returns a
// zero-serial inactive barrier to a reader with after_serial != 0.
// Otherwise false leaves output untouched. Nonzero boot-lifetime serials do not
// wrap, just as in the serialized capture.
// Cached reads do not consume totals; every selected native packet contributes,
// including identical consecutive deltas. A new stream receives a new epoch
// even when the reader missed its preceding teardown.
bool switch2_mouse_capture_latest_input(uint8_t instance, uint32_t after_serial,
                                       Switch2MouseCaptureInput* output);

#if SWITCH_PICO_SWITCH2_USB_BRIDGE
// Core 0 explicitly enables the selected source's ordered native 07/08 relay.
// Starts disabled; false clears the FIFO, repeated true preserves it. Enable
// never replays earlier capture-ring/latest-input data. Selection disables it;
// teardown and capture-serial exhaustion clear it without changing selection.
// Only exact selected Joy-Con 63-byte native payloads (07 left, 08 right) queue.
// The 32-entry FIFO is independent of the raw capture ring; overflow discards
// queued history and retains only the arriving packet.
void switch2_mouse_capture_set_native_stream(uint8_t instance, bool enabled);

// Copy the complete opaque 63-byte payload without its report ID. Returns its
// nonzero boot-lifetime capture serial, never reused, or 0 with report untouched.
// Peek is nondestructive: retry until commit after successful HID submission.
// If the latest selected source or FIFO head is >=500 ms old, discard the FIFO.
// Signed elapsed time tolerates a producer clock just ahead and uint32 rollover.
uint32_t switch2_mouse_capture_peek_native_report(
    uint8_t instance, uint32_t now_ms, uint8_t report[SWITCH2_MOUSE_CAPTURE_NATIVE_INPUT_SIZE]);

// Remove only the exact current head once. False leaves the FIFO unchanged,
// including for tokens invalidated by overflow, disable, teardown or selection.
bool switch2_mouse_capture_commit_native_report(uint8_t instance, uint32_t serial);

// Core 0: one cue for the selected, active Joy-Con's native stream.
// IDs 0..7 only; input must be newer than 500 ms. Accepted requests receive a
// nonzero boot-lifetime token, never reused by reset, selection or cancellation.
bool switch2_mouse_capture_request_sample(uint8_t instance, uint8_t sample_id, uint32_t now_ms,
                                          uint64_t* token);
// 0=pending, 1=verified source ACK (consumed once), -1=failed/stale/expired.
// A request expires 2000 ms after acceptance, including time awaiting dispatch.
int switch2_mouse_capture_sample_result(uint8_t instance, uint64_t token, uint32_t now_ms);
void switch2_mouse_capture_cancel_sample(uint8_t instance);
#endif
