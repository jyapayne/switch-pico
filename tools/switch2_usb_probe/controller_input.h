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
    // Latest opaque native 07/08 byte 8.
    uint8_t native_status;
    // Cumulative signed relative totals within mouse_epoch, not per-poll
    // deltas. Cached polls repeat these totals without consuming motion.
    // Epoch changes on reconnect, including a teardown missed between polls.
    uint32_t mouse_epoch;
    int64_t mouse_total_x;
    int64_t mouse_total_y;
    // Latest opaque native 07/08 byte 13.
    uint8_t mouse_surface;
} probe_controller_input;

// Core 0, before stdio/peripheral initialization.
void probe_controller_input_clock_init(void);
// Core 0, after stdio and before protocol reset or USB startup.
void probe_controller_input_init(void);
// True means flash coordination is ready, not that the radio is ready or a
// controller is connected. Hub mode initializes on Core 0 with Core 1 reserved
// for SRAM-only USB; other modes register both cores and launch the radio there.
// Failure is latched: keep USB and flash-writing protocol operations disabled.
bool probe_controller_input_start(void);
// Core 0 main loop before USB tasks, outside IRQs and application state locks.
// Hub mode cooperatively services CYW43/BTstack, including storage and haptics;
// a no-op before successful start and in dedicated-radio modes.
void probe_controller_input_task(void);
// Core 0 after start(): polls the existing two-second BOOTSEL hold gesture.
// True means a Bluetooth pairing-window request was queued. Long holds NEVER
// clear pairings in this bridge, and this does not inject USB controller input.
bool probe_controller_input_pairing_task(void);
#if SWITCH2_BRIDGE_WII_INPUT
// Main supplies the same validated calibration record advertised to the host.
void probe_controller_input_set_stick_calibration(const uint8_t calibration[9]);
// Native feature changes are output barriers, not Bluetooth/IMU resets.
void probe_controller_input_set_native_features(uint8_t features);
#endif
#if SWITCH2_BRIDGE_FULL_INPUT
// Supply each child's advertised, validated nine-byte stick record. Native
// output stays unavailable until that child's calibration has been supplied.
void probe_controller_input_set_full_stick_calibration(uint8_t instance, const uint8_t calibration[9]);
#endif
// Core0 native07/08 output. Disable discards queued/prepared data; repeated
// enable preserves it. Joy-Con mode relays its bounded FIFO; right-only Wii
// mode synthesizes fresh calibrated sensors and the selected IR pointer.
// Full-controller mode splits one supported gamepad into independent R/L output streams;
// controls remain live while motion is unavailable. Only Wii estimates stationary bias.
// No pairing changes.
void probe_controller_input_set_native_stream(uint8_t instance, bool enabled);
// Copy one63-byte payload without report ID. Returns a boot-unique token, or0
// without changing output. Nondestructive until successful HID submission and
// commit. now_ms uses the Pico boot-ms clock; unavailable/stale input is rejected.
uint32_t probe_controller_input_peek_native_report(uint8_t instance, uint32_t now_ms, uint8_t report[63]);
// Remove only the exact current head once. A stale/replaced token cannot pop a
// new stream's packet. Before flash-ready startup peek/commit return 0/false.
bool probe_controller_input_commit_native_report(uint8_t instance, uint32_t serial);
// Built-in vibration samples only; raw HD-rumble output is not forwarded.
// A nonzero token means queued, not completed. Result:0 pending,1 completion,
// -1 failed/stale. Joy-Con completion is its application ACK; Wii/DualSense
// completion is actual bounded rumble-driver dispatch, not a source application
// ACK or an HD-waveform fidelity claim.
// Reset cancels the request, never stored pairing.
bool probe_controller_input_play_sample(uint8_t instance, uint8_t sample_id, uint64_t* token);
int probe_controller_input_sample_result(uint8_t instance, uint64_t token, uint32_t now_ms);
void probe_controller_input_cancel_sample(uint8_t instance);
// Core0 at250Hz; now_ms uses Pico boot milliseconds. Supplies current mapped
// controls for diagnostic reports; the native sender owns motion consumption.
// Inactive controls are zero except serial; USB supplies its calibrated center.
void probe_controller_input_poll(uint8_t instance, uint32_t now_ms, probe_controller_input* out);

#ifdef __cplusplus
}
#endif
