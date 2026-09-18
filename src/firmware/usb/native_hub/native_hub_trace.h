#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
#ifdef __cplusplus
extern "C" {
#endif

// Trace-build-only root vendor read: C0/5e, value5452, index=child slot,
// length16. Reply: NHTR, version1, status(0 captured/1 busy), slot, reserved0,
// little-endian capture time_us and control generation (both zero when busy).
enum {
    NATIVE_HUB_TRACE_REQUEST = 0x5e,
    NATIVE_HUB_TRACE_VALUE = 0x5452,
    NATIVE_HUB_TRACE_REPLY_SIZE = 16,
};

// Core1: call after EVERY failed selection, including OUT and while frozen.
// Observes the completed decision; never retries or changes the bank.
void native_hub_note_failed_select(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid);
// Core1: call after EVERY successful selection, including while frozen.
// Retains handovers, child SETUP/first IN/OUT, and the first child IN after a
// first-EP0-IN publication notification, even without a handover. Publication
// tickets correlate with a valid CONTROL_CLOCK arm/slot, not necessarily the
// current control. No pre-selection or endpoint/physical acceptance is implied.
void native_hub_note_selected_token(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid);
// Core0 execution tags. Backend lock tags include the source line.
enum {
    NATIVE_HUB_TRACE_PHASE_NONE = 0,
    NATIVE_HUB_TRACE_PHASE_LOG_COPY = 1,
    NATIVE_HUB_TRACE_PHASE_RADIO_POLL = 2,
    NATIVE_HUB_TRACE_PHASE_USB_TASK = 3,
    NATIVE_HUB_TRACE_PHASE_LOG_DRAIN = 4,
    NATIVE_HUB_TRACE_PHASE_PROTOCOL = 5,
    NATIVE_HUB_TRACE_PHASE_SLEEP = 6,
    NATIVE_HUB_TRACE_PHASE_BACKEND_LOCK = 0x10000,
};
uint32_t native_hub_trace_phase(uint32_t phase);

#ifdef __cplusplus
}
#endif
#endif
