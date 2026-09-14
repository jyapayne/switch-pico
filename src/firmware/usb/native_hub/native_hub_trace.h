#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
#ifdef __cplusplus
extern "C" {
#endif

// Core1: selected token wrapper; the original selector owns every hardware decision.
bool native_hub_select_device_traced(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid);
// Core1: call after EVERY failed selection, including OUT and while frozen.
// Observes the completed decision; never retries or changes the bank.
void native_hub_note_failed_select(uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid);
// Core0: call only AFTER restoring the logger's saved interrupt state.
void native_hub_note_log_mask(uint32_t elapsed_us, uint32_t bytes, bool already_masked);
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
