#pragma once

#include <stdint.h>

#include "adapter_usb_mode.h"
#include "controller_state.h"

constexpr uint8_t ADAPTER_MODE_CONTROLLER_SLOT_COUNT = 4;
constexpr uint32_t ADAPTER_MODE_CHORD_HOLD_MS = 3000;
constexpr uint16_t ADAPTER_MODE_CHORD_BUTTON_MASK =
    static_cast<uint16_t>((1u << 4) | (1u << 5) | (1u << 6) |
                          (1u << 7) | (1u << 8));

// Loads persistent configuration on Core 0, consumes watchdog scratch,
// freezes the output implementation, then starts TinyUSB in that exact order.
void adapter_mode_controller_initialize_usb();
AdapterRequestedMode adapter_mode_controller_requested_mode();

// Observes the physical pre-hotkey mask and removes the mode chord from both
// that mask and the state before profile processing. Slot state is isolated by
// connection generation and all time comparisons are uint32-wrap safe.
void adapter_mode_controller_process_input(
    uint8_t slot, bool active, uint32_t connection_generation,
    uint16_t* pre_hotkey_button_mask, uint32_t now_ms,
    ControllerState* state);

// Advances serialized internal mode writes, acknowledgement, recovery, and
// reboot work. Call once per Core-0 loop after processing every input slot.
void adapter_mode_controller_task(uint32_t now_ms);

// Starts the physical ten-second recovery operation: clear only Bluetooth
// pairings, then restore requested mode Auto as the final persisted mutation
// and reboot immediately after that transaction is still current.
void adapter_mode_controller_begin_recovery();
