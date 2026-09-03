#pragma once

#include <stdint.h>

// Correlated normal reboot entry point for management opcode 0x03. Only a
// successful host mode transaction can reset profile runtime and reboot.
bool adapter_reboot_for_mode_transaction(uint32_t transaction_id);

// Schedules a direct ROM USB BOOTSEL reboot after the current endpoint-zero
// control transfer has completed.
bool adapter_reboot_to_bootsel();
