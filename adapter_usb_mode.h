#pragma once

#include <stdint.h>

struct AdapterModeAvailability;

// Persisted user selection. Numeric values are part of adapter configuration
// schema v2 and the USB management protocol.
enum class AdapterRequestedMode : uint8_t {
    kAuto = 0,
    kSwitch = 1,
    kXInput = 2,
    kDInput = 3,
    kMac = 4,
};

// The immutable USB implementation selected before tusb_init(). Auto is a
// requested mode, not an active USB mode.
enum class AdapterUsbMode : uint8_t {
    kSwitch = 0,
    kSwitchProbe = 1,
    kXInput = 2,
};

// Availability of USB mode implementations in this firmware build. All mode
// selection paths consume this single value.
const AdapterModeAvailability& adapter_usb_mode_availability();
AdapterUsbMode adapter_host_probe_mode();
