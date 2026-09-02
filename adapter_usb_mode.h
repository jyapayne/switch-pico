#pragma once

#include <stdint.h>

enum class AdapterUsbMode : uint8_t {
    kSwitchProbe,
    kXInput,
};

AdapterUsbMode adapter_host_probe_mode();
