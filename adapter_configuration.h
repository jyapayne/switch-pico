#pragma once

#include <stddef.h>
#include <stdint.h>

#include "adapter_usb_mode.h"

constexpr uint16_t ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION = 1;
constexpr uint16_t ADAPTER_CONFIGURATION_SCHEMA_VERSION = 2;
constexpr size_t ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE = 4;
constexpr size_t ADAPTER_CONFIGURATION_ENCODED_SIZE = 8;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MIN = 10;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MAX = 300;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT = 60;

struct AdapterConfiguration {
    uint16_t pairing_window_seconds =
        ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT;
    AdapterRequestedMode requested_mode = AdapterRequestedMode::kAuto;
};

struct AdapterModeAvailability {
    bool switch_mode = true;
    bool xinput_mode = true;
    bool dinput_mode = false;
    bool mac_mode = false;
};

AdapterConfiguration adapter_configuration_default();
bool adapter_requested_mode_valid(AdapterRequestedMode requested_mode);
bool adapter_requested_mode_available(
    AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability);
bool adapter_configuration_encode(const AdapterConfiguration& configuration,
                                  uint8_t* output, size_t output_size);
bool adapter_configuration_decode(uint16_t schema_version,
                                  const uint8_t* payload,
                                  size_t payload_size,
                                  AdapterConfiguration* output);
bool adapter_configuration_decode(const uint8_t* payload, size_t payload_size,
                                  AdapterConfiguration* output);
