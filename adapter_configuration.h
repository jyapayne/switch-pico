#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr uint16_t ADAPTER_CONFIGURATION_SCHEMA_VERSION = 1;
constexpr size_t ADAPTER_CONFIGURATION_ENCODED_SIZE = 4;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MIN = 10;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MAX = 300;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT = 60;

struct AdapterConfiguration {
    uint16_t pairing_window_seconds =
        ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT;
};

AdapterConfiguration adapter_configuration_default();
bool adapter_configuration_encode(const AdapterConfiguration& configuration,
                                  uint8_t* output, size_t output_size);
bool adapter_configuration_decode(const uint8_t* payload, size_t payload_size,
                                  AdapterConfiguration* output);
