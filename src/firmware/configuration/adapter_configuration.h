#pragma once

#include <stddef.h>
#include <stdint.h>

#include "adapter/adapter_usb_mode.h"
#include "core/controller_identity.h"

constexpr uint16_t ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION = 1;
constexpr uint16_t ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION = 2;
constexpr uint16_t ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION = 3;
constexpr uint16_t ADAPTER_CONFIGURATION_SCHEMA_VERSION = 4;
constexpr size_t ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE = 4;
constexpr size_t ADAPTER_CONFIGURATION_V2_ENCODED_SIZE = 8;
constexpr size_t ADAPTER_CONFIGURATION_HEADER_SIZE = 8;
constexpr size_t ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY = 16;
constexpr size_t ADAPTER_CONFIGURATION_ENCODED_SIZE =
    ADAPTER_CONFIGURATION_HEADER_SIZE +
    ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY *
        CONTROLLER_IDENTITY_ENCODED_SIZE;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MIN = 10;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_MAX = 300;
constexpr uint16_t ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT = 60;

enum class JoyConMode : uint8_t {
    kPaired = 0,
    kIndividual = 1,
};

struct AdapterConfiguration {
    uint16_t pairing_window_seconds =
        ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT;
    AdapterRequestedMode requested_mode = AdapterRequestedMode::kAuto;
    uint8_t native_switch_controller_count = 0;
    ControllerIdentity native_switch_controllers[
        ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY]{};
    JoyConMode joycon_mode = JoyConMode::kPaired;
};

struct AdapterModeAvailability {
    bool switch_mode = true;
    bool xinput_mode = true;
    bool dinput_mode = false;
    bool mac_mode = false;
};

AdapterConfiguration adapter_configuration_default();
bool adapter_configuration_native_switch_approved(
    const AdapterConfiguration& configuration,
    const ControllerIdentity& identity);
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
