#include "adapter_configuration.h"

namespace {

bool pairing_window_valid(uint16_t pairing_window_seconds) {
    return pairing_window_seconds >= ADAPTER_PAIRING_WINDOW_SECONDS_MIN &&
           pairing_window_seconds <= ADAPTER_PAIRING_WINDOW_SECONDS_MAX;
}

}  // namespace

AdapterConfiguration adapter_configuration_default() {
    return {};
}

bool adapter_requested_mode_valid(AdapterRequestedMode requested_mode) {
    switch (requested_mode) {
        case AdapterRequestedMode::kAuto:
        case AdapterRequestedMode::kSwitch:
        case AdapterRequestedMode::kXInput:
        case AdapterRequestedMode::kDInput:
        case AdapterRequestedMode::kMac:
            return true;
    }
    return false;
}

bool adapter_requested_mode_available(
    AdapterRequestedMode requested_mode,
    const AdapterModeAvailability& availability) {
    switch (requested_mode) {
        case AdapterRequestedMode::kAuto:
            return true;
        case AdapterRequestedMode::kSwitch:
            return availability.switch_mode;
        case AdapterRequestedMode::kXInput:
            return availability.xinput_mode;
        case AdapterRequestedMode::kDInput:
            return availability.dinput_mode;
        case AdapterRequestedMode::kMac:
            return availability.mac_mode;
    }
    return false;
}

bool adapter_configuration_encode(const AdapterConfiguration& configuration,
                                  uint8_t* output, size_t output_size) {
    if (output == nullptr ||
        output_size != ADAPTER_CONFIGURATION_ENCODED_SIZE ||
        !pairing_window_valid(configuration.pairing_window_seconds) ||
        !adapter_requested_mode_valid(configuration.requested_mode)) {
        return false;
    }

    output[0] = static_cast<uint8_t>(configuration.pairing_window_seconds);
    output[1] =
        static_cast<uint8_t>(configuration.pairing_window_seconds >> 8);
    output[2] = static_cast<uint8_t>(configuration.requested_mode);
    output[3] = 0;
    output[4] = 0;
    output[5] = 0;
    output[6] = 0;
    output[7] = 0;
    return true;
}

bool adapter_configuration_decode(uint16_t schema_version,
                                  const uint8_t* payload,
                                  size_t payload_size,
                                  AdapterConfiguration* output) {
    if (payload == nullptr || output == nullptr) {
        return false;
    }

    AdapterConfiguration decoded{};
    if (schema_version == ADAPTER_CONFIGURATION_LEGACY_SCHEMA_VERSION) {
        if (payload_size != ADAPTER_CONFIGURATION_LEGACY_ENCODED_SIZE ||
            payload[2] != 0 || payload[3] != 0) {
            return false;
        }
        decoded.requested_mode = AdapterRequestedMode::kAuto;
    } else if (schema_version == ADAPTER_CONFIGURATION_SCHEMA_VERSION) {
        if (payload_size != ADAPTER_CONFIGURATION_ENCODED_SIZE ||
            payload[3] != 0 || payload[4] != 0 || payload[5] != 0 ||
            payload[6] != 0 || payload[7] != 0) {
            return false;
        }
        decoded.requested_mode =
            static_cast<AdapterRequestedMode>(payload[2]);
        if (!adapter_requested_mode_valid(decoded.requested_mode)) {
            return false;
        }
    } else {
        return false;
    }

    decoded.pairing_window_seconds =
        static_cast<uint16_t>(payload[0]) |
        static_cast<uint16_t>(payload[1] << 8);
    if (!pairing_window_valid(decoded.pairing_window_seconds)) {
        return false;
    }

    *output = decoded;
    return true;
}

bool adapter_configuration_decode(const uint8_t* payload, size_t payload_size,
                                  AdapterConfiguration* output) {
    return adapter_configuration_decode(ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                                        payload, payload_size, output);
}
