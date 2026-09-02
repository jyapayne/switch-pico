#include "adapter_configuration.h"

AdapterConfiguration adapter_configuration_default() {
    return {};
}

bool adapter_configuration_encode(const AdapterConfiguration& configuration,
                                  uint8_t* output, size_t output_size) {
    if (output == nullptr || output_size < ADAPTER_CONFIGURATION_ENCODED_SIZE ||
        configuration.pairing_window_seconds <
            ADAPTER_PAIRING_WINDOW_SECONDS_MIN ||
        configuration.pairing_window_seconds >
            ADAPTER_PAIRING_WINDOW_SECONDS_MAX) {
        return false;
    }

    output[0] = static_cast<uint8_t>(configuration.pairing_window_seconds);
    output[1] =
        static_cast<uint8_t>(configuration.pairing_window_seconds >> 8);
    output[2] = 0;
    output[3] = 0;
    return true;
}

bool adapter_configuration_decode(const uint8_t* payload, size_t payload_size,
                                  AdapterConfiguration* output) {
    if (payload == nullptr || output == nullptr ||
        payload_size != ADAPTER_CONFIGURATION_ENCODED_SIZE ||
        payload[2] != 0 || payload[3] != 0) {
        return false;
    }

    const uint16_t pairing_window_seconds =
        static_cast<uint16_t>(payload[0]) |
        static_cast<uint16_t>(payload[1] << 8);
    if (pairing_window_seconds < ADAPTER_PAIRING_WINDOW_SECONDS_MIN ||
        pairing_window_seconds > ADAPTER_PAIRING_WINDOW_SECONDS_MAX) {
        return false;
    }

    output->pairing_window_seconds = pairing_window_seconds;
    return true;
}
