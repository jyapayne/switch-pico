#include "configuration/adapter_configuration.h"
#include <string.h>

namespace {

bool pairing_window_valid(uint16_t pairing_window_seconds) {
    return pairing_window_seconds >= ADAPTER_PAIRING_WINDOW_SECONDS_MIN &&
           pairing_window_seconds <= ADAPTER_PAIRING_WINDOW_SECONDS_MAX;
}

bool joycon_mode_valid(JoyConMode mode) {
    return mode == JoyConMode::kPaired || mode == JoyConMode::kIndividual;
}

bool native_switch_identity_eligible(const ControllerIdentity& identity) {
    return identity.stable &&
           identity.transport == ControllerTransport::kClassic &&
           identity.vendor_id == 0x057e &&
           (identity.product_id == 0x2009 ||
            identity.product_id == 0x2006 ||
            identity.product_id == 0x2007);
}

}  // namespace

AdapterConfiguration adapter_configuration_default() {
    return {};
}

bool adapter_configuration_native_switch_approved(
    const AdapterConfiguration& configuration,
    const ControllerIdentity& identity) {
    if (!native_switch_identity_eligible(identity) ||
        configuration.native_switch_controller_count >
            ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY) {
        return false;
    }
    for (size_t index = 0;
         index < configuration.native_switch_controller_count; ++index) {
        if (controller_identity_equal(
                identity, configuration.native_switch_controllers[index])) {
            return true;
        }
    }
    return false;
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
        !adapter_requested_mode_valid(configuration.requested_mode) ||
        !joycon_mode_valid(configuration.joycon_mode) ||
        configuration.native_switch_controller_count >
            ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY) {
        return false;
    }

    memset(output, 0, output_size);
    output[0] = static_cast<uint8_t>(configuration.pairing_window_seconds);
    output[1] =
        static_cast<uint8_t>(configuration.pairing_window_seconds >> 8);
    output[2] = static_cast<uint8_t>(configuration.requested_mode);
    output[3] = configuration.native_switch_controller_count;
    output[4] = static_cast<uint8_t>(configuration.joycon_mode);
    for (size_t index = 0;
         index < configuration.native_switch_controller_count; ++index) {
        const ControllerIdentity& identity =
            configuration.native_switch_controllers[index];
        uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE];
        if (!native_switch_identity_eligible(identity) ||
            !controller_identity_encode(identity, encoded, sizeof(encoded))) {
            return false;
        }

        // Sort wire records in place so list order cannot change the CRC.
        size_t position = index;
        while (position > 0) {
            uint8_t* previous = output + ADAPTER_CONFIGURATION_HEADER_SIZE +
                                (position - 1) * sizeof(encoded);
            const int comparison = memcmp(encoded, previous, sizeof(encoded));
            if (comparison == 0) {
                return false;
            }
            if (comparison > 0) {
                break;
            }
            memcpy(previous + sizeof(encoded), previous, sizeof(encoded));
            --position;
        }
        memcpy(output + ADAPTER_CONFIGURATION_HEADER_SIZE +
                   position * sizeof(encoded),
               encoded, sizeof(encoded));
    }
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
    } else if (schema_version == ADAPTER_CONFIGURATION_V2_SCHEMA_VERSION ||
               schema_version == ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION ||
               schema_version == ADAPTER_CONFIGURATION_SCHEMA_VERSION) {
        const bool has_approvals =
            schema_version >= ADAPTER_CONFIGURATION_V3_SCHEMA_VERSION;
        const bool has_joycon_mode =
            schema_version == ADAPTER_CONFIGURATION_SCHEMA_VERSION;
        const size_t expected_size =
            has_approvals ? ADAPTER_CONFIGURATION_ENCODED_SIZE
                          : ADAPTER_CONFIGURATION_V2_ENCODED_SIZE;
        if (payload_size != expected_size ||
            (!has_joycon_mode && payload[4] != 0) ||
            payload[5] != 0 || payload[6] != 0 || payload[7] != 0 ||
            (!has_approvals && payload[3] != 0) ||
            payload[3] >
                ADAPTER_CONFIGURATION_NATIVE_SWITCH_CONTROLLER_CAPACITY) {
            return false;
        }
        decoded.requested_mode =
            static_cast<AdapterRequestedMode>(payload[2]);
        if (!adapter_requested_mode_valid(decoded.requested_mode)) {
            return false;
        }
        if (has_joycon_mode) {
            decoded.joycon_mode = static_cast<JoyConMode>(payload[4]);
            if (!joycon_mode_valid(decoded.joycon_mode)) {
                return false;
            }
        }
        if (has_approvals) {
            decoded.native_switch_controller_count = payload[3];
            size_t offset = ADAPTER_CONFIGURATION_HEADER_SIZE;
            for (size_t index = 0;
                 index < decoded.native_switch_controller_count; ++index) {
                ControllerIdentity& identity =
                    decoded.native_switch_controllers[index];
                if (!controller_identity_decode(
                        payload + offset, CONTROLLER_IDENTITY_ENCODED_SIZE,
                        &identity) ||
                    !native_switch_identity_eligible(identity) ||
                    (index > 0 &&
                     memcmp(payload + offset - CONTROLLER_IDENTITY_ENCODED_SIZE,
                            payload + offset,
                            CONTROLLER_IDENTITY_ENCODED_SIZE) >= 0)) {
                    return false;
                }
                offset += CONTROLLER_IDENTITY_ENCODED_SIZE;
            }
            for (; offset < payload_size; ++offset) {
                if (payload[offset] != 0) {
                    return false;
                }
            }
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
