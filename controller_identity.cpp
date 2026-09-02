#include "controller_identity.h"

#include <string.h>

namespace {

bool controller_identity_valid(const ControllerIdentity& identity) {
    const uint8_t transport = static_cast<uint8_t>(identity.transport);
    if (transport > static_cast<uint8_t>(ControllerTransport::kBle)) {
        return false;
    }
    if (identity.stable) {
        return identity.transport != ControllerTransport::kUnknown;
    }
    return controller_identity_is_global(identity);
}

}  // namespace

ControllerIdentity controller_identity_global() {
    return {};
}

bool controller_identity_is_global(const ControllerIdentity& identity) {
    const ControllerIdentity global{};
    return controller_identity_equal(identity, global);
}

bool controller_identity_equal(const ControllerIdentity& first,
                               const ControllerIdentity& second) {
    return first.stable == second.stable &&
           first.transport == second.transport &&
           first.address_type == second.address_type &&
           memcmp(first.address, second.address, sizeof(first.address)) == 0 &&
           first.vendor_id == second.vendor_id &&
           first.product_id == second.product_id;
}

bool controller_identity_encode(const ControllerIdentity& identity,
                                uint8_t* output, size_t output_size) {
    if (output == nullptr || output_size < CONTROLLER_IDENTITY_ENCODED_SIZE ||
        !controller_identity_valid(identity)) {
        return false;
    }

    output[0] = identity.stable ? 1 : 0;
    output[1] = static_cast<uint8_t>(identity.transport);
    output[2] = identity.address_type;
    output[3] = 0;
    memcpy(&output[4], identity.address, sizeof(identity.address));
    output[10] = static_cast<uint8_t>(identity.vendor_id);
    output[11] = static_cast<uint8_t>(identity.vendor_id >> 8);
    output[12] = static_cast<uint8_t>(identity.product_id);
    output[13] = static_cast<uint8_t>(identity.product_id >> 8);
    return true;
}

bool controller_identity_decode(const uint8_t* input, size_t input_size,
                                ControllerIdentity* output) {
    if (input == nullptr || output == nullptr ||
        input_size != CONTROLLER_IDENTITY_ENCODED_SIZE || input[0] > 1 ||
        input[1] > static_cast<uint8_t>(ControllerTransport::kBle) ||
        input[3] != 0) {
        return false;
    }

    ControllerIdentity decoded{};
    decoded.stable = input[0] != 0;
    decoded.transport = static_cast<ControllerTransport>(input[1]);
    decoded.address_type = input[2];
    memcpy(decoded.address, &input[4], sizeof(decoded.address));
    decoded.vendor_id = static_cast<uint16_t>(input[10]) |
                        static_cast<uint16_t>(input[11] << 8);
    decoded.product_id = static_cast<uint16_t>(input[12]) |
                         static_cast<uint16_t>(input[13] << 8);
    if (!controller_identity_valid(decoded)) {
        return false;
    }

    *output = decoded;
    return true;
}
