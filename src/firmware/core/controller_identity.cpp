#include "core/controller_identity.h"
#include <string.h>

namespace {

constexpr uint16_t kNintendoVendorId = 0x057e;
constexpr uint16_t kJoyConLeftProductId = 0x2067;
constexpr uint16_t kJoyConRightProductId = 0x2066;

bool member_address_valid(uint8_t type, const uint8_t* address) {
    return type == 0 || (type == 1 && (address[0] & 0xc0) == 0xc0);
}

bool partner_empty(const ControllerIdentity& identity) {
    constexpr uint8_t empty[6]{};
    return identity.partner_address_type == 0 &&
           memcmp(identity.partner_address, empty, sizeof(empty)) == 0;
}

bool controller_identity_valid(const ControllerIdentity& identity) {
    const uint8_t transport = static_cast<uint8_t>(identity.transport);
    if (identity.transport == ControllerTransport::kJoyConPair) {
        return controller_identity_is_joycon_pair(identity);
    }
    if (transport > static_cast<uint8_t>(ControllerTransport::kBle) ||
        !partner_empty(identity)) {
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
           first.product_id == second.product_id &&
           first.partner_address_type == second.partner_address_type &&
           memcmp(first.partner_address, second.partner_address,
                  sizeof(first.partner_address)) == 0;
}

bool controller_identity_is_joycon_pair(const ControllerIdentity& identity) {
    return identity.stable &&
           identity.transport == ControllerTransport::kJoyConPair &&
           identity.vendor_id == kNintendoVendorId &&
           identity.product_id == kJoyConLeftProductId &&
           member_address_valid(identity.address_type, identity.address) &&
           member_address_valid(identity.partner_address_type,
                                identity.partner_address) &&
           (identity.address_type != identity.partner_address_type ||
            memcmp(identity.address, identity.partner_address,
                   sizeof(identity.address)) != 0);
}

bool controller_identity_make_joycon_pair(const ControllerIdentity& left,
                                        const ControllerIdentity& right,
                                        ControllerIdentity* output) {
    if (output == nullptr || !left.stable || !right.stable ||
        left.transport != ControllerTransport::kBle ||
        right.transport != ControllerTransport::kBle ||
        left.vendor_id != kNintendoVendorId ||
        right.vendor_id != kNintendoVendorId ||
        left.product_id != kJoyConLeftProductId ||
        right.product_id != kJoyConRightProductId ||
        !partner_empty(left) || !partner_empty(right)) {
        return false;
    }
    ControllerIdentity pair = left;
    pair.transport = ControllerTransport::kJoyConPair;
    pair.partner_address_type = right.address_type;
    memcpy(pair.partner_address, right.address, sizeof(pair.partner_address));
    if (!controller_identity_is_joycon_pair(pair)) {
        return false;
    }
    *output = pair;
    return true;
}

bool controller_identity_joycon_pair_members(const ControllerIdentity& pair,
                                           ControllerIdentity* left,
                                           ControllerIdentity* right) {
    if (left == nullptr || right == nullptr || left == right ||
        !controller_identity_is_joycon_pair(pair)) {
        return false;
    }
    ControllerIdentity decoded_left{};
    decoded_left.stable = true;
    decoded_left.transport = ControllerTransport::kBle;
    decoded_left.address_type = pair.address_type;
    memcpy(decoded_left.address, pair.address, sizeof(decoded_left.address));
    decoded_left.vendor_id = kNintendoVendorId;
    decoded_left.product_id = kJoyConLeftProductId;
    ControllerIdentity decoded_right = decoded_left;
    decoded_right.address_type = pair.partner_address_type;
    memcpy(decoded_right.address, pair.partner_address,
           sizeof(decoded_right.address));
    decoded_right.product_id = kJoyConRightProductId;
    *left = decoded_left;
    *right = decoded_right;
    return true;
}

bool controller_identity_encode(const ControllerIdentity& identity,
                                uint8_t* output, size_t output_size) {
    if (output == nullptr || output_size < CONTROLLER_IDENTITY_ENCODED_SIZE ||
        !controller_identity_valid(identity)) {
        return false;
    }
    if (identity.transport == ControllerTransport::kJoyConPair) {
        output[0] = 1 | (identity.address_type << 1) |
                    (identity.partner_address_type << 2);
        output[1] = static_cast<uint8_t>(identity.transport);
        memcpy(&output[2], identity.address, sizeof(identity.address));
        memcpy(&output[8], identity.partner_address,
               sizeof(identity.partner_address));
        return true;
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
        input_size != CONTROLLER_IDENTITY_ENCODED_SIZE) {
        return false;
    }

    ControllerIdentity decoded{};
    decoded.transport = static_cast<ControllerTransport>(input[1]);
    if (decoded.transport == ControllerTransport::kJoyConPair) {
        if ((input[0] & 0xf9) != 1) {
            return false;
        }
        decoded.stable = true;
        decoded.address_type = (input[0] >> 1) & 1;
        decoded.partner_address_type = (input[0] >> 2) & 1;
        memcpy(decoded.address, &input[2], sizeof(decoded.address));
        memcpy(decoded.partner_address, &input[8],
               sizeof(decoded.partner_address));
        decoded.vendor_id = kNintendoVendorId;
        decoded.product_id = kJoyConLeftProductId;
    } else {
        if (input[0] > 1 || input[3] != 0) {
            return false;
        }
        decoded.stable = input[0] != 0;
        decoded.address_type = input[2];
        memcpy(decoded.address, &input[4], sizeof(decoded.address));
        decoded.vendor_id = static_cast<uint16_t>(input[10]) |
                            static_cast<uint16_t>(input[11] << 8);
        decoded.product_id = static_cast<uint16_t>(input[12]) |
                             static_cast<uint16_t>(input[13] << 8);
    }
    if (!controller_identity_valid(decoded)) {
        return false;
    }

    *output = decoded;
    return true;
}
