#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t CONTROLLER_IDENTITY_ENCODED_SIZE = 14;

enum class ControllerTransport : uint8_t {
    kUnknown = 0,
    kClassic = 1,
    kBle = 2,
};

struct ControllerIdentity {
    bool stable = false;
    ControllerTransport transport = ControllerTransport::kUnknown;
    uint8_t address_type = 0;
    uint8_t address[6]{};
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
};

ControllerIdentity controller_identity_global();
bool controller_identity_is_global(const ControllerIdentity& identity);
bool controller_identity_equal(const ControllerIdentity& first,
                               const ControllerIdentity& second);
bool controller_identity_encode(const ControllerIdentity& identity,
                                uint8_t* output, size_t output_size);
bool controller_identity_decode(const uint8_t* input, size_t input_size,
                                ControllerIdentity* output);
