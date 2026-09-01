#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bluepad32_input_backend.h"

namespace UsbPairingManagement {

constexpr uint8_t kRequestClear = 0x50;
constexpr uint8_t kRequestGet = 0x51;
constexpr uint8_t kRequestRefresh = 0x52;
constexpr uint16_t kRequestValue = 0x5350;
constexpr uint16_t kRequestIndex = 0x4d47;
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kResponseHeaderSize = 12;
constexpr size_t kRecordSize = 8;
constexpr size_t kMaximumResponseSize =
    kResponseHeaderSize +
    BLUEPAD32_PAIRING_RECORD_CAPACITY * kRecordSize;

size_t encode_snapshot(const Bluepad32PairingSnapshot& snapshot,
                       uint8_t* output, size_t output_size);

}  // namespace UsbPairingManagement
