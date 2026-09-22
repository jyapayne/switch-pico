#pragma once

#include <stddef.h>
#include <stdint.h>

// EP0 vendor-request wire protocol shared by every firmware variant and
// switch_pico_bridge.config_manager. Keep this header free of adapter,
// Bluetooth or storage dependencies so the UART firmware can speak the
// subset it supports.
namespace UsbConfigurationManagement {

constexpr uint16_t kRequestValue = 0x5350;
constexpr uint16_t kRequestIndex = 0x0001;
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kRequestHeaderSize = 16;
constexpr size_t kResponseHeaderSize = 20;
constexpr size_t kInfoPayloadSize = 8;
// Board byte of the INFO payload.
constexpr uint8_t kBoardPico = 1;
constexpr uint8_t kBoardPico2W = 2;

enum class Operation : uint8_t {
    kInfo = 0x01,
    kModeSet = 0x02,
    kReboot = 0x03,
    kBootselReboot = 0x04,
    kSwitch2Wake = 0x05,
    kConfigurationRead = 0x10,
    kConfigurationBegin = 0x11,
    kConfigurationChunk = 0x12,
    kConfigurationCommit = 0x13,
    kConfigurationReset = 0x14,
    kTransactionStatus = 0x15,
    kPairingRead = 0x20,
    kPairingRefresh = 0x21,
    kPairingClear = 0x22,
    kRuntimeDiagnostics = 0x23,
    kProfileList = 0x30,
    kProfileSelect = 0x31,
    kProfileRead = 0x32,
    kProfileBegin = 0x33,
    kProfileChunk = 0x34,
    kProfileCommit = 0x35,
    kProfileReset = 0x36,
    kProfileActivate = 0x37,
    kProfileTransactionStatus = 0x38,
    kProfilePlaytest = 0x39,
    kProfileMetadataRead = 0x3a,
    kProfileMetadataSet = 0x3b,
    kProfileIdentify = 0x3c,
    kWiiOrientation = 0x3d,
    kHapticsExperiment = 0x40,
    kHapticsTransportProbe = 0x41,
    kMacroCapture = 0x42,
    kNativeSwitchRumble = 0x43,
    kSwitch2MouseCapture = 0x44,
    kWiiIrGyro = 0x45,
};

enum class Status : uint8_t {
    kOk = 0,
    kPending = 1,
    kMalformed = 2,
    kUnsupportedSchema = 3,
    kTooLarge = 4,
    kOutOfOrder = 5,
    kBadCrc = 6,
    kBusy = 7,
    kStorageError = 8,
};

}  // namespace UsbConfigurationManagement
