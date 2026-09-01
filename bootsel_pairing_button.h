#pragma once

#include <cstdint>

enum class BootselPairingButtonSample : uint8_t {
    kUnread,
    kReleased,
    kPressed,
};
enum class BootselPairingButtonEvent : uint8_t {
    kNone,
    kOpenPairing,
    kClearPairings,
};


class BootselPairingButtonHoldFsm {
public:
    static constexpr uint8_t kPairingHoldSamples = 20;
    static constexpr uint8_t kClearHoldSamples = 100;

    BootselPairingButtonEvent update(BootselPairingButtonSample sample);

private:
    uint8_t pressed_samples_ = 0;
    bool pairing_reported_ = false;
    bool clear_reported_ = false;
};

// Polls BOOTSEL at 10 Hz. Reports pairing at 2 seconds and clearing at
// 10 seconds; each event fires once per continuous hold.
BootselPairingButtonEvent bootsel_pairing_button_task();
