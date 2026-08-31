#pragma once

#include <cstdint>

enum class BootselPairingButtonSample : uint8_t {
    kUnread,
    kReleased,
    kPressed,
};

class BootselPairingButtonHoldFsm {
public:
    static constexpr uint8_t kHoldSamples = 20;

    bool update(BootselPairingButtonSample sample);

private:
    uint8_t pressed_samples_ = 0;
    bool hold_reported_ = false;
};

// Polls BOOTSEL at 10 Hz. Returns true once when a 20-sample hold completes.
bool bootsel_pairing_button_task();
