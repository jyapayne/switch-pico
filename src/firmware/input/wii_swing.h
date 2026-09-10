#pragma once

#include <stdint.h>

// Calibrated Wii acceleration in Nintendo units (4096 counts/g), captured
// independently of whether physical/IR gyro output is enabled or consumed.
struct WiiAccelerometerSample {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    uint32_t sequence = 0;
    uint32_t timestamp_ms = 0;
    bool valid = false;
};

class WiiSwingDetector {
public:
    void reset();
    // Returns an 80ms button pulse. Startup needs settled observations;
    // subsequent strokes need a brief force release and a 200ms cooldown.
    // Repeated reads cannot extend a pulse or build evidence.
    bool update(const WiiAccelerometerSample& sample, uint32_t now_ms,
                uint8_t sensitivity, bool allowed, bool observe_confirmation = false);
    // Lower-threshold evidence is never an individual action. Each observation
    // can confirm only one combined stroke, before or after the full swing.
    bool confirmation(uint32_t* timestamp_ms) const;
    void consume_confirmation();

private:
    int32_t gravity_[3]{};
    uint32_t sequence_ = 0;
    uint32_t sample_ms_ = 0;
    uint32_t quiet_since_ms_ = 0;
    uint32_t candidate_since_ms_ = 0;
    uint32_t fired_ms_ = 0;
    bool initialized_ = false;
    bool quiet_ = false;
    bool candidate_ = false;
    bool armed_ = false;
    bool pulsing_ = false;
    bool fired_ = false;
    uint32_t confirmation_ms_ = 0;
    uint32_t confirmation_since_ms_ = 0;
    uint32_t confirmation_release_ms_ = 0;
    bool have_confirmation_ = false;
    bool confirmation_candidate_ = false;
    bool confirmation_latched_ = false;
    bool confirmation_releasing_ = false;
};

struct WiiSwingGestureResult {
    uint8_t active = 0;
    uint8_t started = 0;
};

// Bits 0/1/2 identify Remote, Nunchuk, and combined gestures. Combining is
// enabled only while both sensors are fresh and the combined binding is allowed.
class WiiSwingGestures {
public:
    void reset();
    // Discard outputs during a macro without losing stroke/rearm history.
    void discard_actions();
    WiiSwingGestureResult update(const WiiAccelerometerSample& remote,
                                 const WiiAccelerometerSample& nunchuk,
                                 uint32_t now_ms, uint8_t remote_sensitivity,
                                 uint8_t nunchuk_sensitivity, uint8_t allowed,
                                 uint8_t combination_window_ms);

private:
    WiiSwingDetector detectors_[2]{};
    bool previous_[2]{};
    bool combining_ = false;
    uint8_t pending_ = 0;
    uint8_t pending_individual_ = 0;
    uint32_t pending_since_ms_ = 0;
    uint8_t active_ = 0;
    uint32_t pulse_since_ms_[3]{};
    uint8_t used_confirmations_ = 0;
    uint32_t used_confirmation_ms_[2]{};
};
