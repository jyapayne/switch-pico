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
    // Returns an 80ms button pulse. Fresh observations must settle before
    // arming/rearming; repeated reads cannot extend a pulse or build evidence.
    bool update(const WiiAccelerometerSample& sample, uint32_t now_ms,
                uint8_t sensitivity, bool allowed);

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
};
