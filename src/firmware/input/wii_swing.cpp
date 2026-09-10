#include "input/wii_swing.h"

namespace {
constexpr uint32_t kStaleMs = 150;
constexpr uint32_t kMaximumSampleGapMs = 50;
constexpr uint32_t kSettleMs = 120;
constexpr uint32_t kPulseMs = 80;
constexpr uint32_t kRearmMs = 250;
constexpr uint32_t kEvidenceMs = 10;
constexpr int32_t kGravity = 4096;
constexpr int64_t square(int32_t value) {
    return static_cast<int64_t>(value) * value;
}
}

void WiiSwingDetector::reset() {
    *this = {};
}

bool WiiSwingDetector::update(const WiiAccelerometerSample& sample,
                              uint32_t now_ms, uint8_t sensitivity, bool allowed) {
    if (!allowed || !sample.valid || sensitivity > 2 ||
        static_cast<int32_t>(now_ms - sample.timestamp_ms) > static_cast<int32_t>(kStaleMs)) {
        reset();
        return false;
    }
    if (pulsing_ && now_ms - fired_ms_ >= kPulseMs) pulsing_ = false;
    if (initialized_ && sample.sequence == sequence_) return pulsing_;
    const uint32_t elapsed = sample.timestamp_ms - sample_ms_;
    if (initialized_ && (elapsed == 0 || elapsed > kMaximumSampleGapMs)) {
        reset();
    }
    sequence_ = sample.sequence;
    sample_ms_ = sample.timestamp_ms;
    const int32_t axes[3] = {sample.x, sample.y, sample.z};
    if (!initialized_) {
        for (unsigned axis = 0; axis < 3; ++axis) gravity_[axis] = axes[axis];
        initialized_ = true;
        return false;
    }

    int64_t magnitude_squared = 0;
    int64_t dynamic_squared = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        magnitude_squared += square(axes[axis]);
        dynamic_squared += square(axes[axis] - gravity_[axis]);
    }
    constexpr int32_t thresholds[3] = {2 * kGravity, 3 * kGravity / 2, kGravity};
    // Gravity alone changes direction during aiming. Require a force-magnitude
    // excursion as well as gravity-subtracted acceleration to reject mere tilt.
    const bool force_excursion = magnitude_squared > square(kGravity * 135 / 100) ||
                                 magnitude_squared < square(kGravity * 65 / 100);
    const bool energetic = dynamic_squared > square(thresholds[sensitivity]) && force_excursion;
    const bool settled = dynamic_squared < square(kGravity / 4) &&
                         magnitude_squared > square(kGravity * 3 / 4) &&
                         magnitude_squared < square(kGravity * 5 / 4);

    if (settled) {
        if (!quiet_) {
            quiet_ = true;
            quiet_since_ms_ = sample_ms_;
        }
        if (sample_ms_ - quiet_since_ms_ >= kSettleMs &&
            (!fired_ || sample_ms_ - fired_ms_ >= kRearmMs)) {
            armed_ = true;
        }
    } else {
        quiet_ = false;
    }
    if (armed_ && energetic) {
        if (!candidate_) {
            candidate_ = true;
            candidate_since_ms_ = sample_ms_;
        } else if (sample_ms_ - candidate_since_ms_ >= kEvidenceMs) {
            pulsing_ = true;
            fired_ = true;
            fired_ms_ = now_ms;
            armed_ = false;
            candidate_ = false;
            quiet_ = false;
        }
    } else {
        candidate_ = false;
    }
    // Track slow orientation changes, but do not absorb a swing into gravity.
    if (!energetic) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            gravity_[axis] += static_cast<int32_t>(
                static_cast<int64_t>(axes[axis] - gravity_[axis]) * elapsed / (100 + elapsed));
        }
    }
    return pulsing_;
}
