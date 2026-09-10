#include "input/wii_swing.h"

namespace {
constexpr uint32_t kStaleMs = 150;
constexpr uint32_t kMaximumSampleGapMs = 50;
constexpr uint32_t kSettleMs = 120;
constexpr uint32_t kPulseMs = 80;
constexpr uint32_t kRearmMs = 200;
constexpr uint32_t kReleaseMs = 20;
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
                              uint32_t now_ms, uint8_t sensitivity, bool allowed,
                              bool observe_confirmation) {
    if (!allowed || !sample.valid || sensitivity > 2 ||
        static_cast<int32_t>(now_ms - sample.timestamp_ms) > static_cast<int32_t>(kStaleMs)) {
        reset();
        return false;
    }
    if (!observe_confirmation) {
        have_confirmation_ = false;
        confirmation_candidate_ = false;
        confirmation_latched_ = false;
        confirmation_releasing_ = false;
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

    // Startup still needs a settled reference. Between strokes, a short
    // lower-force interval is enough; continuous swinging need not stop.
    const bool released = dynamic_squared < square(thresholds[sensitivity] / 2) ||
                          !force_excursion;
    if (fired_ ? released : settled) {
        if (!quiet_) {
            quiet_ = true;
            quiet_since_ms_ = sample_ms_;
        }
        if (sample_ms_ - quiet_since_ms_ >= (fired_ ? kReleaseMs : kSettleMs)) {
            armed_ = true;
        }
    } else {
        quiet_ = false;
    }
    if (observe_confirmation) {
        // Reuse the same calibrated gravity reference and force/tilt guard.
        // Do not lower the individual trigger or change its filtering/rearm.
        const bool confirming = dynamic_squared > square(thresholds[sensitivity] / 2) &&
                                force_excursion;
        if (confirmation_latched_ && released) {
            if (!confirmation_releasing_) {
                confirmation_releasing_ = true;
                confirmation_release_ms_ = sample_ms_;
            }
            if (sample_ms_ - confirmation_release_ms_ >= kReleaseMs)
                confirmation_latched_ = false;
        } else {
            confirmation_releasing_ = false;
        }
        if (!confirmation_latched_ && confirming && (armed_ || fired_)) {
            if (!confirmation_candidate_) {
                confirmation_candidate_ = true;
                confirmation_since_ms_ = sample_ms_;
            } else if (sample_ms_ - confirmation_since_ms_ >= kEvidenceMs) {
                // Reject early rebounds rather than confirming them later.
                if (!fired_ || now_ms - fired_ms_ >= kRearmMs) {
                    confirmation_ms_ = now_ms;
                    have_confirmation_ = true;
                }
                confirmation_latched_ = true;
                confirmation_candidate_ = false;
                confirmation_releasing_ = false;
            }
        } else {
            confirmation_candidate_ = false;
        }
    }
    if (armed_ && energetic) {
        if (!candidate_) {
            candidate_ = true;
            candidate_since_ms_ = sample_ms_;
        } else if (sample_ms_ - candidate_since_ms_ >= kEvidenceMs) {
            // Consume even an early rebound. It must fall below the release
            // threshold again, rather than firing late when cooldown expires.
            if (!fired_ || now_ms - fired_ms_ >= kRearmMs) {
                pulsing_ = true;
                fired_ = true;
                fired_ms_ = now_ms;
            }
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

bool WiiSwingDetector::confirmation(uint32_t* timestamp_ms) const {
    if (!have_confirmation_ || timestamp_ms == nullptr) return false;
    *timestamp_ms = confirmation_ms_;
    return true;
}

void WiiSwingDetector::consume_confirmation() {
    if (!have_confirmation_) return;
    // Count the confirming motion as this source's stroke. Otherwise a weak
    // confirmation growing into a full swing would later leak a single action.
    if (!fired_ || static_cast<int32_t>(confirmation_ms_ - fired_ms_) > 0)
        fired_ms_ = confirmation_ms_;
    fired_ = true;
    armed_ = false;
    candidate_ = false;
    quiet_ = false;
}

void WiiSwingGestures::reset() {
    *this = {};
}

void WiiSwingGestures::discard_actions() {
    pending_ = 0;
    pending_individual_ = 0;
    active_ = 0;
    for (unsigned i = 0; i < 2; ++i) {
        if (detectors_[i].confirmation(&used_confirmation_ms_[i]))
            used_confirmations_ |= 1u << i;
    }
}

WiiSwingGestureResult WiiSwingGestures::update(
    const WiiAccelerometerSample& remote, const WiiAccelerometerSample& nunchuk,
    uint32_t now_ms, uint8_t remote_sensitivity, uint8_t nunchuk_sensitivity,
    uint8_t allowed, uint8_t combination_window_ms) {
    const auto fresh = [now_ms](const WiiAccelerometerSample& sample) {
        return sample.valid &&
            static_cast<int32_t>(now_ms - sample.timestamp_ms) <= static_cast<int32_t>(kStaleMs);
    };
    const bool remote_fresh = fresh(remote);
    const bool nunchuk_fresh = fresh(nunchuk);
    if (!remote_fresh) allowed &= ~5u;
    if (!nunchuk_fresh) allowed &= ~6u;
    if (allowed == 0) {
        reset();
        return {};
    }
    const bool combining = (allowed & 4u) != 0;
    if (combining != combining_) {
        pending_ = 0;
        pending_individual_ = 0;
        combining_ = combining;
        used_confirmations_ = 0;
    }
    active_ &= allowed;
    pending_individual_ &= allowed;
    for (unsigned i = 0; i < 3; ++i) {
        if (now_ms - pulse_since_ms_[i] >= kPulseMs) active_ &= ~(1u << i);
    }
    const bool pulses[2] = {
        detectors_[0].update(remote, now_ms, remote_sensitivity, (allowed & 5u) != 0, combining),
        detectors_[1].update(nunchuk, now_ms, nunchuk_sensitivity, (allowed & 6u) != 0, combining)};
    uint8_t edges = 0;
    for (unsigned i = 0; i < 2; ++i) {
        if (pulses[i] && !previous_[i]) edges |= 1u << i;
        previous_[i] = pulses[i];
    }
    uint32_t confirmation_ms[2]{};
    uint8_t confirmations = 0;
    for (unsigned i = 0; combining && i < 2; ++i) {
        if (detectors_[i].confirmation(&confirmation_ms[i]) &&
            now_ms - confirmation_ms[i] <= combination_window_ms &&
            (!(used_confirmations_ & (1u << i)) ||
             used_confirmation_ms_[i] != confirmation_ms[i])) {
            confirmations |= 1u << i;
        }
    }
    WiiSwingGestureResult result;
    const auto emit = [&](uint8_t bits) {
        bits &= allowed;
        result.started |= bits;
        active_ |= bits;
        for (unsigned i = 0; i < 3; ++i)
            if (bits & (1u << i)) pulse_since_ms_[i] = now_ms;
    };
    const auto combine = [&] {
        emit(4);
        for (unsigned i = 0; i < 2; ++i) {
            if (detectors_[i].confirmation(&used_confirmation_ms_[i])) {
                used_confirmations_ |= 1u << i;
                detectors_[i].consume_confirmation();
            }
        }
        confirmations = 0;
    };
    if (pending_ && now_ms - pending_since_ms_ > combination_window_ms) {
        emit(pending_individual_);
        pending_ = pending_individual_ = 0;
    }
    if (combining) {
        // Match the older pending stroke before a new stroke on the same source.
        // This also makes the inclusive window boundary deterministic.
        const uint8_t opposite = pending_ == 1 ? 2 : pending_ == 2 ? 1 : 0;
        if (opposite && ((edges | confirmations) & opposite)) {
            combine();
            edges &= ~opposite;
            pending_ = pending_individual_ = 0;
        }
        if (edges == 3 || ((edges & 1) && (confirmations & 2)) ||
            ((edges & 2) && (confirmations & 1))) {
            combine();
            edges = 0;
        }
        if (edges) {
            if (pending_) emit(pending_individual_);
            pending_ = edges;
            pending_individual_ = edges & allowed;
            pending_since_ms_ = now_ms;
        }
    } else {
        emit(edges);
    }
    result.active = active_;
    return result;
}
