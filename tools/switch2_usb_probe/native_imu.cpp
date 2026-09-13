#include "native_imu.h"

#include <cmath>
#include <limits.h>

namespace {
constexpr uint32_t kFreshUs = 150000;
constexpr uint32_t kMaximumUpdateGapUs = 50000;
constexpr uint32_t kCalibrationUs = 1500000;
constexpr uint32_t kCalibrationSamples = 64;
constexpr uint32_t kVariationSamples = 16;
// The stationary Wii trace has 0.383 dps / 0.010 g vector RMS variation;
// quantized sample-to-sample steps reach 0.806 dps / 0.030 g. Judge variance
// over distinct samples, with roughly twice that measured noise allowance.
constexpr float kGyroRmsDps = 0.75f;
constexpr float kAccelRmsG = 0.025f;
// One degree between averaged gravity directions, independent of g scale.
constexpr float kGravityDirectionCosSquared = 0.9996954135f;
constexpr float kGravityTimeConstantUs = 1000000.0f;
constexpr float kRadiansPerDegree = 0.017453292519943295f;
constexpr float kTwoPi = 6.2831853071795864769f;
// A two-second complementary heading correction, driven by distinct optical
// sample times rather than polling frequency. Reuse the 150ms sensor lifetime.
constexpr float kOpticalTimeConstantUs = 2000000.0f;
// Reject camera-forward directions within about 14.5 degrees of world vertical.
constexpr float kMinimumOpticalHorizontalSquared = 0.25f * 0.25f;

bool finite_vector(const float* values, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) return false;
    }
    return true;
}

float squared_norm(const float values[3]) {
    return values[0] * values[0] + values[1] * values[1] + values[2] * values[2];
}

bool accumulate_stationary(const float sample[3], uint32_t count, float mean[3],
                          float& variation, float rms_limit) {
    if (count == 1) {
        for (unsigned i = 0; i < 3; ++i) mean[i] = sample[i];
        variation = 0.0f;
        return true;
    }
    // Welford's sum of squared vector deviations avoids subtracting the large
    // uncalibrated bias from a sum of squares. No samples or heap state retained.
    const float weight = 1.0f / static_cast<float>(count);
    float deviation_squared = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
        const float delta = sample[i] - mean[i];
        deviation_squared += delta * delta;
        mean[i] += delta * weight;
        variation += delta * (sample[i] - mean[i]);
    }
    const float variance_limit = rms_limit * rms_limit;
    // A four-RMS excursion discards impulses immediately instead of letting a
    // long quiet prefix dilute them. Ordinary quantization is judged by RMS.
    return std::isfinite(deviation_squared) && std::isfinite(variation) &&
           deviation_squared < 16.0f * variance_limit &&
           (count < kVariationSamples || variation < static_cast<float>(count - 1) * variance_limit);
}

// Match the reference codec's nearest-even rounding without double arithmetic
// or a dependency on the process floating-point rounding mode. Callers ensure
// the input and its rounded result fit int32_t.
int32_t round_even(float value) {
    int32_t integral = static_cast<int32_t>(value);
    const float remainder = value - static_cast<float>(integral);
    if (remainder > 0.5f || (remainder == 0.5f && (integral & 1))) ++integral;
    if (remainder < -0.5f || (remainder == -0.5f && (integral & 1))) --integral;
    return integral;
}

uint32_t ratio_code(float ratio) {
    // Add the midpoint as an integer: (ratio + 1.0f) would first discard low
    // ratio bits. The +1 endpoint is 2^31, outside the unsigned 31-bit field.
    const int32_t offset = round_even(ratio * 1073741824.0f);
    const uint32_t code = static_cast<uint32_t>(offset) + UINT32_C(0x40000000);
    return code > UINT32_C(0x7fffffff) ? UINT32_C(0x7fffffff) : code;
}

int32_t acceleration_code(float acceleration) {
    if (acceleration >= 8.0f) return INT32_MAX;
    if (acceleration <= -8.0f) return INT32_MIN;
    return round_even(acceleration * 268435456.0f);
}

void put_u32(uint8_t* output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}
}  // namespace

extern "C" bool probe_native_imu_pack(const float quaternion_wxyz[4], const float accel_g[3],
                                      uint16_t counter_ticks, uint16_t elapsed_ticks,
                                      int16_t temperature_raw, uint8_t output[30]) {
    if (!quaternion_wxyz || !accel_g || !output || counter_ticks > 0x0fff ||
        elapsed_ticks > 0x0fff || !finite_vector(quaternion_wxyz, 4) || !finite_vector(accel_g, 3)) {
        return false;
    }
    unsigned largest = 0;
    for (unsigned i = 1; i < 4; ++i) {
        if (std::fabs(quaternion_wxyz[i]) > std::fabs(quaternion_wxyz[largest])) largest = i;
    }
    if (quaternion_wxyz[largest] == 0.0f) return false;

    uint32_t ratios[3];
    for (unsigned i = 0; i < 3; ++i) {
        ratios[i] = ratio_code(quaternion_wxyz[(largest + i + 1) & 3] / quaternion_wxyz[largest]);
    }
    output[0] = static_cast<uint8_t>(counter_ticks);
    output[1] = static_cast<uint8_t>((counter_ticks >> 8) | (elapsed_ticks << 4));
    output[2] = static_cast<uint8_t>(elapsed_ticks >> 4);
    output[3] = 0x0c;
    put_u32(output + 4, largest | (ratios[0] << 3));
    put_u32(output + 8, (ratios[0] >> 29) | (ratios[1] << 2));
    put_u32(output + 12, (ratios[1] >> 30) | (ratios[2] << 1));
    for (unsigned i = 0; i < 3; ++i) {
        put_u32(output + 16 + i * 4, static_cast<uint32_t>(acceleration_code(accel_g[i])));
    }
    const uint16_t temperature = static_cast<uint16_t>(temperature_raw);
    output[28] = static_cast<uint8_t>(temperature);
    output[29] = static_cast<uint8_t>(temperature >> 8);
    return true;
}

void ProbeNativeMotion::reset() {
    *this = ProbeNativeMotion{};
}

void ProbeNativeMotion::clear_candidate() {
    candidate_ = false;
    gyro_count_ = 0;
    accel_count_ = 0;
}

void ProbeNativeMotion::invalidate() {
    ready_ = false;
    have_accel_ = false;
    have_gyro_ = false;
    have_optical_reference_ = false;
    have_optical_sample_ = false;
    clear_candidate();
    quaternion_[0] = 1.0f;
    for (unsigned i = 0; i < 3; ++i) {
        quaternion_[i + 1] = 0.0f;
        acceleration_[i] = 0.0f;
        gyro_dps_[i] = 0.0f;
        bias_[i] = 0.0f;
    }
    // Keep the last observed identities until a connection change or explicit
    // reset. Restoring availability cannot turn the same packet into new data.
}

bool ProbeNativeMotion::initialize_orientation() {
    const float norm = std::sqrt(squared_norm(mean_accel_));
    if (!std::isfinite(norm) || norm <= 0.0f) return false;
    const float x = mean_accel_[0] / norm;
    const float y = mean_accel_[1] / norm;
    const float z = mean_accel_[2] / norm;
    const float horizontal = std::sqrt(x * x + y * y);
    quaternion_[3] = 0.0f;
    if (horizontal > 0.0f) {
        // atan2 remains well conditioned near -Z, unlike normalizing [1+z,y,-x,0].
        const float half_angle = 0.5f * std::atan2(horizontal, z);
        const float sine = std::sin(half_angle);
        quaternion_[0] = std::cos(half_angle);
        quaternion_[1] = (y / horizontal) * sine;
        quaternion_[2] = (-x / horizontal) * sine;
    } else {
        quaternion_[0] = z >= 0.0f ? 1.0f : 0.0f;
        quaternion_[1] = z >= 0.0f ? 0.0f : 1.0f;
        quaternion_[2] = 0.0f;
    }
    return true;
}

bool ProbeNativeMotion::integrate(const float gyro_dps[3], uint32_t elapsed_us) {
    if (elapsed_us == 0) return true;
    const float x = gyro_dps[0] - bias_[0];
    const float y = gyro_dps[1] - bias_[1];
    const float z = gyro_dps[2] - bias_[2];
    const float norm_squared = x * x + y * y + z * z;
    if (!std::isfinite(norm_squared)) return false;
    if (norm_squared == 0.0f) return true;
    const float norm = std::sqrt(norm_squared);
    const float half_angle = norm * (static_cast<float>(elapsed_us) * (0.5e-6f * kRadiansPerDegree));
    const float sine_scale = std::sin(half_angle) / norm;
    const float dw = std::cos(half_angle);
    const float dx = x * sine_scale;
    const float dy = y * sine_scale;
    const float dz = z * sine_scale;
    const float w = quaternion_[0];
    const float qx = quaternion_[1];
    const float qy = quaternion_[2];
    const float qz = quaternion_[3];
    // Body-local rates multiply on the right of the body-to-reference rotation.
    quaternion_[0] = w * dw - qx * dx - qy * dy - qz * dz;
    quaternion_[1] = w * dx + qx * dw + qy * dz - qz * dy;
    quaternion_[2] = w * dy - qx * dz + qy * dw + qz * dx;
    quaternion_[3] = w * dz + qx * dy - qy * dx + qz * dw;
    return normalize_orientation();
}

bool ProbeNativeMotion::normalize_orientation() {
    const float length_squared = quaternion_[0] * quaternion_[0] + quaternion_[1] * quaternion_[1] +
                                 quaternion_[2] * quaternion_[2] + quaternion_[3] * quaternion_[3];
    if (!std::isfinite(length_squared) || length_squared <= 0.0f) return false;
    const float reciprocal_length = 1.0f / std::sqrt(length_squared);
    for (float& component : quaternion_) component *= reciprocal_length;
    return true;
}

bool ProbeNativeMotion::correct_gravity(uint32_t elapsed_us) {
    if (elapsed_us == 0) return true;
    const float norm_squared = squared_norm(acceleration_);
    // Do not treat obvious dynamic acceleration as a gravity observation.
    // The real acceleration remains unchanged in the outgoing report.
    if (norm_squared < 0.9f * 0.9f || norm_squared > 1.1f * 1.1f) return true;
    const float reciprocal_norm = 1.0f / std::sqrt(norm_squared);
    const float ax = acceleration_[0] * reciprocal_norm;
    const float ay = acceleration_[1] * reciprocal_norm;
    const float az = acceleration_[2] * reciprocal_norm;
    const float w = quaternion_[0], x = quaternion_[1], y = quaternion_[2], z = quaternion_[3];
    const float tx = 2.0f * (y * az - z * ay);
    const float ty = 2.0f * (z * ax - x * az);
    const float tz = 2.0f * (x * ay - y * ax);
    const float gx = ax + w * tx + y * tz - z * ty;
    const float gy = ay + w * ty + z * tx - x * tz;
    const float gz = az + w * tz + x * ty - y * tx;
    const float horizontal = std::sqrt(gx * gx + gy * gy);
    if (horizontal == 0.0f && gz >= 0.0f) return true;
    // Complementary tilt correction in reference space. Its rotation axis has
    // no reference-Z component: do not invent a yaw observation from gravity.
    // Sensor elapsed time, not Core 0 polling, controls the filter strength.
    const float alpha = static_cast<float>(elapsed_us) /
                        (kGravityTimeConstantUs + static_cast<float>(elapsed_us));
    const float half_angle = 0.5f * alpha * std::atan2(horizontal, gz);
    const float sine = std::sin(half_angle);
    const float cw = std::cos(half_angle);
    const float cx = horizontal > 0.0f ? (gy / horizontal) * sine : sine;
    const float cy = horizontal > 0.0f ? (-gx / horizontal) * sine : 0.0f;
    quaternion_[0] = cw * w - cx * x - cy * y;
    quaternion_[1] = cw * x + cx * w + cy * z;
    quaternion_[2] = cw * y - cx * z + cy * w;
    quaternion_[3] = cw * z + cx * y - cy * x;
    return normalize_orientation();
}

void ProbeNativeMotion::observe_optical_heading(uint32_t now_us, uint32_t reference_generation,
                                                uint32_t sequence, uint32_t sample_us,
                                                float yaw_radians, bool valid) {
    if (!ready_) return;
    if (!have_optical_generation_ || reference_generation != optical_generation_) {
        have_optical_generation_ = true;
        optical_generation_ = reference_generation;
        seen_optical_sequence_ = false;
        have_optical_reference_ = false;
        have_optical_sample_ = false;
    }
    if (!valid || (have_optical_sample_ && now_us - optical_sample_us_ >= kFreshUs)) {
        // Retain the relative anchor while hidden, but never accumulate filter
        // weight for the interval without observations.
        have_optical_sample_ = false;
    }
    const uint32_t sequence_delta = sequence - optical_sequence_;
    if (seen_optical_sequence_ && (sequence_delta == 0 || sequence_delta >= 0x80000000u)) return;
    seen_optical_sequence_ = true;
    optical_sequence_ = sequence;
    if (!valid || !std::isfinite(yaw_radians) || now_us - sample_us >= kFreshUs) {
        have_optical_sample_ = false;
        return;
    }
    // Reject backwards timestamps while both observations could still be
    // fresh. Unsigned differences admit normal microsecond-clock rollover.
    if (have_optical_reference_ && now_us - optical_sample_us_ < kFreshUs &&
        sample_us - optical_sample_us_ >= 0x80000000u) return;
    const float w = quaternion_[0], x = quaternion_[1], y = quaternion_[2], z = quaternion_[3];
    // Native body Y is camera forward. Its world XY projection supplies heading;
    // near vertical, yaw is ill-conditioned and must not become an observation.
    const float forward_x = 2.0f * (x * y - w * z);
    const float forward_y = 1.0f - 2.0f * (x * x + z * z);
    if (forward_x * forward_x + forward_y * forward_y < kMinimumOpticalHorizontalSquared) {
        have_optical_sample_ = false;
        return;
    }
    const uint32_t elapsed_us = have_optical_sample_ ? sample_us - optical_sample_us_ : 0;
    optical_sample_us_ = sample_us;
    have_optical_sample_ = true;
    const float heading = std::atan2(forward_y, forward_x);
    const float bearing = std::remainder(yaw_radians, kTwoPi);
    if (!have_optical_reference_) {
        optical_reference_radians_ = std::remainder(heading + bearing, kTwoPi);
        have_optical_reference_ = true;
        return;
    }
    if (elapsed_us == 0 || elapsed_us >= kFreshUs) return;
    // Optical direction complements gyro heading; concept reference:
    // https://github.com/dolphin-emu/dolphin/blob/master/Source/Core/InputCommon/ControllerInterface/Wiimote/WiimoteController.cpp
    // Independently use a relative yaw anchor and elapsed-time gain, not its
    // per-frame full-vector correction. Translation also changes bar bearing;
    // bar placement/pitch can bias it, so this is not absolute world heading.
    // Positive optical aim-right is a negative world-Z heading change.
    const float error = std::remainder(optical_reference_radians_ - bearing - heading, kTwoPi);
    const float alpha = static_cast<float>(elapsed_us) /
                        (kOpticalTimeConstantUs + static_cast<float>(elapsed_us));
    const float half_angle = 0.5f * alpha * error;
    const float cw = std::cos(half_angle), cz = std::sin(half_angle);
    // Left multiplication about reference-world Z preserves gravity-aligned
    // tilt, and neither the measured acceleration nor learned gyro bias changes.
    quaternion_[0] = cw * w - cz * z;
    quaternion_[1] = cw * x - cz * y;
    quaternion_[2] = cw * y + cz * x;
    quaternion_[3] = cw * z + cz * w;
    if (!normalize_orientation()) invalidate();
}

void ProbeNativeMotion::update(uint32_t now_us, uint32_t connection_generation,
                               const ProbeNativeMotionSample& sample, ProbeNativeMotionBias bias_mode) {
    if (!have_generation_ || connection_generation != connection_generation_ || bias_mode != bias_mode_) {
        reset();
        have_generation_ = true;
        connection_generation_ = connection_generation;
        bias_mode_ = bias_mode;
    }
    const uint32_t elapsed_us = have_update_ ? now_us - update_us_ : 0;
    update_us_ = now_us;
    have_update_ = true;
    if (elapsed_us > kMaximumUpdateGapUs) invalidate();
    if (!sample.accel_valid || !sample.gyro_valid ||
        !finite_vector(sample.accel_g, 3) || !finite_vector(sample.gyro_dps, 3)) {
        invalidate();
        return;
    }

    const bool new_accel = !seen_accel_sequence_ || sample.accel_sequence != accel_sequence_;
    const bool new_gyro = !seen_gyro_sequence_ || sample.gyro_sequence != gyro_sequence_;
    // A fresh replacement cannot conceal a stale interval between observations.
    // Unsigned differences also reject timestamps that move backwards.
    if ((have_accel_ && new_accel && sample.accel_us - accel_us_ > kFreshUs) ||
        (have_gyro_ && new_gyro && sample.gyro_us - gyro_us_ > kFreshUs)) {
        invalidate();
    }
    const bool was_ready = ready_;
    const uint32_t accel_elapsed_us = was_ready && new_accel ? sample.accel_us - accel_us_ : 0;
    float previous_gyro[3];
    if (was_ready && new_gyro) {
        for (unsigned i = 0; i < 3; ++i) previous_gyro[i] = gyro_dps_[i];
    }
    if (new_accel) {
        seen_accel_sequence_ = true;
        accel_sequence_ = sample.accel_sequence;
        accel_us_ = sample.accel_us;
        have_accel_ = true;
        for (unsigned i = 0; i < 3; ++i) acceleration_[i] = sample.accel_g[i];
    }
    if (new_gyro) {
        seen_gyro_sequence_ = true;
        gyro_sequence_ = sample.gyro_sequence;
        gyro_us_ = sample.gyro_us;
        have_gyro_ = true;
        for (unsigned i = 0; i < 3; ++i) gyro_dps_[i] = sample.gyro_dps[i];
    }
    if ((have_accel_ && now_us - accel_us_ >= kFreshUs) || (have_gyro_ && now_us - gyro_us_ >= kFreshUs)) {
        invalidate();
        return;
    }
    // Wii acceleration and gyro arrive on independent packets. Keep a fresh
    // first half while waiting for the other sensor after invalidation.
    if (!have_accel_ || !have_gyro_) return;
    if (was_ready) {
        // Zero-order hold uses actual receipt times, independent of update/USB
        // cadence. Split only when a new rate arrived inside this update span.
        const uint32_t rate_age_us = now_us - gyro_us_;
        if (new_gyro && rate_age_us < elapsed_us) {
            if (!integrate(previous_gyro, elapsed_us - rate_age_us) || !integrate(gyro_dps_, rate_age_us)) {
                invalidate();
            }
        } else if (!integrate(gyro_dps_, elapsed_us)) {
            invalidate();
        }
        if (ready_ && new_accel && !correct_gravity(accel_elapsed_us)) invalidate();
        return;
    }

    if (bias_mode_ == ProbeNativeMotionBias::kAlreadyCalibrated) {
        // Trust only the caller's validated calibrated samples, not an estimated
        // stationary bias. The first acceleration fixes a relative gravity frame.
        for (unsigned i = 0; i < 3; ++i) mean_accel_[i] = acceleration_[i];
        ready_ = initialize_orientation();
        return;
    }

    const float acceleration_norm_squared = squared_norm(acceleration_);
    if (acceleration_norm_squared < 0.85f * 0.85f ||
        acceleration_norm_squared > 1.15f * 1.15f) {
        clear_candidate();
        return;
    }
    if (!candidate_) {
        if (!new_gyro) return;
        candidate_ = true;
        candidate_us_ = gyro_us_;
        gyro_count_ = 0;
        accel_count_ = 0;
    }
    if (new_accel || accel_count_ == 0) {
        if (!accumulate_stationary(acceleration_, ++accel_count_, mean_accel_,
                                   accel_variation_, kAccelRmsG)) {
            clear_candidate();
            return;
        }
        if (accel_count_ == kVariationSamples) {
            for (unsigned i = 0; i < 3; ++i) gravity_reference_[i] = mean_accel_[i];
        } else if (accel_count_ > kVariationSamples) {
            // Anchor the averaged direction rather than the previous packet:
            // slow tilt must not be accepted as a series of small noisy steps.
            float dot = 0.0f;
            for (unsigned i = 0; i < 3; ++i) dot += gravity_reference_[i] * mean_accel_[i];
            if (dot <= 0.0f || dot * dot < squared_norm(gravity_reference_) *
                                         squared_norm(mean_accel_) * kGravityDirectionCosSquared) {
                clear_candidate();
                return;
            }
        }
    }
    if (!new_gyro) return;
    // An absolute angular-rate limit cannot distinguish motion from the bias
    // being estimated. Changing rates, acceleration and gravity direction can;
    // perfectly steady rotation about gravity still requires the user to rest.
    if (!accumulate_stationary(gyro_dps_, ++gyro_count_, mean_gyro_,
                               gyro_variation_, kGyroRmsDps)) {
        clear_candidate();
        return;
    }
    if (gyro_count_ >= kCalibrationSamples && accel_count_ >= kVariationSamples &&
        gyro_us_ - candidate_us_ >= kCalibrationUs) {
        if (!initialize_orientation()) {
            clear_candidate();
            return;
        }
        for (unsigned i = 0; i < 3; ++i) bias_[i] = mean_gyro_[i];
        ready_ = true;
        clear_candidate();
    }
}
