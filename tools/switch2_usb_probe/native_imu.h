#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Encode one native mode-0 IMU block. Inputs use the native-right body frame;
// quaternion wxyz rotates body vectors into the reference frame. Quaternion
// scale/sign do not matter. Finite acceleration saturates at signed Q28 limits.
// Invalid inputs leave output untouched; counter and elapsed must fit 12 bits.
bool probe_native_imu_pack(const float quaternion_wxyz[4], const float accel_g[3],
                           uint16_t counter_ticks, uint16_t elapsed_ticks,
                           int16_t temperature_raw, uint8_t output[30]);

#ifdef __cplusplus
}

struct ProbeNativeMotionSample {
    bool accel_valid = false;
    bool gyro_valid = false;
    uint32_t accel_sequence = 0;
    uint32_t gyro_sequence = 0;
    uint32_t accel_us = 0;
    uint32_t gyro_us = 0;
    float accel_g[3]{};
    float gyro_dps[3]{};
};

enum class ProbeNativeMotionBias : uint8_t {
    kAlreadyCalibrated,
    kTrackStationary,
};

// Core 0 only. Call update even when sensors are unavailable, and consult ready
// before using the orientation. Sequence identities, not polling, admit samples;
// repeated sequences cannot refresh timestamps or contribute to calibration.
// All sources initialize from the first usable fresh sensor pair. Wii optionally
// refines residual bias in the background without resetting orientation. Steady
// rotation about gravity remains indistinguishable from bias without a reference.
// Fresh near-1g acceleration corrects tilt drift. Heading remains gyro-derived
// unless a reliable optical heading observation is supplied.
class ProbeNativeMotion {
public:
    void reset();
    void update(uint32_t now_us, uint32_t connection_generation,
                const ProbeNativeMotionSample& sample,
                ProbeNativeMotionBias bias_mode = ProbeNativeMotionBias::kAlreadyCalibrated);
    // Call after update, using a full observed sensor-bar pair (never inferred).
    // Positive optical yaw means aim-right, the negative reference-world turn.
    // The first sample in each optical generation anchors the current heading;
    // it supplies neither absolute world yaw nor an absolute console cursor.
    void observe_optical_heading(uint32_t now_us, uint32_t reference_generation,
                                 uint32_t sequence, uint32_t sample_us,
                                 float yaw_radians, bool valid);
    bool ready() const { return ready_; }
    const float* quaternion() const { return quaternion_; }
    const float* acceleration() const { return acceleration_; }
    const float* bias() const { return bias_; }

private:
    void invalidate();
    void clear_candidate();
    bool initialize_orientation();
    bool integrate(const float gyro_dps[3], uint32_t elapsed_us);
    bool correct_gravity(uint32_t elapsed_us);
    bool normalize_orientation();
    void track_bias(bool new_accel, bool new_gyro, uint32_t gyro_elapsed_us);

    bool have_generation_ = false;
    ProbeNativeMotionBias bias_mode_ = ProbeNativeMotionBias::kAlreadyCalibrated;
    bool have_update_ = false;
    bool seen_accel_sequence_ = false;
    bool seen_gyro_sequence_ = false;
    bool have_accel_ = false;
    bool have_gyro_ = false;
    bool candidate_ = false;
    bool ready_ = false;
    bool have_optical_generation_ = false;
    bool seen_optical_sequence_ = false;
    bool have_optical_reference_ = false;
    bool have_optical_sample_ = false;
    uint32_t connection_generation_ = 0;
    uint32_t update_us_ = 0;
    uint32_t accel_sequence_ = 0;
    uint32_t gyro_sequence_ = 0;
    uint32_t accel_us_ = 0;
    uint32_t gyro_us_ = 0;
    uint32_t candidate_us_ = 0;
    uint32_t gyro_count_ = 0;
    uint32_t accel_count_ = 0;
    uint32_t optical_generation_ = 0;
    uint32_t optical_sequence_ = 0;
    uint32_t optical_sample_us_ = 0;
    float optical_reference_radians_ = 0.0f;
    float gyro_variation_ = 0.0f;
    float accel_variation_ = 0.0f;
    float quaternion_[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float acceleration_[3]{};
    float gyro_dps_[3]{};
    float bias_[3]{};
    float bias_target_[3]{};
    float mean_gyro_[3]{};
    float mean_accel_[3]{};
    float gravity_reference_[3]{};
};
#endif
