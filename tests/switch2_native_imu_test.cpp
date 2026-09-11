#include "native_imu.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
using Vector = std::array<double, 3>;
using Quaternion = std::array<double, 4>;

void expect(bool okay, const char* message) {
    if (!okay) {
        std::cerr << message << '\n';
        ++failures;
    }
}

// Deliberately decode bit-by-bit rather than mirroring the encoder's word
// boundaries. Double precision here is an independent host-side reference.
uint32_t bits(const uint8_t* bytes, unsigned start, unsigned width) {
    uint32_t result = 0;
    for (unsigned i = 0; i < width; ++i) {
        result |= static_cast<uint32_t>((bytes[(start + i) / 8] >> ((start + i) % 8)) & 1) << i;
    }
    return result;
}

int64_t signed_bits(const uint8_t* bytes, unsigned start, unsigned width) {
    const uint32_t value = bits(bytes, start, width);
    return value & (UINT32_C(1) << (width - 1)) ? static_cast<int64_t>(value) - (INT64_C(1) << width) : value;
}

struct Decoded {
    Quaternion q{};
    Vector acceleration{};
    std::array<int64_t, 3> acceleration_raw{};
    uint32_t counter = 0;
    uint32_t elapsed = 0;
    int64_t temperature = 0;
};

Decoded decode(const uint8_t* bytes) {
    Decoded result;
    expect(bytes[3] == 0x0c, "wire block must use the recovered one-sample format");
    result.counter = bits(bytes, 0, 12);
    result.elapsed = bits(bytes, 12, 12);
    result.temperature = signed_bits(bytes, 224, 16);
    const unsigned largest = bits(bytes, 32, 3);
    expect(largest < 4, "wire quaternion tag must identify an actual component");
    if (largest >= 4) return result;
    Vector ratios{};
    double length_squared = 1.0;
    for (unsigned i = 0; i < 3; ++i) {
        ratios[i] = bits(bytes, 35 + 31 * i, 31) / 1073741824.0 - 1.0;
        length_squared += ratios[i] * ratios[i];
        result.acceleration_raw[i] = signed_bits(bytes, 128 + 32 * i, 32);
        result.acceleration[i] = result.acceleration_raw[i] / 268435456.0;
    }
    result.q[largest] = 1.0 / std::sqrt(length_squared);
    for (unsigned i = 0; i < 3; ++i) result.q[(largest + i + 1) & 3] = ratios[i] * result.q[largest];
    return result;
}

Vector rotate(const Quaternion& q, const Vector& v) {
    const Vector cross{q[2] * v[2] - q[3] * v[1], q[3] * v[0] - q[1] * v[2], q[1] * v[1] - q[2] * v[0]};
    return {v[0] + 2.0 * (q[0] * cross[0] + q[2] * cross[2] - q[3] * cross[1]),
            v[1] + 2.0 * (q[0] * cross[1] + q[3] * cross[0] - q[1] * cross[2]),
            v[2] + 2.0 * (q[0] * cross[2] + q[1] * cross[1] - q[2] * cross[0])};
}

bool close(const Vector& a, const Vector& b, double tolerance = 0.00002) {
    for (unsigned i = 0; i < 3; ++i) {
        if (!std::isfinite(a[i]) || std::abs(a[i] - b[i]) > tolerance) return false;
    }
    return true;
}

Quaternion normalized(const float input[4]) {
    double squared = 0.0;
    for (unsigned i = 0; i < 4; ++i) squared += static_cast<double>(input[i]) * input[i];
    Quaternion result{};
    for (unsigned i = 0; i < 4; ++i) result[i] = input[i] / std::sqrt(squared);
    return result;
}

double camera_heading(const ProbeNativeMotion& motion) {
    const Vector forward = rotate(normalized(motion.quaternion()), {0.0, 1.0, 0.0});
    return std::atan2(forward[1], forward[0]);
}

double heading_change(const ProbeNativeMotion& motion, double initial_heading) {
    return std::remainder(camera_heading(motion) - initial_heading, 2.0 * std::acos(-1.0));
}

void check_encoded_orientation(const float q[4], const float accel[3]) {
    std::array<uint8_t, 32> guarded{};
    guarded.front() = 0x53;
    guarded.back() = 0x79;
    expect(probe_native_imu_pack(q, accel, 0xabc, 0xdef, -12345, guarded.data() + 1), "finite nonzero orientation must encode");
    expect(guarded.front() == 0x53 && guarded.back() == 0x79, "encoder must write exactly thirty bytes");
    const auto decoded = decode(guarded.data() + 1);
    expect(decoded.counter == 0xabc && decoded.elapsed == 0xdef && decoded.temperature == -12345,
           "wire counters and signed temperature must decode independently");
    for (unsigned axis = 0; axis < 3; ++axis) {
        Vector basis{};
        basis[axis] = 1.0;
        expect(close(rotate(decoded.q, basis), rotate(normalized(q), basis)),
               "wire orientation must preserve physical rotation for every largest-component choice");
    }
    expect(close(decoded.acceleration, {accel[0], accel[1], accel[2]}, 0.0000001),
           "wire acceleration must preserve signed Q28 g units");
}

void codec_wire_edges() {
    const float accel[3]{1.25f, -2.5f, 0.0625f};
    const float orientations[][4]{{4.0f, -1.0f, 2.0f, -3.0f}, {-0.5f, -3.0f, 0.25f, 1.0f},
                                  {1.0f, -2.0f, 4.0f, -0.5f}, {-1.0f, 0.5f, -2.0f, -4.0f},
                                  {1.0f, -1.0f, 1.0f, 1.0f}};
    for (const auto& q : orientations) check_encoded_orientation(q, accel);
    const float large = std::numeric_limits<float>::max();
    const float tiny = std::numeric_limits<float>::denorm_min();
    const float large_q[4]{large, -large, large, -large};
    const float tiny_q[4]{tiny, -tiny, tiny, -tiny};
    check_encoded_orientation(large_q, accel);
    check_encoded_orientation(tiny_q, accel);

    uint8_t bytes[30]{};
    const float identity[4]{1.0f, 0.0f, 0.0f, 0.0f};
    const float saturated[3]{large, -large, -8.0f};
    expect(probe_native_imu_pack(identity, saturated, 0, 4095, -32768, bytes), "finite out-of-range acceleration must saturate");
    const auto limits = decode(bytes);
    expect(limits.acceleration_raw == std::array<int64_t, 3>{INT32_MAX, INT32_MIN, INT32_MIN},
           "positive saturation must never wrap to negative acceleration");
    expect(limits.counter == 0 && limits.elapsed == 4095 && limits.temperature == -32768,
           "wire signed and unsigned field boundaries must survive encoding");
    const float ties[3]{0.5f / 268435456.0f, 1.5f / 268435456.0f, -2.5f / 268435456.0f};
    const float fine_q[4]{1.0f, 5.5f / 1073741824.0f, -5.5f / 1073741824.0f, 0.0f};
    expect(probe_native_imu_pack(fine_q, ties, 0, 0, 0, bytes), "sub-count finite inputs must encode");
    expect(decode(bytes).acceleration_raw == std::array<int64_t, 3>{0, 2, -2}, "Q28 halfway values use nearest-even rounding");
    expect(bits(bytes, 35, 31) == UINT32_C(0x40000006) && bits(bytes, 66, 31) == UINT32_C(0x3ffffffa),
           "adding the ratio midpoint must not erase low quaternion bits");
}

void codec_rejects_invalid_inputs() {
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float accel[3]{0.0f, 0.0f, 1.0f};
    std::array<uint8_t, 30> output;
    output.fill(0xa5);
    const auto unchanged = output;
    const float invalid[]{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()};
    for (float value : invalid) {
        q[2] = value;
        expect(!probe_native_imu_pack(q, accel, 1, 1, 0, output.data()), "nonfinite quaternion must be rejected");
        q[2] = 0.0f;
        accel[1] = value;
        expect(!probe_native_imu_pack(q, accel, 1, 1, 0, output.data()), "nonfinite acceleration must be rejected, not saturated");
        accel[1] = 0.0f;
    }
    q[0] = 0.0f;
    expect(!probe_native_imu_pack(q, accel, 1, 1, 0, output.data()), "zero quaternion cannot identify a real orientation");
    q[0] = 1.0f;
    expect(!probe_native_imu_pack(q, accel, 4096, 1, 0, output.data()), "counter overflow must fail rather than overwrite elapsed bits");
    expect(!probe_native_imu_pack(q, accel, 1, 4096, 0, output.data()), "elapsed overflow must fail rather than overwrite format bits");
    expect(output == unchanged, "failed encoding must leave the caller's pending packet unchanged");
}

struct Rig {
    ProbeNativeMotion motion;
    ProbeNativeMotionSample sample{};
    uint32_t now;
    uint32_t generation = 1;
    static constexpr float residual[3]{0.4f, -0.3f, 0.2f};

    explicit Rig(uint32_t start = 0) : now(start) {
        sample.accel_valid = sample.gyro_valid = true;
        sample.accel_g[2] = 1.0f;
        rest();
    }
    void rest() {
        for (unsigned i = 0; i < 3; ++i) sample.gyro_dps[i] = residual[i];
    }
    void fresh(uint32_t elapsed = 25000, bool accel = true, bool gyro = true) {
        now += elapsed;
        if (accel) {
            ++sample.accel_sequence;
            sample.accel_us = now;
        }
        if (gyro) {
            ++sample.gyro_sequence;
            sample.gyro_us = now;
        }
        motion.update(now, generation, sample);
    }
    void settle() {
        for (unsigned i = 0; i < 65; ++i) fresh();
        expect(motion.ready(), "fresh stationary sensors must finish residual bias calibration");
    }
    Decoded packet() {
        uint8_t bytes[30]{};
        expect(motion.ready(), "only calibrated fresh motion may be packed for the consumer");
        expect(probe_native_imu_pack(motion.quaternion(), motion.acceleration(), 0, 0, 0, bytes), "live motion must be encodable");
        return decode(bytes);
    }
};

void startup_needs_count_and_stillness() {
    Rig fast;
    fast.fresh(0);
    for (unsigned i = 0; i < 63; ++i) fast.fresh(1000);
    expect(!fast.motion.ready(), "sixty-four packets alone cannot replace 1.5 seconds of stillness");
    for (unsigned i = 0; i < 56; ++i) fast.fresh();
    expect(!fast.motion.ready(), "calibration must not finish before the stillness duration");
    fast.fresh(37000);
    expect(fast.motion.ready(), "a stationary window may finish at exactly 1.5 seconds");

    Rig slow;
    slow.fresh(0);
    for (unsigned i = 0; i < 62; ++i) slow.fresh(30000);
    expect(!slow.motion.ready(), "elapsed stillness cannot replace sixty-four distinct gyro samples");
    slow.fresh(30000);
    expect(slow.motion.ready(), "the sixty-fourth stationary sample can finish calibration");
    expect(close({slow.motion.bias()[0], slow.motion.bias()[1], slow.motion.bias()[2]},
                 {Rig::residual[0], Rig::residual[1], Rig::residual[2]}, 0.000001),
           "calibration must learn the selected sensor's residual bias");
}

void quantized_wii_bias_calibrates_and_integrates() {
    Rig rig;
    constexpr float device_bias[3]{-13.0625f, 12.4375f, -13.125f};
    constexpr int noise_q10[8]{-576, 192, -320, 576, -192, 320, -64, 64};
    const Vector gravity{8352.0 / 8192.0, 290.0 / 8192.0, -298.0 / 8192.0};
    // Deterministic Wii-scale Q13 steps and Q10 noise, with accel and gyro
    // arriving independently at 200 / 100 Hz. Neither noise nor bias is motion.
    for (unsigned i = 0; i < 320 && !rig.motion.ready(); ++i) {
        rig.sample.accel_g[0] = (8352 + (i & 1 ? 80 : -80)) / 8192.0f;
        rig.sample.accel_g[1] = (290 + (i & 2 ? 40 : -40)) / 8192.0f;
        rig.sample.accel_g[2] = (-298 + (i & 4 ? 43 : -43)) / 8192.0f;
        if (i % 2 == 0) {
            for (unsigned axis = 0; axis < 3; ++axis) {
                rig.sample.gyro_dps[axis] = device_bias[axis] + noise_q10[(i / 2 + axis * 3) % 8] / 1024.0f;
            }
        }
        rig.fresh(5000, true, i % 2 == 0);
    }
    expect(rig.motion.ready(), "quantized stationary Wii sensors with a large own-device bias must calibrate");
    expect(close({rig.motion.bias()[0], rig.motion.bias()[1], rig.motion.bias()[2]},
                 {device_bias[0], device_bias[1], device_bias[2]}, 0.01),
           "stationary variation must average into the measured bias, not a nominal or donor zero");
    const Quaternion reference = rig.packet().q;
    const double gravity_length = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    expect(close(rotate(reference, gravity), {0.0, 0.0, gravity_length}, 0.0002),
           "quantized gravity must establish the physical reference without rescaling acceleration");
    for (unsigned axis = 0; axis < 3; ++axis) {
        rig.sample.gyro_dps[axis] = device_bias[axis];
        rig.sample.accel_g[axis] = static_cast<float>(gravity[axis]);
    }
    rig.fresh(0);
    for (unsigned i = 0; i < 100; ++i) rig.fresh(10000);
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), rotate(reference, {1.0, 0.0, 0.0}), 0.001),
           "learned large residual bias must not become stationary orientation drift");
    rig.sample.gyro_dps[2] += 90.0f;
    rig.fresh(0);
    for (unsigned i = 1; i <= 100; ++i) {
        const double angle = i * std::acos(-1.0) / 200.0;
        rig.sample.accel_g[0] = static_cast<float>(std::cos(angle) * gravity[0] + std::sin(angle) * gravity[1]);
        rig.sample.accel_g[1] = static_cast<float>(-std::sin(angle) * gravity[0] + std::cos(angle) * gravity[1]);
        rig.fresh(10000);
    }
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), rotate(reference, {0.0, 1.0, 0.0}), 0.001),
           "one second of body rotation must subtract the measured Wii bias before integration");
}

void measured_gravity_sets_reference() {
    const float poses[][3]{{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f},
                           {0.000001f, 0.000002f, -1.0f}, {0.3f, 0.4f, 0.8660254f}};
    for (const auto& pose : poses) {
        Rig rig;
        for (unsigned i = 0; i < 3; ++i) rig.sample.accel_g[i] = pose[i];
        rig.settle();
        const auto decoded = rig.packet();
        expect(close(rotate(decoded.q, decoded.acceleration), {0.0, 0.0, 1.0}),
               "measured gravity, including anti-parallel gravity, must align with reference +Z");
        expect(close(decoded.acceleration, {pose[0], pose[1], pose[2]}, 0.0000001),
               "reference initialization must not replace the real body acceleration");
    }
}

void bias_corrected_body_rotation_reaches_wire() {
    Rig rig;
    rig.sample.accel_g[0] = 1.0f;
    rig.sample.accel_g[2] = 0.0f;
    rig.settle();
    for (unsigned i = 0; i < 100; ++i) rig.fresh(10000);
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), {0.0, 0.0, 1.0}),
           "stationary residual bias must not rotate the initialized orientation");
    rig.sample.gyro_dps[2] += 90.0f;
    rig.fresh(0);
    for (unsigned i = 1; i <= 100; ++i) {
        const double angle = i * std::acos(-1.0) / 200.0;
        rig.sample.accel_g[0] = static_cast<float>(std::cos(angle));
        rig.sample.accel_g[1] = static_cast<float>(-std::sin(angle));
        rig.fresh(10000);
    }
    const auto turned = rig.packet();
    expect(close(rotate(turned.q, {1.0, 0.0, 0.0}), {0.0, 1.0, 0.0}) &&
           close(rotate(turned.q, {0.0, 1.0, 0.0}), {0.0, 0.0, -1.0}) &&
           close(rotate(turned.q, {0.0, 0.0, 1.0}), {-1.0, 0.0, 0.0}),
           "one second of corrected body-Z rotation must compose after the nonidentity gravity alignment");
    rig.sample.accel_g[0] = 1.6f;
    rig.sample.accel_g[1] = -0.3f;
    rig.sample.accel_g[2] = 0.8f;
    rig.fresh(0);
    expect(close(rig.packet().acceleration, {1.6, -0.3, 0.8}, 0.0000001), "ready packets must retain real dynamic acceleration");
    expect(close({rig.motion.bias()[0], rig.motion.bias()[1], rig.motion.bias()[2]},
                 {Rig::residual[0], Rig::residual[1], Rig::residual[2]}, 0.000001),
           "motion after calibration must not relearn or chase the gyro bias");
}

void elapsed_time_not_packet_count_controls_rotation() {
    for (uint32_t interval : {5000u, 25000u}) {
        Rig rig;
        rig.settle();
        rig.sample.gyro_dps[2] += 120.0f;
        rig.fresh(0);
        for (uint32_t elapsed = interval; elapsed <= 1000000; elapsed += interval) {
            rig.fresh(interval, true, elapsed % 100000 == 0);
        }
        expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), {-0.5, std::sqrt(0.75), 0.0}),
               "a fresh held gyro rate must integrate elapsed time regardless of polling cadence");
    }
    Rig split;
    split.settle();
    split.sample.gyro_dps[2] += 90.0f;
    ++split.sample.gyro_sequence;
    split.sample.gyro_us = split.now + 10000;
    split.fresh(40000, true, false);
    const double angle = 2.7 * std::acos(-1.0) / 180.0;
    expect(close(rotate(split.packet().q, {1.0, 0.0, 0.0}), {std::cos(angle), std::sin(angle), 0.0}),
           "a rate arriving inside an update interval must only affect time after its receipt");
}

void moving_startup_does_not_calibrate() {
    for (unsigned movement = 0; movement < 4; ++movement) {
        Rig rig;
        for (unsigned i = 0; i < 100; ++i) {
            rig.rest();
            rig.sample.accel_g[0] = 0.0f;
            rig.sample.accel_g[1] = 0.0f;
            rig.sample.accel_g[2] = 1.0f;
            if (movement == 0) {
                // A real slow body-X turn changes gravity even with a constant
                // gyro reading. Its per-packet tilt is smaller than Wii noise.
                const float angle = static_cast<float>(i) * 0.025f * 2.0f * static_cast<float>(std::acos(-1.0) / 180.0);
                rig.sample.gyro_dps[0] += 2.0f;
                rig.sample.accel_g[1] = std::sin(angle);
                rig.sample.accel_g[2] = std::cos(angle);
            }
            if (movement == 1) rig.sample.accel_g[2] = 0.5f;
            if (movement == 2) rig.sample.accel_g[0] = i & 1 ? 0.04f : -0.04f;
            if (movement == 3) rig.sample.gyro_dps[0] = i & 1 ? 1.0f : -1.0f;
            rig.fresh();
        }
        expect(!rig.motion.ready(), "moving startup cannot be mistaken for a stationary calibration window");
        rig.rest();
        rig.sample.accel_g[0] = 0.0f;
        rig.sample.accel_g[1] = 0.0f;
        rig.sample.accel_g[2] = 1.0f;
        for (unsigned i = 0; i < 128; ++i) rig.fresh();
        expect(rig.motion.ready(), "a new stationary window must recover after changing motion ends");
    }
    Rig contaminated;
    for (unsigned i = 0; i < 40; ++i) contaminated.fresh();
    contaminated.sample.gyro_dps[0] = 4.0f;
    contaminated.fresh();
    contaminated.rest();
    for (unsigned i = 0; i < 40; ++i) contaminated.fresh();
    expect(!contaminated.motion.ready(), "a gyro impulse must discard, not pool, the partial bias window");
    for (unsigned i = 0; i < 24; ++i) contaminated.fresh();
    expect(contaminated.motion.ready(), "an uncontaminated replacement window must eventually calibrate");

    Rig interleaved;
    for (unsigned i = 0; i < 55; ++i) interleaved.fresh();
    interleaved.sample.accel_g[0] = 0.1f;
    interleaved.fresh(5000, true, false);
    interleaved.sample.accel_g[0] = 0.0f;
    interleaved.fresh(5000, true, false);
    for (unsigned i = 0; i < 20; ++i) interleaved.fresh();
    expect(!interleaved.motion.ready(), "acceleration-only movement must reset the gyro calibration window too");
    interleaved.settle();
}

void duplicate_sequences_and_interleaved_recovery() {
    Rig rig;
    rig.fresh(0);
    for (unsigned i = 0; i < 400; ++i) {
        // A caller cannot renew sensor lifetime by relabelling receipt time
        // while leaving the actual parser sequence unchanged.
        rig.sample.accel_us = rig.sample.gyro_us = rig.now + 5000;
        rig.fresh(5000, false, false);
    }
    expect(!rig.motion.ready(), "duplicate samples cannot establish calibrated sensor readiness");
    for (unsigned i = 0; i < 128; ++i) rig.fresh(25000, i % 2 == 0, i % 2 != 0);
    expect(rig.motion.ready(), "fresh independently interleaved sensors must recover after a stale reset");
    rig.sample.gyro_dps[2] += 100.0f;
    rig.fresh(10000, false, false);
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}),
           "mutating a duplicate sequence must not inject a new angular rate");
}

void lifecycle_invalidates_bias_and_orientation() {
    for (unsigned fault = 0; fault < 7; ++fault) {
        Rig rig;
        rig.settle();
        if (fault == 0) rig.sample.accel_valid = false;
        if (fault == 1) rig.sample.gyro_valid = false;
        if (fault == 2) rig.sample.accel_g[0] = std::numeric_limits<float>::quiet_NaN();
        if (fault == 3) rig.sample.gyro_dps[0] = std::numeric_limits<float>::infinity();
        if (fault == 4) ++rig.generation;
        if (fault == 5) rig.motion.reset();
        rig.fresh(fault == 6 ? 50001 : 1000);
        expect(!rig.motion.ready(), "lifecycle or invalid sensor input must fail closed immediately");
        rig.sample.accel_valid = rig.sample.gyro_valid = true;
        rig.sample.accel_g[0] = rig.sample.accel_g[1] = 0.0f;
        rig.sample.accel_g[2] = -1.0f;
        rig.sample.gyro_dps[0] = -0.25f;
        rig.sample.gyro_dps[1] = 0.1f;
        rig.sample.gyro_dps[2] = -0.35f;
        rig.settle();
        expect(close({rig.motion.bias()[0], rig.motion.bias()[1], rig.motion.bias()[2]}, {-0.25, 0.1, -0.35}, 0.000001),
               "recovery must calibrate a new bias instead of borrowing the previous connection's bias");
        expect(close(rotate(rig.packet().q, {0.0, 0.0, -1.0}), {0.0, 0.0, 1.0}),
               "recovery must align the new physical pose instead of replaying old orientation");
    }
}

void stale_boundaries_and_hidden_gaps() {
    for (bool stale_gyro : {false, true}) {
        Rig rig;
        rig.settle();
        rig.fresh(50000, stale_gyro, !stale_gyro);
        rig.fresh(50000, stale_gyro, !stale_gyro);
        rig.fresh(49999, stale_gyro, !stale_gyro);
        expect(rig.motion.ready(), "a held sensor remains usable strictly below the 150ms age limit");
        rig.fresh(1, stale_gyro, !stale_gyro);
        expect(!rig.motion.ready(), "either sensor reaching the 150ms limit must invalidate motion");
        rig.settle();
    }
    Rig hidden;
    hidden.settle();
    hidden.sample.gyro_dps[2] += 90.0f;
    hidden.fresh(0);
    hidden.fresh(50000, false, false);
    hidden.fresh(50000, false, false);
    expect(hidden.motion.ready(), "a 50ms update gap is still within the integration limit");
    hidden.fresh(50000);
    expect(hidden.motion.ready(), "a replacement exactly at expiry must preserve the continuously covered interval");
    hidden.fresh(50000, false, false);
    hidden.fresh(50000, false, false);
    hidden.fresh(49999, false, false);
    hidden.fresh(2);
    expect(!hidden.motion.ready(), "fresh replacements cannot hide an intervening stale sample interval");
    hidden.rest();
    hidden.settle();
    expect(close(rotate(hidden.packet().q, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}),
           "stale angular displacement must not replay after recalibration");
}

void clocks_sequences_and_connections_wrap() {
    Rig rig(UINT32_MAX - 2200000u);
    rig.sample.accel_sequence = rig.sample.gyro_sequence = UINT32_MAX - 30u;
    rig.settle();
    rig.sample.gyro_dps[2] += 90.0f;
    rig.fresh(0);
    for (unsigned i = 0; i < 80; ++i) rig.fresh(12500);
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), {0.0, 1.0, 0.0}),
           "microsecond rollover must preserve one second of physical angular displacement");
    ++rig.generation;
    rig.sample.accel_sequence = rig.sample.gyro_sequence = 0;
    rig.rest();
    rig.fresh(0);
    expect(!rig.motion.ready(), "a new connection cannot inherit readiness even if it reuses sample identities");
    rig.settle();
    expect(close(rotate(rig.packet().q, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}),
           "a new connection must establish its own reference orientation");

    Rig calibration_wrap(UINT32_MAX - 750000u);
    calibration_wrap.settle();
    expect(calibration_wrap.motion.ready(), "the stationary window must span microsecond rollover safely");
}

void fresh_gravity_bounds_warm_gyro_tilt_drift() {
    Rig normal, fast;
    for (Rig* rig : {&normal, &fast}) {
        rig->sample.accel_g[0] = 1.0f;
        rig->sample.accel_g[2] = 0.0f;
        rig->settle();
        // The console trace drifted roughly 0.6 degrees/s after startup.
        rig->sample.gyro_dps[1] += 0.6f;
        rig->fresh(0);
    }
    double maximum_tilt = 0.0;
    for (unsigned millisecond = 1; millisecond <= 60000; ++millisecond) {
        fast.fresh(1000, millisecond % 10 == 0, millisecond % 10 == 0);
        if (millisecond % 10 == 0) normal.fresh(10000);
        if (millisecond % 1000 == 0) {
            const Vector gravity = rotate(normal.packet().q, {1.0, 0.0, 0.0});
            const double tilt = std::atan2(std::hypot(gravity[0], gravity[1]), gravity[2]);
            if (tilt > maximum_tilt) maximum_tilt = tilt;
        }
    }
    expect(maximum_tilt < std::acos(-1.0) / 180.0,
           "fresh stationary gravity must keep warming gyro tilt drift below one degree");
    expect(close(rotate(normal.packet().q, {1.0, 0.0, 0.0}),
                 rotate(fast.packet().q, {1.0, 0.0, 0.0}), 0.0002),
           "duplicate accelerometer polling must not strengthen gravity correction");
}

void dynamic_acceleration_does_not_steer_orientation() {
    Rig rig;
    rig.settle();
    rig.sample.gyro_dps[2] += 60.0f;
    rig.sample.accel_g[0] = 2.0f;
    rig.fresh(0);
    for (unsigned i = 0; i < 100; ++i) rig.fresh(10000);
    const auto packet = rig.packet();
    expect(close(rotate(packet.q, {1.0, 0.0, 0.0}), {0.5, std::sqrt(0.75), 0.0}),
           "non-gravity acceleration must not steer gyro orientation");
    expect(close(packet.acceleration, {2.0, 0.0, 1.0}),
           "gravity correction must not replace the consumer's dynamic acceleration");
}

void optical_heading_bounds_drift_without_polling_gain() {
    Rig slow, frequent, gyro_only;
    for (Rig* rig : {&slow, &frequent, &gyro_only}) {
        rig->settle();
        rig->sample.gyro_dps[2] += 0.6f;
        rig->fresh(0);
    }
    const double initial = camera_heading(slow.motion);
    slow.motion.observe_optical_heading(slow.now, 1, 0, slow.now, 0.2f, true);
    frequent.motion.observe_optical_heading(frequent.now, 1, 0, frequent.now, 0.2f, true);
    double maximum_drift = 0.0;
    for (uint32_t step = 1; step <= 6000; ++step) {
        slow.fresh(10000);
        frequent.fresh(10000);
        gyro_only.fresh(10000);
        // The same physical reference at 10Hz and 50Hz, with extra polling of
        // duplicate 50Hz reports, must give essentially the same drift bound.
        if (step % 10 == 0) {
            slow.motion.observe_optical_heading(slow.now, 1, step / 10, slow.now, 0.2f, true);
        }
        frequent.motion.observe_optical_heading(frequent.now, 1, step / 2,
                                                frequent.now, 0.2f, true);
        if (step % 10 == 0) {
            const double drift = std::abs(heading_change(slow.motion, initial));
            if (drift > maximum_drift) maximum_drift = drift;
        }
    }
    expect(maximum_drift < 1.3 * std::acos(-1.0) / 180.0,
           "stable optical bearing must bound a minute of warming gyro yaw drift");
    expect(std::abs(heading_change(slow.motion, camera_heading(frequent.motion))) < 0.0002,
           "optical correction strength must follow sample time, not sample or polling count");
    expect(std::abs(heading_change(gyro_only.motion, initial) - 36.0 * std::acos(-1.0) / 180.0) < 0.0002,
           "without optical observations gravity must leave yaw gyro-derived");
}

void optical_anchor_preserves_pose_and_corrects_only_world_yaw() {
    Rig rig;
    rig.sample.accel_g[0] = 0.3f;
    rig.sample.accel_g[1] = 0.4f;
    rig.sample.accel_g[2] = std::sqrt(0.75f);
    rig.settle();
    // Establish arbitrary nonzero world yaw while preserving this physical tilt.
    for (unsigned i = 0; i < 3; ++i) rig.sample.gyro_dps[i] += 60.0f * rig.sample.accel_g[i];
    rig.fresh(0);
    for (unsigned i = 0; i < 20; ++i) rig.fresh(25000);
    rig.rest();
    rig.fresh(0);
    const Quaternion initial = normalized(rig.motion.quaternion());
    const double initial_heading = camera_heading(rig.motion);
    rig.motion.observe_optical_heading(rig.now, 1, 1, rig.now, 0.45f, true);
    expect(normalized(rig.motion.quaternion()) == initial,
           "the first optical sample must preserve arbitrary existing yaw and tilt exactly");
    rig.sample.accel_g[0] = 1.8f;
    rig.sample.accel_g[1] = -0.2f;
    rig.sample.accel_g[2] = 0.6f;
    rig.fresh(0);
    for (uint32_t step = 1; step <= 400; ++step) {
        rig.fresh(25000);
        if (step % 4 == 0) {
            rig.motion.observe_optical_heading(rig.now, 1, step / 4 + 1, rig.now, 0.8f, true);
        }
    }
    expect(std::abs(heading_change(rig.motion, initial_heading) + 0.35) < 0.004,
           "increasing aim-right optical yaw must converge to negative world yaw, not reanchor away motion");
    const Quaternion corrected = normalized(rig.motion.quaternion());
    for (unsigned axis = 0; axis < 3; ++axis) {
        Vector basis{};
        basis[axis] = 1.0;
        expect(std::abs(rotate(initial, basis)[2] - rotate(corrected, basis)[2]) < 0.000002,
               "optical correction must preserve every body axis's gravity-aligned tilt");
    }
    expect(close(rig.packet().acceleration, {1.8, -0.2, 0.6}, 0.0000001),
           "optical correction must retain real dynamic acceleration in the native packet");
    expect(close({rig.motion.bias()[0], rig.motion.bias()[1], rig.motion.bias()[2]},
                 {Rig::residual[0], Rig::residual[1], Rig::residual[2]}, 0.000001),
           "optical correction must not modify the calibrated raw gyro bias");
}

void optical_duplicates_and_hidden_intervals_do_not_gain_weight() {
    Rig rig;
    rig.settle();
    const double initial_heading = camera_heading(rig.motion);
    uint32_t sequence = 10;
    rig.motion.observe_optical_heading(rig.now, 1, sequence, rig.now, 0.0f, true);
    rig.sample.gyro_dps[2] += 30.0f;
    rig.fresh(0);
    for (unsigned i = 0; i < 40; ++i) {
        rig.fresh(25000);
        rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, 0.0f, false);
    }
    rig.rest();
    rig.fresh(0);
    expect(std::abs(heading_change(rig.motion, initial_heading) - std::acos(-1.0) / 6.0) < 0.00002,
           "hidden or inferred optical reports must not invent a heading correction");
    const double hidden_heading = camera_heading(rig.motion);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, 0.0f, true);
    expect(std::abs(heading_change(rig.motion, hidden_heading)) < 0.000001,
           "reacquisition must not spend correction weight accumulated while hidden");
    for (unsigned i = 0; i < 4; ++i) rig.fresh(25000);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, 0.0f, true);
    expect(heading_change(rig.motion, hidden_heading) < -0.02,
           "continuous reacquisition must correct toward the retained original relative anchor");
    const double corrected_heading = camera_heading(rig.motion);
    for (unsigned i = 0; i < 200; ++i) {
        rig.fresh(1000);
        // A changed value and a newly labelled timestamp are still one packet.
        rig.motion.observe_optical_heading(rig.now, 1, sequence, rig.now, -1.0f, true);
    }
    expect(std::abs(heading_change(rig.motion, corrected_heading)) < 0.000001,
           "duplicate optical sequence identities cannot strengthen correction or inject changed bearings");
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, 0.0f, true);
    expect(std::abs(heading_change(rig.motion, corrected_heading)) < 0.000001,
           "duplicate timestamps cannot keep optical validity alive across a stale interval");
}

void optical_rejects_bad_samples_and_clock_order() {
    Rig rig;
    rig.settle();
    rig.motion.observe_optical_heading(rig.now, 1, 1, rig.now, 0.0f, true);
    uint32_t sequence = 1;
    const double initial_heading = camera_heading(rig.motion);
    // Each rejected identity must remain rejected if its fields are rewritten.
    for (unsigned fault = 0; fault < 4; ++fault) {
        for (unsigned i = 0; i < 4; ++i) rig.fresh(25000);
        const float bearing = fault == 0 ? std::numeric_limits<float>::quiet_NaN() : 1.0f;
        const uint32_t timestamp = fault == 1 ? rig.now - 150000u :
                                   fault == 2 ? rig.now + 1u : rig.now;
        rig.motion.observe_optical_heading(rig.now, 1, ++sequence, timestamp, bearing, fault != 3);
        for (unsigned i = 0; i < 4; ++i) {
            rig.fresh(25000);
            rig.motion.observe_optical_heading(rig.now, 1, sequence, rig.now, 1.0f, true);
        }
        expect(std::abs(heading_change(rig.motion, initial_heading)) < 0.000001,
               "nonfinite, stale, future or unavailable optical identities cannot later be relabelled valid");
    }
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, 0.0f, true);
    const uint32_t last_timestamp = rig.now;
    rig.fresh(25000);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, last_timestamp - 1u, 1.0f, true);
    rig.motion.observe_optical_heading(rig.now, 1, sequence - 1u, rig.now, 1.0f, true);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, last_timestamp, 1.0f, true);
    expect(std::abs(heading_change(rig.motion, initial_heading)) < 0.000001,
           "backwards clocks, out-of-order sequences and zero-time samples must not correct heading");
    // A subsequent ordered sample still contributes its own sensor-time
    // interval after the rejected observations.
    rig.fresh(25000);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now - 1u, 1.0f, true);
    expect(heading_change(rig.motion, initial_heading) < -0.02,
           "a later fresh unique optical sample must still correct after rejected clock order");
}

void optical_generation_and_motion_lifecycle_reanchor() {
    for (unsigned transition = 0; transition < 4; ++transition) {
        Rig rig;
        rig.settle();
        rig.motion.observe_optical_heading(rig.now, UINT32_MAX, 100, rig.now, 0.0f, true);
        for (unsigned i = 0; i < 4; ++i) rig.fresh(25000);
        rig.motion.observe_optical_heading(rig.now, UINT32_MAX, 101, rig.now, 1.0f, true);
        expect(heading_change(rig.motion, std::acos(-1.0) / 2.0) < -0.04,
               "the old optical reference must be active before lifecycle replacement");
        uint32_t optical_generation = UINT32_MAX;
        uint32_t sequence = 102;
        if (transition == 0) {
            optical_generation = 0;
            sequence = 0;
        } else {
            if (transition == 1) {
                rig.sample.gyro_valid = false;
                rig.fresh(0);
                rig.sample.gyro_valid = true;
            }
            if (transition == 2) rig.motion.reset();
            if (transition == 3) ++rig.generation;
            rig.settle();
        }
        const Quaternion before = normalized(rig.motion.quaternion());
        const double heading = camera_heading(rig.motion);
        rig.motion.observe_optical_heading(rig.now, optical_generation, sequence, rig.now, -0.8f, true);
        expect(normalized(rig.motion.quaternion()) == before,
               "optical generation, motion invalidation, reset and reconnect must anchor without an initial jump");
        for (unsigned i = 0; i < 40; ++i) {
            rig.fresh(25000);
            if (i % 4 == 3) {
                rig.motion.observe_optical_heading(rig.now, optical_generation, ++sequence, rig.now, -0.8f, true);
            }
        }
        expect(std::abs(heading_change(rig.motion, heading)) < 0.000002,
               "a replaced optical reference must not pull toward the preceding lifecycle's bearing");
    }
}

void optical_angles_and_sample_clocks_wrap() {
    Rig rig(UINT32_MAX - 1750000u);
    rig.settle();
    const float pi = std::acos(-1.0f);
    const float initial_bearing = pi - 0.02f;
    const double initial_heading = camera_heading(rig.motion);
    uint32_t sequence = UINT32_MAX - 1u;
    rig.motion.observe_optical_heading(rig.now, 1, sequence, rig.now, initial_bearing, true);
    // Matching physical yaw and optical motion cross both wrap boundaries.
    rig.sample.gyro_dps[2] -= 30.0f;
    rig.fresh(0);
    for (unsigned step = 1; step <= 20; ++step) {
        rig.fresh(25000);
        if (step % 4 == 0) {
            const float bearing = std::remainder(initial_bearing + step * 0.025f * pi / 6.0f, 2.0f * pi);
            rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, bearing, true);
        }
    }
    expect(std::abs(heading_change(rig.motion, initial_heading) + std::acos(-1.0) / 12.0) < 0.00002,
           "optical angle, sequence and microsecond wrap must preserve intentional yaw direction without a jump");
    rig.rest();
    rig.fresh(0);
    const float final_bearing = std::remainder(initial_bearing + pi / 12.0f + 0.1f, 2.0f * pi);
    for (unsigned step = 1; step <= 400; ++step) {
        rig.fresh(25000);
        if (step % 4 == 0) {
            rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, final_bearing, true);
        }
    }
    expect(std::abs(heading_change(rig.motion, initial_heading) + std::acos(-1.0) / 12.0 + 0.1) < 0.001,
           "wrapped optical observations must continue correcting toward the original relative anchor");
}

void optical_vertical_forward_defers_the_anchor() {
    Rig rig;
    rig.sample.accel_g[1] = 1.0f;
    rig.sample.accel_g[2] = 0.0f;
    rig.settle();
    const Quaternion vertical = normalized(rig.motion.quaternion());
    uint32_t sequence = 0;
    for (unsigned i = 0; i < 20; ++i) {
        rig.fresh(25000);
        rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, i * 0.1f, true);
    }
    expect(close(rotate(normalized(rig.motion.quaternion()), {1.0, 0.0, 0.0}),
                 rotate(vertical, {1.0, 0.0, 0.0})),
           "near-vertical camera forward must reject ill-conditioned optical yaw");
    // Tilt smoothly away from vertical with matching physical gravity.
    rig.sample.gyro_dps[0] -= 90.0f;
    rig.fresh(0);
    for (unsigned i = 1; i <= 40; ++i) {
        const float angle = i * std::acos(-1.0f) / 80.0f;
        rig.sample.accel_g[1] = std::cos(angle);
        rig.sample.accel_g[2] = std::sin(angle);
        rig.fresh(25000);
    }
    rig.rest();
    rig.fresh(0);
    const Quaternion horizontal = normalized(rig.motion.quaternion());
    const double heading = camera_heading(rig.motion);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, -0.6f, true);
    expect(normalized(rig.motion.quaternion()) == horizontal,
           "the first well-conditioned optical observation must establish the anchor without jumping");
    for (unsigned i = 0; i < 4; ++i) rig.fresh(25000);
    rig.motion.observe_optical_heading(rig.now, 1, ++sequence, rig.now, -0.3f, true);
    expect(heading_change(rig.motion, heading) < -0.01,
           "heading correction must recover after the forward projection leaves vertical");
}

}  // namespace

int main() {
    codec_wire_edges();
    codec_rejects_invalid_inputs();
    startup_needs_count_and_stillness();
    quantized_wii_bias_calibrates_and_integrates();
    measured_gravity_sets_reference();
    bias_corrected_body_rotation_reaches_wire();
    elapsed_time_not_packet_count_controls_rotation();
    moving_startup_does_not_calibrate();
    duplicate_sequences_and_interleaved_recovery();
    lifecycle_invalidates_bias_and_orientation();
    stale_boundaries_and_hidden_gaps();
    clocks_sequences_and_connections_wrap();
    fresh_gravity_bounds_warm_gyro_tilt_drift();
    dynamic_acceleration_does_not_steer_orientation();
    optical_heading_bounds_drift_without_polling_gain();
    optical_anchor_preserves_pose_and_corrects_only_world_yaw();
    optical_duplicates_and_hidden_intervals_do_not_gain_weight();
    optical_rejects_bad_samples_and_clock_order();
    optical_generation_and_motion_lifecycle_reanchor();
    optical_angles_and_sample_clocks_wrap();
    optical_vertical_forward_defers_the_anchor();
    return failures ? 1 : 0;
}
