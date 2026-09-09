#include "input/wii_ir_tracker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        ++failures;
    }
}

bool near(float actual, float expected, float tolerance) {
    return std::isfinite(actual) && std::fabs(actual - expected) <= tolerance;
}

struct Frame {
    std::array<uint16_t, 4> x{};
    std::array<uint16_t, 4> y{};
    uint8_t mask = 0;
};

Frame pair(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by) {
    return {{ax, bx, 0, 0}, {ay, by, 0, 0}, 0x03};
}

struct Camera {
    WiiIrTracker tracker;
    uint32_t now;

    explicit Camera(WiiIrCameraModel model = {}, uint32_t start = 100000,
                    bool relative_motion = false)
        : tracker(model, relative_motion), now(start) {}

    WiiIrTrackingResult update(const Frame& frame, uint32_t elapsed = 10000,
                               float gravity_roll = 0, bool gravity_valid = false) {
        now += elapsed;
        return tracker.update(frame.x.data(), frame.y.data(), frame.mask, now,
                              gravity_roll, gravity_valid);
    }

    WiiIrTrackingResult acquire(const Frame& frame, float gravity_roll = 0,
                                bool gravity_valid = false) {
        expect(!update(frame, 10000, gravity_roll, gravity_valid).tracked,
               "one cold sample must not start tracking");
        const auto result = update(frame, 10000, gravity_roll, gravity_valid);
        expect(result.tracked && result.rebased && !result.inferred,
               "two consistent full-pair samples must acquire with a rebase");
        return result;
    }

    WiiIrTrackingResult settle(const Frame& frame) {
        WiiIrTrackingResult result;
        for (int i = 0; i < 40; ++i) {
            result = update(frame);
            expect(result.tracked && !result.rebased,
                   "an unchanged acquired pair must remain continuously tracked");
        }
        return result;
    }
};

void test_slot_permutation_and_far_reflection() {
    Camera camera;
    const auto original = pair(420, 350, 620, 390);
    camera.acquire(original);
    const auto baseline = camera.settle(original);

    // Swap the endpoint slots, then move them to different slots. The final
    // frame also has a distant reflection that must not replace either end.
    const std::array<Frame, 3> frames{{
        pair(620, 390, 420, 350),
        {{620, 0, 420, 0}, {390, 0, 350, 0}, 0x05},
        {{620, 30, 420, 0}, {390, 700, 350, 0}, 0x07},
    }};
    const std::array<uint8_t, 3> selected_masks{{0x03, 0x05, 0x05}};
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto result = camera.update(frames[i]);
        expect(result.tracked && !result.rebased && !result.inferred,
               "slot changes and a far reflection must not interrupt a matched pair");
        expect(result.pair_mask == selected_masks[i],
               "the selected pair must exclude the reflection and identify current slots");
        expect(near(result.yaw_radians, baseline.yaw_radians, 0.001f) &&
                   near(result.pitch_radians, baseline.pitch_radians, 0.001f),
               "unchanged physical spots must retain their bearings after slot changes");
        expect(near(result.range_in_bar_widths, baseline.range_in_bar_widths, 0.02f),
               "a reflection must not corrupt the tracked range");
    }
}

void test_single_marker_expires_despite_continued_reports() {
    Camera camera;
    const auto full = pair(400, 384, 624, 384);
    camera.acquire(full);
    const auto baseline = camera.settle(full);
    WiiIrTrackingResult result;
    for (int i = 1; i <= 7; ++i) {
        // The surviving endpoint also changes slot from the full-pair frame.
        const Frame single{{0, 0, 0, static_cast<uint16_t>(400 + 2 * i)},
                           {0, 0, 0, 384}, 0x08};
        result = camera.update(single);
        expect(result.tracked && result.inferred && !result.rebased,
               "an unambiguous moving endpoint must track briefly using inferred geometry");
        expect(result.pair_mask == 0x08,
               "inferred tracking must identify only the currently visible endpoint");
    }
    expect(result.yaw_radians < baseline.yaw_radians - 0.0005f,
           "single-marker inference must produce the surviving endpoint's motion");

    const Frame single{{0, 0, 0, 418}, {0, 0, 0, 384}, 0x08};
    expect(!camera.update(single, 20000).tracked,
           "single-marker inference must expire after 80ms from the last full pair");
    for (int i = 0; i < 4; ++i) {
        expect(!camera.update(single).tracked,
               "continued single-marker reports must not restart the inference lifetime");
    }
}

void test_full_pair_return_rebases_inference_correction() {
    Camera camera;
    camera.acquire(pair(400, 384, 624, 384));
    const Frame single{{408, 0, 0, 0}, {384, 0, 0, 0}, 0x01};
    const auto inferred = camera.update(single);
    expect(inferred.tracked && inferred.inferred,
           "a briefly missing endpoint must enter inferred tracking");

    // Inference placed the missing endpoint at x=632. Its real return corrects
    // that assumption, which must not become an emitted aiming delta.
    const auto returned = camera.update(pair(408, 384, 650, 384));
    expect(returned.tracked && !returned.inferred && returned.rebased,
           "a correcting full-pair return must rebase rather than emit an inference jump");
    expect(camera.update(pair(408, 384, 650, 384)).tracked,
           "the returned full pair must remain usable after the correction");
}

void test_zero_marker_loss_and_reacquisition() {
    Camera camera;
    const auto original = pair(400, 350, 620, 350);
    camera.acquire(original);
    camera.settle(original);
    expect(!camera.update(Frame{}).tracked,
           "zero visible markers must stop tracking on the first empty frame");

    const auto returned = pair(430, 350, 650, 350);
    auto result = camera.update(returned);
    if (!result.tracked) {
        result = camera.update(returned);
    }
    expect(result.tracked && result.rebased && !result.inferred,
           "the first tracked pair after loss must rebase, not bridge the missing interval");
    result = camera.update(returned);
    expect(result.tracked && !result.rebased,
           "reacquisition must resume continuous tracking after the initial rebase");
}

void test_configured_pinhole_bearings_and_range() {
    const WiiIrCameraModel model{1000.0f, 800.0f, 480.0f, 360.0f};
    Camera camera(model);
    const auto frame = pair(480, 440, 680, 560);
    camera.acquire(frame);
    const auto result = camera.settle(frame);

    // Normalized endpoints (0,.1), (.2,.25) have span .25. Rotating their
    // midpoint (.1,.175) into the bar frame gives (.185,.08).
    const float expected_yaw = -std::atan2(0.185f, 1.0f);
    const float expected_pitch = -std::atan2(0.08f, std::sqrt(1.0f + 0.185f * 0.185f));
    expect(near(result.yaw_radians, expected_yaw, 0.001f),
           "settled yaw must use the configured center and normalized bar-frame bearing");
    expect(near(result.pitch_radians, expected_pitch, 0.001f),
           "settled pitch must use independent vertical focal length and spherical bearing");
    expect(near(result.range_in_bar_widths, 4.0f, 0.02f),
           "range must be reciprocal normalized span, in bar widths rather than pixels");
}

void test_jitter_attenuation_preserves_slow_motion() {
    Camera camera(WiiIrCameraModel{1000.0f, 1000.0f, 512.0f, 384.0f});
    const auto stationary = pair(412, 384, 612, 384);
    camera.acquire(stationary);
    const auto baseline = camera.settle(stationary);
    float peak_jitter = 0.0f;
    for (int i = 0; i < 64; ++i) {
        // One-pixel endpoint noise gives half-pixel midpoint noise even though
        // the public camera coordinates are integers.
        const auto jitter = pair(static_cast<uint16_t>(i % 2 == 0 ? 411 : 413),
                                 384, 612, 384);
        const auto result = camera.update(jitter, 5000);
        expect(result.tracked && !result.rebased,
               "subpixel midpoint jitter must not cause tracking loss or rebases");
        peak_jitter = std::max(peak_jitter, std::fabs(result.yaw_radians - baseline.yaw_radians));
    }
    const float raw_jitter = std::atan2(0.5f, 1000.0f);
    expect(peak_jitter < raw_jitter * 0.6f,
           "high-frequency subpixel midpoint jitter must be substantially attenuated");

    const auto resting = camera.settle(stationary);
    float previous_yaw = resting.yaw_radians;
    for (int step = 1; step <= 16; ++step) {
        const auto moving = pair(static_cast<uint16_t>(412 - step), 384, 612, 384);
        for (int hold = 0; hold < 8; ++hold) {
            const auto result = camera.update(moving);
            expect(result.tracked && !result.rebased,
                   "slow accumulated movement must remain continuously tracked");
            expect(result.yaw_radians >= previous_yaw - 0.00001f,
                   "monotonic slow motion must not cause filter-driven direction reversals");
            previous_yaw = result.yaw_radians;
        }
    }
    expect(previous_yaw > resting.yaw_radians + 0.006f,
           "half-pixel motion steps must accumulate instead of dying in a per-frame dead zone");
}

void test_optional_gravity_rejects_wrong_pair() {
    const auto vertical = pair(512, 260, 512, 500);
    Camera constrained;
    for (int i = 0; i < 3; ++i) {
        expect(!constrained.update(vertical, 10000, 0.0f, true).tracked,
               "a vertical pair must not acquire against a valid horizontal gravity prior");
    }

    Camera unconstrained;
    unconstrained.acquire(vertical);
    const auto free_result = unconstrained.update(vertical, 10000, 0.0f, false);
    expect(free_result.tracked && !free_result.rebased,
           "the same vertical geometry must remain usable without a valid gravity prior");

    Camera dynamic;
    const auto horizontal = pair(400, 384, 624, 384);
    dynamic.acquire(horizontal, 0.0f, true);
    const auto result = dynamic.update(pair(410, 384, 634, 384), 10000, 1.5707963f, false);
    expect(result.tracked && !result.rebased,
           "an invalid dynamic gravity hint must not disable otherwise continuous IR tracking");
}

Frame rolled_pair(float radians, const WiiIrCameraModel& model) {
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    Frame frame;
    for (size_t i = 0; i < 2; ++i) {
        const float bar_x = i == 0 ? -0.02f : 0.18f;
        const float bar_y = 0.04f;
        const float camera_x = bar_x * cosine - bar_y * sine;
        const float camera_y = bar_x * sine + bar_y * cosine;
        frame.x[i] = static_cast<uint16_t>(std::lround(model.cx + model.fx * camera_x));
        frame.y[i] = static_cast<uint16_t>(std::lround(model.cy + model.fy * camera_y));
    }
    frame.mask = 0x03;
    return frame;
}

void test_roll_through_vertical_preserves_endpoint_order() {
    const WiiIrCameraModel model{1000.0f, 800.0f, 512.0f, 384.0f};
    Camera camera(model);
    const auto initial = rolled_pair(0.0f, model);
    camera.acquire(initial);
    const auto baseline = camera.settle(initial);
    for (int degrees = 5; degrees <= 120; degrees += 5) {
        const float radians = static_cast<float>(degrees) * 0.01745329252f;
        const auto result = camera.update(rolled_pair(radians, model));
        expect(result.tracked && !result.rebased && !result.inferred,
               "rolling a matched pair through vertical must preserve continuous tracking");
        expect(near(result.yaw_radians, baseline.yaw_radians, 0.0025f) &&
                   near(result.pitch_radians, baseline.pitch_radians, 0.0025f),
               "passing 90 degrees of roll must not flip endpoint order or invert bearings");
        expect(near(result.range_in_bar_widths, 5.0f, 0.08f),
               "roll with unequal focal lengths must preserve normalized pair range");
    }
}

void test_timestamp_wrap_preserves_continuous_motion() {
    Camera ordinary({}, 100000);
    Camera wrapping({}, std::numeric_limits<uint32_t>::max() - 50000u);
    const auto initial = pair(400, 384, 624, 384);
    ordinary.acquire(initial);
    const auto baseline = wrapping.acquire(initial);
    float previous_yaw = baseline.yaw_radians;
    for (int step = 1; step <= 16; ++step) {
        const auto frame = pair(static_cast<uint16_t>(400 - 3 * step), 384,
                                static_cast<uint16_t>(624 - 3 * step), 384);
        const auto reference = ordinary.update(frame);
        const auto result = wrapping.update(frame);
        expect(result.tracked && !result.rebased,
               "uint32 timestamp wrap must not interrupt continuous pair motion");
        expect(near(result.yaw_radians, reference.yaw_radians, 0.00001f) &&
                   near(result.pitch_radians, reference.pitch_radians, 0.00001f),
               "wrapped and ordinary clocks must yield the same time-filtered bearings");
        expect(std::fabs(result.yaw_radians - previous_yaw) < 0.02f,
               "timestamp wrap must not create an angular discontinuity");
        previous_yaw = result.yaw_radians;
    }
    expect(previous_yaw > baseline.yaw_radians + 0.015f,
           "motion across timestamp wrap must advance the bearing rather than freeze it");
}

void test_relative_tracking_without_recognizable_bar() {
    const WiiIrCameraModel model{1000.0f, 1000.0f, 512.0f, 384.0f};
    // One endpoint alone, and the four-spot/reflection layout captured on
    // hardware. Neither provides a uniquely identifiable full sensor bar.
    for (unsigned count : {1u, 4u}) {
        Camera camera(model, 100000, true);
        const Frame scene{{184, 193, 687, 739}, {623, 536, 453, 549}, 15};
        WiiIrTrackingResult result;
        for (unsigned step = 0; step <= 160; ++step) {
            Frame frame;
            const unsigned travel = std::min(step, 80u);
            for (unsigned i = 0; i < count; ++i) {
                const unsigned slot = (i + step) % 4;
                frame.x[slot] = scene.x[i] + travel;
                frame.y[slot] = scene.y[i];
                frame.mask |= 1u << slot;
            }
            result = camera.update(frame, 10000, 0.0f, true);
            expect(result.tracked && (step == 0 || !result.rebased),
                   "persistent spots must retain motion beyond 80ms without a full bar");
        }
        float expected = 0;
        for (unsigned i = 0; i < count; ++i) {
            expected -= std::atan((scene.x[i] + 80 - model.cx) / model.fx) -
                        std::atan((scene.x[i] - model.cx) / model.fx);
        }
        expected /= count;
        expect(near(result.yaw_radians, expected, 0.001f),
               "single/multiple-spot motion must retain the measured angular travel");
        expect(near(result.pitch_radians, 0, 0.0001f),
               "slot changes and extra spots must not create vertical motion");
    }
}

void test_relative_loss_and_conflicting_matches_rebase() {
    Camera camera({}, 100000, true);
    const Frame first{{400, 0, 0, 0}, {384, 0, 0, 0}, 1};
    camera.update(first);
    camera.update(Frame{{410, 0, 0, 0}, {384, 0, 0, 0}, 1});
    expect(!camera.update(Frame{}).tracked,
           "relative aiming must stop immediately when every spot disappears");
    auto result = camera.update(Frame{{700, 0, 0, 0}, {384, 0, 0, 0}, 1});
    expect(result.tracked && result.rebased,
           "reacquisition must discard movement across the missing interval");
    result = camera.update(first);
    expect(result.rebased, "an implausible reflection jump must not generate aiming");

    Camera conflicting({}, 100000, true);
    conflicting.update(pair(400, 384, 600, 384));
    result = conflicting.update(pair(410, 384, 590, 384));
    expect(result.rebased,
           "two contradictory matches must not choose an arbitrary aiming direction");
    result = conflicting.update(pair(410, 384, 590, 384));
    expect(result.tracked && !result.rebased && near(result.yaw_radians, 0, 0.0001f),
           "settled observations must resume without replaying rejected movement");
}

}  // namespace

int main() {
    test_slot_permutation_and_far_reflection();
    test_single_marker_expires_despite_continued_reports();
    test_full_pair_return_rebases_inference_correction();
    test_zero_marker_loss_and_reacquisition();
    test_configured_pinhole_bearings_and_range();
    test_jitter_attenuation_preserves_slow_motion();
    test_optional_gravity_rejects_wrong_pair();
    test_roll_through_vertical_preserves_endpoint_order();
    test_timestamp_wrap_preserves_continuous_motion();
    test_relative_tracking_without_recognizable_bar();
    test_relative_loss_and_conflicting_matches_rebase();
    if (failures != 0) {
        std::cerr << failures << " Wii IR tracker test(s) failed\n";
        return 1;
    }
    return 0;
}
