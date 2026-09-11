#pragma once

#include <stdint.h>

// Nominal WiiBrew field of view (33 degrees horizontal, 23 vertical), not
// calibration of the attached camera. Coordinates are the native 1024 x 768.
struct WiiIrCameraModel {
    float fx = 1726.7951f;
    float fy = 1884.9627f;
    float cx = 511.5f;
    float cy = 383.5f;
};

struct WiiIrTrackingResult {
    bool tracked = false;
    bool rebased = false;
    bool inferred = false;
    // Supporting spots used for the tracked measurement.
    uint8_t pair_mask = 0;
    float yaw_radians = 0;
    float pitch_radians = 0;
    // Reciprocal normalized image span: a frontal bar-width proxy, not meters.
    float range_in_bar_widths = 0;
};

// Caller owns synchronization. All history is bounded; no allocation or gyro
// measurements are used. A rebased result must not generate an angular delta.
class WiiIrTracker {
public:
    // Relative motion follows persistent spots without identifying bar ends.
    // Its bearings are accumulated in the camera frame; range is unavailable.
    explicit WiiIrTracker(WiiIrCameraModel model = {}, bool relative_motion = false);
    void reset();
    WiiIrTrackingResult update(const uint16_t x[4], const uint16_t y[4],
                               uint8_t valid_mask, uint32_t now_us,
                               float gravity_roll_radians = 0,
                               bool gravity_valid = false);

private:
    struct Point {
        float x = 0;
        float y = 0;
        float nx = 0;
        float ny = 0;
    };
    struct Pair {
        Point first{};
        Point second{};
        float pixel_span = 0;
        uint8_t mask = 0;
    };
    struct AxisFilter {
        float raw = 0;
        float filtered = 0;
        float velocity = 0;
        float output = 0;
    };

    static bool make_pair(const Point& first, const Point& second,
                          uint8_t mask, float minimum_span,
                          bool gravity_valid, float gravity_x, float gravity_y,
                          Pair& pair);
    static bool match_pair(const Point points[4], const uint8_t masks[4],
                           unsigned count, const Pair& reference,
                           float endpoint_gate, float minimum_span,
                           bool gravity_valid, float gravity_x, float gravity_y,
                           Pair& matched);
    static float filter_axis(AxisFilter& state, float radians, float dt,
                             float inverse_dt, float velocity_alpha, bool rebased);
    WiiIrTrackingResult accept(const Pair& pair, uint32_t now_us, bool inferred);
    WiiIrTrackingResult lose();
    WiiIrTrackingResult follow_spots(const Point points[4], const uint8_t masks[4],
                                    unsigned count, uint32_t now_us);

    WiiIrCameraModel model_;
    float inverse_fx_ = 0;
    float inverse_fy_ = 0;
    bool model_valid_ = false;
    bool relative_motion_ = false;
    Point previous_points_[4]{};
    unsigned previous_count_ = 0;
    Pair geometry_{};
    Pair candidate_{};
    AxisFilter yaw_{};
    AxisFilter pitch_{};
    uint32_t geometry_us_ = 0;
    uint32_t full_pair_us_ = 0;
    uint32_t candidate_us_ = 0;
    uint32_t sample_us_ = 0;
    bool have_geometry_ = false;
    bool have_candidate_ = false;
    bool have_timestamp_ = false;
    bool output_valid_ = false;
    bool was_inferred_ = false;
};
