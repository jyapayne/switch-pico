#include "input/wii_ir_tracker.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansToDegrees = 180.0f / kPi;
constexpr float kColdMinimumSpan = 24.0f;
constexpr float kTrackingMinimumSpan = 12.0f;
constexpr uint32_t kMaximumGapUs = 150000;
constexpr uint32_t kSingleMarkerUs = 80000;
constexpr float kAmbiguityMarginSquared = 64.0f;

float distance_squared(float ax, float ay, float bx, float by) {
    const float dx = ax - bx;
    const float dy = ay - by;
    return dx * dx + dy * dy;
}

float endpoint_gate(uint32_t elapsed_us) {
    return 12.0f + 0.006f * static_cast<float>(elapsed_us);
}

float lowpass_alpha(float cutoff, float dt) {
    const float scaled = 2.0f * kPi * cutoff * dt;
    return scaled / (1.0f + scaled);
}

bool unambiguous(float best, float runner_up) {
    // A close reflection is not evidence of which physical endpoint survived.
    return runner_up > best * 1.25f + kAmbiguityMarginSquared;
}
}  // namespace

WiiIrTracker::WiiIrTracker(WiiIrCameraModel model, bool relative_motion)
    : model_(model), relative_motion_(relative_motion) {
    model_valid_ = isfinite(model_.fx) && model_.fx > 0 &&
                   isfinite(model_.fy) && model_.fy > 0 &&
                   isfinite(model_.cx) && isfinite(model_.cy);
    if (model_valid_) {
        inverse_fx_ = 1.0f / model_.fx;
        inverse_fy_ = 1.0f / model_.fy;
        model_valid_ = isfinite(inverse_fx_) && inverse_fx_ > 0 &&
                       isfinite(inverse_fy_) && inverse_fy_ > 0;
    }
}

void WiiIrTracker::reset() {
    geometry_ = {};
    candidate_ = {};
    yaw_ = {};
    pitch_ = {};
    geometry_us_ = 0;
    full_pair_us_ = 0;
    candidate_us_ = 0;
    sample_us_ = 0;
    have_geometry_ = false;
    have_candidate_ = false;
    have_timestamp_ = false;
    output_valid_ = false;
    previous_count_ = 0;
    was_inferred_ = false;
}

WiiIrTrackingResult WiiIrTracker::lose() {
    output_valid_ = false;
    have_candidate_ = false;
    // Retain only previously accepted geometry, never an outlier or prediction.
    return {};
}

bool WiiIrTracker::make_pair(const Point& first, const Point& second,
                            uint8_t mask, float minimum_span,
                            bool gravity_valid, float gravity_x, float gravity_y,
                            Pair& pair) {
    const float span_squared = distance_squared(first.x, first.y, second.x, second.y);
    if (span_squared < minimum_span * minimum_span) return false;

    const float dx = second.nx - first.nx;
    const float dy = second.ny - first.ny;
    const float normalized_span_squared = dx * dx + dy * dy;
    if (!(normalized_span_squared > 0) || !isfinite(normalized_span_squared)) {
        return false;
    }
    if (gravity_valid) {
        const float dot = dx * gravity_x + dy * gravity_y;
        // Gravity is only a modulo-pi geometry prior. Never use it to reorder
        // an established pair, including when rolling through 90 degrees.
        if (dot * dot < 0.5f * normalized_span_squared) return false;
    }

    pair.first = first;
    pair.second = second;
    pair.pixel_span = sqrtf(span_squared);
    pair.mask = mask;
    return true;
}

bool WiiIrTracker::match_pair(const Point points[4], const uint8_t masks[4],
                             unsigned count, const Pair& reference,
                             float gate, float minimum_span,
                             bool gravity_valid, float gravity_x, float gravity_y,
                             Pair& matched) {
    const float gate_squared = gate * gate;
    const float span_gate = 0.30f * reference.pixel_span + 20.0f;
    float best = INFINITY;
    float runner_up = INFINITY;
    bool found = false;
    for (unsigned i = 0; i < count; ++i) {
        for (unsigned j = i + 1; j < count; ++j) {
            Pair pair;
            if (!make_pair(points[i], points[j], masks[i] | masks[j], minimum_span,
                           gravity_valid, gravity_x, gravity_y, pair) ||
                fabsf(pair.pixel_span - reference.pixel_span) > span_gate) {
                continue;
            }
            for (unsigned ordering = 0; ordering < 2; ++ordering) {
                const float first_distance = distance_squared(
                    pair.first.x, pair.first.y, reference.first.x, reference.first.y);
                const float second_distance = distance_squared(
                    pair.second.x, pair.second.y, reference.second.x, reference.second.y);
                if (first_distance <= gate_squared && second_distance <= gate_squared) {
                    const float score = first_distance + second_distance;
                    if (score < best) {
                        runner_up = best;
                        best = score;
                        matched = pair;
                        found = true;
                    } else if (score < runner_up) {
                        runner_up = score;
                    }
                }
                const Point temporary = pair.first;
                pair.first = pair.second;
                pair.second = temporary;
            }
        }
    }
    return found && unambiguous(best, runner_up);
}

float WiiIrTracker::filter_axis(AxisFilter& state, float radians, float dt,
                               float inverse_dt, float velocity_alpha, bool rebased) {
    if (rebased) {
        state.raw = radians;
        state.filtered = radians;
        state.velocity = 0;
        state.output = radians;
        return radians;
    }

    // The caller admits only strictly increasing timestamps. Estimate speed
    // from unfiltered angles so a fast intentional motion raises the cutoff
    // before the position low-pass would otherwise suppress it.
    const float velocity = (radians - state.raw) * inverse_dt;
    state.raw = radians;
    state.velocity += velocity_alpha * (velocity - state.velocity);
    const float cutoff = fminf(45.0f, 3.0f + 0.2f * fabsf(state.velocity) * kRadiansToDegrees);
    state.filtered += lowpass_alpha(cutoff, dt) * (radians - state.filtered);

    // A play operator, not a per-frame dead zone: slow sub-threshold movement
    // accumulates in the absolute filter state until it crosses the boundary.
    constexpr float hysteresis = 0.01f / kRadiansToDegrees;
    const float difference = state.filtered - state.output;
    if (difference > hysteresis) {
        state.output = state.filtered - hysteresis;
    } else if (difference < -hysteresis) {
        state.output = state.filtered + hysteresis;
    }
    return state.output;
}

WiiIrTrackingResult WiiIrTracker::accept(const Pair& pair, uint32_t now_us,
                                       bool inferred) {
    const float dx = pair.second.nx - pair.first.nx;
    const float dy = pair.second.ny - pair.first.ny;
    const float normalized_span = sqrtf(dx * dx + dy * dy);
    if (!(normalized_span > 0) || !isfinite(normalized_span)) return lose();
    const float inverse_span = 1.0f / normalized_span;
    if (!isfinite(inverse_span)) return lose();
    const float cosine = dx * inverse_span;
    const float sine = dy * inverse_span;
    const float mx = 0.5f * pair.first.nx + 0.5f * pair.second.nx;
    const float my = 0.5f * pair.first.ny + 0.5f * pair.second.ny;
    const float mx_rotated = mx * cosine + my * sine;
    const float my_rotated = -mx * sine + my * cosine;
    if (!isfinite(mx_rotated) || !isfinite(my_rotated)) return lose();
    const float yaw = -atan2f(mx_rotated, 1.0f);
    const float pitch = -atan2f(my_rotated, hypotf(1.0f, mx_rotated));
    const bool rebased = !output_valid_ || !have_geometry_ || (was_inferred_ && !inferred);
    const float dt = static_cast<float>(now_us - geometry_us_) * 0.000001f;
    const float inverse_dt = rebased ? 0.0f : 1.0f / dt;
    const float velocity_alpha = rebased ? 0.0f : lowpass_alpha(10.0f, dt);

    WiiIrTrackingResult result;
    result.tracked = true;
    result.rebased = rebased;
    result.inferred = inferred;
    result.pair_mask = pair.mask;
    result.yaw_radians = filter_axis(yaw_, yaw, dt, inverse_dt, velocity_alpha, rebased);
    result.pitch_radians = filter_axis(pitch_, pitch, dt, inverse_dt, velocity_alpha, rebased);
    result.range_in_bar_widths = inverse_span;

    geometry_ = pair;
    geometry_us_ = now_us;
    if (!inferred) full_pair_us_ = now_us;
    have_geometry_ = true;
    have_candidate_ = false;
    output_valid_ = true;
    was_inferred_ = inferred;
    return result;
}
WiiIrTrackingResult WiiIrTracker::follow_spots(
    const Point points[4], const uint8_t masks[4], unsigned count, uint32_t now_us) {
    struct Flow {
        float dx, dy, yaw, pitch;
        uint8_t mask;
    };
    Flow flows[4]{};
    unsigned matches = 0;
    const uint32_t elapsed = now_us - geometry_us_;
    if (output_valid_ && have_geometry_) {
        const float gate = endpoint_gate(elapsed);
        for (unsigned i = 0; i < count; ++i) {
            float best = INFINITY, runner_up = INFINITY;
            unsigned previous = 0;
            for (unsigned j = 0; j < previous_count_; ++j) {
                const float distance = distance_squared(
                    points[i].x, points[i].y, previous_points_[j].x, previous_points_[j].y);
                if (distance < best) {
                    runner_up = best;
                    best = distance;
                    previous = j;
                } else if (distance < runner_up) {
                    runner_up = distance;
                }
            }
            if (best > gate * gate || !unambiguous(best, runner_up)) continue;
            const Point& old = previous_points_[previous];
            float alternative = INFINITY;
            for (unsigned j = 0; j < count; ++j) {
                if (j == i) continue;
                alternative = fminf(alternative, distance_squared(
                    points[j].x, points[j].y, old.x, old.y));
            }
            // Mutual, unambiguous matches prevent two current reflections
            // from inheriting the same old spot, regardless of camera slot.
            if (!unambiguous(best, alternative)) continue;
            flows[matches++] = {
                points[i].x - old.x, points[i].y - old.y,
                -atan2f(points[i].nx - old.nx, 1.0f + points[i].nx * old.nx),
                -atan2f(points[i].ny - old.ny, 1.0f + points[i].ny * old.ny),
                masks[i]};
        }
    }

    unsigned best_count = 0;
    uint8_t best_set = 0;
    for (unsigned i = 0; i < matches; ++i) {
        const float tolerance = 3.0f + 0.15f * hypotf(flows[i].dx, flows[i].dy);
        unsigned inliers = 0;
        uint8_t set = 0;
        for (unsigned j = 0; j < matches; ++j) {
            if (distance_squared(flows[i].dx, flows[i].dy, flows[j].dx, flows[j].dy) <=
                tolerance * tolerance) {
                ++inliers;
                set |= static_cast<uint8_t>(1u << j);
            }
        }
        if (inliers > best_count) {
            best_count = inliers;
            best_set = set;
        }
    }
    // A lone continuous spot is sufficient. Conflicting matches need a
    // strict majority: never select one of two contradictory movements.
    const bool rebased = best_count == 0 || best_count <= matches / 2;
    float yaw = 0, pitch = 0;
    uint8_t used_mask = 0;
    if (!rebased) {
        for (unsigned i = 0; i < matches; ++i) {
            if (!(best_set & (1u << i))) continue;
            yaw += flows[i].yaw;
            pitch += flows[i].pitch;
            used_mask |= flows[i].mask;
        }
        yaw = yaw_.raw + yaw / best_count;
        pitch = pitch_.raw + pitch / best_count;
    }
    const float dt = static_cast<float>(elapsed) * 0.000001f;
    const float inverse_dt = rebased ? 0.0f : 1.0f / dt;
    const float velocity_alpha = rebased ? 0.0f : lowpass_alpha(10.0f, dt);
    WiiIrTrackingResult result;
    result.tracked = true;
    result.rebased = rebased;
    result.pair_mask = used_mask;
    result.yaw_radians = filter_axis(yaw_, yaw, dt, inverse_dt, velocity_alpha, rebased);
    result.pitch_radians = filter_axis(pitch_, pitch, dt, inverse_dt, velocity_alpha, rebased);
    for (unsigned i = 0; i < count; ++i) previous_points_[i] = points[i];
    previous_count_ = count;
    geometry_us_ = now_us;
    have_geometry_ = true;
    output_valid_ = true;
    return result;
}


WiiIrTrackingResult WiiIrTracker::update(const uint16_t x[4], const uint16_t y[4],
                                       uint8_t valid_mask, uint32_t now_us,
                                       float gravity_roll_radians, bool gravity_valid) {
    if (!model_valid_ || !x || !y) return lose();
    if (have_timestamp_) {
        const uint32_t elapsed = now_us - sample_us_;
        // Unsigned subtraction handles timer wrap. Duplicate or backwards
        // samples cannot advance geometry, cold acquisition, or the filters.
        if (elapsed == 0 || elapsed > 0x7fffffffu) return lose();
    }
    sample_us_ = now_us;
    have_timestamp_ = true;
    if (have_geometry_ && now_us - geometry_us_ > kMaximumGapUs) {
        have_geometry_ = false;
        have_candidate_ = false;
        output_valid_ = false;
    }

    Point points[4];
    uint8_t masks[4];
    unsigned count = 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (!(valid_mask & (1u << i)) || x[i] > 1023 || y[i] > 767) continue;
        Point point;
        point.x = static_cast<float>(x[i]);
        point.y = static_cast<float>(y[i]);
        point.nx = (point.x - model_.cx) * inverse_fx_;
        point.ny = (point.y - model_.cy) * inverse_fy_;
        if (!isfinite(point.nx) || !isfinite(point.ny)) continue;
        points[count] = point;
        masks[count] = static_cast<uint8_t>(1u << i);
        ++count;
    }
    if (count == 0) return lose();
    if (relative_motion_) return follow_spots(points, masks, count, now_us);

    // The caller gates acceleration magnitude and projection. Invalid dynamic
    // acceleration (or a nonfinite angle) must never switch off optical tracking.
    gravity_valid = count >= 2 && gravity_valid && isfinite(gravity_roll_radians);
    const float gravity_x = gravity_valid ? cosf(gravity_roll_radians) : 0.0f;
    const float gravity_y = gravity_valid ? sinf(gravity_roll_radians) : 0.0f;

    if (!have_geometry_) {
        // With no temporal reference, three or four points do not identify the
        // physical bar reliably. Wait rather than guess a plausible reflection.
        if (count != 2) return lose();
        if (have_candidate_ && now_us - candidate_us_ <= kMaximumGapUs) {
            Pair matched;
            if (match_pair(points, masks, count, candidate_, endpoint_gate(now_us - candidate_us_),
                           kColdMinimumSpan, gravity_valid, gravity_x, gravity_y, matched)) {
                return accept(matched, now_us, false);
            }
        }
        Pair pair;
        if (!make_pair(points[0], points[1], masks[0] | masks[1], kColdMinimumSpan,
                       gravity_valid, gravity_x, gravity_y, pair)) {
            return lose();
        }

        const float dx = pair.second.nx - pair.first.nx;
        const float dy = pair.second.ny - pair.first.ny;
        const bool reverse = gravity_valid ? dx * gravity_x + dy * gravity_y < 0 :
                                             (dx < 0 || (dx == 0 && dy < 0));
        if (reverse) {
            const Point temporary = pair.first;
            pair.first = pair.second;
            pair.second = temporary;
        }
        candidate_ = pair;
        candidate_us_ = now_us;
        have_candidate_ = true;
        output_valid_ = false;
        return {};
    }

    const float gate = endpoint_gate(now_us - geometry_us_);
    if (count >= 2) {
        Pair matched;
        if (!match_pair(points, masks, count, geometry_, gate, kTrackingMinimumSpan,
                        gravity_valid, gravity_x, gravity_y, matched)) {
            return lose();
        }
        return accept(matched, now_us, false);
    }

    if (now_us - full_pair_us_ > kSingleMarkerUs) return lose();
    const float first_distance = distance_squared(
        points[0].x, points[0].y, geometry_.first.x, geometry_.first.y);
    const float second_distance = distance_squared(
        points[0].x, points[0].y, geometry_.second.x, geometry_.second.y);
    const bool first_visible = first_distance < second_distance;
    const float best = first_visible ? first_distance : second_distance;
    const float runner_up = first_visible ? second_distance : first_distance;
    if (best > gate * gate || !unambiguous(best, runner_up)) return lose();

    // Translate the last accepted full-pair vector without changing its span or
    // rotation. Repeated inferred frames do NOT extend the full-pair deadline.
    Pair inferred = geometry_;
    const Point& previous = first_visible ? geometry_.first : geometry_.second;
    const float delta_x = points[0].x - previous.x;
    const float delta_y = points[0].y - previous.y;
    const float delta_nx = delta_x * inverse_fx_;
    const float delta_ny = delta_y * inverse_fy_;
    inferred.first.x += delta_x;
    inferred.first.y += delta_y;
    inferred.first.nx += delta_nx;
    inferred.first.ny += delta_ny;
    inferred.second.x += delta_x;
    inferred.second.y += delta_y;
    inferred.second.nx += delta_nx;
    inferred.second.ny += delta_ny;
    inferred.mask = masks[0];
    return accept(inferred, now_us, true);
}
