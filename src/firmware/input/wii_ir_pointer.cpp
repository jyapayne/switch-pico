#include "input/wii_ir_pointer.h"
#include "input/wii_ir_tracker.h"

#include <math.h>
#include <string.h>
#include "pico/critical_section.h"
#include "pico/time.h"

namespace {
constexpr uint32_t kStaleUs = 150000;
#ifndef SWITCH_PICO_WII_IR_GYRO
constexpr int32_t kMaximumPendingQ8 = 4096 * 256;
#endif
constexpr uint16_t kButtonA = 0x0008;
constexpr uint16_t kButtonB = 0x0004;
constexpr uint16_t kButtonOne = 0x0002;
constexpr WiiIrCameraModel kCamera{};
#ifdef SWITCH_PICO_WII_IR_GYRO
constexpr float kDegreesPerRadian = 57.295779513f;
WiiIrTracker g_tracker{kCamera, true};
#else
WiiIrTracker g_tracker{kCamera};
#endif
critical_section_t g_lock;
bool g_initialized;
uint8_t g_owner = 0xff;
uint32_t g_connection_generation;
uint32_t g_generation;
uint32_t g_sequence;
uint32_t g_received_us;
uint32_t g_samples;
uint32_t g_sent;
uint32_t g_rebaselines;
uint16_t g_x[4], g_y[4];
uint16_t g_raw_buttons;
uint8_t g_valid_mask;
uint8_t g_buttons;
uint8_t g_sent_buttons;
bool g_have_sample;
bool g_tracking;
bool g_clutch;
int32_t g_filtered_x_q8, g_filtered_y_q8;
int32_t g_pending_x_q8, g_pending_y_q8;

#ifdef SWITCH_PICO_WII_IR_GYRO
constexpr uint32_t kOutputStallUs = 100000;
// Angular position is Q8 at 50 units/degree (12800 units/degree). Spread
// displacement over three 5 ms samples: raw = angular_units * 4457/59904.
constexpr int32_t kGyroNumerator = 4457;
constexpr int32_t kGyroDenominator = 59904;
constexpr int32_t kMaximumGyro = 720 * 13371 / 936;
bool g_infrared;
bool g_motion_enabled;
bool g_output_armed;
bool g_have_gyro_send;
uint32_t g_output_progress_us;
uint32_t g_gyro_sent_us;
uint32_t g_gyro_sent;
int16_t g_sent_gyro_z, g_sent_gyro_y;
ControllerMotionSample g_real_motion{};
// Fractional Q8 units, denominator kGyroNumerator. These retain exactly the
// displacement represented by integral raw gyro values across USB reports.
int32_t g_fraction_x, g_fraction_y;
#endif

int32_t clamp(int32_t value, int32_t limit) {
    return value < -limit ? -limit : (value > limit ? limit : value);
}

void stop_tracking(bool reset_model = true) {
    if (reset_model) g_tracker.reset();
    bool pending = g_pending_x_q8 || g_pending_y_q8;
#ifdef SWITCH_PICO_WII_IR_GYRO
    pending = pending || g_fraction_x || g_fraction_y;
    g_fraction_x = 0;
    g_fraction_y = 0;
#endif
    if (g_tracking || pending) {
        ++g_generation;
        ++g_rebaselines;
    }
    g_tracking = false;
    g_pending_x_q8 = 0;
    g_pending_y_q8 = 0;
}

#ifdef SWITCH_PICO_WII_IR_GYRO
void reset_gyro_source() {
    g_infrared = false;
    g_output_armed = false;
}

void expire_output(uint32_t now) {
    if (g_infrared && g_output_armed &&
        static_cast<int32_t>(now - g_output_progress_us) >=
            static_cast<int32_t>(kOutputStallUs)) {
        stop_tracking();
        ++g_generation;
        g_output_armed = false;
    }
}

void set_pending(int32_t& pending, int32_t& fraction, int64_t value) {
    // Bound backlog to one report, without clipping ordinary fast sweeps at
    // the old 180 deg/s limit. Association rejects outliers before this stage.
    constexpr int64_t maximum = static_cast<int64_t>(kMaximumGyro) * kGyroDenominator;
    if (value > maximum) value = maximum;
    if (value < -maximum) value = -maximum;
    pending = static_cast<int32_t>(value / kGyroNumerator);
    fraction = static_cast<int32_t>(value % kGyroNumerator);
}

int32_t pending_gyro(int32_t pending, int32_t fraction) {
    return (pending * kGyroNumerator + fraction) / kGyroDenominator;
}

int32_t consumed_gyro(int32_t consumed_q8) {
    // The integer Q8 displacement uniquely identifies its raw gyro count:
    // one raw count spans >13 Q8 units, so recover it by nearest rounding.
    int32_t scaled = consumed_q8 * kGyroNumerator;
    scaled += scaled < 0 ? -kGyroDenominator / 2 : kGyroDenominator / 2;
    return scaled / kGyroDenominator;
}
#endif

void expire(uint32_t now) {
    // A USB scheduling timestamp can precede a newer Core 1 sample observed
    // before this lock was acquired. A small negative age is not a timeout.
    if (g_have_sample && static_cast<int32_t>(now - g_received_us) >=
                             static_cast<int32_t>(kStaleUs)) {
        stop_tracking();
        g_buttons = 0;
        // Losing IR must not silently select physical gyro. Keep the chosen
        // source and emit zero until a fresh tracking baseline is available.
    }
}

void put16(uint8_t* p, uint16_t value) {
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
}
void put32(uint8_t* p, uint32_t value) {
    put16(p, static_cast<uint16_t>(value));
    put16(p + 2, static_cast<uint16_t>(value >> 16));
}

bool claim_owner(uint8_t slot, uint32_t connection_generation) {
    if (g_owner != 0xff && g_owner != slot) return false;
    if (g_owner != slot || g_connection_generation != connection_generation) {
        stop_tracking();
        ++g_generation;
        g_owner = slot;
        g_connection_generation = connection_generation;
        g_have_sample = false;
#ifdef SWITCH_PICO_WII_IR_GYRO
        reset_gyro_source();
        g_motion_enabled = false;
        g_real_motion = {};
#endif
    }
    return true;
}

// Caller holds g_lock. Both public diagnostics use this directly rather than
// nesting public functions on the same non-recursive striped spinlock.
void pointer_diagnostics(uint8_t* data, uint32_t now) {
    data[0] = 1;
    data[1] = g_owner;
    data[2] = g_valid_mask;
    data[3] = (g_tracking ? 1 : 0) | (g_clutch ? 2 : 0) |
              (g_have_sample && now - g_received_us < kStaleUs ? 4 : 0);
    put32(data + 4, g_sequence);
    put32(data + 8, g_samples);
    put32(data + 12, g_sent);
    for (unsigned i = 0; i < 4; ++i) {
        put16(data + 16 + 4 * i, g_x[i]);
        put16(data + 18 + 4 * i, g_y[i]);
    }
    put16(data + 32, g_raw_buttons);
    data[34] = g_buttons;
    data[35] = g_sent_buttons;
    put32(data + 36, g_have_sample ? now - g_received_us : UINT32_MAX);
    put32(data + 40, g_rebaselines);
    put16(data + 44, static_cast<uint16_t>(g_pending_x_q8 / 256));
    put16(data + 46, static_cast<uint16_t>(g_pending_y_q8 / 256));
}
}  // namespace

void wii_ir_pointer_init() {
    if (g_initialized) return;
    // The backend holds an exclusive lock while calling us. Do not consume
    // another scarce exclusive lock or nest any striped lock inside this one.
    critical_section_init_with_lock_num(&g_lock, next_striped_spin_lock_num());
    g_initialized = true;
}

void wii_ir_pointer_reset() {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    stop_tracking();
    ++g_generation;
    g_buttons = 0;
    g_owner = 0xff;
    // Force an explicit release after reset if the host saw a held button.
    g_have_sample = false;
    g_valid_mask = 0;
#ifdef SWITCH_PICO_WII_IR_GYRO
    reset_gyro_source();
    g_motion_enabled = false;
    g_real_motion = {};
#endif
    critical_section_exit(&g_lock);
}

void wii_ir_pointer_disconnect(uint8_t slot) {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    if (g_owner == slot) {
        stop_tracking();
        ++g_generation;
        g_owner = 0xff;
        g_buttons = 0;
        g_have_sample = false;
        g_valid_mask = 0;
#ifdef SWITCH_PICO_WII_IR_GYRO
        reset_gyro_source();
        g_motion_enabled = false;
        g_real_motion = {};
#endif
    }
    critical_section_exit(&g_lock);
}

void wii_ir_pointer_observe(uint8_t slot, uint32_t connection_generation,
                          uint32_t sequence, uint16_t buttons,
                          const uint16_t x[4], const uint16_t y[4],
                          uint8_t valid_mask, bool nunchuk_c) {
    if (!g_initialized) return;
    const uint32_t now = time_us_32();
    critical_section_enter_blocking(&g_lock);
    if (!claim_owner(slot, connection_generation)) {
        critical_section_exit(&g_lock);
        return;
    }
    if (g_have_sample && sequence == g_sequence) {
        critical_section_exit(&g_lock);
        return;
    }
    expire(now);
#ifdef SWITCH_PICO_WII_IR_GYRO
    expire_output(now);
#endif
    g_have_sample = true;
    g_sequence = sequence;
    g_received_us = now;
    ++g_samples;
    memcpy(g_x, x, sizeof(g_x));
    memcpy(g_y, y, sizeof(g_y));
    g_raw_buttons = buttons;
    g_valid_mask = valid_mask & 0x0f;
    g_buttons = ((buttons & kButtonA) ? 1 : 0) | ((buttons & kButtonB) ? 2 : 0);
    g_clutch = (buttons & kButtonOne) != 0;
#ifdef SWITCH_PICO_WII_IR_GYRO
    // C + 1 is the reposition chord. Keep 1 + 2 motionless while the
    // separate source-switch gesture is held; 1 alone remains a game button.
    g_clutch = g_clutch && (nunchuk_c || (buttons & 0x0001) != 0);
#else
    (void)nunchuk_c;
#endif

    bool usable = !g_clutch;
#ifdef SWITCH_PICO_WII_IR_GYRO
    usable = usable && (!g_infrared || g_motion_enabled);
#endif
    if (!usable) {
        stop_tracking();
        critical_section_exit(&g_lock);
        return;
    }
    const WiiIrTrackingResult tracked = g_tracker.update(x, y, g_valid_mask, now);
    if (!tracked.tracked) {
        // Discard output, but retain bounded association history so a brief
        // occlusion does not make another reflection become the reference.
        stop_tracking(false);
        critical_section_exit(&g_lock);
        return;
    }
#ifdef SWITCH_PICO_WII_IR_GYRO
    constexpr float kUnitsPerRadian = 12800.0f * kDegreesPerRadian;
    // Conservative pitch tuning against the physical MotionPlus reference.
    // Keep camera geometry and the working horizontal response unchanged.
    constexpr float kPitchAimGain = 1.5f;
    const int32_t px = static_cast<int32_t>(lroundf(tracked.yaw_radians * kUnitsPerRadian));
    const int32_t py = static_cast<int32_t>(
        lroundf(tracked.pitch_radians * (kUnitsPerRadian * kPitchAimGain)));
#else
    // Mouse output keeps roughly two counts/pixel at the image center, but
    // uses the same angular tracking and filtering as gyro output.
    const int32_t px = static_cast<int32_t>(lroundf(tracked.yaw_radians * kCamera.fx * 512.0f));
    const int32_t py = static_cast<int32_t>(lroundf(tracked.pitch_radians * kCamera.fy * 512.0f));
#endif
    if (!g_tracking || tracked.rebased) {
        stop_tracking(false);
        g_tracking = true;
        g_filtered_x_q8 = px;
        g_filtered_y_q8 = py;
        ++g_rebaselines;
    } else {
        const int32_t dx = px - g_filtered_x_q8;
        const int32_t dy = py - g_filtered_y_q8;
        g_filtered_x_q8 = px;
        g_filtered_y_q8 = py;
#ifdef SWITCH_PICO_WII_IR_GYRO
        if (g_infrared && g_motion_enabled && g_output_armed) {
            set_pending(g_pending_x_q8, g_fraction_x,
                        (static_cast<int64_t>(g_pending_x_q8) + dx) *
                            kGyroNumerator + g_fraction_x);
            set_pending(g_pending_y_q8, g_fraction_y,
                        (static_cast<int64_t>(g_pending_y_q8) + dy) *
                            kGyroNumerator + g_fraction_y);
        }
#else
        g_pending_x_q8 = clamp(g_pending_x_q8 + dx, kMaximumPendingQ8);
        g_pending_y_q8 = clamp(g_pending_y_q8 + dy, kMaximumPendingQ8);
#endif
    }
    critical_section_exit(&g_lock);
}

bool wii_ir_mouse_peek(WiiIrMouseReport* report) {
    if (!g_initialized || report == nullptr) return false;
    critical_section_enter_blocking(&g_lock);
    expire(time_us_32());
    report->dx = static_cast<int8_t>(clamp(g_pending_x_q8 / 256, 127));
    report->dy = static_cast<int8_t>(clamp(g_pending_y_q8 / 256, 127));
    report->buttons = g_buttons;
    report->generation = g_generation;
    const bool pending = report->dx || report->dy || g_buttons != g_sent_buttons;
    critical_section_exit(&g_lock);
    return pending;
}

void wii_ir_mouse_commit(const WiiIrMouseReport& report) {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    if (report.generation == g_generation) {
        g_pending_x_q8 -= static_cast<int32_t>(report.dx) * 256;
        g_pending_y_q8 -= static_cast<int32_t>(report.dy) * 256;
    }
    // Even a raced reset must release buttons that USB actually accepted.
    g_sent_buttons = report.buttons;
    ++g_sent;
    critical_section_exit(&g_lock);
}

size_t wii_ir_pointer_diagnostics(uint8_t* buffer, size_t capacity) {
    if (!g_initialized || buffer == nullptr) return 0;
    uint8_t data[WII_IR_MOUSE_DIAGNOSTIC_SIZE]{};
    critical_section_enter_blocking(&g_lock);
    const uint32_t now = time_us_32();
    expire(now);
    pointer_diagnostics(data, now);
    critical_section_exit(&g_lock);
    const size_t size = capacity < sizeof(data) ? capacity : sizeof(data);
    memcpy(buffer, data, size);
    return size;
}

#ifdef SWITCH_PICO_WII_IR_GYRO
bool wii_ir_gyro_select(uint8_t slot, uint32_t connection_generation,
                        bool infrared) {
    if (!g_initialized) return false;
    critical_section_enter_blocking(&g_lock);
    if (!claim_owner(slot, connection_generation)) {
        critical_section_exit(&g_lock);
        return false;
    }
    expire(time_us_32());
    if (g_infrared != infrared) {
        stop_tracking();
        ++g_generation;
        g_infrared = infrared;
        g_output_armed = false;
    }
    critical_section_exit(&g_lock);
    return true;
}

void wii_ir_gyro_update_motion(uint8_t slot, uint32_t connection_generation,
                              bool enabled, const ControllerMotionSample& sample) {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    if (claim_owner(slot, connection_generation)) {
        const uint32_t now = time_us_32();
        expire(now);
        expire_output(now);
        if (g_motion_enabled != enabled) {
            stop_tracking();
            ++g_generation;
            g_motion_enabled = enabled;
        }
        // Physical gyro stays in the normal controller state. IR prepare
        // writes all gyro axes itself and uses only these real accel axes.
        g_real_motion.accel_x = sample.accel_x;
        g_real_motion.accel_y = sample.accel_y;
        g_real_motion.accel_z = sample.accel_z;
    }
    critical_section_exit(&g_lock);
}

void wii_ir_gyro_reset_output() {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    stop_tracking();
    ++g_generation;
    g_output_armed = false;
    g_output_progress_us = 0;
    critical_section_exit(&g_lock);
}

bool wii_ir_gyro_prepare(uint8_t slot, uint32_t now_us, ControllerState* state,
                         WiiIrGyroReport* report) {
    if (!g_initialized || state == nullptr || report == nullptr) return false;
    critical_section_enter_blocking(&g_lock);
    expire(now_us);
    expire_output(now_us);
    if (g_owner != slot || !g_infrared) {
        critical_section_exit(&g_lock);
        return false;
    }
    int32_t raw_x = 0, raw_y = 0;
    if (g_motion_enabled && g_tracking && !g_clutch && g_output_armed) {
        raw_x = pending_gyro(g_pending_x_q8, g_fraction_x);
        raw_y = pending_gyro(g_pending_y_q8, g_fraction_y);
    }
    ControllerMotionSample sample{};
    if (g_motion_enabled) {
        sample.accel_x = g_real_motion.accel_x;
        sample.accel_y = g_real_motion.accel_y;
        sample.accel_z = g_real_motion.accel_z;
        // Match the physical Wii yaw axis after Bluepad32 normalization:
        // horizontal camera travel rotates Z, not the longitudinal roll X.
        sample.gyro_z = static_cast<int16_t>(-raw_x);
        sample.gyro_y = static_cast<int16_t>(raw_y);
    }
    state->motion_sample_count =
        g_motion_enabled ? CONTROLLER_MOTION_SAMPLE_CAPACITY : 0;
    for (ControllerMotionSample& destination : state->motion_samples) {
        destination = sample;
    }
    report->generation = g_generation;
    report->consumed_x_q8 = raw_x * kGyroDenominator / kGyroNumerator;
    report->consumed_y_q8 = raw_y * kGyroDenominator / kGyroNumerator;
    report->submitted_us = now_us;
    critical_section_exit(&g_lock);
    return true;
}

void wii_ir_gyro_commit(const WiiIrGyroReport& report) {
    if (!g_initialized) return;
    const int32_t raw_x = consumed_gyro(report.consumed_x_q8);
    const int32_t raw_y = consumed_gyro(report.consumed_y_q8);
    critical_section_enter_blocking(&g_lock);
    if (report.generation == g_generation && g_infrared) {
        set_pending(g_pending_x_q8, g_fraction_x,
                    g_pending_x_q8 * kGyroNumerator + g_fraction_x -
                        raw_x * kGyroDenominator);
        set_pending(g_pending_y_q8, g_fraction_y,
                    g_pending_y_q8 * kGyroNumerator + g_fraction_y -
                        raw_y * kGyroDenominator);
        if (!g_output_armed) {
            // Start from the next fresh camera baseline, not motion collected
            // while the host was negotiating or unavailable.
            stop_tracking();
            g_output_armed = true;
        }
        g_output_progress_us = report.submitted_us;
    }
    // These fields describe what USB actually accepted, including a report
    // whose generation raced a source switch or tracking rebaseline.
    g_sent_gyro_y = static_cast<int16_t>(raw_y);
    g_sent_gyro_z = static_cast<int16_t>(-raw_x);
    g_gyro_sent_us = report.submitted_us;
    g_have_gyro_send = true;
    ++g_gyro_sent;
    critical_section_exit(&g_lock);
}

size_t wii_ir_gyro_diagnostics(uint8_t* output, size_t capacity) {
    if (!g_initialized || output == nullptr) return 0;
    uint8_t data[WII_IR_GYRO_DIAGNOSTIC_SIZE]{};
    critical_section_enter_blocking(&g_lock);
    const uint32_t now = time_us_32();
    expire(now);
    expire_output(now);
    pointer_diagnostics(data, now);
    data[48] = g_infrared ? 1 : 0;
    data[49] = g_motion_enabled ? 1 : 0;
    put16(data + 52, static_cast<uint16_t>(g_sent_gyro_y));
    put16(data + 54, static_cast<uint16_t>(g_sent_gyro_z));
    put32(data + 56, g_gyro_sent);
    put32(data + 60, g_have_gyro_send ? now - g_gyro_sent_us : UINT32_MAX);
    critical_section_exit(&g_lock);
    const size_t size = capacity < sizeof(data) ? capacity : sizeof(data);
    memcpy(output, data, size);
    return size;
}
#endif
