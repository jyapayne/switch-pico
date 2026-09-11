#include "input/wii_ir_pointer.h"

#include <cstdint>
#include <cmath>
#include <limits>
#include <iostream>

static uint32_t clock_us;
uint32_t time_us_32() { return clock_us; }

namespace {
int failures;

void expect(bool okay, const char* message) {
    if (!okay) { std::cerr << message << '\n'; ++failures; }
}

void observe(uint8_t slot, uint32_t connection_generation, uint32_t sequence,
             int dx, int dy, uint16_t buttons = 0, uint8_t mask = 3,
             bool nunchuk_c = false) {
    clock_us += 5000;
    const uint16_t x[4] = {static_cast<uint16_t>(400 + dx), static_cast<uint16_t>(624 + dx), 0, 0};
    const uint16_t y[4] = {static_cast<uint16_t>(384 + dy), static_cast<uint16_t>(384 + dy), 0, 0};
    wii_ir_pointer_observe(slot, connection_generation, sequence, buttons, x, y, mask, nunchuk_c,
                           0.0f, false);
}

struct Rig {
    uint8_t slot = 2;
    uint32_t connection_generation = 41;
    uint32_t sequence = 0;
    int horizontal = 0;
    int vertical = 0;

    Rig() {
        wii_ir_pointer_init();
        wii_ir_pointer_reset();
#if SWITCH2_BRIDGE_WII_INPUT
        expect(wii_ir_pointer_configure_screen(660, 370, 0, 0, 1320, 740),
               "explicit native viewport and count spans must be configurable");
#endif
        wii_ir_mouse_set_output_enabled(true);
        clock_us = 100000;
#if SWITCH2_BRIDGE_WII_INPUT
        // Prime the real upstream startup/glitch/smoothing history, then
        // consume its finite startup tail before comparing relative motion.
        settle(0, 0);
#endif
        WiiIrMouseReport release{};
        if (wii_ir_mouse_peek(&release)) wii_ir_mouse_commit(release);
        input(0, 0);
        input(0, 0);
    }

    void input(int dx, int dy, uint16_t buttons = 0, uint8_t mask = 3,
               bool nunchuk_c = false) {
        horizontal = dx;
        vertical = dy;
        observe(slot, connection_generation, ++sequence, dx, dy, buttons, mask, nunchuk_c);
    }

    void baseline(int dx, int dy) {
#if SWITCH2_BRIDGE_WII_INPUT
        for (unsigned i = 0; i < 16; ++i) {
            input(dx, dy);
            if (peek().tracking) return;
        }
        expect(false, "fresh stationary full-pair reports must establish a native baseline");
#else
        input(dx, dy);
        input(dx, dy);
#endif
    }

    void settle(int dx, int dy, uint16_t buttons = 0) {
        for (int i = 0; i < 80; ++i) input(dx, dy, buttons);
    }

    void move_to(int dx, int dy) {
        const int from_x = horizontal;
        const int from_y = vertical;
        for (int i = 1; i <= 20; ++i) {
            input(from_x + (dx - from_x) * i / 20, from_y + (dy - from_y) * i / 20);
        }
        settle(dx, dy);
    }

    WiiIrMouseReport peek() {
        WiiIrMouseReport report{};
        wii_ir_mouse_peek(&report, 32767);
        return report;
    }
};

void native_movement_and_legacy_limits_conserve_output() {
    Rig rig;
    rig.move_to(100, -100);
    const auto native = rig.peek();
    expect(native.dx < -127 && native.dy > 127,
           "large two-axis motion must remain signed and exceed the legacy 8-bit range");
    const auto retry = rig.peek();
    expect(retry.dx == native.dx && retry.dy == native.dy && retry.generation == native.generation,
           "peeking an unsubmitted native report must not consume movement");

    WiiIrMouseReport legacy{};
    expect(wii_ir_mouse_peek(&legacy) && legacy.dx == -127 && legacy.dy == 127,
           "legacy callers must retain the signed 8-bit HID movement limits");
    WiiIrMouseReport bounded{};
    expect(wii_ir_mouse_peek(&bounded, 31) && bounded.dx == -31 && bounded.dy == 31,
           "both signed axes must obey the caller's movement bound");
    expect(!wii_ir_mouse_peek(&bounded, 0) && bounded.dx == 0 && bounded.dy == 0 && bounded.tracking,
           "a zero movement allowance must leave tracking live without consuming displacement");

    wii_ir_mouse_commit(legacy);
    const auto remainder = rig.peek();
    expect(remainder.dx == native.dx - legacy.dx && remainder.dy == native.dy - legacy.dy,
           "committing a bounded legacy report must leave its full native remainder");
    wii_ir_mouse_commit(remainder);
    expect(!wii_ir_mouse_peek(&bounded, 32767) && bounded.dx == 0 && bounded.dy == 0 && bounded.tracking,
           "committed movement must not be sent again and idle tracking must remain visible");
}

void prepared_report_preserves_newer_observations() {
    Rig rig;
    rig.move_to(80, -80);
    const auto prepared = rig.peek();
    rig.move_to(120, -120);
    const auto accumulated = rig.peek();
    expect(accumulated.dx < prepared.dx && accumulated.dy > prepared.dy,
           "fresh observations must continue accumulating while a USB report waits");
    wii_ir_mouse_commit(prepared);
    const auto remaining = rig.peek();
    expect(remaining.dx == accumulated.dx - prepared.dx && remaining.dy == accumulated.dy - prepared.dy,
           "a successful cached report must consume only its own displacement, not newer samples");
}

void disabled_output_rebaselines_without_losing_owner() {
    Rig rig;
    rig.move_to(100, -100);
    const auto old = rig.peek();
    wii_ir_mouse_set_output_enabled(false);
    WiiIrMouseReport disabled{};
    expect(!wii_ir_mouse_peek(&disabled, 32767) && disabled.dx == 0 && disabled.dy == 0 && !disabled.tracking,
           "disabling output must immediately discard pending movement and tracking");
    expect(disabled.owner == rig.slot && disabled.connection_generation == rig.connection_generation &&
               disabled.generation != old.generation,
           "gating must invalidate prepared reports without relinquishing the selected owner");

    rig.settle(-150, 100, 0x000c);
    expect(!wii_ir_mouse_peek(&disabled, 32767) && disabled.dx == 0 && disabled.dy == 0 && disabled.buttons == 3,
           "disabled camera observations must retain legacy button diagnostics but never queue output");
    uint8_t diagnostics[WII_IR_MOUSE_DIAGNOSTIC_SIZE]{};
    wii_ir_pointer_diagnostics(diagnostics, sizeof(diagnostics));
    expect(diagnostics[2] == 3 && diagnostics[32] == 0x0c && diagnostics[34] == 3,
           "disabled output must continue reporting observed spots and physical buttons in diagnostics");

    wii_ir_mouse_set_output_enabled(true);
    const auto reopened = rig.peek();
    expect(reopened.dx == 0 && reopened.dy == 0 && !reopened.tracking &&
               reopened.owner == rig.slot && reopened.generation != disabled.generation,
           "reopening output must await a fresh camera baseline without replaying disabled movement");
    rig.baseline(-150, 100);
    const auto baseline = rig.peek();
    expect(baseline.tracking && baseline.dx == 0 && baseline.dy == 0,
           "fresh observations after reopening must establish a motionless baseline");

    rig.move_to(-120, 70);
    const auto resumed = rig.peek();
    expect(resumed.dx < 0 && resumed.dx > -127 && resumed.dy > 0 && resumed.dy < 127,
           "reopened output must contain the small new motion, not the larger disabled repositioning");
    wii_ir_mouse_commit(old);
    const auto after_late_commit = rig.peek();
    expect(after_late_commit.dx == resumed.dx && after_late_commit.dy == resumed.dy,
           "a late commit from before disabling must not consume newly acquired movement");

    wii_ir_mouse_set_output_enabled(false);
    wii_ir_pointer_reset();
    rig.slot = 4;
    rig.connection_generation = 99;
    rig.settle(0, 0);
    expect(!wii_ir_mouse_peek(&disabled, 32767) && !disabled.tracking && disabled.owner == 4 &&
               disabled.connection_generation == 99 && disabled.dx == 0 && disabled.dy == 0,
           "reset and source reselection must not silently reopen a disabled output gate");
}

void repeated_enable_preserves_live_tracking() {
    Rig rig;
    rig.move_to(50, -50);
    const auto before = rig.peek();
    wii_ir_mouse_set_output_enabled(true);
    const auto unchanged = rig.peek();
    expect(unchanged.tracking && unchanged.generation == before.generation &&
               unchanged.dx == before.dx && unchanged.dy == before.dy,
           "repeated enable must neither reset live tracking nor discard pending displacement");
    for (int i = 1; i <= 20; ++i) {
        wii_ir_mouse_set_output_enabled(true);
        rig.input(50 + i, -50 - i);
    }
    const auto moved = rig.peek();
    expect(moved.tracking && moved.generation == before.generation && moved.dx < before.dx && moved.dy > before.dy,
           "a continuously asserted readiness gate must allow subsequent camera motion");
}

void duplicate_samples_cannot_refresh_stale_tracking() {
    Rig rig;
    rig.move_to(80, -80);
    const auto prepared = rig.peek();
    for (int i = 0; i < 29; ++i) {
        observe(rig.slot, rig.connection_generation, rig.sequence, -100, 100, 0x000c);
    }
    const auto fresh = rig.peek();
    expect(fresh.tracking && fresh.dx == prepared.dx && fresh.dy == prepared.dy && fresh.buttons == 0,
           "duplicate sequences must not update camera movement, buttons, or the freshness deadline");
    observe(rig.slot, rig.connection_generation, rig.sequence, -100, 100, 0x000c);
    WiiIrMouseReport stale{};
    expect(!wii_ir_mouse_peek(&stale, 32767) && !stale.tracking && stale.dx == 0 && stale.dy == 0,
           "tracking and queued movement must expire at 150ms despite duplicate observations");
    expect(stale.owner == rig.slot && stale.connection_generation == rig.connection_generation &&
               stale.generation != prepared.generation,
           "stale tracking must invalidate prepared movement without forgetting the connected owner");
    rig.baseline(-100, 100);
    const auto reacquired = rig.peek();
    expect(reacquired.tracking && reacquired.dx == 0 && reacquired.dy == 0,
           "reacquisition after stale tracking must not include the stale interval");
    rig.move_to(-70, 70);
    const auto current = rig.peek();
    wii_ir_mouse_commit(prepared);
    const auto after = rig.peek();
    expect(after.dx == current.dx && after.dy == current.dy,
           "an expired report cannot consume movement from a fresh tracking generation");
}

void owner_generation_and_tracking_loss_invalidate_reports() {
    Rig rig;
    WiiIrMouseReport idle{};
    expect(!wii_ir_mouse_peek(&idle, 32767) && idle.tracking && idle.owner == rig.slot &&
               idle.connection_generation == rig.connection_generation,
           "a stationary source must expose tracking and connection identity even without pending output");
    rig.move_to(80, -80);
    const auto old = rig.peek();
    observe(7, 900, 1, -100, 100, 0x000c);
    wii_ir_pointer_disconnect(7);
    const auto owned = rig.peek();
    expect(owned.owner == rig.slot && owned.connection_generation == rig.connection_generation &&
               owned.dx == old.dx && owned.dy == old.dy && owned.buttons == 0,
           "an unrelated source or disconnect must not steal ownership or alter movement");

    ++rig.connection_generation;
    rig.sequence = 0;
    rig.baseline(-100, 100);
    const auto reconnected = rig.peek();
    expect(reconnected.tracking && reconnected.dx == 0 && reconnected.dy == 0 &&
               reconnected.connection_generation == rig.connection_generation && reconnected.generation != old.generation,
           "reusing a slot with a new connection generation must discard the old association and backlog");
    rig.move_to(-70, 70);
    const auto current = rig.peek();
    wii_ir_mouse_commit(old);
    const auto after = rig.peek();
    expect(after.dx == current.dx && after.dy == current.dy,
           "a previous connection's report cannot consume current movement");

#if SWITCH2_BRIDGE_WII_INPUT
    // Upstream holds eight missing reports; only the ninth is true loss.
    for (unsigned i = 0; i < 8; ++i) rig.input(-70, 70, 0, 0);
#endif
    rig.input(-70, 70, 0, 0);
    expect(!wii_ir_mouse_peek(&idle, 32767) && !idle.tracking && idle.dx == 0 && idle.dy == 0,
           "true camera tracking loss must drop unsent movement");
    wii_ir_pointer_disconnect(rig.slot);
    expect(!wii_ir_mouse_peek(&idle, 32767) && !idle.tracking && idle.owner == 0xff && idle.connection_generation == 0,
           "disconnect must clear tracking and connection identity even without pending movement");
}

void legacy_buttons_and_clutch_remain_independent_of_movement() {
    Rig rig;
    rig.input(0, 0, 0x000c);
    WiiIrMouseReport buttons{};
    expect(wii_ir_mouse_peek(&buttons) && buttons.buttons == 3 && buttons.dx == 0 && buttons.dy == 0,
           "legacy mouse buttons must remain observable independently of camera movement");
    wii_ir_mouse_commit(buttons);
    expect(!wii_ir_mouse_peek(&buttons), "committing stationary legacy buttons must clear their pending change");
#if SWITCH2_BRIDGE_WII_INPUT
    for (int i = 1; i <= 20; ++i) rig.input(i, -i, 0x0003, 3, true);
    const auto held = rig.peek();
    expect(held.tracking && held.dx < 0 && held.dy > 0 && held.buttons == 0,
           "native controller buttons and Nunchuk C must not activate either legacy clutch chord");
#else
    rig.input(20, -20, 0x0002);
    expect(wii_ir_mouse_peek(&buttons) && buttons.buttons == 0 && !buttons.tracking && buttons.dx == 0 && buttons.dy == 0,
           "the legacy 1-button clutch must suppress movement while still releasing buttons");
    wii_ir_mouse_commit(buttons);
    rig.settle(100, -100, 0x0002);
    rig.input(100, -100);
    rig.input(100, -100);
    expect(!wii_ir_mouse_peek(&buttons) && buttons.tracking && buttons.dx == 0 && buttons.dy == 0,
           "releasing the legacy clutch must rebaseline rather than replay repositioning");
    rig.move_to(120, -120);
    const auto moved = rig.peek();
    expect(moved.dx < 0 && moved.dy > 0, "movement must resume normally after the legacy clutch is released");
#endif
}

#if SWITCH2_BRIDGE_WII_INPUT
void native_cold_baseline_and_unique_camera_reports() {
    Rig rig;
    wii_ir_pointer_reset();
    const uint16_t x[4] = {300, 524, 0, 0};
    const uint16_t y[4] = {384, 384, 0, 0};
    clock_us += 5000;
    wii_ir_pointer_observe(rig.slot, rig.connection_generation, ++rig.sequence,
                           0, x, y, 3, false, 0.0f, true);
    for (unsigned i = 0; i < 64; ++i) {
        wii_ir_pointer_observe(rig.slot, rig.connection_generation, rig.sequence,
                               0, x, y, 3, false, 0.0f, true);
        const auto held = rig.peek();
        expect(!held.tracking && held.dx == 0 && held.dy == 0,
               "polls and duplicate sequences must not advance upstream's initial glitch hold");
    }
    rig.baseline(-100, 0);
    const auto first = rig.peek();
    expect(first.tracking && first.dx == 0 && first.dy == 0 && first.optical_valid,
           "first accepted upstream position must become a motionless relative baseline");
    expect(first.optical_sequence == rig.sequence && first.optical_received_us == clock_us &&
               std::fabs(first.optical_yaw_radians - std::atan(99.0f * 0.39f / 512.0f)) < 0.00001f,
           "optical heading must use mirrored raw X with upstream calc_yaw in radians");
    uint8_t diagnostics[WII_IR_MOUSE_DIAGNOSTIC_SIZE]{};
    expect(wii_ir_pointer_diagnostics(diagnostics, sizeof(diagnostics)) == 48 &&
               diagnostics[16] == 0x2c && diagnostics[17] == 0x01 &&
               diagnostics[18] == 0x80 && diagnostics[19] == 0x01 &&
               (diagnostics[3] & 0x19) == 0x19,
           "raw48 diagnostics must retain original unmirrored pixels and upstream tracking flags");
}

void native_glitch_hold_excludes_optical_heading() {
    Rig rig;
    const auto before = rig.peek();
    for (unsigned frame = 0; frame < 6; ++frame) {
        rig.input(250, 0);
        const auto held = rig.peek();
        expect(held.tracking && !held.optical_valid && held.dx == 0 && held.dy == 0 &&
                   held.generation == before.generation,
               "upstream glitch-held raw geometry must not move the pointer or feed optical heading");
    }
    rig.input(250, 0);
    const auto accepted = rig.peek();
    expect(accepted.tracking && accepted.optical_valid && accepted.dx < 0 &&
               accepted.generation == before.generation,
           "upstream accepting a persistent glitch must restore heading confidence without a custom rebase");
}

void native_missing_report_grace_and_true_loss() {
    Rig rig;
    rig.input(20, -20); // Leave an upstream smoothing tail when the dots vanish.
    const auto settled = rig.peek();
    wii_ir_mouse_commit(settled);
    for (unsigned missing = 1; missing <= 8; ++missing) {
        rig.input(20, -20, 0, 0);
        const auto held = rig.peek();
        expect(held.tracking && !held.optical_valid && held.dx == 0 && held.dy == 0 &&
                   held.generation == settled.generation,
               "upstream's eight missing-frame grace must hold position without movement or rebasing");
        // Core0 polling cannot age the frame-count error policy.
        for (unsigned poll = 0; poll < 16; ++poll) {
            const auto polled = rig.peek();
            expect(polled.tracking && polled.dx == 0 && polled.dy == 0,
                   "USB peeks must neither age missing-frame grace nor synthesize movement");
        }
    }
    rig.input(20, -20, 0, 0);
    const auto lost = rig.peek();
    expect(!lost.tracking && !lost.optical_valid && lost.dx == 0 && lost.dy == 0 &&
               lost.generation != settled.generation,
           "ninth missing camera report must expose upstream's real loss and invalidate pending generations");
    rig.input(-100, 100);
    const auto acquired = rig.peek();
    expect(acquired.tracking && acquired.optical_valid && acquired.dx == 0 && acquired.dy == 0,
           "upstream loss recovery must rebaseline rather than bridge the absent camera interval");
    rig.move_to(-70, 70);
    expect(rig.peek().dx < 0 && rig.peek().dy > 0,
           "real movement must resume after loss recovery");
}

void native_single_end_does_not_supply_full_optical_heading() {
    Rig rig;
    int vertical = 0;
    for (int i = 1; i <= 80; ++i) {
        rig.input(0, i, 0, 1);
        const auto report = rig.peek();
        expect(report.tracking && !report.optical_valid,
               "upstream single-end continuation must not claim a full-pair optical heading");
        vertical += report.dy;
        wii_ir_mouse_commit(report);
    }
    expect(vertical < -140,
           "continuous single-end observations must retain upstream's relative movement");
    rig.input(0, 80);
    expect(rig.peek().optical_valid,
           "a full pair returning must restore the native optical observer metadata");
}

void native_roll_convention_and_invalid_gravity_retention() {
    Rig rig;
    wii_ir_pointer_reset();
    // A raw +90-degree bar becomes horizontal only after mirrored camera X
    // and upstream's +90-degree rotation. Raw X travel becomes corrected Y.
    int horizontal = 0, vertical = 0;
    for (unsigned i = 0; i < 100; ++i) {
        const uint16_t x[4] = {static_cast<uint16_t>(512 + (i > 30 ? 30 : 0)),
                               static_cast<uint16_t>(512 + (i > 30 ? 30 : 0)), 0, 0};
        const uint16_t y[4] = {272, 496, 0, 0};
        clock_us += 5000;
        wii_ir_pointer_observe(rig.slot, rig.connection_generation, ++rig.sequence,
                               0, x, y, 3, false,
                               i == 0 ? 1.57079632679f : std::numeric_limits<float>::quiet_NaN(),
                               (i % 2) == 0);
        const auto report = rig.peek();
        if (i > 30) {
            expect(report.tracking && report.optical_valid,
                   "invalid new gravity must retain the last valid roll for upstream full-pair tracking");
            horizontal += report.dx;
            vertical += report.dy;
        }
        wii_ir_mouse_commit(report);
    }
    expect(vertical > 0 && vertical > 2 * std::abs(horizontal),
           "raw +90-degree roll must map travel predominantly to shared positive Y despite smoothing history");
}
void native_relative_motion_is_not_clipped_to_viewport() {
    Rig rig;
    wii_ir_pointer_reset();
    for (int i = 0; i < 80; ++i) {
        rig.input(0, 220);
        wii_ir_mouse_commit(rig.peek());
    }
    int vertical = 0;
    bool tracked = true;
    for (int i = 1; i <= 110; ++i) {
        rig.input(0, 220 + (i < 30 ? i : 30));
        const auto report = rig.peek();
        tracked = tracked && report.tracking;
        vertical += report.dy;
        wii_ir_mouse_commit(report);
    }
    expect(tracked && vertical < -40 && vertical > -80,
           "visible vertical motion outside the viewport must still reach a relative mouse");

    expect(wii_ir_pointer_configure_screen(1, 1, 0, 0, 32767, 32767),
           "the narrowest supported viewport and largest count spans must be accepted");
    for (int i = 0; i < 80; ++i) {
        rig.input(300, 0);
        wii_ir_mouse_commit(rig.peek());
    }
    int horizontal = 0;
    bool bounded = true;
    for (int i = 0; i < 24; ++i) {
        rig.input(290, 0);
        const auto report = rig.peek();
        bounded = bounded && report.dx >= 0 && report.dx <= 4096 &&
                  report.dy >= -4096 && report.dy <= 4096;
        horizontal += report.dx;
        wii_ir_mouse_commit(report);
    }
    expect(bounded && horizontal > 0,
           "wide off-viewport coordinates must not overflow or reverse bounded relative movement");
}

#endif

}  // namespace

int main() {
    native_movement_and_legacy_limits_conserve_output();
    prepared_report_preserves_newer_observations();
    disabled_output_rebaselines_without_losing_owner();
    repeated_enable_preserves_live_tracking();
    duplicate_samples_cannot_refresh_stale_tracking();
    owner_generation_and_tracking_loss_invalidate_reports();
    legacy_buttons_and_clutch_remain_independent_of_movement();
#if SWITCH2_BRIDGE_WII_INPUT
    native_cold_baseline_and_unique_camera_reports();
    native_glitch_hold_excludes_optical_heading();
    native_missing_report_grace_and_true_loss();
    native_single_end_does_not_supply_full_optical_heading();
    native_roll_convention_and_invalid_gravity_retention();
    native_relative_motion_is_not_clipped_to_viewport();
#endif
    if (failures) return 1;
    std::cout << "IR pointer output contract passed\n";
}
