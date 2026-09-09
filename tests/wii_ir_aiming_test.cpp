#include "input/wii_ir_pointer.h"

#include <cmath>
#include <cstdint>
#include <iostream>

static uint32_t clock_us;
uint32_t time_us_32() { return clock_us; }

namespace {
int failures;
void expect(bool okay, const char* message) {
    if (!okay) { std::cerr << message << '\n'; ++failures; }
}
struct Rotation {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
};
struct Rig {
    uint32_t sequence = 0;
    uint8_t visible_mask;
    explicit Rig(uint8_t mask = 3) : visible_mask(mask) {
        wii_ir_pointer_init();
        wii_ir_pointer_reset();
        clock_us = 100000;
        update_motion();
        expect(wii_ir_gyro_select(0, 1, true), "IR source must be selectable");
        send();  // Arm output before establishing a fresh camera baseline.
        for (int i = 0; i < 12; ++i) {
            input(0, 0);
            if (i % 3 == 2) send();
        }
    }
    void update_motion() {
        const ControllerMotionSample physical{0, 0, 4096, 11, 22, 33};
        wii_ir_gyro_update_motion(0, 1, true, physical);
    }
    void input(int dx, int dy, uint16_t buttons = 0, bool nunchuk_c = false) {
        clock_us += 5000;
        update_motion();
        const uint16_t x[4] = {static_cast<uint16_t>(400 + dx), static_cast<uint16_t>(624 + dx), 0, 0};
        const uint16_t y[4] = {static_cast<uint16_t>(384 + dy), static_cast<uint16_t>(384 + dy), 0, 0};
        wii_ir_pointer_observe(0, 1, ++sequence, buttons, x, y, visible_mask, nunchuk_c);
    }
    ControllerState prepare(WiiIrGyroReport& ticket) {
        ControllerState state{};
        expect(wii_ir_gyro_prepare(0, clock_us, &state, &ticket), "selected IR must supply motion");
        expect(state.motion_sample_count == 3, "IR reports contain three 5ms samples");
        for (const auto& sample : state.motion_samples) {
            expect(sample.accel_x == 0 && sample.accel_y == 0 && sample.accel_z == 4096,
                   "changing aiming axes must preserve the accelerometer");
        }
        return state;
    }
    ControllerMotionSample send() {
        WiiIrGyroReport ticket{};
        const ControllerState state = prepare(ticket);
        const auto first = state.motion_samples[0];
        for (const auto& sample : state.motion_samples) {
            expect(sample.gyro_x == first.gyro_x && sample.gyro_y == first.gyro_y && sample.gyro_z == first.gyro_z,
                   "the report's angular displacement must be spread consistently over 15ms");
        }
        wii_ir_gyro_commit(ticket);
        return first;
    }
    Rotation sweep(int horizontal, int vertical, uint16_t buttons = 0, bool nunchuk_c = false) {
        Rotation total;
        for (int i = 1; i <= 300; ++i) {
            const int travel = i < 100 ? i : 100;
            input(horizontal * travel, vertical * travel, buttons, nunchuk_c);
            if (i % 3 == 0) {
                const auto sample = send();
                total.x += sample.gyro_x;
                total.y += sample.gyro_y;
                total.z += sample.gyro_z;
            }
        }
        return total;
    }
};

void horizontal_aim_matches_physical_yaw() {
    for (int direction : {-1, 1}) {
        Rig rig;
        const auto rotation = rig.sweep(direction, 0);
        // The physical Wii reference has gravity and horizontal yaw on Z.
        // Check angular displacement, not merely a nonzero output channel.
        const double expected = std::atan(100.0 / 1726.7951);
        const double yaw = rotation.z * 0.015 / 818.5;
        expect(std::abs(yaw - direction * expected) < expected * 0.05,
               "IR yaw must match the physical rotation implied by camera travel");
        expect(rotation.x == 0 && std::abs(rotation.y) < std::abs(rotation.z) / 50 + 3,
               "horizontal pointing must not produce roll or vertical aiming");
    }
}

void vertical_aim_retains_second_gyro_axis() {
    for (int direction : {-1, 1}) {
        Rig rig;
        const auto rotation = rig.sweep(0, direction);
        expect(rotation.y * direction < 0, "vertical IR direction and gyro Y must remain unchanged");
        expect(rotation.x == 0 && rotation.z == 0, "vertical IR must not generate horizontal/roll movement");
    }
}

void reposition_requires_c_and_one() {
    {
        Rig rig;
        expect(rig.sweep(1, 0, 0x0002, false).z > 0,
               "1 alone must no longer pause IR aiming");
    }
    {
        Rig rig;
        expect(rig.sweep(1, 0, 0, true).z > 0,
               "Nunchuk C alone must not pause IR aiming");
    }
    Rig rig;
    const auto held = rig.sweep(1, 1, 0x0002, true);
    expect(held.x == 0 && held.y == 0 && held.z == 0,
           "C + 1 must suppress aiming on every gyro axis while repositioning");
    for (int i = 0; i < 6; ++i) {
        rig.input(100, 100);
        if (i % 3 == 2) {
            const auto released = rig.send();
            expect(released.gyro_x == 0 && released.gyro_y == 0 && released.gyro_z == 0,
                   "releasing C + 1 must establish a new baseline, not replay held motion");
        }
    }
    int64_t movement = 0;
    for (int i = 1; i <= 30; ++i) {
        rig.input(100 + i, 100);
        if (i % 3 == 0) movement += rig.send().gyro_z;
    }
    expect(movement > 0, "horizontal aiming must resume after reposition release");
}

void failed_send_preserves_horizontal_motion_once() {
    Rig rig;
    for (int i = 0; i < 3; ++i) rig.input(20, 0);
    WiiIrGyroReport first{};
    const auto waiting = rig.prepare(first).motion_samples[0];
    expect(waiting.gyro_z > 0, "horizontal motion must be pending before the failed send");
    WiiIrGyroReport retry{};
    const auto repeated = rig.prepare(retry).motion_samples[0];
    expect(repeated.gyro_z == waiting.gyro_z && repeated.gyro_x == 0,
           "an uncommitted send must retain horizontal movement on its correct axis");
    wii_ir_gyro_commit(first);
    const auto consumed = rig.send();
    expect(consumed.gyro_x == 0 && consumed.gyro_y == 0 && consumed.gyro_z == 0,
           "successfully sent motion must not be emitted twice");
}
void single_spot_retains_horizontal_aiming() {
    Rig rig(1);
    const auto rotation = rig.sweep(1, 0);
    const double expected = std::atan((500 - 511.5) / 1726.7951) -
                            std::atan((400 - 511.5) / 1726.7951);
    expect(std::abs(rotation.z * 0.015 / 818.5 - expected) < expected * 0.05,
           "one persistent spot must deliver its full yaw without an 80ms timeout");
    expect(rotation.x == 0 && rotation.y == 0,
           "single-spot horizontal travel must not create roll or pitch");
}

}  // namespace

int main() {
    horizontal_aim_matches_physical_yaw();
    vertical_aim_retains_second_gyro_axis();
    reposition_requires_c_and_one();
    failed_send_preserves_horizontal_motion_once();
    single_spot_retains_horizontal_aiming();
    if (failures) return 1;
    std::cout << "IR aiming output axes passed\n";
}
