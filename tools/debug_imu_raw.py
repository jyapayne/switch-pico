#!/usr/bin/env python3
"""
Raw SDL3 IMU diagnostic tool.

Prints every gyro/accel sensor event directly from SDL3, bypassing all
bridge logic. Use this to confirm SDL3 is delivering sensor events before
debugging conversion or axis mapping issues.

Usage:
    uv run python tools/debug_imu_raw.py
    uv run python tools/debug_imu_raw.py --count 500   # stop after N gyro events
    uv run python tools/debug_imu_raw.py --no-bias     # skip bias calibration window
"""

import argparse
import ctypes
import math
import sys
import time

import sdl3

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SDL_SENSOR_ACCEL = 1
SDL_SENSOR_GYRO  = 2
GRAVITY          = 9.80665          # m/s²
LSB_PER_G        = 4096.0           # Nintendo accel scale
LSB_PER_RAD_S    = 818.5            # Nintendo gyro scale
BIAS_SAMPLES     = 200              # ~1 second at 200 Hz

def main():
    parser = argparse.ArgumentParser(description="Raw SDL3 IMU diagnostic")
    parser.add_argument("--count", type=int, default=0,
                        help="Stop after this many gyro events (0 = run forever)")
    parser.add_argument("--no-bias", action="store_true",
                        help="Skip bias calibration window, print raw values immediately")
    parser.add_argument("--raw", action="store_true",
                        help="Also print converted Nintendo-native raw counts")
    args = parser.parse_args()

    # Init SDL3 with gamepad + sensor support
    if not sdl3.SDL_Init(sdl3.SDL_INIT_GAMEPAD | sdl3.SDL_INIT_EVENTS):
        print(f"SDL_Init failed: {sdl3.SDL_GetError().decode()}", file=sys.stderr)
        sys.exit(1)

    sdl3.SDL_SetGamepadEventsEnabled(True)

    # Find first gamepad
    count = ctypes.c_int(0)
    ids = sdl3.SDL_GetJoysticks(ctypes.byref(count))
    if not ids or count.value == 0:
        print("No joysticks/gamepads found.", file=sys.stderr)
        sdl3.SDL_Quit()
        sys.exit(1)

    gamepad = None
    instance_id = None
    for i in range(count.value):
        if sdl3.SDL_IsGamepad(ids[i]):
            gamepad = sdl3.SDL_OpenGamepad(ids[i])
            instance_id = ids[i]
            break
    sdl3.SDL_free(ids)

    if not gamepad:
        print("No gamepad found (only non-gamepad joysticks detected).", file=sys.stderr)
        sdl3.SDL_Quit()
        sys.exit(1)

    name = sdl3.SDL_GetGamepadName(gamepad)
    print(f"Gamepad: {name.decode() if name else 'unknown'} (instance_id={instance_id})")

    # Check sensor support
    has_accel = bool(sdl3.SDL_GamepadHasSensor(gamepad, SDL_SENSOR_ACCEL))
    has_gyro  = bool(sdl3.SDL_GamepadHasSensor(gamepad, SDL_SENSOR_GYRO))
    print(f"  Accelerometer supported: {has_accel}")
    print(f"  Gyroscope supported:     {has_gyro}")

    if not (has_accel and has_gyro):
        print("\nThis controller does not expose IMU sensors to SDL3.")
        print("Possible reasons:")
        print("  - Controller doesn't have IMU (Xbox, generic gamepads)")
        print("  - Missing kernel driver (Linux: hid-nintendo not loaded)")
        print("  - SDL3 HIDAPI disabled for this controller")
        sdl3.SDL_CloseGamepad(gamepad)
        sdl3.SDL_Quit()
        sys.exit(1)

    # Enable sensors
    ok_accel = bool(sdl3.SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_ACCEL, True))
    ok_gyro  = bool(sdl3.SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_GYRO, True))
    print(f"  Accelerometer enabled:   {ok_accel}")
    print(f"  Gyroscope enabled:       {ok_gyro}")

    if not (ok_accel and ok_gyro):
        print(f"\nFailed to enable sensors: {sdl3.SDL_GetError().decode()}")
        sdl3.SDL_CloseGamepad(gamepad)
        sdl3.SDL_Quit()
        sys.exit(1)

    print()
    if args.no_bias:
        print("Skipping bias calibration. Showing raw values immediately.")
    else:
        print(f"Hold controller STILL — collecting {BIAS_SAMPLES} gyro samples for bias calibration...")
    print("Press Ctrl+C to stop.\n")
    print(f"{'EVENT':<8} {'AX':>8} {'AY':>8} {'AZ':>8}  {'GX':>8} {'GY':>8} {'GZ':>8}  {'STATUS'}")
    print("-" * 80)

    # State
    last_accel    = (0.0, 0.0, 0.0)
    bias          = [0.0, 0.0, 0.0]
    bias_count    = 0
    bias_locked   = args.no_bias
    gyro_events   = 0
    last_print    = time.monotonic()
    event         = sdl3.SDL_Event()

    try:
        while True:
            while sdl3.SDL_PollEvent(ctypes.byref(event)):
                t = event.type

                if t == sdl3.SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
                    gs = event.gsensor
                    # Only handle events from our gamepad
                    if gs.which != instance_id:
                        continue

                    sensor_type = gs.sensor
                    d = gs.data          # c_float_Array_3

                    if sensor_type == SDL_SENSOR_ACCEL:
                        last_accel = (float(d[0]), float(d[1]), float(d[2]))
                        continue

                    if sensor_type != SDL_SENSOR_GYRO:
                        continue

                    gx, gy, gz = float(d[0]), float(d[1]), float(d[2])

                    # Bias accumulation
                    if not bias_locked:
                        if bias_count < BIAS_SAMPLES:
                            bias[0] += gx
                            bias[1] += gy
                            bias[2] += gz
                            bias_count += 1
                        if bias_count >= BIAS_SAMPLES:
                            bias = [b / BIAS_SAMPLES for b in bias]
                            bias_locked = True
                            print(f"  [BIAS LOCKED] bias_rad_s=({bias[0]:.5f}, {bias[1]:.5f}, {bias[2]:.5f})\n")
                        continue  # Don't print during calibration

                    gyro_events += 1
                    ax, ay, az = last_accel
                    ux, uy, uz = gx - bias[0], gy - bias[1], gz - bias[2]

                    now = time.monotonic()
                    if now - last_print >= 0.1:   # 10 Hz display update
                        last_print = now
                        # In m/s² and rad/s (SDL values)
                        status = f"events={gyro_events}"
                        if args.raw:
                            # Nintendo-native counts (reversed SDL axis mapping)
                            nx = int(-uz * LSB_PER_RAD_S)
                            ny = int(-ux * LSB_PER_RAD_S)
                            nz = int( uy * LSB_PER_RAD_S)
                            nax = int(-az / GRAVITY * LSB_PER_G)
                            nay = int(-ax / GRAVITY * LSB_PER_G)
                            naz = int( ay / GRAVITY * LSB_PER_G)
                            status += f"  raw_g=({nax},{nay},{naz}) raw_gyro=({nx},{ny},{nz})"
                        print(
                            f"{'GYRO':<8} "
                            f"{ax:>8.3f} {ay:>8.3f} {az:>8.3f}  "
                            f"{ux:>8.4f} {uy:>8.4f} {uz:>8.4f}  "
                            f"{status}"
                        )

                elif t == sdl3.SDL_EVENT_GAMEPAD_REMOVED:
                    print("\nGamepad disconnected.")
                    break

            if args.count and gyro_events >= args.count:
                print(f"\nReached {args.count} gyro events. Done.")
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\nStopped.")

    print(f"\nTotal gyro events received: {gyro_events}")
    if bias_locked:
        print(f"Final bias (rad/s): ({bias[0]:.5f}, {bias[1]:.5f}, {bias[2]:.5f})")
        print(f"Bias magnitude: {math.sqrt(sum(b**2 for b in bias)):.5f} rad/s "
              f"= {math.sqrt(sum(b**2 for b in bias)) * 180/math.pi:.3f} deg/s")

    sdl3.SDL_CloseGamepad(gamepad)
    sdl3.SDL_Quit()

if __name__ == "__main__":
    main()
