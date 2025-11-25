# Gyro Support Plan

## Checklist
- [x] Review current UART input/rumble plumbing (`switch-pico.cpp` and `controller_uart_bridge.py`) to confirm the existing 0xAA input frame (buttons/hat/sticks) and 0xBB rumble return path.
- [x] Map the current firmware IMU hooks (`switch_pro_driver.cpp`/`switch_pro_descriptors.h` `imuData` field, `is_imu_enabled` flag, report construction cadence) and note where IMU payloads should be injected.
- [x] Pull expected IMU packet format, sample cadence (3 samples/report), and unit scaling from references: `Nintendo_Switch_Reverse_Engineering/imu_sensor_notes.md`, `hid-nintendo.c`, and NXBT (`nxbt/controller/protocol.py`).
- [x] Examine GP2040-CE’s motion implementation (e.g., `GP2040-CE/src/addons/imu` and Switch report handling) for framing, calibration defaults, and rate control patterns to reuse.
- [x] Decide on UART motion framing: header/length/checksum scheme, sample packing (likely 3 samples of accel+gyro int16 LE), endian/order, and compatibility with existing 8-byte frames (avoid breaking current host builds).
- [ ] Define Switch-facing IMU payload layout inside `SwitchProReport` (axis order, sign conventions, zero/neutral sample) and ensure it matches the reverse-engineered descriptors.
- [ ] Add firmware-side data structures/buffers for incoming IMU samples (triple-buffer if mirroring Joy-Con 3-sample bursts) and default zeroing when IMU is disabled/missing.
- [ ] Extend UART parser in `switch-pico.cpp` to accept the new motion frame(s), validate checksum, and stash samples atomically alongside button state.
- [ ] Gate IMU injection on the host’s `TOGGLE_IMU` feature flag (`is_imu_enabled`) and ensure reports carry motion data only when enabled; default to zeros otherwise.
- [ ] Apply calibration/scaling constants: choose defaults from references (e.g., 0.06103 dps/LSB gyro, accel per imu_sensor_notes) and document where to adjust for sensor-specific offsets.
- [ ] Update host bridge to enable SDL sensor support (`SDL_GameControllerSetSensorEnabled`, `SDL_CONTROLLERAXIS` vs sensor events) and capture gyro (and accel if needed) at the required rate.
- [ ] Buffer and pack host IMU readings into the agreed UART motion frame, including timestamping/rate smoothing so the firmware sees stable 200 Hz-ish samples (3 per 5 ms report).
- [ ] Keep backward compatibility: allow running without IMU-capable controllers (send zero motion) and keep rumble unchanged.
- [ ] Add logging/metrics: lightweight counters for dropped/late IMU frames and a debug toggle to inspect raw samples.
- [ ] Test matrix: host-only sensor capture sanity check; loopback UART frame validator; firmware USB capture with Switch (or nxbt PC host) verifying IMU report contents and that `TOGGLE_IMU` on/off behaves; regression check that buttons/rumble remain stable.

## Findings to date
- Firmware: `SwitchProReport.imuData[36]` exists but is always zero; `is_imu_enabled` is set only via `TOGGLE_IMU` feature report; `switch_pro_task` always sends `switch_report`, so IMU injection should happen before the memcmp/send path.
- IMU payload layout (standard 0x30/31/32/33): bytes 13-24 are accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z (all Int16 LE); bytes 25-48 repeat two more samples (3 samples total, ~5 ms apart). Matches `imuData[36]` size and `hid-nintendo.c` parsing (`imu_raw_bytes` split into 3 `joycon_imu_data` structs).
- Scaling from references: accel ≈ 0.000244 G/LSB (±8000 mG), gyro ≈ 0.06103 dps/LSB (±2000 dps) or 0.070 dps/LSB with ST’s +15% headroom; Switch internally also uses rev/s conversions. Typical packet cadence ~15 ms with 3 samples (≈200 Hz sampling).
- NXBT reference: only injects IMU when `imu_enabled` flag is true; drops a 36-byte sample block at offset 14 (0-based) in the report. Example data is static; good for offset confirmation.
- GP2040-CE reference: Switch Pro driver mirrors this project—`imuData` zeroed, no motion handling yet. No reusable IMU framing, but report/keepalive cadence matches.

## UART IMU framing decision (breaking change OK)
- New host→Pico frame (versioned) replaces the old 8-byte 0xAA packet:
  - Byte0: `0xAA` header
  - Byte1: `0x02` version
  - Byte2: `payload_len` (44 for the layout below)
  - Byte3-4: buttons LE (same masks as before)
  - Byte5: hat
  - Byte6-9: sticks `lx, ly, rx, ry` (0-255 as before)
  - Byte10: `imu_sample_count` (host should send 3; firmware may accept 0-3)
  - Byte11-46: IMU samples, 3 blocks of 12 bytes each:
    - For sample i: `accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z` (all int16 LE, Pro Controller axis/sign convention)
  - Byte47: checksum = (sum of bytes 0 through the last payload byte) & 0xFF
- Host behavior: always populate 3 samples per packet (~200 Hz, 5 ms spacing) with reference/default scaling; send zeros and `imu_sample_count=0` if IMU unavailable/disabled. Buttons/sticks unchanged.
- Firmware behavior: parse the new frame, validate `payload_len` and checksum, then atomically store button/stick plus up to `imu_sample_count` samples. If `is_imu_enabled` is true, copy samples into `switch_report.imuData` in the 3× sample layout; otherwise zero the IMU block.
- Axis orientation: match Pro Controller orientation (no additional flips beyond standard report ordering).

## Open Questions
- All answered:
  - Include both accelerometer and gyro (full IMU).
  - Reference/default scaling is acceptable (no per-device calibration).
  - Mirror 3-sample bursts (~200 Hz, 5 ms spacing).
  - Use Pro Controller axis orientation/sign.
  - Breaking UART framing change is acceptable (use the versioned packet above).
