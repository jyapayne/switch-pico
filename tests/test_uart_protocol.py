"""Tests for UART v2 protocol serialization in switch_pico_uart."""

import struct
import pytest
from switch_pico_bridge.switch_pico_uart import (
    SwitchReport,
    IMUSample,
    SwitchDpad,
    UART_HEADER,
    UART_PROTOCOL_VERSION,
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_RAD_S,
    MS2_PER_G,
    compute_checksum,
)


def test_v2_frame_with_imu_samples():
    """V2 frame with 3 IMU samples should be 48 bytes with correct layout."""
    r = SwitchReport(
        buttons=0,
        imu_samples=[
            IMUSample(100, -200, 4096, 50, -50, 0),
            IMUSample(101, -201, 4097, 51, -51, 1),
            IMUSample(102, -202, 4098, 52, -52, 2),
        ],
    )
    data = r.to_bytes()
    assert len(data) == 48, f"Expected 48 bytes, got {len(data)}"
    assert data[0] == UART_HEADER  # 0xAA
    assert data[1] == UART_PROTOCOL_VERSION  # 0x02
    assert data[2] == 44  # payload_len
    assert data[10] == 3  # imu_count
    # Verify checksum
    assert data[-1] == compute_checksum(data[:-1])
    # Verify first sample accel_x (int16 LE at byte 11)
    ax0 = struct.unpack_from("<h", data, 11)[0]
    assert ax0 == 100, f"Expected accel_x=100, got {ax0}"
    # Verify first sample gyro_z (int16 LE at bytes 21-22)
    gz0 = struct.unpack_from("<h", data, 21)[0]
    assert gz0 == 0, f"Expected gyro_z=0, got {gz0}"


def test_v2_frame_no_imu():
    """V2 frame with no IMU samples should be 12 bytes."""
    r = SwitchReport(
        buttons=0x0004, hat=SwitchDpad.CENTER, lx=128, ly=128, rx=128, ry=128
    )
    data = r.to_bytes()
    assert len(data) == 12, f"Expected 12 bytes, got {len(data)}"
    assert data[0] == UART_HEADER
    assert data[1] == UART_PROTOCOL_VERSION
    assert data[2] == 8  # payload_len
    assert data[10] == 0  # imu_count
    assert data[-1] == compute_checksum(data[:-1])


def test_checksum_validation():
    """Checksum should match sum of all preceding bytes & 0xFF."""
    r = SwitchReport(buttons=0x0001)
    data = r.to_bytes()
    expected_checksum = sum(data[:-1]) & 0xFF
    assert data[-1] == expected_checksum
    # Corrupt a byte and verify mismatch
    corrupted = bytearray(data)
    corrupted[3] ^= 0xFF  # flip bits in first payload byte
    recalculated = sum(corrupted[:-1]) & 0xFF
    assert corrupted[-1] != recalculated, "Checksum should not match corrupted data"


def test_accel_scale_gravity():
    """1G (9.80665 m/s²) should convert to ~4096 raw counts."""
    # convert_accel_to_raw(9.80665) ≈ 4096
    raw = int(round((MS2_PER_G / MS2_PER_G) * ACCEL_LSB_PER_G))
    assert abs(raw - 4096) <= 5, f"Expected ~4096 for 1G, got {raw}"


def test_gyro_scale_one_rad():
    """1.0 rad/s should convert to ~818 raw counts."""
    raw = int(round(1.0 * GYRO_LSB_PER_RAD_S))
    assert abs(raw - 818) <= 5, f"Expected ~818 for 1 rad/s, got {raw}"


def test_imu_sample_dataclass():
    """IMUSample fields accept int16 range values."""
    s = IMUSample(
        accel_x=32767, accel_y=-32768, accel_z=0, gyro_x=100, gyro_y=-100, gyro_z=1000
    )
    assert s.accel_x == 32767
    assert s.accel_y == -32768
    assert s.gyro_z == 1000
    # Values outside int16 range are clamped in to_bytes()
    s2 = IMUSample(accel_x=99999)
    r = SwitchReport(imu_samples=[s2])
    data = r.to_bytes()
    ax = struct.unpack_from("<h", data, 11)[0]
    assert ax == 32767, f"Expected clamped value 32767, got {ax}"


def test_backward_compat_switch_report():
    """SwitchReport with no imu_samples produces valid v2 frame (backward compat)."""
    r = SwitchReport(buttons=0x000A, lx=200, ly=50, rx=128, ry=128)
    data = r.to_bytes()
    assert len(data) == 12
    assert data[1] == 0x02  # still v2
    # Buttons at bytes 3-4
    buttons = struct.unpack_from("<H", data, 3)[0]
    assert buttons == 0x000A
    # lx at byte 6
    assert data[6] == 200


def test_max_imu_samples_capped():
    """Providing >3 IMU samples should cap at 3."""
    samples = [IMUSample(i, 0, 0, 0, 0, 0) for i in range(5)]
    r = SwitchReport(imu_samples=samples)
    data = r.to_bytes()
    assert len(data) == 48  # 3 samples, not 5
    assert data[10] == 3
    assert data[2] == 44  # payload_len for 3 samples
