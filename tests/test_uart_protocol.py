"""Tests for UART v3 protocol serialization in switch_pico_uart."""

import struct
import pytest
from switch_pico_bridge.switch_pico_uart import (
    SwitchReport,
    IMUSample,
    SwitchDpad,
    PicoUART,
    UART_HEADER,
    UART_PROTOCOL_VERSION,
    UART_SLOT_COUNT,
    RUMBLE_HEADER,
    RUMBLE_TYPE_DECODED,
    RUMBLE_TYPE_SLOT,
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_RAD_S,
    MS2_PER_G,
    compute_checksum,
)


class BufferedSerial:
    def __init__(self, data: bytes = b""):
        self._data = bytearray(data)

    @property
    def in_waiting(self) -> int:
        return len(self._data)

    def read(self, size: int) -> bytes:
        data = bytes(self._data[:size])
        del self._data[:size]
        return data

    def feed(self, data: bytes) -> None:
        self._data.extend(data)


def make_rumble_frame(low: int, high: int, slot: int = 0) -> bytes:
    frame = bytes([RUMBLE_HEADER, RUMBLE_TYPE_SLOT, slot, low, high])
    return frame + bytes([compute_checksum(frame)])


def make_legacy_rumble_frame(low: int, high: int) -> bytes:
    frame = bytes([RUMBLE_HEADER, RUMBLE_TYPE_DECODED, low, high])
    return frame + bytes([compute_checksum(frame)])


def make_uart(data: bytes = b"") -> tuple[PicoUART, BufferedSerial]:
    uart = object.__new__(PicoUART)
    serial_port = BufferedSerial(data)
    uart.serial = serial_port
    uart._buffer = bytearray()
    return uart, serial_port


def test_v3_frame_with_imu_samples():
    """V3 frame with 3 IMU samples should be 49 bytes with correct layout."""
    r = SwitchReport(
        buttons=0,
        imu_samples=[
            IMUSample(100, -200, 4096, 50, -50, 0),
            IMUSample(101, -201, 4097, 51, -51, 1),
            IMUSample(102, -202, 4098, 52, -52, 2),
        ],
    )
    data = r.to_bytes()
    assert len(data) == 49, f"Expected 49 bytes, got {len(data)}"
    assert data[0] == UART_HEADER  # 0xAA
    assert data[1] == UART_PROTOCOL_VERSION  # 0x03
    assert data[2] == 44  # payload_len
    assert data[3] == 0  # slot
    assert data[11] == 3  # imu_count
    # Verify checksum
    assert data[-1] == compute_checksum(data[:-1])
    # Verify first sample accel_x (int16 LE at byte 12)
    ax0 = struct.unpack_from("<h", data, 12)[0]
    assert ax0 == 100, f"Expected accel_x=100, got {ax0}"
    # Verify first sample gyro_z (int16 LE at bytes 22-23)
    gz0 = struct.unpack_from("<h", data, 22)[0]
    assert gz0 == 0, f"Expected gyro_z=0, got {gz0}"


def test_v3_frame_no_imu():
    """V3 frame with no IMU samples should be 13 bytes."""
    r = SwitchReport(
        buttons=0x0004, hat=SwitchDpad.CENTER, lx=128, ly=128, rx=128, ry=128
    )
    data = r.to_bytes()
    assert len(data) == 13, f"Expected 13 bytes, got {len(data)}"
    assert data[0] == UART_HEADER
    assert data[1] == UART_PROTOCOL_VERSION
    assert data[2] == 8  # payload_len
    assert data[3] == 0  # slot
    assert data[11] == 0  # imu_count
    assert data[-1] == compute_checksum(data[:-1])


def test_v3_frame_addresses_slot():
    """The slot byte selects which emulated controller receives the report."""
    data = SwitchReport(buttons=0x0001).to_bytes(slot=3)
    assert data[3] == 3
    assert struct.unpack_from("<H", data, 4)[0] == 0x0001
    assert data[-1] == compute_checksum(data[:-1])
    with pytest.raises(ValueError):
        SwitchReport().to_bytes(slot=UART_SLOT_COUNT)
    with pytest.raises(ValueError):
        SwitchReport().to_bytes(slot=-1)


def test_checksum_validation():
    """Checksum should match sum of all preceding bytes & 0xFF."""
    r = SwitchReport(buttons=0x0001)
    data = r.to_bytes()
    expected_checksum = sum(data[:-1]) & 0xFF
    assert data[-1] == expected_checksum
    # Corrupt a byte and verify mismatch
    corrupted = bytearray(data)
    corrupted[4] ^= 0xFF  # flip bits in first payload byte
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
    ax = struct.unpack_from("<h", data, 12)[0]
    assert ax == 32767, f"Expected clamped value 32767, got {ax}"


def test_switch_report_payload_layout():
    """Buttons and axes land at the documented v3 payload offsets."""
    r = SwitchReport(buttons=0x000A, lx=200, ly=50, rx=128, ry=128)
    data = r.to_bytes()
    assert len(data) == 13
    assert data[1] == 0x03
    # Buttons at bytes 4-5
    buttons = struct.unpack_from("<H", data, 4)[0]
    assert buttons == 0x000A
    # lx at byte 7
    assert data[7] == 200


def test_max_imu_samples_capped():
    """Providing >3 IMU samples should cap at 3."""
    samples = [IMUSample(i, 0, 0, 0, 0, 0) for i in range(5)]
    r = SwitchReport(imu_samples=samples)
    data = r.to_bytes()
    assert len(data) == 49  # 3 samples, not 5
    assert data[11] == 3
    assert data[2] == 44  # payload_len for 3 samples


def test_decoded_rumble_frame_survives_fragmented_input():
    frame = make_rumble_frame(64, 192, slot=1)
    uart, serial_port = make_uart(frame[:3])

    assert uart.read_rumble() is None

    serial_port.feed(frame[3:])
    assert uart.read_rumble() == pytest.approx((1, 64 / 255.0, 192 / 255.0))


def test_decoded_rumble_frame_resynchronizes_after_garbage():
    uart, _ = make_uart(b"\x00\xffnot-a-frame" + make_rumble_frame(12, 34))

    assert uart.read_rumble() == pytest.approx((0, 12 / 255.0, 34 / 255.0))


def test_decoded_rumble_frame_rejects_bad_checksum():
    corrupted = bytearray(make_rumble_frame(25, 50))
    corrupted[-1] ^= 0x01
    uart, _ = make_uart(bytes(corrupted) + make_rumble_frame(75, 100, slot=2))

    assert uart.read_rumble() == pytest.approx((2, 75 / 255.0, 100 / 255.0))


def test_decoded_rumble_zero_and_full_magnitudes():
    uart, _ = make_uart(make_rumble_frame(0, 0) + make_rumble_frame(255, 255, slot=3))

    assert uart.read_rumble() == (0, 0.0, 0.0)
    assert uart.read_rumble() == (3, 1.0, 1.0)


def test_legacy_rumble_frame_maps_to_slot_zero():
    """Pre-multi-controller firmware sends 5-byte frames without a slot byte."""
    uart, _ = make_uart(make_legacy_rumble_frame(10, 20) + make_rumble_frame(30, 40, slot=1))

    assert uart.read_rumble() == pytest.approx((0, 10 / 255.0, 20 / 255.0))
    assert uart.read_rumble() == pytest.approx((1, 30 / 255.0, 40 / 255.0))


def test_rumble_frame_with_out_of_range_slot_is_skipped():
    uart, _ = make_uart(make_rumble_frame(1, 2, slot=UART_SLOT_COUNT) + make_rumble_frame(3, 4))

    assert uart.read_rumble() == pytest.approx((0, 3 / 255.0, 4 / 255.0))


def test_reboot_bootsel_frame_matches_firmware_contract():
    """0xAA 0xFE len(8) cmd(1) 'BOOTSEL' checksum: 12 bytes, the parser's minimum frame."""
    frame = PicoUART.reboot_bootsel_frame()
    assert frame[:3] == bytes([UART_HEADER, 0xFE, 8])
    assert frame[3] == 0x01
    assert frame[4:11] == b"BOOTSEL"
    assert len(frame) == 12
    assert frame[-1] == compute_checksum(frame[:-1])
