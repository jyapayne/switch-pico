"""RAM-only, neutral RVL-CNT-01 protocol for the DolphinBar experiment.

Wire formats: https://wiibrew.org/wiki/Wiimote . Report handling facts were
cross-checked against rnconrad/WiimoteEmulator and Dolphin's WiimoteEmu.
This module contains no transport, physical inputs, audio output, or file I/O.
"""

from __future__ import annotations

_OUTPUT_LENGTHS = {
    0x10: 1,
    0x11: 1,
    0x12: 2,
    0x13: 1,
    0x14: 1,
    0x15: 1,
    0x16: 21,
    0x17: 6,
    0x18: 21,
    0x19: 1,
    0x1A: 1,
}
_INPUT_LENGTHS = {
    0x20: 6,
    0x21: 21,
    0x22: 4,
    0x30: 2,
    0x31: 5,
    0x32: 10,
    0x33: 17,
    0x34: 21,
    0x35: 21,
    0x36: 21,
    0x37: 21,
    0x3D: 21,
    0x3E: 21,
    0x3F: 21,
}


def _hid_descriptor() -> bytes:
    # Generic Desktop/Game Pad application; vendor-defined, opaque byte arrays.
    descriptor = bytearray.fromhex("05 01 09 05 a1 01 15 00 26 ff 00 75 08 06 00 ff")
    for lengths, item in ((_OUTPUT_LENGTHS, 0x91), (_INPUT_LENGTHS, 0x81)):
        for report_id, size in lengths.items():
            descriptor.extend((0x85, report_id, 0x95, size, 0x09, 0x01, item, 0x00))
    descriptor.append(0xC0)
    return bytes(descriptor)


HID_DESCRIPTOR = _hid_descriptor()

# High eight accelerometer bits: zero-g = 0x80, one-g = 0x9a.
# A stationary, face-up remote measures (0g, 0g, +1g); low bits are zero.
_ACCEL_ZERO = 0x80
_ACCEL_ONE = 0x9A
_ACCEL = bytes((_ACCEL_ZERO, _ACCEL_ZERO, _ACCEL_ONE))
_BUTTONS = b"\x00\x00"
_ABSENT = b"\xff" * 21
_NEUTRAL_REPORTS = {
    0x30: b"\xa1\x30" + _BUTTONS,
    0x31: b"\xa1\x31" + _BUTTONS + _ACCEL,
    0x32: b"\xa1\x32" + _BUTTONS + _ABSENT[:8],
    0x33: b"\xa1\x33" + _BUTTONS + _ACCEL + _ABSENT[:12],
    0x34: b"\xa1\x34" + _BUTTONS + _ABSENT[:19],
    0x35: b"\xa1\x35" + _BUTTONS + _ACCEL + _ABSENT[:16],
    0x36: b"\xa1\x36" + _BUTTONS + _ABSENT[:19],
    0x37: b"\xa1\x37" + _BUTTONS + _ACCEL + _ABSENT[:16],
    0x3D: b"\xa1\x3d" + _ABSENT,
    # The interleaved button fields carry the high and low nibbles of Z's
    # eight-bit value, not the low bits used by normal accelerometer reports.
    0x3E: bytes(
        (
            0xA1,
            0x3E,
            ((_ACCEL_ONE >> 4) & 3) << 5,
            ((_ACCEL_ONE >> 6) & 3) << 5,
            _ACCEL_ZERO,
        )
    )
    + _ABSENT[:18],
    0x3F: bytes(
        (0xA1, 0x3F, (_ACCEL_ONE & 3) << 5, ((_ACCEL_ONE >> 2) & 3) << 5, _ACCEL_ZERO)
    )
    + _ABSENT[:18],
}


def _calibration_block(data: bytes) -> bytes:
    return data + bytes(((sum(data) + 0x55) & 0xFF,))


def _new_eeprom() -> bytearray:
    eeprom = bytearray(0x1700)
    # Symmetric factory reference points in all four camera quadrants. These
    # are calibration constants only, never emitted as observed IR spots.
    points = ((128, 128), (896, 128), (128, 640), (896, 640))
    packed = bytearray()
    for index in (0, 2):
        x1, y1 = points[index]
        x2, y2 = points[index + 1]
        high = ((y1 >> 8) << 6) | ((x1 >> 8) << 4) | ((y2 >> 8) << 2) | (x2 >> 8)
        packed.extend((x1 & 0xFF, y1 & 0xFF, high, x2 & 0xFF, y2 & 0xFF))
    ir_calibration = _calibration_block(bytes(packed))
    accel_calibration = _calibration_block(
        bytes(
            (
                _ACCEL_ZERO,
                _ACCEL_ZERO,
                _ACCEL_ZERO,
                0,
                _ACCEL_ONE,
                _ACCEL_ONE,
                _ACCEL_ONE,
                0,
                0x40,
            )
        )
    )
    eeprom[0x00:0x0B] = ir_calibration
    eeprom[0x0B:0x16] = ir_calibration
    eeprom[0x16:0x20] = accel_calibration
    eeprom[0x20:0x2A] = accel_calibration
    return eeprom


class Wiimote:
    """An original remote with no buttons pressed, extension, or visible IR.

    ``handle_output`` consumes report ID + its exact descriptor-sized payload;
    callers remove the Bluetooth 0xa2/0x52 header. Every returned packet already
    includes 0xa1. Drain command replies before requesting periodic reports.
    There is no internal reply queue: a read returns at most 368 packets.
    EEPROM writes change only this instance's RAM, not the simulated sensor.
    """

    def __init__(self) -> None:
        self.ir_enabled = False  # Report 0x13 also controls the status flag/I2C.
        self.ir_secondary_enabled = False  # Report 0x1a, the second camera gate.
        self.report_mode = 0x30
        self.rumble = False
        self.leds = 0
        self.speaker_enabled = False
        self.speaker_muted = False
        self.speaker_data = b""  # Last accepted FIFO packet only; never played.
        self._continuous = False
        self._report_pending = True
        self._interleaved_next = 0x3E
        self._pair_pending = False
        self._eeprom = _new_eeprom()
        self._speaker = bytearray(0x0A)
        self._camera = bytearray(0x5B)
        self._camera[0x37:0x5B] = b"\xff" * 36

    @staticmethod
    def _ack(report_id: int, error: int = 0) -> bytes:
        return bytes((0xA1, 0x22, 0, 0, report_id, error))

    @staticmethod
    def _read_reply(address: int, data: bytes = b"", error: int = 0) -> bytes:
        size = 16 if error else len(data)
        return bytes(
            (
                0xA1,
                0x21,
                0,
                0,
                ((size - 1) << 4) | error,
                (address >> 8) & 0xFF,
                address & 0xFF,
            )
        ) + data.ljust(16, b"\x00")

    def status_report(self) -> bytes:
        """Return full battery, no extension, and the actual LED/feature flags."""
        flags = (
            self.leds | (int(self.speaker_enabled) << 2) | (int(self.ir_enabled) << 3)
        )
        return bytes((0xA1, 0x20, 0, 0, flags, 0, 0, 0xC0))

    def _memory(
        self, flags: int, address: int, size: int, writing: bool = False
    ) -> tuple[bytearray | None, int, int]:
        """Resolve the whole transfer before mutation; never resize a bank."""
        space = flags & 0x0C
        if space == 0x0C:
            return None, 0, 6  # Invalid address space, not EEPROM or I2C.
        if space == 0:
            offset = address & 0xFFFF  # EEPROM mirrors every 64 KiB.
            if offset + size <= len(self._eeprom):
                return self._eeprom, offset, 0
            return None, 0, 8

        peripheral = (address >> 16) & 0xFE
        offset = address & 0xFF  # Peripheral register high address byte is ignored.
        if peripheral in (0xA4, 0xA6):
            return None, 0, 7  # No extension or MotionPlus on the I2C bus.
        if peripheral == 0xA2:
            if offset + size <= len(self._speaker):
                return self._speaker, offset, 0
        elif peripheral == 0xB0:
            if not self.ir_enabled:
                return None, 0, 7
            if offset + size <= 0x34:
                return self._camera, offset, 0
            if 0x37 <= offset and offset + size <= len(self._camera):
                if writing:
                    return None, 0, 7  # Sensor output is read-only.
                return self._camera, offset, 0
        return None, 0, 8

    def handle_output(self, report: bytes) -> list[bytes]:
        """Apply a well-formed command atomically and return its wire replies.

        Unsupported/malformed commands return error 3, absent I2C extensions
        error 7, invalid addresses error 8, invalid address-space selection 6.
        Malformed reports never even change rumble. Empty input has no report
        ID to acknowledge and is ignored. Valid writes always ACK; feature/mode
        commands ACK only when requested by bit 1. Status, reads, rumble and
        speaker streaming have their own reply rules, as on the original remote.
        """
        if not report:
            return []
        report_id = report[0]
        expected = _OUTPUT_LENGTHS.get(report_id)
        if expected is None or len(report) != expected + 1:
            return [self._ack(report_id, 3)]
        flags = report[1]

        if report_id == 0x12 and report[2] not in _NEUTRAL_REPORTS:
            return [self._ack(report_id, 3)]
        if report_id == 0x18 and not 1 <= flags >> 3 <= 20:
            return [self._ack(report_id, 3)]

        if report_id in (0x16, 0x17):
            address = int.from_bytes(report[2:5], "big")
            size = (
                report[5] if report_id == 0x16 else int.from_bytes(report[5:7], "big")
            )
            if size == 0 or (report_id == 0x16 and size > 16):
                if report_id == 0x17:
                    return [self._read_reply(address, error=3)]
                return [self._ack(report_id, 3)]
            # Register zero of the speaker is a streaming FIFO, not a write
            # spanning configuration registers. Keep only the latest samples.
            speaker_fifo = (
                report_id == 0x16
                and flags & 0x0C in (4, 8)
                and (address >> 16) & 0xFE == 0xA2
                and address & 0xFF == 0
            )
            if speaker_fifo:
                self.speaker_data = report[6 : 6 + size]
                self.rumble = bool(flags & 1)
                return [self._ack(report_id)]
            bank, offset, error = self._memory(flags, address, size, report_id == 0x16)
            if error:
                if report_id == 0x17:
                    return [self._read_reply(address, error=error)]
                return [self._ack(report_id, error)]
            self.rumble = bool(flags & 1)
            if report_id == 0x16:
                bank[offset : offset + size] = report[6 : 6 + size]
                return [self._ack(report_id)]
            return [
                self._read_reply(
                    address + index,
                    bytes(bank[offset + index : offset + min(index + 16, size)]),
                )
                for index in range(0, size, 16)
            ]

        self.rumble = bool(flags & 1)
        if report_id == 0x10:
            return []
        if report_id == 0x11:
            self.leds = flags & 0xF0
        elif report_id == 0x12:
            self.report_mode = report[2]
            self._continuous = bool(flags & 4)
            self._report_pending = True
            self._interleaved_next = 0x3E
            self._pair_pending = False
        elif report_id == 0x13:
            self.ir_enabled = bool(flags & 4)
        elif report_id == 0x14:
            self.speaker_enabled = bool(flags & 4)
        elif report_id == 0x15:
            return [self.status_report()]
        elif report_id == 0x18:
            self.speaker_data = report[2 : 2 + (flags >> 3)]
            return []
        elif report_id == 0x19:
            self.speaker_muted = bool(flags & 4)
        elif report_id == 0x1A:
            self.ir_secondary_enabled = bool(flags & 4)
        return [self._ack(report_id)] if flags & 2 else []

    def periodic_report(self) -> bytes | None:
        """Return one 100-Hz tick's neutral report, or None if unchanged.

        Interleaved modes always finish their 0x3e/0x3f pair, even with continuous
        reporting disabled. The requested mode remains stable for logging.
        Immutable packets are shared; idle/continuous ticks do not allocate.
        """
        if not (self._continuous or self._report_pending or self._pair_pending):
            return None
        self._report_pending = False
        if self.report_mode in (0x3E, 0x3F):
            report_id = self._interleaved_next
            self._pair_pending = report_id == 0x3E
            self._interleaved_next = 0x3F if self._pair_pending else 0x3E
            return _NEUTRAL_REPORTS[report_id]
        return _NEUTRAL_REPORTS[self.report_mode]
