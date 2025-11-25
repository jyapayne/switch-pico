#!/usr/bin/env python3
"""
Lightweight helpers for talking to the switch-pico firmware over UART.

This module exposes the raw report structure plus a small convenience wrapper
so other scripts can do things like "press a button" or "move a stick" without
depending on SDL. It mirrors the framing in ``switch-pico.cpp``:

  Host -> Pico : 0xAA, buttons (LE16), hat, lx, ly, rx, ry
  Pico -> Host : 0xBB, 0x01, 8 rumble bytes, checksum (sum of first 10 bytes)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import Iterable, Mapping, Optional, Tuple, Union

import serial

UART_HEADER = 0xAA
RUMBLE_HEADER = 0xBB
RUMBLE_TYPE_RUMBLE = 0x01
UART_BAUD = 921600


class SwitchButton(IntFlag):
    # Mirrors the masks defined in switch_pro_descriptors.h
    Y = 1 << 0
    B = 1 << 1
    A = 1 << 2
    X = 1 << 3
    L = 1 << 4
    R = 1 << 5
    ZL = 1 << 6
    ZR = 1 << 7
    MINUS = 1 << 8
    PLUS = 1 << 9
    LCLICK = 1 << 10
    RCLICK = 1 << 11
    HOME = 1 << 12
    CAPTURE = 1 << 13


class SwitchHat(IntEnum):
    TOP = 0x00
    TOP_RIGHT = 0x01
    RIGHT = 0x02
    BOTTOM_RIGHT = 0x03
    BOTTOM = 0x04
    BOTTOM_LEFT = 0x05
    LEFT = 0x06
    TOP_LEFT = 0x07
    CENTER = 0x08


def clamp_byte(value: Union[int, float]) -> int:
    """Clamp a numeric value to the 0-255 byte range."""
    return max(0, min(255, int(value)))


def normalize_stick_value(value: Union[int, float]) -> int:
    """
    Convert a normalized float (-1..1) or raw byte (0..255) to the stick range.

    Floats are treated as -1.0 = full negative deflection, 0.0 = center,
    1.0 = full positive deflection. Integers are assumed to already be in the
    0-255 range.
    """
    if isinstance(value, float):
        value = max(-1.0, min(1.0, value))
        value = int(round((value + 1.0) * 255 / 2.0))
    return clamp_byte(value)


def axis_to_stick(value: int, deadzone: int) -> int:
    """Convert a signed axis value to 0-255 stick range with deadzone."""
    if abs(value) < deadzone:
        value = 0
    scaled = int((value + 32768) * 255 / 65535)
    return clamp_byte(scaled)


def trigger_to_button(value: int, threshold: int) -> bool:
    """Return True if analog trigger crosses digital threshold."""
    return value >= threshold


def dpad_to_hat(flags: Mapping[str, bool]) -> SwitchHat:
    """Translate DPAD button flags into a Switch hat value."""
    up = flags.get("up", False)
    down = flags.get("down", False)
    left = flags.get("left", False)
    right = flags.get("right", False)

    if up and right:
        return SwitchHat.TOP_RIGHT
    if up and left:
        return SwitchHat.TOP_LEFT
    if down and right:
        return SwitchHat.BOTTOM_RIGHT
    if down and left:
        return SwitchHat.BOTTOM_LEFT
    if up:
        return SwitchHat.TOP
    if down:
        return SwitchHat.BOTTOM
    if right:
        return SwitchHat.RIGHT
    if left:
        return SwitchHat.LEFT
    return SwitchHat.CENTER


@dataclass
class SwitchReport:
    buttons: int = 0
    hat: SwitchHat = SwitchHat.CENTER
    lx: int = 128
    ly: int = 128
    rx: int = 128
    ry: int = 128

    def to_bytes(self) -> bytes:
        """Serialize the report into the UART packet format."""
        return struct.pack(
            "<BHBBBBB", UART_HEADER, self.buttons & 0xFFFF, self.hat & 0xFF, self.lx, self.ly, self.rx, self.ry
        )


class PicoUART:
    def __init__(self, port: str, baudrate: int = UART_BAUD) -> None:
        """Open a UART connection to the Pico with non-blocking IO."""
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0.0,
            write_timeout=0.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._buffer = bytearray()

    def send_report(self, report: SwitchReport) -> None:
        """Send a controller report to the Pico."""
        self.serial.write(report.to_bytes())

    def read_rumble_payload(self) -> Optional[bytes]:
        """
        Drain available UART bytes into an internal buffer, then extract one rumble frame.

        Frame format:
          0: 0xBB (RUMBLE_HEADER)
          1: type (0x01 for rumble)
          2-9: 8-byte rumble payload
          10: checksum (sum of first 10 bytes) & 0xFF
        """
        waiting = self.serial.in_waiting
        if waiting:
            self._buffer.extend(self.serial.read(waiting))

        while True:
            if not self._buffer:
                return None

            start = self._buffer.find(bytes([RUMBLE_HEADER]))
            if start < 0:
                self._buffer.clear()
                return None

            if len(self._buffer) - start < 11:
                if start > 0:
                    del self._buffer[:start]
                return None

            frame = self._buffer[start:start + 11]
            checksum = sum(frame[:10]) & 0xFF

            if frame[1] == RUMBLE_TYPE_RUMBLE and checksum == frame[10]:
                payload = bytes(frame[2:10])
                del self._buffer[:start + 11]
                return payload

            del self._buffer[:start + 1]

    def close(self) -> None:
        """Close the UART connection."""
        self.serial.close()


def decode_rumble(payload: bytes) -> Tuple[float, float]:
    """Return normalized rumble amplitudes (0.0-1.0) for left/right."""
    if len(payload) < 8:
        return 0.0, 0.0
    if payload == b"\x00\x01\x40\x40\x00\x01\x40\x40":
        return 0.0, 0.0
    right_raw = ((payload[1] & 0x03) << 8) | payload[0]
    left_raw = ((payload[5] & 0x03) << 8) | payload[4]
    if left_raw < 8 and right_raw < 8:
        return 0.0, 0.0
    left = min(max(left_raw / 1023.0, 0.0), 1.0)
    right = min(max(right_raw / 1023.0, 0.0), 1.0)
    return left, right


@dataclass
class SwitchControllerState:
    """Mutable controller state with helpers for building reports."""

    report: SwitchReport = field(default_factory=SwitchReport)

    def press(self, *buttons: Union[SwitchButton, int]) -> None:
        """Set one or more buttons as pressed."""
        for button in buttons:
            self.report.buttons |= int(button)

    def release(self, *buttons: Union[SwitchButton, int]) -> None:
        """Release one or more buttons."""
        for button in buttons:
            self.report.buttons &= ~int(button)

    def set_buttons(self, buttons: Iterable[Union[SwitchButton, int]]) -> None:
        """Replace the current button bitmask with the provided buttons."""
        self.report.buttons = 0
        self.press(*buttons)

    def set_hat(self, hat: Union[SwitchHat, int]) -> None:
        """Set the DPAD/hat value directly."""
        self.report.hat = int(hat) & 0xFF

    def move_left_stick(self, x: Union[int, float], y: Union[int, float]) -> None:
        """Move the left stick using normalized floats (-1..1) or raw bytes (0-255)."""
        self.report.lx = normalize_stick_value(x)
        self.report.ly = normalize_stick_value(y)

    def move_right_stick(self, x: Union[int, float], y: Union[int, float]) -> None:
        """Move the right stick using normalized floats (-1..1) or raw bytes (0-255)."""
        self.report.rx = normalize_stick_value(x)
        self.report.ry = normalize_stick_value(y)

    def neutral(self) -> None:
        """Clear all input back to the neutral controller state."""
        self.report.buttons = 0
        self.report.hat = SwitchHat.CENTER
        self.report.lx = 128
        self.report.ly = 128
        self.report.rx = 128
        self.report.ry = 128


class SwitchUARTClient:
    """
    High-level helper to send controller actions to the Pico and poll rumble.

    Example:
        with SwitchUARTClient("/dev/cu.usbserial-0001") as client:
            client.press(SwitchButton.A)
            time.sleep(0.1)
            client.release(SwitchButton.A)
            client.move_left_stick(0.0, -1.0)  # push up
    """

    def __init__(self, port: str, baud: int = UART_BAUD, send_interval: float = 0.0) -> None:
        self.uart = PicoUART(port, baud)
        self.state = SwitchControllerState()
        self.send_interval = max(0.0, send_interval)
        self._last_send = 0.0

    def send(self) -> None:
        """Send the current state to the Pico, throttled by send_interval if set."""
        now = time.monotonic()
        if self.send_interval and (now - self._last_send) < self.send_interval:
            return
        self.uart.send_report(self.state.report)
        self._last_send = now

    def press(self, *buttons: int) -> None:
        self.state.press(*buttons)
        self.send()

    def release(self, *buttons: int) -> None:
        self.state.release(*buttons)
        self.send()

    def set_buttons(self, buttons: Iterable[int]) -> None:
        self.state.set_buttons(buttons)
        self.send()

    def set_hat(self, hat: int) -> None:
        self.state.set_hat(hat)
        self.send()

    def move_left_stick(self, x: Union[int, float], y: Union[int, float]) -> None:
        self.state.move_left_stick(x, y)
        self.send()

    def move_right_stick(self, x: Union[int, float], y: Union[int, float]) -> None:
        self.state.move_right_stick(x, y)
        self.send()

    def neutral(self) -> None:
        self.state.neutral()
        self.send()

    def poll_rumble(self) -> Optional[Tuple[float, float]]:
        """
        Poll for the latest rumble payload and return normalized amplitudes.
        Returns None if no rumble frame was available.
        """
        payload = self.uart.read_rumble_payload()
        if payload:
            return decode_rumble(payload)
        return None

    def close(self) -> None:
        self.uart.close()

    def __enter__(self) -> "SwitchUARTClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
