#!/usr/bin/env python3
"""Manage switch-pico USB modes, configuration, profiles, and pairings."""

from __future__ import annotations

import argparse
import json
import secrets
import struct
import sys
import time
import zlib
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, cast

import usb.core

USB_IDENTITIES = (
    (0x057E, 0x2009),
    (0xCAFE, 0x4010),
    (0xCAFE, 0x4020),
    (0xCAFE, 0x4021),
)
REQUEST_VALUE = 0x5350
REQUEST_INDEX = 0x0001
PROTOCOL_VERSION = 1
REQUEST_HEADER_SIZE = 16
RESPONSE_HEADER_SIZE = 20
MAXIMUM_REQUEST_SIZE = 64
MAXIMUM_RESPONSE_SIZE = 293
MAXIMUM_CHUNK_SIZE = 40
USB_TIMEOUT_MS = 1000
DEFAULT_OPERATION_TIMEOUT_SECONDS = 15.0
HOST_TRANSACTION_ID_MASK = 0x7FFFFFFF

OP_INFO = 0x01
OP_MODE_SET = 0x02
OP_REBOOT = 0x03
OP_BOOTSEL_REBOOT = 0x04
OP_CONFIGURATION_READ = 0x10
OP_CONFIGURATION_BEGIN = 0x11
OP_CONFIGURATION_CHUNK = 0x12
OP_CONFIGURATION_COMMIT = 0x13
OP_CONFIGURATION_RESET = 0x14
OP_TRANSACTION_STATUS = 0x15
OP_PAIRING_READ = 0x20
OP_PAIRING_REFRESH = 0x21
OP_PAIRING_CLEAR = 0x22
OP_RUNTIME_DIAGNOSTICS = 0x23
OP_PROFILE_LIST = 0x30
OP_PROFILE_SELECT = 0x31
OP_PROFILE_READ = 0x32
OP_PROFILE_BEGIN = 0x33
OP_PROFILE_CHUNK = 0x34
OP_PROFILE_COMMIT = 0x35
OP_PROFILE_RESET = 0x36
OP_PROFILE_ACTIVATE = 0x37
OP_PROFILE_TRANSACTION_STATUS = 0x38

STATUS_OK = 0
STATUS_PENDING = 1
STATUS_NAMES = {
    2: "malformed request",
    3: "unsupported schema",
    4: "value too large",
    5: "out-of-order transaction",
    6: "CRC mismatch",
    7: "device busy",
    8: "storage failure",
}

CONFIGURATION_SCHEMA_VERSION = 2
CONFIGURATION_SIZE = 8
PAIRING_WINDOW_SECONDS_MIN = 10
PAIRING_WINDOW_SECONDS_MAX = 300
REQUESTED_MODE_AUTO = 0
REQUESTED_MODE_SWITCH = 1
REQUESTED_MODE_XINPUT = 2
REQUESTED_MODE_DINPUT = 3
REQUESTED_MODE_MAC = 4
REQUESTED_MODE_NAMES = ("auto", "switch", "xinput", "dinput", "mac")
SELECTABLE_MODE_NAMES = REQUESTED_MODE_NAMES
ACTIVE_MODE_SWITCH = 0
ACTIVE_MODE_SWITCH_PROBE = 1
ACTIVE_MODE_XINPUT = 2
ACTIVE_MODE_DINPUT = 3
ACTIVE_MODE_MAC = 4
ACTIVE_MODE_NAMES = ("Switch", "Switch probe", "XInput", "DInput", "Mac")
# USB management info byte 5 capability flags.
CAPABILITY_INPUT = 1 << 0
CAPABILITY_RUMBLE = 1 << 1
CAPABILITY_MOTION = 1 << 2
CAPABILITY_MASK = CAPABILITY_INPUT | CAPABILITY_RUMBLE | CAPABILITY_MOTION
PAIRING_RECORD_SIZE = 8
PAIRING_RECORD_CAPACITY = 16
TRANSPORT_UNKNOWN = 0
TRANSPORT_CLASSIC = 1
TRANSPORT_BLE = 2
PROFILE_LEGACY_SCHEMA_VERSION = 1
PROFILE_SCHEMA_VERSION = 2
PROFILE_SIZE = 256
PROFILE_CAPACITY = 4
PROFILE_IDENTITY_CAPACITY = 16
PROFILE_LIST_CAPACITY = PROFILE_IDENTITY_CAPACITY + 1
CONTROLLER_IDENTITY_SIZE = 14
PROFILE_LIST_ROW_SIZE = 16
PROFILE_NONE_BUTTON = 0xFF
PROFILE_MACRO_STEP_CAPACITY = 8
PROFILE_MACRO_STEP_SIZE = 19
PROFILE_MAXIMUM_WAIT_MS = 10000
PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD = 0x8000
PROFILE_DEFAULT_DIGITAL_THRESHOLD = 22934

LOGICAL_BUTTONS = (
    "south",
    "east",
    "west",
    "north",
    "left_shoulder",
    "right_shoulder",
    "select",
    "start",
    "system",
    "capture",
    "left_stick",
    "right_stick",
    "dpad_up",
    "dpad_down",
    "dpad_left",
    "dpad_right",
)
RUMBLE_POLICIES = ("none", "rumble", "led", "rumble_and_led")
TURBO_MODES = ("off", "turbo", "auto_burst")
MACRO_STEP_TYPES = ("state", "end")
MACRO_OVERRIDE_NAMES = (
    "buttons",
    "left_stick",
    "right_stick",
    "left_trigger",
    "right_trigger",
)
MACRO_OVERRIDE_MASK = (1 << len(MACRO_OVERRIDE_NAMES)) - 1


class ConfigManagerError(RuntimeError):
    """Expected discovery, USB transport, or management protocol failure."""


class UsbDevice(Protocol):
    bus: int | None
    address: int | None
    port_numbers: tuple[int, ...] | None

    def ctrl_transfer(
        self,
        bm_request_type: int,
        request: int,
        value: int = 0,
        index: int = 0,
        data_or_w_length: Any = None,
        timeout: int | None = None,
    ) -> Any:
        ...


@dataclass(frozen=True)
class Envelope:
    operation: int
    status: int
    flags: int
    schema_version: int
    generation: int
    payload_crc: int
    payload: bytes


@dataclass(frozen=True)
class DeviceInfo:
    firmware_version: tuple[int, int, int]
    board: int
    active_mode: int
    capabilities: int
    maximum_configuration_size: int

    def mode_name(self) -> str:
        try:
            return ACTIVE_MODE_NAMES[self.active_mode]
        except IndexError as exc:
            raise ConfigManagerError(
                f"unknown active USB mode {self.active_mode}"
            ) from exc

    def capability_names(self) -> tuple[str, ...]:
        if self.capabilities == 0:
            return ("unreported",)
        names = ["input"]
        if self.capabilities & CAPABILITY_RUMBLE:
            names.append("rumble")
        if self.capabilities & CAPABILITY_MOTION:
            names.append("motion")
        return tuple(names)

    def capability_summary(self) -> str:
        names = self.capability_names()
        return "input only" if names == ("input",) else ", ".join(names)


@dataclass(frozen=True)
class RuntimeDiagnostics:
    initialization_stage: int
    rumble_timer_ticks: int
    configuration_timer_ticks: int
    controller_reports: int
    host_rumble_requests: int
    local_feedback_requests: int
    rumble_dispatches: int
    active_slots: int
    rumble_capable_slots: int
    feedback_pending_slots: int
    rumble_pending_slots: int

@dataclass(frozen=True)
class AdapterConfiguration:
    pairing_window_seconds: int
    generation: int
    crc: int
    requested_mode: int = REQUESTED_MODE_AUTO


@dataclass(frozen=True)
class TransactionStatus:
    transaction_id: int
    received_size: int
    expected_size: int
    expected_crc: int
    stored_generation: int
    stored_crc: int
    status: int


@dataclass(frozen=True)
class PairingRecord:
    transport: int
    address_type: int
    address: bytes

    @property
    def address_text(self) -> str:
        return ":".join(f"{octet:02X}" for octet in self.address)

    @property
    def transport_text(self) -> str:
        if self.transport == TRANSPORT_CLASSIC:
            return "Classic"
        if self.transport == TRANSPORT_BLE:
            address_types = {
                0: "public",
                1: "random",
                2: "public identity",
                3: "random identity",
            }
            suffix = address_types.get(
                self.address_type, f"type {self.address_type}"
            )
            return f"BLE ({suffix})"
        return f"unknown transport {self.transport}"


@dataclass(frozen=True)
class PairingSnapshot:
    generation: int
    pending: bool
    overflow: bool
    records: tuple[PairingRecord, ...]


def _require_int(value: Any, name: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ConfigManagerError(
            f"{name} must be an integer from {minimum} to {maximum}"
        )
    return value


def _require_bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise ConfigManagerError(f"{name} must be true or false")
    return value


def _require_object(
    value: Any, fields: Sequence[str], name: str
) -> dict[str, Any]:
    if type(value) is not dict:
        raise ConfigManagerError(f"{name} must be a JSON object")
    expected = set(fields)
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        details: list[str] = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if unknown:
            details.append("unknown " + ", ".join(unknown))
        raise ConfigManagerError(f"invalid {name}: {'; '.join(details)}")
    return value


def _require_enum(value: Any, choices: Sequence[str], name: str) -> int:
    if type(value) is not str or value not in choices:
        raise ConfigManagerError(
            f"{name} must be one of {', '.join(choices)}"
        )
    return choices.index(value)


def _button_index(value: Any, name: str) -> int:
    if value is None:
        return PROFILE_NONE_BUTTON
    if type(value) is not str or value not in LOGICAL_BUTTONS:
        raise ConfigManagerError(
            f"{name} must be a logical button name or null"
        )
    return LOGICAL_BUTTONS.index(value)


def _button_name(value: int) -> str | None:
    if value == PROFILE_NONE_BUTTON:
        return None
    return LOGICAL_BUTTONS[value]


def _button_mask_from_json(value: Any, name: str) -> int:
    if type(value) is not list:
        raise ConfigManagerError(f"{name} must be a JSON array")
    mask = 0
    for entry in value:
        index = _button_index(entry, name)
        if index == PROFILE_NONE_BUTTON:
            raise ConfigManagerError(f"{name} cannot contain null")
        bit = 1 << index
        if mask & bit:
            raise ConfigManagerError(f"{name} contains a duplicate button")
        mask |= bit
    return mask


def _button_mask_to_json(mask: int) -> list[str]:
    return [
        name
        for index, name in enumerate(LOGICAL_BUTTONS)
        if mask & (1 << index)
    ]


@dataclass(frozen=True)
class ControllerIdentity:
    stable: bool
    transport: int
    address_type: int
    address: bytes
    vendor_id: int
    product_id: int

    def __post_init__(self) -> None:
        _require_bool(self.stable, "identity stable")
        _require_int(self.transport, "identity transport", 0, 2)
        _require_int(self.address_type, "identity address_type", 0, 0xFF)
        if type(self.address) is not bytes or len(self.address) != 6:
            raise ConfigManagerError("identity address must contain six bytes")
        _require_int(self.vendor_id, "identity vendor_id", 0, 0xFFFF)
        _require_int(self.product_id, "identity product_id", 0, 0xFFFF)
        if self.stable:
            if self.transport not in (TRANSPORT_CLASSIC, TRANSPORT_BLE):
                raise ConfigManagerError(
                    "stable identity transport must be Classic or BLE"
                )
        elif (
            self.transport != TRANSPORT_UNKNOWN
            or self.address_type != 0
            or self.address != bytes(6)
            or self.vendor_id != 0
            or self.product_id != 0
        ):
            raise ConfigManagerError(
                "unstable identity must be the all-zero global fallback"
            )

    @classmethod
    def global_fallback(cls) -> ControllerIdentity:
        return cls(False, TRANSPORT_UNKNOWN, 0, bytes(6), 0, 0)

    @classmethod
    def from_bytes(cls, payload: bytes) -> ControllerIdentity:
        payload = bytes(payload)
        if len(payload) != CONTROLLER_IDENTITY_SIZE:
            raise ConfigManagerError("invalid controller identity size")
        stable, transport, address_type, reserved = payload[:4]
        if stable not in (0, 1) or reserved != 0:
            raise ConfigManagerError("invalid controller identity encoding")
        vendor_id, product_id = struct.unpack_from("<HH", payload, 10)
        return cls(
            stable=bool(stable),
            transport=transport,
            address_type=address_type,
            address=payload[4:10],
            vendor_id=vendor_id,
            product_id=product_id,
        )

    def to_bytes(self) -> bytes:
        return (
            bytes(
                [
                    int(self.stable),
                    self.transport,
                    self.address_type,
                    0,
                ]
            )
            + self.address
            + struct.pack("<HH", self.vendor_id, self.product_id)
        )

    @property
    def is_global_fallback(self) -> bool:
        return not self.stable

    @property
    def address_text(self) -> str:
        return ":".join(f"{octet:02X}" for octet in self.address)

    @property
    def transport_text(self) -> str:
        if self.transport == TRANSPORT_CLASSIC:
            return "Classic"
        if self.transport == TRANSPORT_BLE:
            return "BLE"
        return "Unknown"


@dataclass(frozen=True)
class ProfileListEntry:
    identity: ControllerIdentity
    active_profile_index: int

    def __post_init__(self) -> None:
        if not isinstance(self.identity, ControllerIdentity):
            raise ConfigManagerError(
                "profile-list identity must be a ControllerIdentity"
            )
        _require_int(
            self.active_profile_index,
            "active profile index",
            0,
            PROFILE_CAPACITY - 1,
        )


@dataclass(frozen=True)
class StickConfig:
    center_x: int
    center_y: int
    inner_deadzone: int
    outer_saturation: int
    curve_q8_8: int
    invert_x: bool
    invert_y: bool

    def __post_init__(self) -> None:
        _require_int(self.center_x, "stick center_x", -0x8000, 0x7FFF)
        _require_int(self.center_y, "stick center_y", -0x8000, 0x7FFF)
        _require_int(
            self.inner_deadzone, "stick inner_deadzone", 0, 0x7FFF
        )
        _require_int(
            self.outer_saturation, "stick outer_saturation", 1, 0x7FFF
        )
        if self.inner_deadzone >= self.outer_saturation:
            raise ConfigManagerError(
                "stick inner_deadzone must be below outer_saturation"
            )
        _require_int(self.curve_q8_8, "stick curve_q8_8", 1, 0xFFFF)
        _require_bool(self.invert_x, "stick invert_x")
        _require_bool(self.invert_y, "stick invert_y")

    @classmethod
    def from_bytes(cls, payload: bytes) -> StickConfig:
        if len(payload) != 16 or payload[11:] != bytes(5):
            raise ConfigManagerError("invalid stick configuration encoding")
        center_x, center_y, inner, outer, curve_q8_8, flags = struct.unpack(
            "<hhHHHB", payload[:11]
        )
        if flags & ~0x03:
            raise ConfigManagerError("invalid stick inversion flags")
        return cls(
            center_x,
            center_y,
            inner,
            outer,
            curve_q8_8,
            bool(flags & 1),
            bool(flags & 2),
        )

    def to_bytes(self) -> bytes:
        flags = int(self.invert_x) | (int(self.invert_y) << 1)
        return struct.pack(
            "<hhHHHB5x",
            self.center_x,
            self.center_y,
            self.inner_deadzone,
            self.outer_saturation,
            self.curve_q8_8,
            flags,
        )

    def to_json_object(self) -> dict[str, Any]:
        return {
            "center_x": self.center_x,
            "center_y": self.center_y,
            "inner_deadzone": self.inner_deadzone,
            "outer_saturation": self.outer_saturation,
            "curve_q8_8": self.curve_q8_8,
            "invert_x": self.invert_x,
            "invert_y": self.invert_y,
        }

    @classmethod
    def from_json_object(cls, value: Any, name: str) -> StickConfig:
        fields = (
            "center_x",
            "center_y",
            "inner_deadzone",
            "outer_saturation",
            "curve_q8_8",
            "invert_x",
            "invert_y",
        )
        obj = _require_object(value, fields, name)
        return cls(
            _require_int(obj["center_x"], f"{name}.center_x", -0x8000, 0x7FFF),
            _require_int(obj["center_y"], f"{name}.center_y", -0x8000, 0x7FFF),
            _require_int(
                obj["inner_deadzone"],
                f"{name}.inner_deadzone",
                0,
                0x7FFF,
            ),
            _require_int(
                obj["outer_saturation"],
                f"{name}.outer_saturation",
                1,
                0x7FFF,
            ),
            _require_int(
                obj["curve_q8_8"], f"{name}.curve_q8_8", 1, 0xFFFF
            ),
            _require_bool(obj["invert_x"], f"{name}.invert_x"),
            _require_bool(obj["invert_y"], f"{name}.invert_y"),
        )


@dataclass(frozen=True)
class TriggerConfig:
    lower_deadzone: int
    upper_saturation: int
    curve_q8_8: int
    digital_threshold: int

    def __post_init__(self) -> None:
        _require_int(
            self.lower_deadzone, "trigger lower_deadzone", 0, 0xFFFF
        )
        _require_int(
            self.upper_saturation, "trigger upper_saturation", 1, 0xFFFF
        )
        if self.lower_deadzone >= self.upper_saturation:
            raise ConfigManagerError(
                "trigger lower_deadzone must be below upper_saturation"
            )
        _require_int(self.curve_q8_8, "trigger curve_q8_8", 1, 0xFFFF)
        _require_int(
            self.digital_threshold, "trigger digital_threshold", 0, 0xFFFF
        )

    @classmethod
    def from_bytes(cls, payload: bytes) -> TriggerConfig:
        if len(payload) != 10 or payload[8:] != b"\x00\x00":
            raise ConfigManagerError("invalid trigger configuration encoding")
        return cls(*struct.unpack("<HHHH", payload[:8]))

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<HHHH2x",
            self.lower_deadzone,
            self.upper_saturation,
            self.curve_q8_8,
            self.digital_threshold,
        )

    def to_json_object(self) -> dict[str, int]:
        return {
            "lower_deadzone": self.lower_deadzone,
            "upper_saturation": self.upper_saturation,
            "curve_q8_8": self.curve_q8_8,
            "digital_threshold": self.digital_threshold,
        }

    @classmethod
    def from_json_object(cls, value: Any, name: str) -> TriggerConfig:
        fields = (
            "lower_deadzone",
            "upper_saturation",
            "curve_q8_8",
            "digital_threshold",
        )
        obj = _require_object(value, fields, name)
        return cls(
            _require_int(
                obj["lower_deadzone"],
                f"{name}.lower_deadzone",
                0,
                0xFFFF,
            ),
            _require_int(
                obj["upper_saturation"],
                f"{name}.upper_saturation",
                1,
                0xFFFF,
            ),
            _require_int(
                obj["curve_q8_8"], f"{name}.curve_q8_8", 1, 0xFFFF
            ),
            _require_int(
                obj["digital_threshold"],
                f"{name}.digital_threshold",
                0,
                0xFFFF,
            ),
        )

def _migrate_legacy_trigger_threshold(trigger: TriggerConfig) -> TriggerConfig:
    if (
        trigger.digital_threshold
        != PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD
    ):
        return trigger
    return TriggerConfig(
        trigger.lower_deadzone,
        trigger.upper_saturation,
        trigger.curve_q8_8,
        PROFILE_DEFAULT_DIGITAL_THRESHOLD,
    )


@dataclass(frozen=True)
class MacroStep:
    step_type: int
    override_flags: int
    duration_ms: int
    output_button_mask: int
    left_stick_x: int
    left_stick_y: int
    right_stick_x: int
    right_stick_y: int
    left_trigger: int
    right_trigger: int

    def __post_init__(self) -> None:
        _require_int(self.step_type, "macro step type", 0, 1)
        _require_int(
            self.override_flags, "macro override flags", 0, MACRO_OVERRIDE_MASK
        )
        _require_int(
            self.duration_ms,
            "macro duration_ms",
            0,
            PROFILE_MAXIMUM_WAIT_MS,
        )
        _require_int(
            self.output_button_mask, "macro output button mask", 0, 0xFFFF
        )
        for name in (
            "left_stick_x",
            "left_stick_y",
            "right_stick_x",
            "right_stick_y",
        ):
            _require_int(
                getattr(self, name), f"macro {name}", -0x8000, 0x7FFF
            )
        _require_int(self.left_trigger, "macro left_trigger", 0, 0xFFFF)
        _require_int(self.right_trigger, "macro right_trigger", 0, 0xFFFF)
        if self.step_type == 1 and any(
            (
                self.override_flags,
                self.duration_ms,
                self.output_button_mask,
                self.left_stick_x,
                self.left_stick_y,
                self.right_stick_x,
                self.right_stick_y,
                self.left_trigger,
                self.right_trigger,
            )
        ):
            raise ConfigManagerError("end macro step must otherwise be zero")
        if self.step_type == 0:
            if (
                not self.override_flags & 1
                and self.output_button_mask != 0
            ):
                raise ConfigManagerError(
                    "macro buttons require the buttons override"
                )
            if (
                not self.override_flags & 2
                and (self.left_stick_x != 0 or self.left_stick_y != 0)
            ):
                raise ConfigManagerError(
                    "macro left stick values require the left_stick override"
                )
            if (
                not self.override_flags & 4
                and (self.right_stick_x != 0 or self.right_stick_y != 0)
            ):
                raise ConfigManagerError(
                    "macro right stick values require the right_stick override"
                )
            if not self.override_flags & 8 and self.left_trigger != 0:
                raise ConfigManagerError(
                    "macro left trigger requires the left_trigger override"
                )
            if not self.override_flags & 16 and self.right_trigger != 0:
                raise ConfigManagerError(
                    "macro right trigger requires the right_trigger override"
                )

    @classmethod
    def end(cls) -> MacroStep:
        return cls(1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    @classmethod
    def from_bytes(cls, payload: bytes) -> MacroStep:
        if len(payload) != PROFILE_MACRO_STEP_SIZE or payload[18] != 0:
            raise ConfigManagerError("invalid macro step encoding")
        return cls(*struct.unpack("<BBHHhhhhHH", payload[:18]))

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<BBHHhhhhHHB",
            self.step_type,
            self.override_flags,
            self.duration_ms,
            self.output_button_mask,
            self.left_stick_x,
            self.left_stick_y,
            self.right_stick_x,
            self.right_stick_y,
            self.left_trigger,
            self.right_trigger,
            0,
        )

    def to_json_object(self) -> dict[str, Any]:
        return {
            "type": MACRO_STEP_TYPES[self.step_type],
            "overrides": [
                name
                for index, name in enumerate(MACRO_OVERRIDE_NAMES)
                if self.override_flags & (1 << index)
            ],
            "duration_ms": self.duration_ms,
            "output_buttons": _button_mask_to_json(
                self.output_button_mask
            ),
            "left_stick": {
                "x": self.left_stick_x,
                "y": self.left_stick_y,
            },
            "right_stick": {
                "x": self.right_stick_x,
                "y": self.right_stick_y,
            },
            "triggers": {
                "left": self.left_trigger,
                "right": self.right_trigger,
            },
        }

    @classmethod
    def from_json_object(cls, value: Any, name: str) -> MacroStep:
        fields = (
            "type",
            "overrides",
            "duration_ms",
            "output_buttons",
            "left_stick",
            "right_stick",
            "triggers",
        )
        obj = _require_object(value, fields, name)
        overrides = obj["overrides"]
        if type(overrides) is not list:
            raise ConfigManagerError(f"{name}.overrides must be a JSON array")
        override_flags = 0
        for override in overrides:
            index = _require_enum(
                override, MACRO_OVERRIDE_NAMES, f"{name}.overrides"
            )
            bit = 1 << index
            if override_flags & bit:
                raise ConfigManagerError(
                    f"{name}.overrides contains a duplicate"
                )
            override_flags |= bit
        left = _require_object(
            obj["left_stick"], ("x", "y"), f"{name}.left_stick"
        )
        right = _require_object(
            obj["right_stick"], ("x", "y"), f"{name}.right_stick"
        )
        triggers = _require_object(
            obj["triggers"], ("left", "right"), f"{name}.triggers"
        )
        return cls(
            _require_enum(obj["type"], MACRO_STEP_TYPES, f"{name}.type"),
            override_flags,
            _require_int(
                obj["duration_ms"],
                f"{name}.duration_ms",
                0,
                PROFILE_MAXIMUM_WAIT_MS,
            ),
            _button_mask_from_json(
                obj["output_buttons"], f"{name}.output_buttons"
            ),
            _require_int(
                left["x"], f"{name}.left_stick.x", -0x8000, 0x7FFF
            ),
            _require_int(
                left["y"], f"{name}.left_stick.y", -0x8000, 0x7FFF
            ),
            _require_int(
                right["x"], f"{name}.right_stick.x", -0x8000, 0x7FFF
            ),
            _require_int(
                right["y"], f"{name}.right_stick.y", -0x8000, 0x7FFF
            ),
            _require_int(
                triggers["left"],
                f"{name}.triggers.left",
                0,
                0xFFFF,
            ),
            _require_int(
                triggers["right"],
                f"{name}.triggers.right",
                0,
                0xFFFF,
            ),
        )


@dataclass(frozen=True)
class ControllerProfile:
    button_map: tuple[int, ...]
    left_stick: StickConfig
    right_stick: StickConfig
    left_trigger: TriggerConfig
    right_trigger: TriggerConfig
    weak_rumble_scale: int
    strong_rumble_scale: int
    confirmation_policy: int
    switching_chord: int
    macro_trigger: int
    macro_cancel: int
    macro_steps: tuple[MacroStep, ...]
    turbo_modes: tuple[int, ...]

    def __post_init__(self) -> None:
        if type(self.button_map) is not tuple or len(self.button_map) != len(
            LOGICAL_BUTTONS
        ):
            raise ConfigManagerError(
                "button map must contain 16 logical mappings"
            )
        for mapping in self.button_map:
            if type(mapping) is not int or (
                mapping != PROFILE_NONE_BUTTON
                and not 0 <= mapping < len(LOGICAL_BUTTONS)
            ):
                raise ConfigManagerError("invalid logical button mapping")
        if not isinstance(self.left_stick, StickConfig) or not isinstance(
            self.right_stick, StickConfig
        ):
            raise ConfigManagerError("profile sticks must be StickConfig values")
        if not isinstance(self.left_trigger, TriggerConfig) or not isinstance(
            self.right_trigger, TriggerConfig
        ):
            raise ConfigManagerError(
                "profile triggers must be TriggerConfig values"
            )
        _require_int(
            self.weak_rumble_scale, "weak rumble scale", 0, 0xFF
        )
        _require_int(
            self.strong_rumble_scale, "strong rumble scale", 0, 0xFF
        )
        _require_int(
            self.confirmation_policy,
            "confirmation policy",
            0,
            len(RUMBLE_POLICIES) - 1,
        )
        _require_int(self.switching_chord, "switching chord", 0, 0xFFFF)
        for value, name in (
            (self.macro_trigger, "macro trigger"),
            (self.macro_cancel, "macro cancel"),
        ):
            if type(value) is not int or (
                value != PROFILE_NONE_BUTTON
                and not 0 <= value < len(LOGICAL_BUTTONS)
            ):
                raise ConfigManagerError(f"invalid {name}")
        if (
            type(self.macro_steps) is not tuple
            or not 1 <= len(self.macro_steps) <= PROFILE_MACRO_STEP_CAPACITY
            or not all(isinstance(step, MacroStep) for step in self.macro_steps)
        ):
            raise ConfigManagerError("macro must contain one to eight steps")
        if any(step.step_type != 0 for step in self.macro_steps[:-1]):
            raise ConfigManagerError("only the final macro step may be end")
        if self.macro_steps[-1] != MacroStep.end():
            raise ConfigManagerError("final macro step must be canonical end")
        if type(self.turbo_modes) is not tuple or len(
            self.turbo_modes
        ) != len(LOGICAL_BUTTONS):
            raise ConfigManagerError("Turbo modes must contain 16 entries")
        for mode in self.turbo_modes:
            _require_int(mode, "Turbo mode", 0, len(TURBO_MODES) - 1)

    @classmethod
    def default(cls) -> ControllerProfile:
        stick = StickConfig(0, 0, 0, 0x7FFF, 256, False, False)
        trigger = TriggerConfig(
            0, 0xFFFF, 256, PROFILE_DEFAULT_DIGITAL_THRESHOLD
        )
        return cls(
            button_map=tuple(range(len(LOGICAL_BUTTONS))),
            left_stick=stick,
            right_stick=stick,
            left_trigger=trigger,
            right_trigger=trigger,
            weak_rumble_scale=0xFF,
            strong_rumble_scale=0xFF,
            confirmation_policy=3,
            switching_chord=0,
            macro_trigger=PROFILE_NONE_BUTTON,
            macro_cancel=PROFILE_NONE_BUTTON,
            macro_steps=(MacroStep.end(),),
            turbo_modes=(0,) * len(LOGICAL_BUTTONS),
        )

    @classmethod
    def from_bytes(cls, payload: bytes) -> ControllerProfile:
        payload = bytes(payload)
        if len(payload) != PROFILE_SIZE:
            raise ConfigManagerError("invalid profile size")
        version, size = struct.unpack_from("<HH", payload)
        if (
            version
            not in (PROFILE_LEGACY_SCHEMA_VERSION, PROFILE_SCHEMA_VERSION)
            or size != PROFILE_SIZE
        ):
            raise ConfigManagerError("unsupported profile schema")
        if payload[75] != 0 or payload[81] != 0:
            raise ConfigManagerError("profile reserved fields must be zero")
        if payload[98:100] != b"\x00\x00" or payload[252:] != bytes(4):
            raise ConfigManagerError("profile reserved fields must be zero")
        macro_count = payload[80]
        if not 1 <= macro_count <= PROFILE_MACRO_STEP_CAPACITY:
            raise ConfigManagerError("invalid macro step count")
        all_steps = tuple(
            MacroStep.from_bytes(
                payload[
                    100
                    + index * PROFILE_MACRO_STEP_SIZE : 100
                    + (index + 1) * PROFILE_MACRO_STEP_SIZE
                ]
            )
            for index in range(PROFILE_MACRO_STEP_CAPACITY)
        )
        if any(
            step != MacroStep.end() for step in all_steps[macro_count:]
        ):
            raise ConfigManagerError("unused macro steps must be canonical end")
        left_trigger = TriggerConfig.from_bytes(payload[52:62])
        right_trigger = TriggerConfig.from_bytes(payload[62:72])
        if version == PROFILE_LEGACY_SCHEMA_VERSION:
            left_trigger = _migrate_legacy_trigger_threshold(left_trigger)
            right_trigger = _migrate_legacy_trigger_threshold(right_trigger)
        return cls(
            button_map=tuple(payload[4:20]),
            left_stick=StickConfig.from_bytes(payload[20:36]),
            right_stick=StickConfig.from_bytes(payload[36:52]),
            left_trigger=left_trigger,
            right_trigger=right_trigger,
            weak_rumble_scale=payload[72],
            strong_rumble_scale=payload[73],
            confirmation_policy=payload[74],
            switching_chord=struct.unpack_from("<H", payload, 76)[0],
            macro_trigger=payload[78],
            macro_cancel=payload[79],
            macro_steps=all_steps[:macro_count],
            turbo_modes=tuple(payload[82:98]),
        )

    def to_bytes(self) -> bytes:
        payload = bytearray(PROFILE_SIZE)
        struct.pack_into(
            "<HH", payload, 0, PROFILE_SCHEMA_VERSION, PROFILE_SIZE
        )
        payload[4:20] = bytes(self.button_map)
        payload[20:36] = self.left_stick.to_bytes()
        payload[36:52] = self.right_stick.to_bytes()
        payload[52:62] = self.left_trigger.to_bytes()
        payload[62:72] = self.right_trigger.to_bytes()
        payload[72:76] = bytes(
            (
                self.weak_rumble_scale,
                self.strong_rumble_scale,
                self.confirmation_policy,
                0,
            )
        )
        struct.pack_into("<H", payload, 76, self.switching_chord)
        payload[78:82] = bytes(
            (
                self.macro_trigger,
                self.macro_cancel,
                len(self.macro_steps),
                0,
            )
        )
        payload[82:98] = bytes(self.turbo_modes)
        for index in range(PROFILE_MACRO_STEP_CAPACITY):
            step = (
                self.macro_steps[index]
                if index < len(self.macro_steps)
                else MacroStep.end()
            )
            offset = 100 + index * PROFILE_MACRO_STEP_SIZE
            payload[offset : offset + PROFILE_MACRO_STEP_SIZE] = step.to_bytes()
        return bytes(payload)

    def to_json_object(self) -> dict[str, Any]:
        return {
            "schema_version": PROFILE_SCHEMA_VERSION,
            "size": PROFILE_SIZE,
            "button_map": {
                name: _button_name(self.button_map[index])
                for index, name in enumerate(LOGICAL_BUTTONS)
            },
            "sticks": {
                "left": self.left_stick.to_json_object(),
                "right": self.right_stick.to_json_object(),
            },
            "triggers": {
                "left": self.left_trigger.to_json_object(),
                "right": self.right_trigger.to_json_object(),
            },
            "rumble": {
                "weak_scale": self.weak_rumble_scale,
                "strong_scale": self.strong_rumble_scale,
                "confirmation_policy": RUMBLE_POLICIES[
                    self.confirmation_policy
                ],
            },
            "switching_chord": _button_mask_to_json(self.switching_chord),
            "macro": {
                "trigger": _button_name(self.macro_trigger),
                "cancel": _button_name(self.macro_cancel),
                "steps": [
                    step.to_json_object() for step in self.macro_steps
                ],
            },
            "turbo": {
                name: TURBO_MODES[self.turbo_modes[index]]
                for index, name in enumerate(LOGICAL_BUTTONS)
            },
        }

    def to_json(self) -> str:
        return json.dumps(self.to_json_object(), indent=2) + "\n"

    @classmethod
    def from_json_object(cls, value: Any) -> ControllerProfile:
        fields = (
            "schema_version",
            "size",
            "button_map",
            "sticks",
            "triggers",
            "rumble",
            "switching_chord",
            "macro",
            "turbo",
        )
        obj = _require_object(value, fields, "profile")
        schema_version = _require_int(
            obj["schema_version"], "profile.schema_version", 0, 0xFFFF
        )
        if (
            schema_version
            not in (PROFILE_LEGACY_SCHEMA_VERSION, PROFILE_SCHEMA_VERSION)
            or _require_int(obj["size"], "profile.size", 0, 0xFFFF)
            != PROFILE_SIZE
        ):
            raise ConfigManagerError("unsupported profile schema")
        button_map = _require_object(
            obj["button_map"], LOGICAL_BUTTONS, "profile.button_map"
        )
        sticks = _require_object(
            obj["sticks"], ("left", "right"), "profile.sticks"
        )
        triggers = _require_object(
            obj["triggers"], ("left", "right"), "profile.triggers"
        )
        rumble = _require_object(
            obj["rumble"],
            ("weak_scale", "strong_scale", "confirmation_policy"),
            "profile.rumble",
        )
        macro = _require_object(
            obj["macro"], ("trigger", "cancel", "steps"), "profile.macro"
        )
        turbo = _require_object(
            obj["turbo"], LOGICAL_BUTTONS, "profile.turbo"
        )
        steps = macro["steps"]
        if type(steps) is not list or not (
            1 <= len(steps) <= PROFILE_MACRO_STEP_CAPACITY
        ):
            raise ConfigManagerError(
                "profile.macro.steps must contain one to eight steps"
            )
        left_trigger = TriggerConfig.from_json_object(
            triggers["left"], "profile.triggers.left"
        )
        right_trigger = TriggerConfig.from_json_object(
            triggers["right"], "profile.triggers.right"
        )
        if schema_version == PROFILE_LEGACY_SCHEMA_VERSION:
            left_trigger = _migrate_legacy_trigger_threshold(left_trigger)
            right_trigger = _migrate_legacy_trigger_threshold(right_trigger)
        return cls(
            button_map=tuple(
                _button_index(
                    button_map[name], f"profile.button_map.{name}"
                )
                for name in LOGICAL_BUTTONS
            ),
            left_stick=StickConfig.from_json_object(
                sticks["left"], "profile.sticks.left"
            ),
            right_stick=StickConfig.from_json_object(
                sticks["right"], "profile.sticks.right"
            ),
            left_trigger=left_trigger,
            right_trigger=right_trigger,
            weak_rumble_scale=_require_int(
                rumble["weak_scale"],
                "profile.rumble.weak_scale",
                0,
                0xFF,
            ),
            strong_rumble_scale=_require_int(
                rumble["strong_scale"],
                "profile.rumble.strong_scale",
                0,
                0xFF,
            ),
            confirmation_policy=_require_enum(
                rumble["confirmation_policy"],
                RUMBLE_POLICIES,
                "profile.rumble.confirmation_policy",
            ),
            switching_chord=_button_mask_from_json(
                obj["switching_chord"], "profile.switching_chord"
            ),
            macro_trigger=_button_index(
                macro["trigger"], "profile.macro.trigger"
            ),
            macro_cancel=_button_index(
                macro["cancel"], "profile.macro.cancel"
            ),
            macro_steps=tuple(
                MacroStep.from_json_object(
                    step, f"profile.macro.steps[{index}]"
                )
                for index, step in enumerate(steps)
            ),
            turbo_modes=tuple(
                _require_enum(
                    turbo[name], TURBO_MODES, f"profile.turbo.{name}"
                )
                for name in LOGICAL_BUTTONS
            ),
        )

    @classmethod
    def from_json(cls, payload: str) -> ControllerProfile:
        try:
            value = json.loads(payload, object_pairs_hook=_unique_json_object)
        except json.JSONDecodeError as exc:
            raise ConfigManagerError(f"invalid profile JSON: {exc.msg}") from exc
        return cls.from_json_object(value)


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ConfigManagerError(f"duplicate JSON field {key}")
        result[key] = value
    return result


def _crc32(payload: bytes) -> int:
    return zlib.crc32(payload) & 0xFFFFFFFF


def _host_transaction_id() -> int:
    return (secrets.randbits(31) & HOST_TRANSACTION_ID_MASK) or 1


def encode_request(operation: int, payload: bytes = b"") -> bytes:
    if len(payload) + REQUEST_HEADER_SIZE > MAXIMUM_REQUEST_SIZE:
        raise ConfigManagerError("management request exceeds EP0 limit")
    return struct.pack(
        "<4sBBBBHHI",
        b"SPMG",
        PROTOCOL_VERSION,
        operation,
        0,
        0,
        len(payload),
        0,
        _crc32(payload),
    ) + payload


def parse_response(payload: bytes, expected_operation: int) -> Envelope:
    if len(payload) < RESPONSE_HEADER_SIZE:
        raise ConfigManagerError("short management response")
    (
        magic,
        version,
        operation,
        status,
        flags,
        payload_size,
        schema_version,
        generation,
        payload_crc,
    ) = struct.unpack_from("<4sBBBBHHII", payload)
    if magic != b"SPMG":
        raise ConfigManagerError("device does not implement switch-pico management")
    if version != PROTOCOL_VERSION:
        raise ConfigManagerError(
            f"unsupported management protocol version {version}"
        )
    if operation != expected_operation:
        raise ConfigManagerError(
            f"unexpected management operation 0x{operation:02x}"
        )
    if len(payload) != RESPONSE_HEADER_SIZE + payload_size:
        raise ConfigManagerError("invalid management payload size")
    body = bytes(payload[RESPONSE_HEADER_SIZE:])
    if _crc32(body) != payload_crc:
        raise ConfigManagerError("management response CRC mismatch")
    return Envelope(
        operation=operation,
        status=status,
        flags=flags,
        schema_version=schema_version,
        generation=generation,
        payload_crc=payload_crc,
        payload=body,
    )


def _raise_status(envelope: Envelope, *, pending_ok: bool = False) -> None:
    if envelope.status == STATUS_OK:
        return
    if envelope.status == STATUS_PENDING and pending_ok:
        return
    if envelope.status == STATUS_PENDING:
        raise ConfigManagerError("device operation is still pending")
    raise ConfigManagerError(
        STATUS_NAMES.get(
            envelope.status, f"unknown device status {envelope.status}"
        )
    )


def _control_in(device: UsbDevice, operation: int) -> Envelope:
    payload = device.ctrl_transfer(
        0xC0,
        operation,
        REQUEST_VALUE,
        REQUEST_INDEX,
        MAXIMUM_RESPONSE_SIZE,
        timeout=USB_TIMEOUT_MS,
    )
    return parse_response(bytes(payload), operation)


def _control_out(
    device: UsbDevice, operation: int, payload: bytes = b""
) -> None:
    request = encode_request(operation, payload)
    device.ctrl_transfer(
        0x40,
        operation,
        REQUEST_VALUE,
        REQUEST_INDEX,
        request,
        timeout=USB_TIMEOUT_MS,
    )


def read_info(device: UsbDevice) -> DeviceInfo:
    envelope = _control_in(device, OP_INFO)
    _raise_status(envelope)
    if len(envelope.payload) != 8:
        raise ConfigManagerError("invalid device-info payload")
    active_mode = envelope.payload[4]
    capabilities = envelope.payload[5]
    if active_mode >= len(ACTIVE_MODE_NAMES):
        raise ConfigManagerError(f"unknown active USB mode {active_mode}")
    if capabilities & ~CAPABILITY_MASK:
        raise ConfigManagerError(
            f"unknown device capability flags 0x{capabilities:02x}"
        )
    if capabilities != 0 and not capabilities & CAPABILITY_INPUT:
        raise ConfigManagerError(
            "device capability flags omit required input support"
        )
    return DeviceInfo(
        firmware_version=(
            envelope.payload[0],
            envelope.payload[1],
            envelope.payload[2],
        ),
        board=envelope.payload[3],
        active_mode=active_mode,
        capabilities=capabilities,
        maximum_configuration_size=struct.unpack_from(
            "<H", envelope.payload, 6
        )[0],
    )

def read_runtime_diagnostics(device: UsbDevice) -> RuntimeDiagnostics:
    envelope = _control_in(device, OP_RUNTIME_DIAGNOSTICS)
    _raise_status(envelope)
    if len(envelope.payload) != 32:
        raise ConfigManagerError("invalid runtime-diagnostics payload")
    counters = struct.unpack_from("<7I", envelope.payload)
    return RuntimeDiagnostics(
        *counters,
        active_slots=envelope.payload[28],
        rumble_capable_slots=envelope.payload[29],
        feedback_pending_slots=envelope.payload[30],
        rumble_pending_slots=envelope.payload[31],
    )


def read_configuration(device: UsbDevice) -> AdapterConfiguration:
    envelope = _control_in(device, OP_CONFIGURATION_READ)
    _raise_status(envelope)
    if (
        envelope.schema_version != CONFIGURATION_SCHEMA_VERSION
        or len(envelope.payload) != CONFIGURATION_SIZE
        or envelope.payload[3:] != bytes(5)
    ):
        raise ConfigManagerError("unsupported configuration object")
    pairing_window_seconds = struct.unpack_from("<H", envelope.payload)[0]
    requested_mode = envelope.payload[2]
    if not (
        PAIRING_WINDOW_SECONDS_MIN
        <= pairing_window_seconds
        <= PAIRING_WINDOW_SECONDS_MAX
    ):
        raise ConfigManagerError("invalid stored pairing-window duration")
    if requested_mode >= len(REQUESTED_MODE_NAMES):
        raise ConfigManagerError(
            f"invalid stored requested USB mode {requested_mode}"
        )
    return AdapterConfiguration(
        pairing_window_seconds=pairing_window_seconds,
        generation=envelope.generation,
        crc=envelope.payload_crc,
        requested_mode=requested_mode,
    )


def read_transaction_status(device: UsbDevice) -> TransactionStatus:
    envelope = _control_in(device, OP_TRANSACTION_STATUS)
    _raise_status(envelope, pending_ok=True)
    if len(envelope.payload) != 20:
        raise ConfigManagerError("invalid transaction-status payload")
    values = struct.unpack("<IHHIII", envelope.payload)
    return TransactionStatus(*values, status=envelope.status)


def _wait_for_transaction(
    device: UsbDevice, transaction_id: int, timeout: float
) -> TransactionStatus:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = read_transaction_status(device)
        if status.transaction_id != transaction_id:
            raise ConfigManagerError("device reported a different transaction")
        if status.status == STATUS_OK:
            return status
        time.sleep(0.05)
    raise ConfigManagerError("configuration commit did not finish")


def write_configuration(
    device: UsbDevice, configuration: AdapterConfiguration, timeout: float
) -> TransactionStatus:
    if not (
        PAIRING_WINDOW_SECONDS_MIN
        <= configuration.pairing_window_seconds
        <= PAIRING_WINDOW_SECONDS_MAX
    ):
        raise ConfigManagerError(
            "pairing window must be between 10 and 300 seconds"
        )
    if (
        type(configuration.requested_mode) is not int
        or not 0 <= configuration.requested_mode < len(REQUESTED_MODE_NAMES)
    ):
        raise ConfigManagerError("invalid requested USB mode")
    payload = struct.pack(
        "<HB5x",
        configuration.pairing_window_seconds,
        configuration.requested_mode,
    )
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_CONFIGURATION_BEGIN,
        struct.pack(
            "<IHHI",
            transaction_id,
            CONFIGURATION_SCHEMA_VERSION,
            len(payload),
            _crc32(payload),
        ),
    )
    for offset in range(0, len(payload), MAXIMUM_CHUNK_SIZE):
        chunk = payload[offset : offset + MAXIMUM_CHUNK_SIZE]
        _control_out(
            device,
            OP_CONFIGURATION_CHUNK,
            struct.pack("<IHH", transaction_id, offset, len(chunk)) + chunk,
        )
    _control_out(
        device, OP_CONFIGURATION_COMMIT, struct.pack("<I", transaction_id)
    )
    return _wait_for_transaction(device, transaction_id, timeout)


def reset_configuration(device: UsbDevice, timeout: float) -> TransactionStatus:
    transaction_id = _host_transaction_id()
    _control_out(
        device, OP_CONFIGURATION_RESET, struct.pack("<I", transaction_id)
    )
    return _wait_for_transaction(device, transaction_id, timeout)


def set_mode(
    device: UsbDevice, requested_mode: int, timeout: float
) -> TransactionStatus:
    if type(requested_mode) is not int or requested_mode not in (
        REQUESTED_MODE_AUTO,
        REQUESTED_MODE_SWITCH,
        REQUESTED_MODE_XINPUT,
        REQUESTED_MODE_DINPUT,
        REQUESTED_MODE_MAC,
    ):
        raise ConfigManagerError("requested USB mode is not available")
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_MODE_SET,
        struct.pack("<IB", transaction_id, requested_mode),
    )
    return _wait_for_transaction(device, transaction_id, timeout)


def request_reboot(device: UsbDevice, transaction_id: int) -> None:
    _require_int(
        transaction_id, "transaction ID", 1, HOST_TRANSACTION_ID_MASK
    )
    _control_out(device, OP_REBOOT, struct.pack("<I", transaction_id))

def request_bootsel_reboot(device: UsbDevice) -> None:
    _control_out(device, OP_BOOTSEL_REBOOT)


def _mode_is_active(requested_mode: int, active_mode: int) -> bool:
    if requested_mode == REQUESTED_MODE_AUTO:
        return active_mode in (ACTIVE_MODE_SWITCH_PROBE, ACTIVE_MODE_XINPUT)
    if requested_mode == REQUESTED_MODE_SWITCH:
        return active_mode == ACTIVE_MODE_SWITCH
    if requested_mode == REQUESTED_MODE_XINPUT:
        return active_mode == ACTIVE_MODE_XINPUT
    if requested_mode == REQUESTED_MODE_DINPUT:
        return active_mode == ACTIVE_MODE_DINPUT
    if requested_mode == REQUESTED_MODE_MAC:
        return active_mode == ACTIVE_MODE_MAC
    return False


def parse_profile_list(envelope: Envelope) -> tuple[ProfileListEntry, ...]:
    _raise_status(envelope)
    if envelope.schema_version not in (
        PROFILE_LEGACY_SCHEMA_VERSION,
        PROFILE_SCHEMA_VERSION,
    ):
        raise ConfigManagerError("unsupported profile-list schema")
    if not envelope.payload:
        raise ConfigManagerError("short profile-list payload")
    count = envelope.payload[0]
    if (
        not 1 <= count <= PROFILE_LIST_CAPACITY
        or len(envelope.payload) != 1 + count * PROFILE_LIST_ROW_SIZE
    ):
        raise ConfigManagerError("invalid profile-list count")
    entries: list[ProfileListEntry] = []
    identities: set[ControllerIdentity] = set()
    for index in range(count):
        offset = 1 + index * PROFILE_LIST_ROW_SIZE
        identity = ControllerIdentity.from_bytes(
            envelope.payload[offset : offset + CONTROLLER_IDENTITY_SIZE]
        )
        active_profile_index = envelope.payload[offset + 14]
        if envelope.payload[offset + 15] != 0:
            raise ConfigManagerError("profile-list reserved field is nonzero")
        if index == 0 and not identity.is_global_fallback:
            raise ConfigManagerError(
                "profile list does not begin with global fallback"
            )
        if index != 0 and identity.is_global_fallback:
            raise ConfigManagerError("duplicate global fallback profile entry")
        if identity in identities:
            raise ConfigManagerError("duplicate identity in profile list")
        identities.add(identity)
        entries.append(ProfileListEntry(identity, active_profile_index))
    return tuple(entries)


def list_profiles(device: UsbDevice) -> tuple[ProfileListEntry, ...]:
    return parse_profile_list(_control_in(device, OP_PROFILE_LIST))


def _validate_profile_index(profile_index: int) -> None:
    _require_int(
        profile_index,
        "profile index",
        0,
        PROFILE_CAPACITY - 1,
    )


def select_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
) -> None:
    _validate_profile_index(profile_index)
    _control_out(
        device,
        OP_PROFILE_SELECT,
        identity.to_bytes() + bytes((profile_index,)),
    )


def read_selected_profile(device: UsbDevice) -> ControllerProfile:
    envelope = _control_in(device, OP_PROFILE_READ)
    _raise_status(envelope)
    if envelope.schema_version not in (
        PROFILE_LEGACY_SCHEMA_VERSION,
        PROFILE_SCHEMA_VERSION,
    ):
        raise ConfigManagerError("unsupported profile schema")
    return ControllerProfile.from_bytes(envelope.payload)


def read_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
) -> ControllerProfile:
    select_profile(device, identity, profile_index)
    return read_selected_profile(device)


def read_profile_transaction_status(device: UsbDevice) -> TransactionStatus:
    envelope = _control_in(device, OP_PROFILE_TRANSACTION_STATUS)
    _raise_status(envelope, pending_ok=True)
    if (
        envelope.schema_version != PROFILE_SCHEMA_VERSION
        or len(envelope.payload) != 20
    ):
        raise ConfigManagerError("invalid profile transaction-status payload")
    values = struct.unpack("<IHHIII", envelope.payload)
    return TransactionStatus(*values, status=envelope.status)


def _wait_for_profile_transaction(
    device: UsbDevice, transaction_id: int, timeout: float
) -> TransactionStatus:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        envelope = _control_in(device, OP_PROFILE_TRANSACTION_STATUS)
        if (
            envelope.schema_version != PROFILE_SCHEMA_VERSION
            or len(envelope.payload) != 20
        ):
            raise ConfigManagerError(
                "invalid profile transaction-status payload"
            )
        values = struct.unpack("<IHHIII", envelope.payload)
        status = TransactionStatus(*values, status=envelope.status)
        if status.transaction_id != transaction_id:
            raise ConfigManagerError(
                "device reported a different profile transaction"
            )
        _raise_status(envelope, pending_ok=True)
        if status.status == STATUS_OK:
            return status
        time.sleep(0.05)
    raise ConfigManagerError("profile commit did not finish")


def write_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
    profile: ControllerProfile,
    timeout: float,
) -> TransactionStatus:
    _validate_profile_index(profile_index)
    if not isinstance(profile, ControllerProfile):
        raise ConfigManagerError("profile must be a ControllerProfile")
    payload = profile.to_bytes()
    transaction_id = _host_transaction_id()
    begin = (
        struct.pack("<I", transaction_id)
        + identity.to_bytes()
        + struct.pack(
            "<BBHHI",
            profile_index,
            0,
            PROFILE_SCHEMA_VERSION,
            len(payload),
            _crc32(payload),
        )
    )
    _control_out(device, OP_PROFILE_BEGIN, begin)
    for offset in range(0, len(payload), MAXIMUM_CHUNK_SIZE):
        chunk = payload[offset : offset + MAXIMUM_CHUNK_SIZE]
        _control_out(
            device,
            OP_PROFILE_CHUNK,
            struct.pack("<IHH", transaction_id, offset, len(chunk)) + chunk,
        )
    _control_out(device, OP_PROFILE_COMMIT, struct.pack("<I", transaction_id))
    return _wait_for_profile_transaction(device, transaction_id, timeout)


def reset_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int | None,
    timeout: float,
) -> TransactionStatus:
    wire_index = PROFILE_NONE_BUTTON
    if profile_index is not None:
        _validate_profile_index(profile_index)
        wire_index = profile_index
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_PROFILE_RESET,
        struct.pack("<I", transaction_id)
        + identity.to_bytes()
        + bytes((wire_index,)),
    )
    return _wait_for_profile_transaction(device, transaction_id, timeout)


def activate_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
    timeout: float,
) -> TransactionStatus:
    _validate_profile_index(profile_index)
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_PROFILE_ACTIVATE,
        struct.pack("<I", transaction_id)
        + identity.to_bytes()
        + bytes((profile_index,)),
    )
    return _wait_for_profile_transaction(device, transaction_id, timeout)


def parse_pairing_snapshot(envelope: Envelope) -> PairingSnapshot:
    _raise_status(envelope, pending_ok=True)
    if len(envelope.payload) < 4:
        raise ConfigManagerError("short pairing snapshot")
    record_count = envelope.payload[0]
    required = 4 + record_count * PAIRING_RECORD_SIZE
    if (
        record_count > PAIRING_RECORD_CAPACITY
        or len(envelope.payload) != required
    ):
        raise ConfigManagerError("invalid pairing record count")
    records: list[PairingRecord] = []
    offset = 4
    for _ in range(record_count):
        records.append(
            PairingRecord(
                transport=envelope.payload[offset],
                address_type=envelope.payload[offset + 1],
                address=envelope.payload[offset + 2 : offset + 8],
            )
        )
        offset += PAIRING_RECORD_SIZE
    return PairingSnapshot(
        generation=envelope.generation,
        pending=envelope.status == STATUS_PENDING,
        overflow=bool(envelope.flags & 1 or envelope.payload[1] & 1),
        records=tuple(records),
    )


def read_pairings(device: UsbDevice) -> PairingSnapshot:
    return parse_pairing_snapshot(_control_in(device, OP_PAIRING_READ))


def _wait_for_pairings(
    device: UsbDevice, previous_generation: int, timeout: float
) -> PairingSnapshot:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = read_pairings(device)
        if not snapshot.pending and snapshot.generation != previous_generation:
            return snapshot
        time.sleep(0.05)
    raise ConfigManagerError("Pico did not finish the pairing operation")


def refresh_pairings(device: UsbDevice, timeout: float) -> PairingSnapshot:
    initial = read_pairings(device)
    _control_out(device, OP_PAIRING_REFRESH)
    return _wait_for_pairings(device, initial.generation, timeout)


def clear_pairings(device: UsbDevice, timeout: float) -> PairingSnapshot:
    initial = read_pairings(device)
    _control_out(device, OP_PAIRING_CLEAR)
    snapshot = _wait_for_pairings(device, initial.generation, timeout)
    if snapshot.records:
        raise ConfigManagerError("Pico reported pairings after clear completed")
    return snapshot


def _candidate_devices() -> Iterable[UsbDevice]:
    for vendor_id, product_id in USB_IDENTITIES:
        devices = cast(
            Iterable[UsbDevice] | None,
            usb.core.find(
                find_all=True, idVendor=vendor_id, idProduct=product_id
            ),
        )
        if devices is not None:
            yield from devices


def find_pico(
    bus: int | None, address: int | None, timeout: float = 3.0
) -> UsbDevice:
    deadline = time.monotonic() + timeout
    failures: list[Exception] = []
    while True:
        matches: list[UsbDevice] = []
        for device in _candidate_devices():
            if bus is not None and getattr(device, "bus", None) != bus:
                continue
            if address is not None and getattr(device, "address", None) != address:
                continue
            try:
                _ = read_info(device)
            except (ConfigManagerError, usb.core.USBError) as exc:
                failures.append(exc)
                continue
            matches.append(device)
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            locations = ", ".join(
                f"{device.bus}:{device.address}" for device in matches
            )
            raise ConfigManagerError(
                f"multiple switch-pico devices found ({locations}); "
                "select one with --bus and --address"
            )
        if time.monotonic() >= deadline:
            break
        time.sleep(0.05)
    if failures:
        raise ConfigManagerError(
            "matching USB devices were found, but none accepted the "
            f"management request; last error: {failures[-1]}"
        ) from failures[-1]
    raise ConfigManagerError("no USB-connected switch-pico firmware found")


_UsbPhysicalLocation = tuple[int, tuple[int, ...]]
_UsbEnumerationIdentity = tuple[int, int]


@dataclass(frozen=True)
class _ReenumerationSnapshot:
    previous_device: UsbDevice
    previous_bus: int | None
    previous_address: int | None
    selected_location: _UsbPhysicalLocation | None
    other_locations: frozenset[_UsbPhysicalLocation]
    other_enumerations: frozenset[_UsbEnumerationIdentity]


def _physical_location(device: UsbDevice) -> _UsbPhysicalLocation | None:
    bus = getattr(device, "bus", None)
    try:
        port_numbers = getattr(device, "port_numbers", None)
    except (AttributeError, NotImplementedError):
        return None
    if bus is None or port_numbers is None:
        return None
    ports = tuple(port_numbers)
    if not ports:
        return None
    return bus, ports


def _enumeration_identity(
    device: UsbDevice,
) -> _UsbEnumerationIdentity | None:
    bus = getattr(device, "bus", None)
    address = getattr(device, "address", None)
    if bus is None or address is None:
        return None
    return bus, address


def _capture_reenumeration_snapshot(
    previous_device: UsbDevice,
) -> _ReenumerationSnapshot:
    previous_bus = getattr(previous_device, "bus", None)
    previous_address = getattr(previous_device, "address", None)
    previous_enumeration = _enumeration_identity(previous_device)
    selected_location = _physical_location(previous_device)
    other_locations: set[_UsbPhysicalLocation] = set()
    other_enumerations: set[_UsbEnumerationIdentity] = set()
    other_count = 0
    for device in _candidate_devices():
        enumeration = _enumeration_identity(device)
        location = _physical_location(device)
        if (
            device is previous_device
            or (
                previous_enumeration is not None
                and enumeration == previous_enumeration
            )
            or (
                selected_location is not None
                and location == selected_location
            )
        ):
            continue
        other_count += 1
        if location is not None:
            other_locations.add(location)
        if enumeration is not None:
            other_enumerations.add(enumeration)

    if selected_location is None and other_count:
        raise ConfigManagerError(
            "USB port topology is unavailable; cannot safely reboot while "
            "multiple switch-pico adapters are connected"
        )
    return _ReenumerationSnapshot(
        previous_device,
        previous_bus,
        previous_address,
        selected_location,
        frozenset(other_locations),
        frozenset(other_enumerations),
    )


def _is_previous_enumeration(
    device: UsbDevice, snapshot: _ReenumerationSnapshot
) -> bool:
    if device is snapshot.previous_device:
        return True
    identity = _enumeration_identity(device)
    return (
        identity is not None
        and snapshot.previous_bus is not None
        and snapshot.previous_address is not None
        and identity == (snapshot.previous_bus, snapshot.previous_address)
    )


def _is_reenumeration_candidate(
    device: UsbDevice, snapshot: _ReenumerationSnapshot
) -> bool:
    location = _physical_location(device)
    enumeration = _enumeration_identity(device)
    if location is not None:
        if location in snapshot.other_locations:
            return False
    elif enumeration is not None and enumeration in snapshot.other_enumerations:
        return False

    if snapshot.selected_location is not None:
        return location == snapshot.selected_location
    return (
        snapshot.previous_bus is None
        or getattr(device, "bus", None) == snapshot.previous_bus
    )


def _wait_for_reenumeration(
    snapshot: _ReenumerationSnapshot, timeout: float
) -> UsbDevice:
    deadline = time.monotonic() + timeout
    disappeared = False
    failures: list[Exception] = []
    while True:
        candidates = list(_candidate_devices())
        if not disappeared and not any(
            _is_previous_enumeration(device, snapshot)
            for device in candidates
        ):
            disappeared = True
        if disappeared:
            matches: list[UsbDevice] = []
            for device in candidates:
                if not _is_reenumeration_candidate(device, snapshot):
                    continue
                try:
                    _ = read_info(device)
                except (ConfigManagerError, usb.core.USBError) as exc:
                    failures.append(exc)
                    continue
                matches.append(device)
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                locations = ", ".join(
                    f"{device.bus}:{device.address}" for device in matches
                )
                if snapshot.selected_location is None:
                    raise ConfigManagerError(
                        "USB port topology is unavailable; multiple "
                        "switch-pico devices make reboot identity ambiguous "
                        f"({locations})"
                    )
                raise ConfigManagerError(
                    "multiple switch-pico devices re-enumerated on the "
                    f"selected USB port ({locations})"
                )
        if time.monotonic() >= deadline:
            break
        time.sleep(0.05)
    if not disappeared:
        raise ConfigManagerError(
            "Pico did not disappear from USB after the reboot request"
        )
    if failures:
        raise ConfigManagerError(
            "Pico re-enumerated on the selected USB port, but did not accept "
            f"the management request; last error: {failures[-1]}"
        ) from failures[-1]
    if snapshot.selected_location is not None:
        raise ConfigManagerError(
            "Pico did not re-enumerate on its original physical USB port "
            "after reboot"
        )
    raise ConfigManagerError("Pico did not re-enumerate after reboot")


def configure_mode(
    device: UsbDevice, requested_mode: int, timeout: float
) -> tuple[UsbDevice, bool]:
    if type(requested_mode) is not int or requested_mode not in (
        REQUESTED_MODE_AUTO,
        REQUESTED_MODE_SWITCH,
        REQUESTED_MODE_XINPUT,
        REQUESTED_MODE_DINPUT,
        REQUESTED_MODE_MAC,
    ):
        raise ConfigManagerError("requested USB mode is not available")
    before_info = read_info(device)
    before_configuration = read_configuration(device)
    if (
        before_configuration.requested_mode == requested_mode
        and _mode_is_active(requested_mode, before_info.active_mode)
    ):
        return device, False
    reenumeration_snapshot = _capture_reenumeration_snapshot(device)

    transaction = set_mode(device, requested_mode, timeout)
    request_reboot(device, transaction.transaction_id)
    reenumerated = _wait_for_reenumeration(
        reenumeration_snapshot, timeout
    )
    after_info = read_info(reenumerated)
    after_configuration = read_configuration(reenumerated)
    if after_configuration.requested_mode != requested_mode:
        raise ConfigManagerError(
            "requested USB mode was not stored after reboot"
        )
    if not _mode_is_active(requested_mode, after_info.active_mode):
        raise ConfigManagerError(
            f"device activated {after_info.mode_name()} instead of "
            f"{REQUESTED_MODE_NAMES[requested_mode]}"
        )
    return reenumerated, True


def _print_pairings(snapshot: PairingSnapshot) -> None:
    if not snapshot.records:
        print("No stored pairings.")
        return
    for index, record in enumerate(snapshot.records, start=1):
        print(f"{index}: {record.transport_text} {record.address_text}")
    if snapshot.overflow:
        print("Warning: additional pairings did not fit in the response.")


def _print_profiles(entries: Sequence[ProfileListEntry]) -> None:
    for index, entry in enumerate(entries):
        identity = entry.identity
        if identity.is_global_fallback:
            description = "global fallback"
        else:
            description = (
                f"{identity.transport_text} {identity.address_text} "
                f"address-type {identity.address_type} "
                f"VID:PID {identity.vendor_id:04X}:{identity.product_id:04X}"
            )
        print(
            f"{index}: {description} "
            f"(active profile {entry.active_profile_index + 1})"
        )


def _resolve_profile_identity(
    entries: Sequence[ProfileListEntry], identity_index: int
) -> ControllerIdentity:
    if not 0 <= identity_index < len(entries):
        raise ConfigManagerError(
            f"identity index {identity_index} is out of range; "
            f"use profiles list to see indices 0 through {len(entries) - 1}"
        )
    return entries[identity_index].identity


def _load_profile(path: Path) -> ControllerProfile:
    try:
        payload = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigManagerError(
            f"could not read profile JSON {path}: {exc}"
        ) from exc
    return ControllerProfile.from_json(payload)


def _save_profile(path: Path, profile: ControllerProfile) -> None:
    try:
        path.write_text(profile.to_json(), encoding="utf-8")
    except OSError as exc:
        raise ConfigManagerError(
            f"could not write profile JSON {path}: {exc}"
        ) from exc


def _profile_number(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "profile must be a number from 1 to 4"
        ) from exc
    if not 1 <= number <= PROFILE_CAPACITY:
        raise argparse.ArgumentTypeError("profile must be a number from 1 to 4")
    return number - 1


def _identity_index(value: str) -> int:
    try:
        index = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "identity must be a non-negative list index"
        ) from exc
    if index < 0:
        raise argparse.ArgumentTypeError(
            "identity must be a non-negative list index"
        )
    return index


def _add_identity_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--identity",
        type=_identity_index,
        default=0,
        metavar="N",
        help="identity index from profiles list (default: 0)",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="switch-pico-config",
        description=(
            "Manage switch-pico USB modes, persistent configuration, "
            "profiles, and pairings."
        ),
    )
    parser.add_argument("--bus", type=int, help="USB bus number")
    parser.add_argument("--address", type=int, help="USB device address")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_OPERATION_TIMEOUT_SECONDS,
        help=(
            "operation timeout in seconds "
            f"(default: {DEFAULT_OPERATION_TIMEOUT_SECONDS:g})"
        ),
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="show firmware and configuration status")
    commands.add_parser(
        "diagnostics", help="show live Bluetooth and rumble pipeline counters"
    )
    reboot = commands.add_parser(
        "reboot", help="reboot into a firmware or ROM target"
    )
    reboot.add_argument("target", choices=("bootsel",))
    mode = commands.add_parser("mode", help="select the persistent USB mode")
    mode.add_argument("mode", choices=SELECTABLE_MODE_NAMES)

    config = commands.add_parser("config", help="read or change configuration")
    config_commands = config.add_subparsers(dest="config_command", required=True)
    config_commands.add_parser("show", help="show persistent configuration")
    config_set = config_commands.add_parser("set", help="write configuration")
    config_set.add_argument(
        "--pairing-window-seconds",
        required=True,
        type=int,
        metavar="SECONDS",
    )
    config_reset = config_commands.add_parser("reset", help="restore defaults")
    config_reset.add_argument("--yes", action="store_true")

    profiles = commands.add_parser(
        "profiles", help="list, import, export, or select controller profiles"
    )
    profile_commands = profiles.add_subparsers(
        dest="profile_command", required=True
    )
    profile_commands.add_parser(
        "list", help="list profile identities and active profiles"
    )
    profile_export = profile_commands.add_parser(
        "export", help="export a profile as JSON"
    )
    profile_export.add_argument(
        "profile_index", type=_profile_number, metavar="PROFILE"
    )
    profile_export.add_argument("path", type=Path, metavar="PATH")
    _add_identity_argument(profile_export)
    profile_import = profile_commands.add_parser(
        "import", help="import a profile from JSON"
    )
    profile_import.add_argument(
        "profile_index", type=_profile_number, metavar="PROFILE"
    )
    profile_import.add_argument("path", type=Path, metavar="PATH")
    _add_identity_argument(profile_import)
    profile_reset = profile_commands.add_parser(
        "reset", help="reset one or all profiles"
    )
    profile_reset.add_argument(
        "profile_index",
        type=lambda value: None if value == "all" else _profile_number(value),
        metavar="PROFILE|all",
    )
    _add_identity_argument(profile_reset)
    profile_reset.add_argument("--yes", action="store_true")
    profile_activate = profile_commands.add_parser(
        "activate", help="activate a profile"
    )
    profile_activate.add_argument(
        "profile_index", type=_profile_number, metavar="PROFILE"
    )
    _add_identity_argument(profile_activate)

    pairings = commands.add_parser("pairings", help="list or clear pairings")
    pairing_commands = pairings.add_subparsers(
        dest="pairing_command", required=True
    )
    pairing_commands.add_parser("list", help="list stored pairings")
    pairing_clear = pairing_commands.add_parser("clear", help="clear pairings")
    pairing_clear.add_argument("--yes", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout <= 0:
        print("error: --timeout must be positive", file=sys.stderr)
        return 2
    if (
        args.command == "config"
        and args.config_command == "reset"
        and not args.yes
    ):
        print("error: config reset requires --yes", file=sys.stderr)
        return 2
    if (
        args.command == "pairings"
        and args.pairing_command == "clear"
        and not args.yes
    ):
        print("error: pairings clear requires --yes", file=sys.stderr)
        return 2
    if (
        args.command == "profiles"
        and args.profile_command == "reset"
        and not args.yes
    ):
        print("error: profiles reset requires --yes", file=sys.stderr)
        return 2

    imported_profile: ControllerProfile | None = None
    try:
        if args.command == "profiles" and args.profile_command == "import":
            imported_profile = _load_profile(args.path)
        device = find_pico(args.bus, args.address, args.timeout)
        if args.command == "status":
            info = read_info(device)
            configuration = read_configuration(device)
            version = ".".join(str(part) for part in info.firmware_version)
            print(f"Firmware: {version}")
            print(f"Board: Pico 2 W ({info.board})")
            print(
                "Requested USB mode: "
                f"{REQUESTED_MODE_NAMES[configuration.requested_mode]}"
            )
            print(f"Active USB mode: {info.mode_name()}")
            print(f"Mode capabilities: {info.capability_summary()}")
            print(f"Configuration generation: {configuration.generation}")
            print(f"Configuration CRC: {configuration.crc:08x}")
            print(
                "Pairing window: "
                f"{configuration.pairing_window_seconds} seconds"
            )
        elif args.command == "diagnostics":
            diagnostics = read_runtime_diagnostics(device)
            print(f"Initialization stage: {diagnostics.initialization_stage}")
            print(f"Rumble timer ticks: {diagnostics.rumble_timer_ticks}")
            print(
                "Configuration timer ticks: "
                f"{diagnostics.configuration_timer_ticks}"
            )
            print(f"Controller reports: {diagnostics.controller_reports}")
            print(
                "Host rumble requests: "
                f"{diagnostics.host_rumble_requests}"
            )
            print(
                "Local feedback requests: "
                f"{diagnostics.local_feedback_requests}"
            )
            print(f"Rumble dispatches: {diagnostics.rumble_dispatches}")
            print(f"Active slots: {diagnostics.active_slots}")
            print(
                "Rumble-capable slots: "
                f"{diagnostics.rumble_capable_slots}"
            )
            print(
                "Feedback-pending slots: "
                f"{diagnostics.feedback_pending_slots}"
            )
            print(
                "Rumble-pending slots: "
                f"{diagnostics.rumble_pending_slots}"
            )
        elif args.command == "reboot":
            request_bootsel_reboot(device)
            print("Rebooting into USB BOOTSEL mode.")
        elif args.command == "mode":
            requested_mode = REQUESTED_MODE_NAMES.index(args.mode)
            _, changed = configure_mode(device, requested_mode, args.timeout)
            if changed:
                print(f"USB mode changed to {args.mode}.")
            else:
                print(f"USB mode is already {args.mode}.")
        elif args.command == "config":
            if args.config_command == "show":
                configuration = read_configuration(device)
                print(
                    f"pairing_window_seconds={configuration.pairing_window_seconds}"
                )
                print(
                    "requested_mode="
                    f"{REQUESTED_MODE_NAMES[configuration.requested_mode]}"
                )
                print(f"generation={configuration.generation}")
                print(f"crc={configuration.crc:08x}")
            elif args.config_command == "set":
                before = read_configuration(device)
                status = write_configuration(
                    device,
                    AdapterConfiguration(
                        pairing_window_seconds=args.pairing_window_seconds,
                        generation=before.generation,
                        crc=before.crc,
                        requested_mode=before.requested_mode,
                    ),
                    args.timeout,
                )
                print(
                    "Stored configuration generation "
                    f"{status.stored_generation} "
                    f"(CRC {status.stored_crc:08x})."
                )
            else:
                status = reset_configuration(device, args.timeout)
                print(
                    "Reset configuration at generation "
                    f"{status.stored_generation}."
                )
        elif args.command == "profiles":
            entries = list_profiles(device)
            if args.profile_command == "list":
                _print_profiles(entries)
            else:
                identity = _resolve_profile_identity(entries, args.identity)
                if args.profile_command == "export":
                    profile = read_profile(
                        device, identity, args.profile_index
                    )
                    _save_profile(args.path, profile)
                    print(
                        f"Exported profile {args.profile_index + 1} "
                        f"for identity {args.identity} to {args.path}."
                    )
                elif args.profile_command == "import":
                    status = write_profile(
                        device,
                        identity,
                        args.profile_index,
                        cast(ControllerProfile, imported_profile),
                        args.timeout,
                    )
                    print(
                        f"Stored profile {args.profile_index + 1} "
                        f"for identity {args.identity} at generation "
                        f"{status.stored_generation} "
                        f"(CRC {status.stored_crc:08x})."
                    )
                elif args.profile_command == "reset":
                    reset_profile(
                        device, identity, args.profile_index, args.timeout
                    )
                    target = (
                        "all profiles"
                        if args.profile_index is None
                        else f"profile {args.profile_index + 1}"
                    )
                    print(f"Reset {target} for identity {args.identity}.")
                else:
                    activate_profile(
                        device, identity, args.profile_index, args.timeout
                    )
                    print(
                        f"Activated profile {args.profile_index + 1} "
                        f"for identity {args.identity}."
                    )
        elif args.pairing_command == "list":
            _print_pairings(refresh_pairings(device, args.timeout))
        else:
            before = refresh_pairings(device, args.timeout)
            clear_pairings(device, args.timeout)
            print(f"Cleared {len(before.records)} stored pairing(s).")
    except ConfigManagerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except usb.core.USBError as exc:
        print(f"error: USB access failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
