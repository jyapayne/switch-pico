#!/usr/bin/env python3
"""Manage switch-pico USB modes, configuration, profiles, and pairings."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import secrets
import struct
import sys
import time
import zlib
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Protocol, cast

import usb.core

USB_IDENTITIES = (
    (0x057E, 0x2009),
    (0x057E, 0x2068),  # Native hub root only; its Joy-Con children are not adapters.
    (0xCAFE, 0x4010),
    (0xCAFE, 0x4020),
    (0xCAFE, 0x4021),
)
REQUEST_VALUE = 0x5350
REQUEST_INDEX = 0x0001
PROTOCOL_VERSION = 1
REQUEST_HEADER_SIZE = 16
RESPONSE_HEADER_SIZE = 20
MAXIMUM_REQUEST_SIZE = 80
MAXIMUM_RESPONSE_SIZE = 837
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
OP_PROFILE_PLAYTEST = 0x39
OP_PROFILE_METADATA_READ = 0x3A
OP_PROFILE_METADATA_SET = 0x3B
OP_PROFILE_IDENTIFY = 0x3C
OP_WII_ORIENTATION = 0x3D
OP_HAPTICS_EXPERIMENT = 0x40
OP_HAPTICS_TRANSPORT_PROBE = 0x41
OP_MACRO_CAPTURE = 0x42
OP_NATIVE_SWITCH_RUMBLE = 0x43
NATIVE_SWITCH_RUMBLE_SCHEMA_VERSION = 2
NATIVE_SWITCH_RUMBLE_ROW_SIZE = 80
NATIVE_SWITCH_RUMBLE_SLOT_COUNT = 4
NATIVE_SWITCH_RUMBLE_LATENCY_NOTE = (
    "Latency percentiles are host-receipt-to-HCI-submission histogram upper "
    "bounds in 250 us buckets (tail uses observed maximum), not physical latency. "
    "Coalesced held-state commands require no new packet and are excluded."
)
MACRO_CAPTURE_SCHEMA_VERSION = 1
MACRO_CAPTURE_STATES = (
    "idle",
    "recording",
    "stopped",
    "full",
    "timed_out",
    "disconnected",
)

STATUS_OK = 0
STATUS_PENDING = 1
STATUS_UNSUPPORTED_SCHEMA = 3
STATUS_NAMES = {
    2: "malformed request",
    3: "unsupported schema",
    4: "value too large",
    5: "out-of-order transaction",
    6: "CRC mismatch",
    7: "device busy",
    8: "storage failure",
}

CONFIGURATION_SCHEMA_VERSION = 4
JOYCON_MODE_PAIRED = 0
JOYCON_MODE_INDIVIDUAL = 1
JOYCON_MODE_NAMES = ("paired", "individual")
CONFIGURATION_SIZE = 232
NATIVE_SWITCH_CONTROLLER_CAPACITY = 16
NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE = (
    "Native rumble requires a genuine qualified Nintendo Switch Pro Controller "
    "or Joy-Con. Approval applies to this physical controller across all profiles; "
    "matching VID/PID is not automatic proof of clone support."
)
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
ACTIVE_MODE_NATIVE_HUB = 5
ACTIVE_MODE_NAMES = (
    "Switch",
    "Switch probe",
    "XInput",
    "DInput",
    "Mac",
    "Native Joy-Con hub",
)
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
TRANSPORT_JOYCON_PAIR = 3
PROFILE_LEGACY_SCHEMA_VERSION = 1
PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION = 2
PROFILE_CONTROL_MAPPING_SCHEMA_VERSION = 3
PROFILE_ACTION_CONTROL_SCHEMA_VERSION = 4
PROFILE_SPARSE_MACRO_SCHEMA_VERSION = 5
PROFILE_EXPANDED_SCHEMA_VERSION = 6
PROFILE_EXTRA_CONTROL_SCHEMA_VERSION = 7
PROFILE_SWING_SCHEMA_VERSION = 8
PROFILE_SCHEMA_VERSION = 9
PROFILE_LEGACY_SIZE = 256
PROFILE_SIZE = 384
PROFILE_CAPACITY = 8
PROFILE_IDENTITY_CAPACITY = 16
PROFILE_LIST_CAPACITY = PROFILE_IDENTITY_CAPACITY + 1
CONTROLLER_IDENTITY_SIZE = 14
PROFILE_LIST_ROW_SIZE = 48
PROFILE_NONE_BUTTON = 0xFF
PROFILE_MACRO_COUNT = 4
PROFILE_MACRO_STEP_CAPACITY = 16
PROFILE_MACRO_STEPS_PER_MACRO = 8
PROFILE_LEGACY_MACRO_STEP_CAPACITY = 8
PROFILE_MACRO_STREAM_SIZE = 136
PROFILE_MACRO_STEP_SIZE = 19
PROFILE_MAXIMUM_WAIT_MS = 10000
PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD = 0x8000
PROFILE_DEFAULT_DIGITAL_THRESHOLD = 22934
PROFILE_TURBO_RATE_MIN = 1
PROFILE_TURBO_RATE_MAX = 30
PROFILE_TURBO_DUTY_MIN = 1
PROFILE_TURBO_DUTY_MAX = 99
PROFILE_TURBO_BURST_MIN = 1
PROFILE_TURBO_BURST_MAX = 255
PROFILE_MACRO_REPEAT_MIN = 1
PROFILE_MACRO_REPEAT_MAX = 255
PROFILE_COMBINATION_WINDOW_MIN = 30
PROFILE_COMBINATION_WINDOW_MAX = 200
PROFILE_COMBINATION_WINDOW_DEFAULT = 100
PROFILE_PLAYTEST_LEGACY_SCHEMA_VERSION = 2
PROFILE_PLAYTEST_LEGACY_SIZE = 54
PROFILE_PLAYTEST_EXTRA_BUTTON_SCHEMA_VERSION = 3
PROFILE_PLAYTEST_EXTRA_BUTTON_SIZE = 55
PROFILE_PLAYTEST_TOPOLOGY_SCHEMA_VERSION = 4
PROFILE_PLAYTEST_SCHEMA_VERSION = 5
PROFILE_PLAYTEST_SIZE = 56
PROFILE_PLAYTEST_LAYOUTS = (
    None,
    "joycon2-left",
    "joycon2-right",
    "joycon2-pair",
    "wii-remote",
    "wii-nunchuk",
    "wii-horizontal",
    "wii-vertical",
)
PROFILE_PLAYTEST_SLOT_COUNT = 4
PROFILE_METADATA_SCHEMA_VERSION = 1
PROFILE_METADATA_MAX_BYTES = 31
PROFILE_METADATA_VALUE_SIZE = 32
PROFILE_METADATA_SIZE = 288
HAPTICS_EXPERIMENT_SCHEMA_VERSION = 5
HAPTICS_EXPERIMENT_SIZE = 84
HAPTICS_EXPERIMENT_SLOT_COUNT = 4
HAPTICS_TRANSPORT_PROBE_SCHEMA_VERSION = 3
HAPTICS_TRANSPORT_PROBE_SIZE = 176
HAPTICS_EXPERIMENT_STATES = (
    "idle",
    "pending",
    "running",
    "completed",
    "stopped",
    "disconnected",
    "unsupported",
    "error",
)
HAPTICS_EXPERIMENT_MODES = ("fixture", "gameplay")
HAPTICS_EXPERIMENT_ERRORS = {
    0: "none",
    1: "unsupported controller or Bluetooth protocol",
    2: "insufficient Bluetooth MTU",
    3: "controller connection missing or lost",
    4: "Bluetooth can-send or stop timed out",
    5: "Bluetooth request or send failed",
    6: "queued controller output; wait for prior output to drain, then retry",
}
HAPTICS_EXPERIMENT_ENABLE_HINT = (
    "Install firmware built with SWITCH_PICO_HAPTICS_EXPERIMENT=ON "
    "and connect a DualSense or DualSense Edge over Bluetooth Classic."
)
HAPTICS_EXPERIMENT_EVIDENCE_NOTE = (
    "Send timestamps measure firmware/HCI submission, not physical actuator "
    "onset or playback; USB ACK only accepts a request. "
    "The initial 1.024 s of priming silence is intentional, not transport delay."
)
HAPTICS_GAMEPLAY_EVIDENCE_NOTE = (
    "Send timestamps measure firmware/HCI submission, not physical actuator "
    "onset or playback; USB ACK only accepts a request. Gameplay streams "
    "continuously, including silence without host commands; first-tone "
    "timestamps remain zero until the first nonzero PCM."
)
HAPTICS_GAMEPLAY_ARMING_NOTE = (
    "PC-controlled gameplay arming does not persist across power cycles. "
    "Firmware built with SWITCH_PICO_HD_RUMBLE=ON automatically arms the first "
    "eligible DualSense that becomes ready, in any slot. One native stream is "
    "selected at a time; use gameplay --slot to select another controller."
)
HAPTICS_GAMEPLAY_TIMING = {
    "sample_rate_hz": 3000,
    "switch_command_window_us": 8000,
    "switch_watchdog_us": 50000,
    "xinput_command_policy": "held_until_changed_or_stopped",
    "xinput_carrier_hz": {"left_low": 160, "right_high": 320},
    "band_gains": {"low": 2.0, "high": 2.0},
    "response_exponent": 0.8,
}
HAPTICS_TRANSPORT_PROBE_UNSUPPORTED_HINT = (
    "Firmware does not support haptics transport profile operation 0x41. "
    "Install updated firmware built with SWITCH_PICO_HAPTICS_EXPERIMENT=ON "
    "and transport-probe support; haptics-experiment status still uses 0x40."
)
HAPTICS_TRANSPORT_PROBE_EVIDENCE_NOTE = (
    "Durations are inclusive and may overlap or nest (poll/read/send/write); "
    "do not sum their totals. first_tone_send_return_us is the low 32-bit "
    "Pico uptime timestamp after l2cap_send returns successfully, not physical "
    "actuator onset or playback. ACL extrema are observed samples, not an "
    "exact occupancy timeline; completion counters select this connection "
    "handle. Running snapshots are correlated to one run, not one instant."
)

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
OUTPUT_CONTROLS = LOGICAL_BUTTONS + ("left_trigger", "right_trigger")
EXTRA_BUTTONS = ("c", "gl", "gr", "left_sl", "left_sr", "right_sl", "right_sr")
LOGICAL_CONTROLS = OUTPUT_CONTROLS + EXTRA_BUTTONS
PROFILE_LOGICAL_CONTROL_MASK = (1 << len(LOGICAL_CONTROLS)) - 1
RUMBLE_POLICIES = ("none", "rumble", "led", "rumble_and_led")
TURBO_MODES = ("off", "turbo", "auto_burst", "burst")
SHIFT_MODES = ("off", "hold", "toggle")
MACRO_PLAYBACK_MODES = ("once", "while_held", "toggle", "repeat")
SWING_SENSITIVITIES = ("low", "medium", "high")
SHORTCUT_SELECTOR_BUTTONS = LOGICAL_BUTTONS[:4] + LOGICAL_BUTTONS[12:]
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
    ) -> Any: ...


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
    switch2_ingress_drops: int | None = None
    switch2_output_drops: int | None = None


@dataclass(frozen=True)
class NativeSwitchRumbleSlot:
    slot: int
    parser_type: int
    firmware_high: int
    firmware_low: int
    flags: int
    generation: int
    received_commands: int
    submitted_reports: int
    dropped_commands: int
    resynchronizations: int
    raw_commands: int
    quantized_commands: int
    congested_attempts: int
    completed_commands: int
    p50_upper_us: int
    p95_upper_us: int
    p99_upper_us: int
    max_latency_us: int
    queue_depth: int
    last_wire_low_u32: int
    last_wire_high_u32: int
    max_encode_us: int
    coalesced_commands: int

    def to_json_object(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "connected": bool(self.flags & 1),
            "approved": bool(self.flags & 2),
            "active": bool(self.flags & 4),
            "mono": bool(self.flags & 8),
            "feedback": bool(self.flags & 16),
            "last_wire_hex": struct.pack(
                "<II", self.last_wire_low_u32, self.last_wire_high_u32
            ).hex(),
        }


@dataclass(frozen=True)
class HapticsExperimentDiagnostics:
    run_id: int
    connection_generation: int
    start_us: int
    generated_packets: int
    sent_packets: int
    skipped_packets: int
    send_failures: int
    can_send_requests: int
    synchronous_callbacks: int
    max_generate_us: int
    max_send_gap_us: int
    max_lateness_us: int
    max_request_wait_us: int
    first_tone_due_us: int
    first_tone_sent_us: int
    last_sent_us: int
    elapsed_us: int
    state: int
    slot: int | None
    last_error: int
    mode: int
    host_updates: int
    dropped_updates: int
    packet_frames: int
    last_packet_nonzero: bool = False

    @property
    def state_name(self) -> str:
        return HAPTICS_EXPERIMENT_STATES[self.state]

    @property
    def mode_name(self) -> str:
        return HAPTICS_EXPERIMENT_MODES[self.mode]

    @property
    def error_name(self) -> str:
        return HAPTICS_EXPERIMENT_ERRORS.get(
            self.last_error, f"unknown error {self.last_error}"
        )

    @property
    def firmware_supported(self) -> bool:
        return not (self.state_name == "unsupported" and self.slot is None)

    @property
    def first_tone_submission_delay_us(self) -> int | None:
        if self.first_tone_sent_us == 0:
            return None
        return (self.first_tone_sent_us - self.first_tone_due_us) & 0xFFFFFFFF

    def to_json_object(self) -> dict[str, Any]:
        values = {
            **asdict(self),
            "schema_version": HAPTICS_EXPERIMENT_SCHEMA_VERSION,
            "state_name": self.state_name,
            "mode_name": self.mode_name,
            "error_name": self.error_name,
            "firmware_supported": self.firmware_supported,
            "first_tone_submission_delay_us": self.first_tone_submission_delay_us,
        }
        if self.mode == 1:
            values["gameplay"] = {
                **HAPTICS_GAMEPLAY_TIMING,
                "stereo_frames_per_packet": self.packet_frames,
                "lookback_us": self.packet_frames * 1000000 / 3000,
            }
            values["evidence_note"] = HAPTICS_GAMEPLAY_EVIDENCE_NOTE
            values["arming_note"] = HAPTICS_GAMEPLAY_ARMING_NOTE
        else:
            phase_packets = 768 // self.packet_frames
            values["pattern"] = {
                "sample_rate_hz": 3000,
                "stereo_frames_per_packet": self.packet_frames,
                "packet_interval_us": self.packet_frames * 1000000 / 3000,
                "peak_amplitude": 32,
                "priming_silence_packets": 3072 // self.packet_frames,
                "cycles": 4,
                "phases": [
                    {"channel": "left", "frequency_hz": 100, "packets": phase_packets},
                    {"channel": "silence", "packets": phase_packets},
                    {"channel": "right", "frequency_hz": 200, "packets": phase_packets},
                    {"channel": "silence", "packets": phase_packets},
                ],
                "trailing_silence_packets": 3072 // self.packet_frames,
                "total_packets": 18432 // self.packet_frames,
                "initial_mode_packet_stereo_frames": 0,
                "duration_us": 6144000,
            }
            values["evidence_note"] = HAPTICS_EXPERIMENT_EVIDENCE_NOTE
        return values


@dataclass(frozen=True)
class MacroCaptureEvent:
    at_us: int
    buttons: int
    left_x: int
    left_y: int
    right_x: int
    right_y: int
    left_trigger: int
    right_trigger: int


@dataclass(frozen=True)
class MacroCapturePage:
    run_id: int
    connection_generation: int
    elapsed_us: int
    slot: int
    state: int
    channels: int
    total_events: int
    first_index: int
    axis_quantum: int
    trigger_quantum: int
    max_duration_ms: int
    max_events: int
    events: tuple[MacroCaptureEvent, ...]

    @property
    def state_name(self) -> str:
        return MACRO_CAPTURE_STATES[self.state]

    def to_json_object(self) -> dict[str, Any]:
        return {**asdict(self), "state_name": self.state_name}


@dataclass(frozen=True)
class HapticsTransportProbe:
    run_id: int
    connection_generation: int
    connection_handle: int
    timer_wakes: int
    max_timer_lateness_us: int
    total_timer_lateness_us: int
    send_calls: int
    max_send_us: int
    total_send_us: int
    write_calls: int
    max_write_us: int
    total_write_us: int
    read_calls: int
    read_packets: int
    max_read_us: int
    total_read_us: int
    poll_calls: int
    max_poll_us: int
    total_poll_us: int
    completion_events: int
    completed_packets: int
    max_completion_gap_us: int
    max_outstanding_acl: int
    min_free_acl: int
    first_tone_send_return_us: int
    active: bool
    max_permission_wait_us: int
    total_permission_wait_us: int
    permission_callbacks: int
    max_poll_gap_us: int
    controller_acl_packet_bytes: int
    controller_acl_packet_count: int
    requested_sys_khz: int
    measured_sys_khz: int
    measured_usb_khz: int
    core_voltage_mv: int
    flash_clock_divider: int
    cyw43_pio_divider256: int
    temperature_millicelsius: int
    host_completed_writes: int
    acl_writes: int
    other_writes: int
    write_failures: int
    packet_read_optimized: int

    def to_json_object(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "schema_version": HAPTICS_TRANSPORT_PROBE_SCHEMA_VERSION,
            "evidence_note": HAPTICS_TRANSPORT_PROBE_EVIDENCE_NOTE,
        }


@dataclass(frozen=True)
class AdapterConfiguration:
    pairing_window_seconds: int
    generation: int
    crc: int
    requested_mode: int = REQUESTED_MODE_AUTO
    native_switch_controllers: tuple[ControllerIdentity, ...] = ()
    schema_version: int = CONFIGURATION_SCHEMA_VERSION
    joycon_mode: int = JOYCON_MODE_PAIRED


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

    def __post_init__(self) -> None:
        if self.transport not in (TRANSPORT_CLASSIC, TRANSPORT_BLE):
            raise ConfigManagerError(
                "pairing record must identify a physical Bluetooth peer"
            )

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
            suffix = address_types.get(self.address_type, f"type {self.address_type}")
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


def _require_object(value: Any, fields: Sequence[str], name: str) -> dict[str, Any]:
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
        raise ConfigManagerError(f"{name} must be one of {', '.join(choices)}")
    return choices.index(value)


def _button_index(value: Any, name: str) -> int:
    if value is None:
        return PROFILE_NONE_BUTTON
    if type(value) is not str or value not in LOGICAL_BUTTONS:
        raise ConfigManagerError(f"{name} must be a logical button name or null")
    return LOGICAL_BUTTONS.index(value)


def _button_name(value: int) -> str | None:
    if value == PROFILE_NONE_BUTTON:
        return None
    return LOGICAL_BUTTONS[value]


def _control_index(
    value: Any, name: str, *, schema_version: int = PROFILE_SCHEMA_VERSION
) -> int:
    if value is None:
        return PROFILE_NONE_BUTTON
    controls = (
        LOGICAL_CONTROLS
        if schema_version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION
        else OUTPUT_CONTROLS
    )
    if type(value) is not str or value not in controls:
        choices = ", ".join(controls)
        raise ConfigManagerError(f"{name} must be null or one of: {choices}")
    return controls.index(value)


def _control_name(value: int) -> str | None:
    if value == PROFILE_NONE_BUTTON:
        return None
    return LOGICAL_CONTROLS[value]


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


def _control_mask_from_json(
    value: Any, name: str, *, schema_version: int = PROFILE_SCHEMA_VERSION
) -> int:
    if type(value) is not list:
        raise ConfigManagerError(f"{name} must be a JSON array")
    mask = 0
    for entry in value:
        index = _control_index(entry, name, schema_version=schema_version)
        if index == PROFILE_NONE_BUTTON:
            raise ConfigManagerError(f"{name} cannot contain null")
        bit = 1 << index
        if mask & bit:
            raise ConfigManagerError(f"{name} contains a duplicate control")
        mask |= bit
    return mask


def _control_mask_to_json(mask: int) -> list[str]:
    return [name for index, name in enumerate(LOGICAL_CONTROLS) if mask & (1 << index)]


def _button_mask_to_json(mask: int) -> list[str]:
    return [name for index, name in enumerate(LOGICAL_BUTTONS) if mask & (1 << index)]


@dataclass(frozen=True)
class ControllerIdentity:
    stable: bool
    transport: int
    address_type: int
    address: bytes
    vendor_id: int
    product_id: int
    partner_address_type: int = 0
    partner_address: bytes = bytes(6)

    def __post_init__(self) -> None:
        _require_bool(self.stable, "identity stable")
        _require_int(self.transport, "identity transport", 0, TRANSPORT_JOYCON_PAIR)
        _require_int(self.address_type, "identity address_type", 0, 0xFF)
        if type(self.address) is not bytes or len(self.address) != 6:
            raise ConfigManagerError("identity address must contain six bytes")
        _require_int(self.vendor_id, "identity vendor_id", 0, 0xFFFF)
        _require_int(self.product_id, "identity product_id", 0, 0xFFFF)
        _require_int(
            self.partner_address_type, "identity partner_address_type", 0, 0xFF
        )
        if type(self.partner_address) is not bytes or len(self.partner_address) != 6:
            raise ConfigManagerError("identity partner_address must contain six bytes")
        if self.is_joycon_pair:
            if (
                not self.stable
                or self.vendor_id != 0x057E
                or self.product_id != 0x2067
                or self.address_type not in (0, 1)
                or self.partner_address_type not in (0, 1)
                or (self.address_type == 1 and self.address[0] & 0xC0 != 0xC0)
                or (
                    self.partner_address_type == 1
                    and self.partner_address[0] & 0xC0 != 0xC0
                )
                or (self.address_type, self.address)
                == (self.partner_address_type, self.partner_address)
            ):
                raise ConfigManagerError("invalid Joy-Con 2 pair identity")
            return
        if self.partner_address_type != 0 or self.partner_address != bytes(6):
            raise ConfigManagerError(
                "physical identity cannot contain a partner address"
            )
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
    def make_joycon_pair(
        cls, left: ControllerIdentity, right: ControllerIdentity
    ) -> ControllerIdentity:
        for member, product_id in ((left, 0x2067), (right, 0x2066)):
            if (
                not isinstance(member, ControllerIdentity)
                or not member.stable
                or member.transport != TRANSPORT_BLE
                or member.vendor_id != 0x057E
                or member.product_id != product_id
                or member.address_type not in (0, 1)
            ):
                raise ConfigManagerError(
                    "pair members must be stable BLE Joy-Con 2 (L) and (R) identities"
                )
        return cls(
            True,
            TRANSPORT_JOYCON_PAIR,
            left.address_type,
            left.address,
            0x057E,
            0x2067,
            right.address_type,
            right.address,
        )

    def joycon_pair_members(self) -> tuple[ControllerIdentity, ControllerIdentity]:
        if not self.is_joycon_pair:
            raise ConfigManagerError("identity is not a Joy-Con 2 pair")
        return (
            ControllerIdentity(
                True,
                TRANSPORT_BLE,
                self.address_type,
                self.address,
                0x057E,
                0x2067,
            ),
            ControllerIdentity(
                True,
                TRANSPORT_BLE,
                self.partner_address_type,
                self.partner_address,
                0x057E,
                0x2066,
            ),
        )

    @classmethod
    def from_bytes(cls, payload: bytes) -> ControllerIdentity:
        payload = bytes(payload)
        if len(payload) != CONTROLLER_IDENTITY_SIZE:
            raise ConfigManagerError("invalid controller identity size")
        if payload[1] == TRANSPORT_JOYCON_PAIR:
            flags = payload[0]
            if flags & ~0x07 or not flags & 1:
                raise ConfigManagerError("invalid Joy-Con 2 pair flags")
            return cls(
                True,
                TRANSPORT_JOYCON_PAIR,
                (flags >> 1) & 1,
                payload[2:8],
                0x057E,
                0x2067,
                (flags >> 2) & 1,
                payload[8:14],
            )
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
        if self.is_joycon_pair:
            return (
                bytes(
                    (
                        1 | (self.address_type << 1) | (self.partner_address_type << 2),
                        TRANSPORT_JOYCON_PAIR,
                    )
                )
                + self.address
                + self.partner_address
            )
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
    def is_joycon_pair(self) -> bool:
        return self.transport == TRANSPORT_JOYCON_PAIR

    @property
    def address_text(self) -> str:
        return ":".join(f"{octet:02X}" for octet in self.address)

    @property
    def partner_address_text(self) -> str:
        return ":".join(f"{octet:02X}" for octet in self.partner_address)

    @property
    def transport_text(self) -> str:
        if self.transport == TRANSPORT_CLASSIC:
            return "Classic"
        if self.transport == TRANSPORT_BLE:
            return "BLE"
        if self.is_joycon_pair:
            return "BLE pair"
        return "Unknown"

    def to_json_object(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "stable": self.stable,
            "address": self.address_text,
            "address_type": self.address_type,
            "transport": self.transport_text,
            "vendor_id": self.vendor_id,
            "product_id": self.product_id,
            "is_joycon_pair": self.is_joycon_pair,
        }
        if self.is_joycon_pair:
            left, right = self.joycon_pair_members()
            result.update(
                partner_address=self.partner_address_text,
                partner_address_type=self.partner_address_type,
                members={
                    "left": left.to_json_object(),
                    "right": right.to_json_object(),
                },
            )
        return result


@dataclass(frozen=True)
class ProfileListEntry:
    identity: ControllerIdentity
    active_profile_index: int
    alias: str = ""

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
        if (
            type(self.alias) is not str
            or "\x00" in self.alias
            or len(self.alias.encode("utf-8")) > PROFILE_METADATA_MAX_BYTES
        ):
            raise ConfigManagerError(
                "controller alias must contain at most 31 UTF-8 bytes"
            )


@dataclass(frozen=True)
class ProfileMetadata:
    alias: str
    profile_names: tuple[str, ...]

    def __post_init__(self) -> None:
        if len(self.profile_names) != PROFILE_CAPACITY:
            raise ConfigManagerError("profile metadata must contain eight names")
        for label, value in (
            ("controller alias", self.alias),
            *(
                (f"profile {index + 1} name", name)
                for index, name in enumerate(self.profile_names)
            ),
        ):
            if (
                type(value) is not str
                or "\x00" in value
                or len(value.encode("utf-8")) > PROFILE_METADATA_MAX_BYTES
            ):
                raise ConfigManagerError(f"{label} must contain at most 31 UTF-8 bytes")


@dataclass(frozen=True)
class ProfilePlaytest:
    connected: bool
    slot_index: int | None
    connection_generation: int
    state_generation: int
    identity: ControllerIdentity | None
    button_mask: int
    left_stick: tuple[int, int]
    right_stick: tuple[int, int]
    triggers: tuple[int, int]
    battery: int
    capabilities: int
    motion: tuple[int, int, int, int, int, int] | None
    extra_buttons: int = 0
    layout: str | None = None

    def to_json_object(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "slot": self.slot_index,
            "connection_generation": self.connection_generation,
            "state_generation": self.state_generation,
            "identity": (
                self.identity.to_json_object() if self.identity is not None else None
            ),
            "layout": self.layout,
            "buttons": _button_mask_to_json(self.button_mask),
            "extra_buttons": [
                name
                for index, name in enumerate(EXTRA_BUTTONS)
                if self.extra_buttons & (1 << index)
            ],
            "left_stick": {
                "x": self.left_stick[0],
                "y": self.left_stick[1],
            },
            "right_stick": {
                "x": self.right_stick[0],
                "y": self.right_stick[1],
            },
            "triggers": {
                "left": self.triggers[0],
                "right": self.triggers[1],
            },
            "battery": (
                round((self.battery - 1) / 254 * 100) if self.battery != 0 else None
            ),
            "capabilities": [
                name
                for bit, name in enumerate(
                    ("rumble", "lightbar", "player_leds", "motion")
                )
                if self.capabilities & (1 << bit)
            ],
            "motion": (
                {
                    "accel": list(self.motion[:3]),
                    "gyro": list(self.motion[3:]),
                }
                if self.motion is not None
                else None
            ),
        }


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
        _require_int(self.inner_deadzone, "stick inner_deadzone", 0, 0x7FFF)
        _require_int(self.outer_saturation, "stick outer_saturation", 1, 0x7FFF)
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
            _require_int(obj["curve_q8_8"], f"{name}.curve_q8_8", 1, 0xFFFF),
            _require_bool(obj["invert_x"], f"{name}.invert_x"),
            _require_bool(obj["invert_y"], f"{name}.invert_y"),
        )


@dataclass(frozen=True)
class TriggerConfig:
    lower_deadzone: int
    upper_saturation: int
    curve_q8_8: int
    digital_threshold: int
    output: int

    def __post_init__(self) -> None:
        _require_int(self.lower_deadzone, "trigger lower_deadzone", 0, 0xFFFF)
        _require_int(self.upper_saturation, "trigger upper_saturation", 1, 0xFFFF)
        if self.lower_deadzone >= self.upper_saturation:
            raise ConfigManagerError(
                "trigger lower_deadzone must be below upper_saturation"
            )
        _require_int(self.curve_q8_8, "trigger curve_q8_8", 1, 0xFFFF)
        _require_int(self.digital_threshold, "trigger digital_threshold", 0, 0xFFFF)
        if (
            type(self.output) is not int
            or self.output != PROFILE_NONE_BUTTON
            and not 0 <= self.output < len(OUTPUT_CONTROLS)
        ):
            raise ConfigManagerError("invalid trigger output mapping")

    @classmethod
    def from_bytes(
        cls,
        payload: bytes,
        *,
        schema_version: int,
        source_index: int,
    ) -> TriggerConfig:
        if len(payload) != 10 or payload[9] != 0:
            raise ConfigManagerError("invalid trigger configuration encoding")
        if schema_version < PROFILE_CONTROL_MAPPING_SCHEMA_VERSION and payload[8] != 0:
            raise ConfigManagerError("invalid legacy trigger configuration")
        output = (
            payload[8]
            if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION
            else len(LOGICAL_BUTTONS) + source_index
        )
        lower, upper, curve_q8_8, threshold = struct.unpack("<HHHH", payload[:8])
        return cls(lower, upper, curve_q8_8, threshold, output)

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<HHHHBB",
            self.lower_deadzone,
            self.upper_saturation,
            self.curve_q8_8,
            self.digital_threshold,
            self.output,
            0,
        )

    def to_json_object(self) -> dict[str, int | str | None]:
        return {
            "lower_deadzone": self.lower_deadzone,
            "upper_saturation": self.upper_saturation,
            "curve_q8_8": self.curve_q8_8,
            "digital_threshold": self.digital_threshold,
            "output": _control_name(self.output),
        }

    @classmethod
    def from_json_object(
        cls,
        value: Any,
        name: str,
        *,
        schema_version: int,
        source_index: int,
    ) -> TriggerConfig:
        fields = (
            "lower_deadzone",
            "upper_saturation",
            "curve_q8_8",
            "digital_threshold",
        )
        if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION:
            fields += ("output",)
        obj = _require_object(value, fields, name)
        output = (
            _control_index(obj["output"], f"{name}.output")
            if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION
            else len(LOGICAL_BUTTONS) + source_index
        )
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
            _require_int(obj["curve_q8_8"], f"{name}.curve_q8_8", 1, 0xFFFF),
            _require_int(
                obj["digital_threshold"],
                f"{name}.digital_threshold",
                0,
                0xFFFF,
            ),
            output,
        )


def _migrate_legacy_trigger_threshold(trigger: TriggerConfig) -> TriggerConfig:
    if trigger.digital_threshold != PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD:
        return trigger
    return TriggerConfig(
        trigger.lower_deadzone,
        trigger.upper_saturation,
        trigger.curve_q8_8,
        PROFILE_DEFAULT_DIGITAL_THRESHOLD,
        trigger.output,
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
        _require_int(self.output_button_mask, "macro output button mask", 0, 0xFFFF)
        for name in (
            "left_stick_x",
            "left_stick_y",
            "right_stick_x",
            "right_stick_y",
        ):
            _require_int(getattr(self, name), f"macro {name}", -0x8000, 0x7FFF)
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
            if not self.override_flags & 1 and self.output_button_mask != 0:
                raise ConfigManagerError("macro buttons require the buttons override")
            if not self.override_flags & 2 and (
                self.left_stick_x != 0 or self.left_stick_y != 0
            ):
                raise ConfigManagerError(
                    "macro left stick values require the left_stick override"
                )
            if not self.override_flags & 4 and (
                self.right_stick_x != 0 or self.right_stick_y != 0
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

    def to_sparse_bytes(self) -> bytes:
        if self.step_type != 0:
            raise ConfigManagerError("only state steps use sparse encoding")
        payload = bytearray(struct.pack("<BH", self.override_flags, self.duration_ms))
        if self.override_flags & 1:
            payload.extend(struct.pack("<H", self.output_button_mask))
        if self.override_flags & 2:
            payload.extend(struct.pack("<hh", self.left_stick_x, self.left_stick_y))
        if self.override_flags & 4:
            payload.extend(struct.pack("<hh", self.right_stick_x, self.right_stick_y))
        if self.override_flags & 8:
            payload.extend(struct.pack("<H", self.left_trigger))
        if self.override_flags & 16:
            payload.extend(struct.pack("<H", self.right_trigger))
        return bytes(payload)

    @classmethod
    def from_sparse_bytes(cls, payload: bytes, name: str) -> tuple[MacroStep, int]:
        if len(payload) < 3:
            raise ConfigManagerError(f"{name} is truncated")
        flags, duration = struct.unpack_from("<BH", payload)
        if flags & ~MACRO_OVERRIDE_MASK:
            raise ConfigManagerError(f"{name} has invalid override flags")
        offset = 3

        def take(fmt: str) -> tuple[int, ...]:
            nonlocal offset
            size = struct.calcsize(fmt)
            if offset + size > len(payload):
                raise ConfigManagerError(f"{name} is truncated")
            values = struct.unpack_from(fmt, payload, offset)
            offset += size
            return values

        buttons = take("<H")[0] if flags & 1 else 0
        left_x, left_y = take("<hh") if flags & 2 else (0, 0)
        right_x, right_y = take("<hh") if flags & 4 else (0, 0)
        left_trigger = take("<H")[0] if flags & 8 else 0
        right_trigger = take("<H")[0] if flags & 16 else 0
        return (
            cls(
                0,
                flags,
                duration,
                buttons,
                left_x,
                left_y,
                right_x,
                right_y,
                left_trigger,
                right_trigger,
            ),
            offset,
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
            "output_buttons": _button_mask_to_json(self.output_button_mask),
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
            index = _require_enum(override, MACRO_OVERRIDE_NAMES, f"{name}.overrides")
            bit = 1 << index
            if override_flags & bit:
                raise ConfigManagerError(f"{name}.overrides contains a duplicate")
            override_flags |= bit
        left = _require_object(obj["left_stick"], ("x", "y"), f"{name}.left_stick")
        right = _require_object(obj["right_stick"], ("x", "y"), f"{name}.right_stick")
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
            _button_mask_from_json(obj["output_buttons"], f"{name}.output_buttons"),
            _require_int(left["x"], f"{name}.left_stick.x", -0x8000, 0x7FFF),
            _require_int(left["y"], f"{name}.left_stick.y", -0x8000, 0x7FFF),
            _require_int(right["x"], f"{name}.right_stick.x", -0x8000, 0x7FFF),
            _require_int(right["y"], f"{name}.right_stick.y", -0x8000, 0x7FFF),
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
class ProfileShortcuts:
    modifier: int = PROFILE_NONE_BUTTON
    profiles: tuple[int, ...] = (PROFILE_NONE_BUTTON,) * PROFILE_CAPACITY

    def __post_init__(self) -> None:
        if type(self.modifier) is not int or (
            self.modifier != PROFILE_NONE_BUTTON
            and not 0 <= self.modifier < len(LOGICAL_CONTROLS)
        ):
            raise ConfigManagerError("invalid shortcut modifier")
        if type(self.profiles) is not tuple or len(self.profiles) != PROFILE_CAPACITY:
            raise ConfigManagerError("shortcuts must contain eight profile selectors")
        selected: set[int] = set()
        for selector in self.profiles:
            if type(selector) is not int:
                raise ConfigManagerError("invalid shortcut selector")
            if selector == PROFILE_NONE_BUTTON:
                continue
            if not (0 <= selector < 4 or 12 <= selector < 16):
                raise ConfigManagerError(
                    "shortcut selector must be a face or D-pad button"
                )
            if selector in selected or selector == self.modifier:
                raise ConfigManagerError(
                    "shortcut selectors must be unique and differ from modifier"
                )
            if self.modifier == PROFILE_NONE_BUTTON:
                raise ConfigManagerError("enabled shortcuts require a modifier")
            selected.add(selector)

    def to_json_object(self) -> dict[str, Any]:
        return {
            "modifier": _control_name(self.modifier),
            "profiles": [_button_name(value) for value in self.profiles],
        }

    @classmethod
    def from_json_object(
        cls, value: Any, *, schema_version: int = PROFILE_SCHEMA_VERSION
    ) -> ProfileShortcuts:
        obj = _require_object(value, ("modifier", "profiles"), "profile.shortcuts")
        if type(obj["profiles"]) is not list:
            raise ConfigManagerError("profile.shortcuts.profiles must be an array")
        return cls(
            _control_index(
                obj["modifier"],
                "profile.shortcuts.modifier",
                schema_version=schema_version,
            ),
            tuple(
                _button_index(selector, f"profile.shortcuts.profiles[{index}]")
                for index, selector in enumerate(obj["profiles"])
            ),
        )


@dataclass(frozen=True)
class ProfileShift:
    mode: int = 0
    modifier: int = PROFILE_NONE_BUTTON
    button_map: tuple[int, ...] = tuple(range(len(LOGICAL_BUTTONS)))
    extra_button_map: tuple[int, ...] = (PROFILE_NONE_BUTTON,) * len(EXTRA_BUTTONS)

    def __post_init__(self) -> None:
        _require_int(self.mode, "Shift mode", 0, len(SHIFT_MODES) - 1)
        if type(self.modifier) is not int or (
            self.modifier != PROFILE_NONE_BUTTON
            and not 0 <= self.modifier < len(LOGICAL_CONTROLS)
        ):
            raise ConfigManagerError("invalid Shift modifier")
        if self.mode != 0 and self.modifier == PROFILE_NONE_BUTTON:
            raise ConfigManagerError("enabled Shift requires a modifier")
        if type(self.button_map) is not tuple or len(self.button_map) != len(
            LOGICAL_BUTTONS
        ):
            raise ConfigManagerError("Shift button map must contain 16 mappings")
        if type(self.extra_button_map) is not tuple or len(
            self.extra_button_map
        ) != len(EXTRA_BUTTONS):
            raise ConfigManagerError(
                "Shift extra button map must contain seven mappings"
            )
        for output in (*self.button_map, *self.extra_button_map):
            if type(output) is not int or (
                output != PROFILE_NONE_BUTTON and not 0 <= output < len(LOGICAL_BUTTONS)
            ):
                raise ConfigManagerError("Shift outputs must be buttons or null")

    def to_json_object(self) -> dict[str, Any]:
        return {
            "mode": SHIFT_MODES[self.mode],
            "modifier": _control_name(self.modifier),
            "button_map": {
                name: _button_name(self.button_map[index])
                for index, name in enumerate(LOGICAL_BUTTONS)
            },
            "extra_button_map": {
                name: _button_name(self.extra_button_map[index])
                for index, name in enumerate(EXTRA_BUTTONS)
            },
        }

    @classmethod
    def from_json_object(
        cls, value: Any, *, schema_version: int = PROFILE_SCHEMA_VERSION
    ) -> ProfileShift:
        fields = ["mode", "modifier", "button_map"]
        if schema_version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION:
            fields.append("extra_button_map")
        obj = _require_object(value, fields, "profile.shift")
        mappings = _require_object(
            obj["button_map"], LOGICAL_BUTTONS, "profile.shift.button_map"
        )
        extras = (
            _require_object(
                obj["extra_button_map"], EXTRA_BUTTONS, "profile.shift.extra_button_map"
            )
            if schema_version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION
            else dict.fromkeys(EXTRA_BUTTONS)
        )
        return cls(
            _require_enum(obj["mode"], SHIFT_MODES, "profile.shift.mode"),
            _control_index(
                obj["modifier"], "profile.shift.modifier", schema_version=schema_version
            ),
            tuple(
                _button_index(mappings[name], f"profile.shift.button_map.{name}")
                for name in LOGICAL_BUTTONS
            ),
            tuple(
                _button_index(extras[name], f"profile.shift.extra_button_map.{name}")
                for name in EXTRA_BUTTONS
            ),
        )


@dataclass(frozen=True)
class TurboSettings:
    rate_hz: int = 15
    duty_percent: int = 50
    burst_count: int = 3

    def __post_init__(self) -> None:
        _require_int(
            self.rate_hz, "Turbo rate", PROFILE_TURBO_RATE_MIN, PROFILE_TURBO_RATE_MAX
        )
        _require_int(
            self.duty_percent,
            "Turbo duty",
            PROFILE_TURBO_DUTY_MIN,
            PROFILE_TURBO_DUTY_MAX,
        )
        _require_int(
            self.burst_count,
            "Turbo burst count",
            PROFILE_TURBO_BURST_MIN,
            PROFILE_TURBO_BURST_MAX,
        )

    def to_json_object(self) -> dict[str, Any]:
        return {
            "rate_hz": self.rate_hz,
            "duty_percent": self.duty_percent,
            "burst_count": self.burst_count,
        }

    @classmethod
    def from_json_object(cls, value: Any, name: str) -> TurboSettings:
        obj = _require_object(value, ("rate_hz", "duty_percent", "burst_count"), name)
        return cls(obj["rate_hz"], obj["duty_percent"], obj["burst_count"])


@dataclass(frozen=True)
class ControllerMacro:
    trigger_mask: int
    cancel_control: int
    steps: tuple[MacroStep, ...]
    playback: int = 0
    repeat_count: int = 1

    def __post_init__(self) -> None:
        _require_int(
            self.trigger_mask,
            "macro trigger chord",
            0,
            PROFILE_LOGICAL_CONTROL_MASK,
        )
        if type(self.cancel_control) is not int or (
            self.cancel_control != PROFILE_NONE_BUTTON
            and not 0 <= self.cancel_control < len(LOGICAL_CONTROLS)
        ):
            raise ConfigManagerError("invalid macro cancel control")
        if (
            type(self.steps) is not tuple
            or len(self.steps) > PROFILE_MACRO_STEPS_PER_MACRO
            or not all(
                isinstance(step, MacroStep) and step.step_type == 0
                for step in self.steps
            )
        ):
            raise ConfigManagerError("macro must contain zero to eight state steps")
        _require_int(self.playback, "macro playback", 0, len(MACRO_PLAYBACK_MODES) - 1)
        _require_int(
            self.repeat_count,
            "macro repeat count",
            PROFILE_MACRO_REPEAT_MIN,
            PROFILE_MACRO_REPEAT_MAX,
        )
        if (
            self.trigger_mask
            and self.steps
            and self.playback != 0
            and not any(step.duration_ms for step in self.steps)
        ):
            raise ConfigManagerError("looping macros require a nonzero cycle duration")

    @classmethod
    def empty(cls) -> ControllerMacro:
        return cls(0, PROFILE_NONE_BUTTON, ())

    def to_json_object(self) -> dict[str, Any]:
        return {
            "trigger": _control_mask_to_json(self.trigger_mask),
            "cancel": _control_name(self.cancel_control),
            "steps": [step.to_json_object() for step in self.steps],
            "playback": MACRO_PLAYBACK_MODES[self.playback],
            "repeat_count": self.repeat_count,
        }

    @classmethod
    def from_json_object(
        cls, value: Any, name: str, *, schema_version: int = PROFILE_SCHEMA_VERSION
    ) -> ControllerMacro:
        fields = ["trigger", "cancel", "steps"]
        if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION:
            fields.extend(("playback", "repeat_count"))
        obj = _require_object(value, fields, name)
        steps = obj["steps"]
        if type(steps) is not list or len(steps) > PROFILE_MACRO_STEPS_PER_MACRO:
            raise ConfigManagerError(f"{name}.steps must contain zero to eight steps")
        return cls(
            _control_mask_from_json(
                obj["trigger"], f"{name}.trigger", schema_version=schema_version
            ),
            _control_index(
                obj["cancel"], f"{name}.cancel", schema_version=schema_version
            ),
            tuple(
                MacroStep.from_json_object(step, f"{name}.steps[{index}]")
                for index, step in enumerate(steps)
            ),
            (
                _require_enum(obj["playback"], MACRO_PLAYBACK_MODES, f"{name}.playback")
                if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION
                else 0
            ),
            obj["repeat_count"]
            if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION
            else 1,
        )


def _validate_swing_action(button: int, macro: int, modifier: int) -> None:
    if type(button) is not int or (
        button != PROFILE_NONE_BUTTON and not 0 <= button < len(LOGICAL_BUTTONS)
    ):
        raise ConfigManagerError("invalid swing output button")
    if type(macro) is not int or (
        macro != PROFILE_NONE_BUTTON and not 0 <= macro < PROFILE_MACRO_COUNT
    ):
        raise ConfigManagerError("invalid swing macro index")
    if button != PROFILE_NONE_BUTTON and macro != PROFILE_NONE_BUTTON:
        raise ConfigManagerError(
            "swing action must select either a button or a macro, not both"
        )
    if type(modifier) is not int or (
        modifier != PROFILE_NONE_BUTTON and not 0 <= modifier < len(LOGICAL_CONTROLS)
    ):
        raise ConfigManagerError("invalid swing modifier")


def _swing_macro_index(value: Any, name: str) -> int:
    if value is None:
        return PROFILE_NONE_BUTTON
    return _require_int(value, name, 1, PROFILE_MACRO_COUNT) - 1


@dataclass(frozen=True)
class ProfileSwing:
    button: int = PROFILE_NONE_BUTTON
    sensitivity: int = 1
    modifier: int = PROFILE_NONE_BUTTON
    macro: int = PROFILE_NONE_BUTTON

    def __post_init__(self) -> None:
        _validate_swing_action(self.button, self.macro, self.modifier)
        _require_int(
            self.sensitivity, "swing sensitivity", 0, len(SWING_SENSITIVITIES) - 1
        )

    def to_json_object(self) -> dict[str, Any]:
        return {
            "button": _button_name(self.button),
            "sensitivity": SWING_SENSITIVITIES[self.sensitivity],
            "modifier": _control_name(self.modifier),
            "macro": None if self.macro == PROFILE_NONE_BUTTON else self.macro + 1,
        }

    @classmethod
    def from_json_object(
        cls,
        value: Any,
        name: str = "profile.swing",
        *,
        schema_version: int = PROFILE_SCHEMA_VERSION,
    ) -> ProfileSwing:
        fields = ["button", "sensitivity", "modifier"]
        if schema_version >= PROFILE_SCHEMA_VERSION:
            fields.append("macro")
        obj = _require_object(value, fields, name)
        return cls(
            _button_index(obj["button"], f"{name}.button"),
            _require_enum(
                obj["sensitivity"], SWING_SENSITIVITIES, f"{name}.sensitivity"
            ),
            _control_index(
                obj["modifier"], f"{name}.modifier", schema_version=schema_version
            ),
            _swing_macro_index(obj["macro"], f"{name}.macro")
            if schema_version >= PROFILE_SCHEMA_VERSION
            else PROFILE_NONE_BUTTON,
        )


@dataclass(frozen=True)
class ProfileCombinedSwing:
    button: int = PROFILE_NONE_BUTTON
    macro: int = PROFILE_NONE_BUTTON
    modifier: int = PROFILE_NONE_BUTTON

    def __post_init__(self) -> None:
        _validate_swing_action(self.button, self.macro, self.modifier)

    def to_json_object(self) -> dict[str, Any]:
        return {
            "button": _button_name(self.button),
            "macro": None if self.macro == PROFILE_NONE_BUTTON else self.macro + 1,
            "modifier": _control_name(self.modifier),
        }

    @classmethod
    def from_json_object(cls, value: Any) -> ProfileCombinedSwing:
        name = "profile.combined_swing"
        obj = _require_object(value, ("button", "macro", "modifier"), name)
        return cls(
            _button_index(obj["button"], f"{name}.button"),
            _swing_macro_index(obj["macro"], f"{name}.macro"),
            _control_index(obj["modifier"], f"{name}.modifier"),
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
    motion_toggle_chord: int
    macros: tuple[ControllerMacro, ...]
    turbo_modes: tuple[int, ...]
    shortcuts: ProfileShortcuts = ProfileShortcuts()
    shift: ProfileShift = ProfileShift()
    turbo_defaults: TurboSettings = TurboSettings()
    turbo_overrides: tuple[TurboSettings | None, ...] = (None,) * len(LOGICAL_BUTTONS)
    extra_button_map: tuple[int, ...] = (PROFILE_NONE_BUTTON,) * len(EXTRA_BUTTONS)
    swing: ProfileSwing = ProfileSwing()
    nunchuk_swing: ProfileSwing = ProfileSwing()
    combined_swing: ProfileCombinedSwing = ProfileCombinedSwing()
    combination_window_ms: int = PROFILE_COMBINATION_WINDOW_DEFAULT

    def __post_init__(self) -> None:
        if type(self.button_map) is not tuple or len(self.button_map) != len(
            LOGICAL_BUTTONS
        ):
            raise ConfigManagerError("button map must contain 16 logical mappings")
        if type(self.extra_button_map) is not tuple or len(
            self.extra_button_map
        ) != len(EXTRA_BUTTONS):
            raise ConfigManagerError("extra button map must contain seven mappings")
        for mapping in (*self.button_map, *self.extra_button_map):
            if type(mapping) is not int or (
                mapping != PROFILE_NONE_BUTTON
                and not 0 <= mapping < len(OUTPUT_CONTROLS)
            ):
                raise ConfigManagerError("invalid logical control mapping")
        if not isinstance(self.left_stick, StickConfig) or not isinstance(
            self.right_stick, StickConfig
        ):
            raise ConfigManagerError("profile sticks must be StickConfig values")
        if not isinstance(self.left_trigger, TriggerConfig) or not isinstance(
            self.right_trigger, TriggerConfig
        ):
            raise ConfigManagerError("profile triggers must be TriggerConfig values")
        routed_triggers = [
            trigger.output
            for trigger in (self.left_trigger, self.right_trigger)
            if len(LOGICAL_BUTTONS) <= trigger.output < len(OUTPUT_CONTROLS)
        ]
        if len(routed_triggers) != len(set(routed_triggers)):
            raise ConfigManagerError(
                "left and right trigger cannot target the same analog trigger"
            )
        _require_int(self.weak_rumble_scale, "weak rumble scale", 0, 0xFF)
        _require_int(self.strong_rumble_scale, "strong rumble scale", 0, 0xFF)
        _require_int(
            self.confirmation_policy,
            "confirmation policy",
            0,
            len(RUMBLE_POLICIES) - 1,
        )
        _require_int(
            self.switching_chord,
            "switching chord",
            0,
            PROFILE_LOGICAL_CONTROL_MASK,
        )
        _require_int(
            self.motion_toggle_chord,
            "motion toggle chord",
            0,
            PROFILE_LOGICAL_CONTROL_MASK,
        )
        if (
            type(self.macros) is not tuple
            or len(self.macros) != PROFILE_MACRO_COUNT
            or not all(isinstance(macro, ControllerMacro) for macro in self.macros)
        ):
            raise ConfigManagerError("profile must contain four macros")
        total_steps = sum(len(macro.steps) for macro in self.macros)
        if total_steps > PROFILE_MACRO_STEP_CAPACITY:
            raise ConfigManagerError(
                "profile macros exceed the sixteen-step shared pool"
            )
        encoded_size = sum(
            len(step.to_sparse_bytes()) for macro in self.macros for step in macro.steps
        )
        if encoded_size > PROFILE_MACRO_STREAM_SIZE:
            raise ConfigManagerError("profile macros exceed the 136-byte sparse stream")
        triggers = [
            macro.trigger_mask for macro in self.macros if macro.trigger_mask != 0
        ]
        if len(triggers) != len(set(triggers)):
            raise ConfigManagerError("macro trigger chords must be unique")
        if type(self.turbo_modes) is not tuple or len(self.turbo_modes) != len(
            LOGICAL_BUTTONS
        ):
            raise ConfigManagerError("Turbo modes must contain 16 entries")
        for mode in self.turbo_modes:
            _require_int(mode, "Turbo mode", 0, len(TURBO_MODES) - 1)
        if not isinstance(self.shortcuts, ProfileShortcuts):
            raise ConfigManagerError("profile shortcuts must be ProfileShortcuts")
        if not isinstance(self.shift, ProfileShift):
            raise ConfigManagerError("profile Shift must be ProfileShift")
        for name, gesture_type in (
            ("swing", ProfileSwing),
            ("nunchuk_swing", ProfileSwing),
            ("combined_swing", ProfileCombinedSwing),
        ):
            gesture = getattr(self, name)
            if not isinstance(gesture, gesture_type):
                raise ConfigManagerError(
                    f"profile {name} must be {gesture_type.__name__}"
                )
            if gesture.macro != PROFILE_NONE_BUTTON:
                target = self.macros[gesture.macro]
                if not target.steps or not any(
                    step.duration_ms for step in target.steps
                ):
                    raise ConfigManagerError(
                        f"profile.{name} macro {gesture.macro + 1} must contain "
                        "at least one step and a positive total duration"
                    )
        _require_int(
            self.combination_window_ms,
            "profile.combination_window_ms",
            PROFILE_COMBINATION_WINDOW_MIN,
            PROFILE_COMBINATION_WINDOW_MAX,
        )
        if not isinstance(self.turbo_defaults, TurboSettings):
            raise ConfigManagerError("Turbo defaults must be TurboSettings")
        if (
            type(self.turbo_overrides) is not tuple
            or len(self.turbo_overrides) != len(LOGICAL_BUTTONS)
            or not all(
                settings is None or isinstance(settings, TurboSettings)
                for settings in self.turbo_overrides
            )
        ):
            raise ConfigManagerError(
                "Turbo overrides must contain 16 settings or null entries"
            )

    @classmethod
    def default(cls) -> ControllerProfile:
        stick = StickConfig(0, 0, 0, 0x7FFF, 256, False, False)
        left_trigger = TriggerConfig(
            0,
            0xFFFF,
            256,
            PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            LOGICAL_CONTROLS.index("left_trigger"),
        )
        right_trigger = TriggerConfig(
            0,
            0xFFFF,
            256,
            PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            LOGICAL_CONTROLS.index("right_trigger"),
        )
        return cls(
            button_map=tuple(range(len(LOGICAL_BUTTONS))),
            left_stick=stick,
            right_stick=stick,
            left_trigger=left_trigger,
            right_trigger=right_trigger,
            weak_rumble_scale=0xFF,
            strong_rumble_scale=0xFF,
            confirmation_policy=3,
            switching_chord=0,
            motion_toggle_chord=0,
            macros=tuple(ControllerMacro.empty() for _ in range(PROFILE_MACRO_COUNT)),
            turbo_modes=(0,) * len(LOGICAL_BUTTONS),
        )

    @classmethod
    def from_bytes(cls, payload: bytes) -> ControllerProfile:
        payload = bytes(payload)
        if len(payload) not in (PROFILE_LEGACY_SIZE, PROFILE_SIZE):
            raise ConfigManagerError("invalid profile size")
        version, size = struct.unpack_from("<HH", payload)
        expected_size = (
            PROFILE_SIZE
            if version >= PROFILE_EXPANDED_SCHEMA_VERSION
            else PROFILE_LEGACY_SIZE
        )
        if (
            version < PROFILE_LEGACY_SCHEMA_VERSION
            or version > PROFILE_SCHEMA_VERSION
            or size != expected_size
            or len(payload) != expected_size
        ):
            raise ConfigManagerError("unsupported profile schema")
        has_control_mapping = version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION
        has_action_controls = version >= PROFILE_ACTION_CONTROL_SCHEMA_VERSION
        sparse_macros = version >= PROFILE_SPARSE_MACRO_SCHEMA_VERSION
        has_extra_buttons = version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION
        has_swing = version >= PROFILE_SWING_SCHEMA_VERSION
        has_combined_swing = version >= PROFILE_SCHEMA_VERSION
        control_count = (
            len(LOGICAL_CONTROLS) if has_extra_buttons else len(OUTPUT_CONTROLS)
        )
        if sparse_macros:
            if payload[75] & 0xCC:
                raise ConfigManagerError("profile action flags are invalid")
        elif has_action_controls:
            if payload[75] & 0xC0:
                raise ConfigManagerError("profile action flags are invalid")
            if payload[252:] != bytes(4):
                raise ConfigManagerError("profile reserved fields must be zero")
        elif payload[75] != 0 or payload[252:] != bytes(4):
            raise ConfigManagerError("legacy profile reserved fields must be zero")
        if not has_control_mapping and (
            payload[81] != 0 or payload[98:100] != b"\x00\x00"
        ):
            raise ConfigManagerError("legacy profile reserved fields must be zero")

        left_trigger = TriggerConfig.from_bytes(
            payload[52:62], schema_version=version, source_index=0
        )
        right_trigger = TriggerConfig.from_bytes(
            payload[62:72], schema_version=version, source_index=1
        )
        if version == PROFILE_LEGACY_SCHEMA_VERSION:
            left_trigger = _migrate_legacy_trigger_threshold(left_trigger)
            right_trigger = _migrate_legacy_trigger_threshold(right_trigger)

        if sparse_macros:
            switching_chord = struct.unpack_from("<H", payload, 76)[0] | (
                (payload[75] & 0x03) << 16
            )
            motion_toggle_chord = struct.unpack_from("<H", payload, 78)[0] | (
                ((payload[75] >> 4) & 0x03) << 16
            )
            if has_extra_buttons:
                if any(value & 0x80 for value in payload[358:364]):
                    raise ConfigManagerError("invalid extra control mask")
                switching_chord |= payload[362] << 18
                motion_toggle_chord |= payload[363] << 18
            turbo_modes = tuple(payload[80:96])
            macros: list[ControllerMacro] = []
            stream_offset = 0
            total_steps = 0
            for macro_index in range(PROFILE_MACRO_COUNT):
                offset = 96 + macro_index * 6
                descriptor = payload[offset : offset + 6]
                if descriptor[2] & 0x80 or descriptor[3] != stream_offset:
                    raise ConfigManagerError("invalid sparse macro descriptor")
                trigger_mask = struct.unpack_from("<H", descriptor)[0] | (
                    (descriptor[2] & 0x03) << 16
                )
                if has_extra_buttons:
                    trigger_mask |= payload[358 + macro_index] << 18
                cancel = (descriptor[2] >> 2) & 0x1F
                if cancel >= control_count and cancel != 0x1F:
                    raise ConfigManagerError("invalid sparse macro cancel control")
                step_count = descriptor[4]
                encoded_size = descriptor[5]
                if (
                    step_count > PROFILE_MACRO_STEPS_PER_MACRO
                    or total_steps + step_count > PROFILE_MACRO_STEP_CAPACITY
                    or stream_offset + encoded_size > PROFILE_MACRO_STREAM_SIZE
                ):
                    raise ConfigManagerError("invalid sparse macro bounds")
                consumed = 0
                steps: list[MacroStep] = []
                for step_index in range(step_count):
                    step, step_size = MacroStep.from_sparse_bytes(
                        payload[
                            120 + stream_offset + consumed : 120
                            + stream_offset
                            + encoded_size
                        ],
                        f"profile.macros[{macro_index}].steps[{step_index}]",
                    )
                    steps.append(step)
                    consumed += step_size
                if consumed != encoded_size:
                    raise ConfigManagerError("invalid sparse macro size")
                macros.append(
                    ControllerMacro(
                        trigger_mask,
                        PROFILE_NONE_BUTTON if cancel == 0x1F else cancel,
                        tuple(steps),
                        payload[336 + macro_index * 2]
                        if version >= PROFILE_EXPANDED_SCHEMA_VERSION
                        else 0,
                        payload[337 + macro_index * 2]
                        if version >= PROFILE_EXPANDED_SCHEMA_VERSION
                        else 1,
                    )
                )
                stream_offset += consumed
                total_steps += step_count
            if payload[120 + stream_offset : PROFILE_LEGACY_SIZE] != bytes(
                PROFILE_MACRO_STREAM_SIZE - stream_offset
            ):
                raise ConfigManagerError("nonzero sparse macro padding")
        else:
            switching_chord = struct.unpack_from("<H", payload, 76)[0]
            motion_toggle_chord = (
                struct.unpack_from("<H", payload, 98)[0] if has_control_mapping else 0
            )
            if has_control_mapping:
                trigger_mask = struct.unpack_from("<H", payload, 78)[0]
                cancel_control = payload[81]
            else:
                legacy_trigger = payload[78]
                if legacy_trigger != PROFILE_NONE_BUTTON and not (
                    0 <= legacy_trigger < len(LOGICAL_BUTTONS)
                ):
                    raise ConfigManagerError("invalid legacy macro trigger")
                trigger_mask = (
                    0 if legacy_trigger == PROFILE_NONE_BUTTON else 1 << legacy_trigger
                )
                cancel_control = payload[79]
            if (
                cancel_control != PROFILE_NONE_BUTTON
                and cancel_control >= control_count
            ):
                raise ConfigManagerError("invalid legacy macro cancel control")
            if has_action_controls:
                switching_chord |= (payload[75] & 0x03) << 16
                trigger_mask |= ((payload[75] >> 2) & 0x03) << 16
                motion_toggle_chord |= ((payload[75] >> 4) & 0x03) << 16
            legacy_count = payload[80]
            if not 1 <= legacy_count <= PROFILE_LEGACY_MACRO_STEP_CAPACITY:
                raise ConfigManagerError("invalid legacy macro step count")
            legacy_steps = tuple(
                MacroStep.from_bytes(
                    payload[
                        100 + index * PROFILE_MACRO_STEP_SIZE : 100
                        + (index + 1) * PROFILE_MACRO_STEP_SIZE
                    ]
                )
                for index in range(PROFILE_LEGACY_MACRO_STEP_CAPACITY)
            )
            if any(
                step != MacroStep.end() for step in legacy_steps[legacy_count - 1 :]
            ):
                raise ConfigManagerError("invalid legacy macro end padding")
            state_steps = legacy_steps[: legacy_count - 1]
            if any(step.step_type != 0 for step in state_steps):
                raise ConfigManagerError("invalid legacy macro state step")
            macros = [
                ControllerMacro(trigger_mask, cancel_control, state_steps),
                *(ControllerMacro.empty() for _ in range(PROFILE_MACRO_COUNT - 1)),
            ]
            turbo_modes = tuple(payload[82:98])
        shortcuts = ProfileShortcuts()
        shift = ProfileShift()
        turbo_defaults = TurboSettings()
        turbo_overrides: list[TurboSettings | None] = [None] * len(LOGICAL_BUTTONS)
        if version >= PROFILE_EXPANDED_SCHEMA_VERSION:
            if any(
                value != PROFILE_NONE_BUTTON and value >= control_count
                for value in (payload[256], payload[266])
            ):
                raise ConfigManagerError("invalid profile modifier")
            shortcuts = ProfileShortcuts(payload[256], tuple(payload[257:265]))
            shift = ProfileShift(
                payload[265],
                payload[266],
                tuple(payload[267:283]),
                tuple(payload[351:358])
                if has_extra_buttons
                else (PROFILE_NONE_BUTTON,) * len(EXTRA_BUTTONS),
            )
            turbo_defaults = TurboSettings(*payload[283:286])
            override_mask = struct.unpack_from("<H", payload, 286)[0]
            settings_offset = 288
            for button in range(len(LOGICAL_BUTTONS)):
                if override_mask & (1 << button):
                    turbo_overrides[button] = TurboSettings(
                        *payload[settings_offset : settings_offset + 3]
                    )
                    settings_offset += 3
            if payload[settings_offset:336] != bytes(336 - settings_offset):
                raise ConfigManagerError("nonzero Turbo override padding")
            reserved_offset = (
                376
                if has_combined_swing
                else 367
                if has_swing
                else 364
                if has_extra_buttons
                else 344
            )
            if any(payload[reserved_offset:]):
                raise ConfigManagerError("profile reserved fields must be zero")
        elif any(mode > 2 for mode in turbo_modes):
            raise ConfigManagerError("invalid legacy Turbo mode")

        return cls(
            button_map=tuple(payload[4:20]),
            left_stick=StickConfig.from_bytes(payload[20:36]),
            right_stick=StickConfig.from_bytes(payload[36:52]),
            left_trigger=left_trigger,
            right_trigger=right_trigger,
            weak_rumble_scale=payload[72],
            strong_rumble_scale=payload[73],
            confirmation_policy=payload[74],
            switching_chord=switching_chord,
            motion_toggle_chord=motion_toggle_chord,
            macros=tuple(macros),
            turbo_modes=turbo_modes,
            shortcuts=shortcuts,
            shift=shift,
            turbo_defaults=turbo_defaults,
            turbo_overrides=tuple(turbo_overrides),
            extra_button_map=tuple(payload[344:351])
            if has_extra_buttons
            else (PROFILE_NONE_BUTTON,) * len(EXTRA_BUTTONS),
            swing=(
                ProfileSwing(*payload[364:368])
                if has_combined_swing
                else ProfileSwing(*payload[364:367])
                if has_swing
                else ProfileSwing()
            ),
            nunchuk_swing=ProfileSwing(*payload[368:372])
            if has_combined_swing
            else ProfileSwing(),
            combined_swing=ProfileCombinedSwing(*payload[372:375])
            if has_combined_swing
            else ProfileCombinedSwing(),
            combination_window_ms=payload[375]
            if has_combined_swing
            else PROFILE_COMBINATION_WINDOW_DEFAULT,
        )

    def to_bytes(self) -> bytes:
        payload = bytearray(PROFILE_SIZE)
        struct.pack_into("<HH", payload, 0, PROFILE_SCHEMA_VERSION, PROFILE_SIZE)
        payload[4:20] = bytes(self.button_map)
        payload[20:36] = self.left_stick.to_bytes()
        payload[36:52] = self.right_stick.to_bytes()
        payload[52:62] = self.left_trigger.to_bytes()
        payload[62:72] = self.right_trigger.to_bytes()
        payload[72:75] = bytes(
            (
                self.weak_rumble_scale,
                self.strong_rumble_scale,
                self.confirmation_policy,
            )
        )
        payload[75] = ((self.switching_chord >> 16) & 0x03) | (
            ((self.motion_toggle_chord >> 16) & 0x03) << 4
        )
        struct.pack_into("<H", payload, 76, self.switching_chord & 0xFFFF)
        struct.pack_into("<H", payload, 78, self.motion_toggle_chord & 0xFFFF)
        payload[80:96] = bytes(self.turbo_modes)

        stream = bytearray()
        for macro_index, macro in enumerate(self.macros):
            encoded_steps = b"".join(step.to_sparse_bytes() for step in macro.steps)
            descriptor_offset = 96 + macro_index * 6
            struct.pack_into(
                "<H", payload, descriptor_offset, macro.trigger_mask & 0xFFFF
            )
            cancel = (
                0x1F
                if macro.cancel_control == PROFILE_NONE_BUTTON
                else macro.cancel_control
            )
            payload[descriptor_offset + 2] = ((macro.trigger_mask >> 16) & 0x03) | (
                cancel << 2
            )
            payload[descriptor_offset + 3] = len(stream)
            payload[descriptor_offset + 4] = len(macro.steps)
            payload[descriptor_offset + 5] = len(encoded_steps)
            stream.extend(encoded_steps)
            payload[336 + macro_index * 2] = macro.playback
            payload[337 + macro_index * 2] = macro.repeat_count
            payload[358 + macro_index] = (macro.trigger_mask >> 18) & 0x7F
        if len(stream) > PROFILE_MACRO_STREAM_SIZE:
            raise ConfigManagerError("profile macros exceed the 136-byte sparse stream")
        payload[120 : 120 + len(stream)] = stream
        payload[256] = self.shortcuts.modifier
        payload[257:265] = bytes(self.shortcuts.profiles)
        payload[265] = self.shift.mode
        payload[266] = self.shift.modifier
        payload[267:283] = bytes(self.shift.button_map)
        payload[283:286] = bytes(
            (
                self.turbo_defaults.rate_hz,
                self.turbo_defaults.duty_percent,
                self.turbo_defaults.burst_count,
            )
        )
        override_mask = 0
        settings_offset = 288
        for button, settings in enumerate(self.turbo_overrides):
            if settings is not None:
                override_mask |= 1 << button
                payload[settings_offset : settings_offset + 3] = bytes(
                    (
                        settings.rate_hz,
                        settings.duty_percent,
                        settings.burst_count,
                    )
                )
                settings_offset += 3
        struct.pack_into("<H", payload, 286, override_mask)
        payload[344:351] = bytes(self.extra_button_map)
        payload[351:358] = bytes(self.shift.extra_button_map)
        payload[362] = (self.switching_chord >> 18) & 0x7F
        payload[363] = (self.motion_toggle_chord >> 18) & 0x7F
        payload[364:368] = bytes(
            (
                self.swing.button,
                self.swing.sensitivity,
                self.swing.modifier,
                self.swing.macro,
            )
        )
        payload[368:372] = bytes(
            (
                self.nunchuk_swing.button,
                self.nunchuk_swing.sensitivity,
                self.nunchuk_swing.modifier,
                self.nunchuk_swing.macro,
            )
        )
        payload[372:376] = bytes(
            (
                self.combined_swing.button,
                self.combined_swing.macro,
                self.combined_swing.modifier,
                self.combination_window_ms,
            )
        )
        return bytes(payload)

    def to_json_object(self) -> dict[str, Any]:
        return {
            "schema_version": PROFILE_SCHEMA_VERSION,
            "size": PROFILE_SIZE,
            "button_map": {
                name: _control_name(self.button_map[index])
                for index, name in enumerate(LOGICAL_BUTTONS)
            },
            "extra_button_map": {
                name: _control_name(self.extra_button_map[index])
                for index, name in enumerate(EXTRA_BUTTONS)
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
                "confirmation_policy": RUMBLE_POLICIES[self.confirmation_policy],
            },
            "switching_chord": _control_mask_to_json(self.switching_chord),
            "motion_toggle_chord": _control_mask_to_json(self.motion_toggle_chord),
            "macros": [macro.to_json_object() for macro in self.macros],
            "turbo": {
                name: TURBO_MODES[self.turbo_modes[index]]
                for index, name in enumerate(LOGICAL_BUTTONS)
            },
            "shortcuts": self.shortcuts.to_json_object(),
            "shift": self.shift.to_json_object(),
            "swing": self.swing.to_json_object(),
            "nunchuk_swing": self.nunchuk_swing.to_json_object(),
            "combined_swing": self.combined_swing.to_json_object(),
            "combination_window_ms": self.combination_window_ms,
            "turbo_settings": {
                "defaults": self.turbo_defaults.to_json_object(),
                "overrides": {
                    LOGICAL_BUTTONS[index]: settings.to_json_object()
                    for index, settings in enumerate(self.turbo_overrides)
                    if settings is not None
                },
            },
        }

    def to_json(self) -> str:
        return json.dumps(self.to_json_object(), indent=2) + "\n"

    @classmethod
    def from_json_object(cls, value: Any) -> ControllerProfile:
        if type(value) is not dict:
            raise ConfigManagerError("profile must be a JSON object")
        schema_version = _require_int(
            value.get("schema_version"),
            "profile.schema_version",
            0,
            0xFFFF,
        )
        if (
            schema_version < PROFILE_LEGACY_SCHEMA_VERSION
            or schema_version > PROFILE_SCHEMA_VERSION
        ):
            raise ConfigManagerError("unsupported profile schema")
        fields = [
            "schema_version",
            "size",
            "button_map",
            "sticks",
            "triggers",
            "rumble",
            "switching_chord",
            "turbo",
        ]
        if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION:
            fields.append("motion_toggle_chord")
        fields.append(
            "macros"
            if schema_version >= PROFILE_SPARSE_MACRO_SCHEMA_VERSION
            else "macro"
        )
        if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION:
            fields.extend(("shortcuts", "shift", "turbo_settings"))
        if schema_version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION:
            fields.append("extra_button_map")
        if schema_version >= PROFILE_SWING_SCHEMA_VERSION:
            fields.append("swing")
        if schema_version >= PROFILE_SCHEMA_VERSION:
            fields.extend(("nunchuk_swing", "combined_swing", "combination_window_ms"))
        obj = _require_object(value, fields, "profile")
        expected_size = (
            PROFILE_SIZE
            if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION
            else PROFILE_LEGACY_SIZE
        )
        if _require_int(obj["size"], "profile.size", 0, 0xFFFF) != expected_size:
            raise ConfigManagerError("unsupported profile schema")
        button_map = _require_object(
            obj["button_map"], LOGICAL_BUTTONS, "profile.button_map"
        )
        extras = (
            _require_object(
                obj["extra_button_map"], EXTRA_BUTTONS, "profile.extra_button_map"
            )
            if schema_version >= PROFILE_EXTRA_CONTROL_SCHEMA_VERSION
            else dict.fromkeys(EXTRA_BUTTONS)
        )
        sticks = _require_object(obj["sticks"], ("left", "right"), "profile.sticks")
        triggers = _require_object(
            obj["triggers"], ("left", "right"), "profile.triggers"
        )
        rumble = _require_object(
            obj["rumble"],
            ("weak_scale", "strong_scale", "confirmation_policy"),
            "profile.rumble",
        )
        turbo = _require_object(obj["turbo"], LOGICAL_BUTTONS, "profile.turbo")
        left_trigger = TriggerConfig.from_json_object(
            triggers["left"],
            "profile.triggers.left",
            schema_version=schema_version,
            source_index=0,
        )
        right_trigger = TriggerConfig.from_json_object(
            triggers["right"],
            "profile.triggers.right",
            schema_version=schema_version,
            source_index=1,
        )
        if schema_version == PROFILE_LEGACY_SCHEMA_VERSION:
            left_trigger = _migrate_legacy_trigger_threshold(left_trigger)
            right_trigger = _migrate_legacy_trigger_threshold(right_trigger)

        mask_parser = (
            (
                lambda value, name: _control_mask_from_json(
                    value, name, schema_version=schema_version
                )
            )
            if schema_version >= PROFILE_ACTION_CONTROL_SCHEMA_VERSION
            else _button_mask_from_json
        )
        switching_chord = mask_parser(obj["switching_chord"], "profile.switching_chord")
        motion_toggle_chord = (
            mask_parser(
                obj["motion_toggle_chord"],
                "profile.motion_toggle_chord",
            )
            if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION
            else 0
        )
        if schema_version >= PROFILE_SPARSE_MACRO_SCHEMA_VERSION:
            macro_values = obj["macros"]
            if (
                type(macro_values) is not list
                or len(macro_values) != PROFILE_MACRO_COUNT
            ):
                raise ConfigManagerError("profile.macros must contain four macros")
            macros = tuple(
                ControllerMacro.from_json_object(
                    macro, f"profile.macros[{index}]", schema_version=schema_version
                )
                for index, macro in enumerate(macro_values)
            )
        else:
            macro = _require_object(
                obj["macro"], ("trigger", "cancel", "steps"), "profile.macro"
            )
            steps = macro["steps"]
            if type(steps) is not list or not (
                1 <= len(steps) <= PROFILE_LEGACY_MACRO_STEP_CAPACITY
            ):
                raise ConfigManagerError(
                    "profile.macro.steps must contain one to eight steps"
                )
            decoded_steps = tuple(
                MacroStep.from_json_object(step, f"profile.macro.steps[{index}]")
                for index, step in enumerate(steps)
            )
            if decoded_steps[-1] != MacroStep.end() or any(
                step.step_type != 0 for step in decoded_steps[:-1]
            ):
                raise ConfigManagerError(
                    "legacy macro must end with one canonical end step"
                )
            if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION:
                trigger_mask = mask_parser(macro["trigger"], "profile.macro.trigger")
                cancel_control = (
                    _control_index(
                        macro["cancel"],
                        "profile.macro.cancel",
                        schema_version=schema_version,
                    )
                    if schema_version >= PROFILE_ACTION_CONTROL_SCHEMA_VERSION
                    else _button_index(macro["cancel"], "profile.macro.cancel")
                )
            else:
                trigger = _button_index(macro["trigger"], "profile.macro.trigger")
                trigger_mask = 0 if trigger == PROFILE_NONE_BUTTON else 1 << trigger
                cancel_control = _button_index(macro["cancel"], "profile.macro.cancel")
            macros = (
                ControllerMacro(trigger_mask, cancel_control, decoded_steps[:-1]),
                *(ControllerMacro.empty() for _ in range(PROFILE_MACRO_COUNT - 1)),
            )

        shortcuts = ProfileShortcuts()
        shift = ProfileShift()
        turbo_defaults = TurboSettings()
        turbo_overrides: list[TurboSettings | None] = [None] * len(LOGICAL_BUTTONS)
        if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION:
            shortcuts = ProfileShortcuts.from_json_object(
                obj["shortcuts"], schema_version=schema_version
            )
            shift = ProfileShift.from_json_object(
                obj["shift"], schema_version=schema_version
            )
            settings = _require_object(
                obj["turbo_settings"],
                ("defaults", "overrides"),
                "profile.turbo_settings",
            )
            turbo_defaults = TurboSettings.from_json_object(
                settings["defaults"], "profile.turbo_settings.defaults"
            )
            overrides = settings["overrides"]
            if type(overrides) is not dict or any(
                name not in LOGICAL_BUTTONS for name in overrides
            ):
                raise ConfigManagerError(
                    "Turbo overrides must map button names to settings"
                )
            for name, override in overrides.items():
                turbo_overrides[LOGICAL_BUTTONS.index(name)] = (
                    TurboSettings.from_json_object(
                        override, f"profile.turbo_settings.overrides.{name}"
                    )
                )

        return cls(
            button_map=tuple(
                (
                    _control_index
                    if schema_version >= PROFILE_CONTROL_MAPPING_SCHEMA_VERSION
                    else _button_index
                )(button_map[name], f"profile.button_map.{name}")
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
                rumble["weak_scale"], "profile.rumble.weak_scale", 0, 0xFF
            ),
            strong_rumble_scale=_require_int(
                rumble["strong_scale"], "profile.rumble.strong_scale", 0, 0xFF
            ),
            confirmation_policy=_require_enum(
                rumble["confirmation_policy"],
                RUMBLE_POLICIES,
                "profile.rumble.confirmation_policy",
            ),
            switching_chord=switching_chord,
            motion_toggle_chord=motion_toggle_chord,
            macros=macros,
            turbo_modes=tuple(
                _require_enum(
                    turbo[name],
                    TURBO_MODES
                    if schema_version >= PROFILE_EXPANDED_SCHEMA_VERSION
                    else TURBO_MODES[:3],
                    f"profile.turbo.{name}",
                )
                for name in LOGICAL_BUTTONS
            ),
            shortcuts=shortcuts,
            shift=shift,
            turbo_defaults=turbo_defaults,
            turbo_overrides=tuple(turbo_overrides),
            extra_button_map=tuple(
                _control_index(extras[name], f"profile.extra_button_map.{name}")
                for name in EXTRA_BUTTONS
            ),
            swing=(
                ProfileSwing.from_json_object(
                    obj["swing"], schema_version=schema_version
                )
                if schema_version >= PROFILE_SWING_SCHEMA_VERSION
                else ProfileSwing()
            ),
            nunchuk_swing=(
                ProfileSwing.from_json_object(
                    obj["nunchuk_swing"], "profile.nunchuk_swing"
                )
                if schema_version >= PROFILE_SCHEMA_VERSION
                else ProfileSwing()
            ),
            combined_swing=(
                ProfileCombinedSwing.from_json_object(obj["combined_swing"])
                if schema_version >= PROFILE_SCHEMA_VERSION
                else ProfileCombinedSwing()
            ),
            combination_window_ms=(
                obj["combination_window_ms"]
                if schema_version >= PROFILE_SCHEMA_VERSION
                else PROFILE_COMBINATION_WINDOW_DEFAULT
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
    return (
        struct.pack(
            "<4sBBBBHHI",
            b"SPMG",
            PROTOCOL_VERSION,
            operation,
            0,
            0,
            len(payload),
            0,
            _crc32(payload),
        )
        + payload
    )


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
        raise ConfigManagerError(f"unsupported management protocol version {version}")
    if operation != expected_operation:
        raise ConfigManagerError(f"unexpected management operation 0x{operation:02x}")
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
        STATUS_NAMES.get(envelope.status, f"unknown device status {envelope.status}")
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


def _control_out(device: UsbDevice, operation: int, payload: bytes = b"") -> None:
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
        raise ConfigManagerError("device capability flags omit required input support")
    return DeviceInfo(
        firmware_version=(
            envelope.payload[0],
            envelope.payload[1],
            envelope.payload[2],
        ),
        board=envelope.payload[3],
        active_mode=active_mode,
        capabilities=capabilities,
        maximum_configuration_size=struct.unpack_from("<H", envelope.payload, 6)[0],
    )


def read_macro_capture(
    device: UsbDevice,
    run_id: int = 0,
    first_index: int = 0,
) -> MacroCapturePage:
    _require_int(run_id, "capture run ID", 0, 0xFFFFFFFF)
    _require_int(first_index, "capture first index", 0, 128)
    _control_out(device, OP_MACRO_CAPTURE, struct.pack("<BIH", 2, run_id, first_index))
    envelope = _control_in(device, OP_MACRO_CAPTURE)
    _raise_status(envelope)
    payload = envelope.payload
    if envelope.schema_version != MACRO_CAPTURE_SCHEMA_VERSION:
        raise ConfigManagerError("unsupported macro capture schema")
    if envelope.flags or len(payload) < 32:
        raise ConfigManagerError("invalid macro capture header")
    actual_run, generation, elapsed = struct.unpack_from("<III", payload)
    slot, state, channels, count = struct.unpack_from("<4B", payload, 12)
    total, first, axis, trigger, duration, limit = struct.unpack_from(
        "<HHHHIB", payload, 16
    )
    if (
        envelope.generation != actual_run
        or (run_id and actual_run != run_id)
        or state >= len(MACRO_CAPTURE_STATES)
        or slot >= 4
        or channels == 0
        or channels > 31
        or count > 32
        or total > limit
        or not 1 <= limit <= 128
        or first != first_index
        or first + count > total
        or len(payload) != 32 + count * 20
        or not 1 <= axis <= 32767
        or not 1 <= trigger <= 65535
        or not 1 <= duration <= 80000
        or elapsed > duration * 1000
        or any(payload[29:32])
    ):
        raise ConfigManagerError("invalid macro capture bounds or run identity")
    events = []
    previous = -1
    for index in range(count):
        offset = 32 + index * 20
        event = MacroCaptureEvent(*struct.unpack_from("<IHhhhhHH", payload, offset))
        if (
            event.at_us <= previous
            or event.at_us > elapsed
            or any(payload[offset + 18 : offset + 20])
        ):
            raise ConfigManagerError("invalid macro capture event timeline")
        previous = event.at_us
        events.append(event)
    return MacroCapturePage(
        actual_run,
        generation,
        elapsed,
        slot,
        state,
        channels,
        total,
        first,
        axis,
        trigger,
        duration,
        limit,
        tuple(events),
    )


def start_macro_capture(
    device: UsbDevice,
    slot: int,
    connection_generation: int,
    *,
    channels: int = 1,
    max_events: int = 8,
    axis_quantum: int = 512,
    trigger_quantum: int = 1024,
    max_duration_ms: int = 10000,
) -> MacroCapturePage:
    for value, name, low, high in (
        (slot, "capture slot", 0, 3),
        (connection_generation, "capture generation", 0, 0xFFFFFFFF),
        (channels, "capture channels", 1, 31),
        (max_events, "capture capacity", 1, 128),
        (axis_quantum, "capture axis quantum", 1, 32767),
        (trigger_quantum, "capture trigger quantum", 1, 65535),
        (max_duration_ms, "capture duration", 1, 80000),
    ):
        _require_int(value, name, low, high)
    _control_out(
        device,
        OP_MACRO_CAPTURE,
        struct.pack(
            "<BBIBBHHI",
            1,
            slot,
            connection_generation,
            channels,
            max_events,
            axis_quantum,
            trigger_quantum,
            max_duration_ms,
        ),
    )
    page = read_macro_capture(device)
    if page.slot != slot or page.connection_generation != connection_generation:
        raise ConfigManagerError("capture controller changed during start")
    return page


def stop_macro_capture(device: UsbDevice, run_id: int) -> MacroCapturePage:
    _require_int(run_id, "capture run ID", 1, 0xFFFFFFFF)
    _control_out(device, OP_MACRO_CAPTURE, struct.pack("<BI", 0, run_id))
    return read_macro_capture(device, run_id)


def collect_macro_capture(device: UsbDevice, run_id: int) -> MacroCapturePage:
    page = read_macro_capture(device, run_id)
    if page.state_name in ("idle", "recording"):
        raise ConfigManagerError("stop recording before collecting macro steps")
    events = list(page.events)
    while len(events) < page.total_events:
        following = read_macro_capture(device, run_id, len(events))
        if (
            following.run_id != page.run_id
            or following.connection_generation != page.connection_generation
            or following.elapsed_us != page.elapsed_us
            or following.total_events != page.total_events
            or following.state != page.state
            or following.channels != page.channels
            or following.slot != page.slot
            or following.axis_quantum != page.axis_quantum
            or following.trigger_quantum != page.trigger_quantum
            or following.max_duration_ms != page.max_duration_ms
            or following.max_events != page.max_events
            or not following.events
        ):
            raise ConfigManagerError("macro capture changed while reading pages")
        if events and following.events[0].at_us <= events[-1].at_us:
            raise ConfigManagerError("macro capture pages overlap")
        events.extend(following.events)
    return MacroCapturePage(
        page.run_id,
        page.connection_generation,
        page.elapsed_us,
        page.slot,
        page.state,
        page.channels,
        page.total_events,
        0,
        page.axis_quantum,
        page.trigger_quantum,
        page.max_duration_ms,
        page.max_events,
        tuple(events),
    )


def capture_macro_steps(page: MacroCapturePage) -> tuple[MacroStep, ...]:
    if (
        page.state_name in ("idle", "recording")
        or page.first_index != 0
        or len(page.events) != page.total_events
        or not page.events
        or page.events[0].at_us != 0
    ):
        raise ConfigManagerError("a complete stopped capture is required")
    steps = []
    for index, event in enumerate(page.events):
        end_us = (
            page.events[index + 1].at_us
            if index + 1 < len(page.events)
            else page.elapsed_us
        )
        duration = (end_us + 500) // 1000 - (event.at_us + 500) // 1000
        steps.append(
            MacroStep(
                0,
                page.channels,
                duration,
                event.buttons,
                event.left_x,
                event.left_y,
                event.right_x,
                event.right_y,
                event.left_trigger,
                event.right_trigger,
            )
        )
    return tuple(steps)


def read_runtime_diagnostics(device: UsbDevice) -> RuntimeDiagnostics:
    envelope = _control_in(device, OP_RUNTIME_DIAGNOSTICS)
    _raise_status(envelope)
    if len(envelope.payload) not in (32, 40):
        raise ConfigManagerError("invalid runtime-diagnostics payload")
    counters = struct.unpack_from("<7I", envelope.payload)
    ingress_drops, output_drops = (
        struct.unpack_from("<2I", envelope.payload, 32)
        if len(envelope.payload) == 40
        else (None, None)
    )
    return RuntimeDiagnostics(
        *counters,
        active_slots=envelope.payload[28],
        rumble_capable_slots=envelope.payload[29],
        feedback_pending_slots=envelope.payload[30],
        rumble_pending_slots=envelope.payload[31],
        switch2_ingress_drops=ingress_drops,
        switch2_output_drops=output_drops,
    )


def parse_native_switch_rumble(
    envelope: Envelope,
) -> tuple[NativeSwitchRumbleSlot, ...]:
    _raise_status(envelope)
    if (
        envelope.schema_version != NATIVE_SWITCH_RUMBLE_SCHEMA_VERSION
        or len(envelope.payload)
        != NATIVE_SWITCH_RUMBLE_ROW_SIZE * NATIVE_SWITCH_RUMBLE_SLOT_COUNT
    ):
        raise ConfigManagerError("unsupported native Nintendo rumble diagnostics")
    slots = []
    for slot in range(NATIVE_SWITCH_RUMBLE_SLOT_COUNT):
        values = struct.unpack_from(
            "<4B19I", envelope.payload, slot * NATIVE_SWITCH_RUMBLE_ROW_SIZE
        )
        if values[0] != slot or values[4] & ~0x1F:
            raise ConfigManagerError("invalid native Nintendo rumble diagnostic row")
        slots.append(NativeSwitchRumbleSlot(*values))
    return tuple(slots)


def read_native_switch_rumble(device: UsbDevice) -> tuple[NativeSwitchRumbleSlot, ...]:
    return parse_native_switch_rumble(_control_in(device, OP_NATIVE_SWITCH_RUMBLE))


def _print_native_switch_rumble_status(
    slots: tuple[NativeSwitchRumbleSlot, ...], *, json_output: bool
) -> None:
    if json_output:
        print(
            json.dumps(
                {
                    "schema_version": NATIVE_SWITCH_RUMBLE_SCHEMA_VERSION,
                    "latency_note": NATIVE_SWITCH_RUMBLE_LATENCY_NOTE,
                    "slots": [slot.to_json_object() for slot in slots],
                }
            )
        )
        return
    print(NATIVE_SWITCH_RUMBLE_LATENCY_NOTE)
    for slot in slots:
        values = slot.to_json_object()
        print(
            f"Slot {slot.slot}: parser type {slot.parser_type}, firmware bytes "
            f"{slot.firmware_high:02x}:{slot.firmware_low:02x}, generation {slot.generation}"
        )
        print(
            "  "
            + " ".join(
                f"{name}={str(values[name]).lower()}"
                for name in ("connected", "approved", "active", "mono", "feedback")
            )
        )
        print(
            f"  Commands: received={slot.received_commands} completed={slot.completed_commands} "
            f"raw={slot.raw_commands} quantized={slot.quantized_commands} "
            f"dropped={slot.dropped_commands} coalesced={slot.coalesced_commands}"
        )
        print(
            f"  Reports: submitted={slot.submitted_reports} "
            f"congested={slot.congested_attempts} resynchronizations={slot.resynchronizations} "
            f"queue_depth={slot.queue_depth}"
        )
        print(
            f"  Latency upper bounds (us): p50={slot.p50_upper_us} "
            f"p95={slot.p95_upper_us} p99={slot.p99_upper_us}; "
            f"max={slot.max_latency_us}; max_encode={slot.max_encode_us}"
        )
        print(f"  Last wire bytes: {values['last_wire_hex']}")


def parse_haptics_experiment(envelope: Envelope) -> HapticsExperimentDiagnostics:
    _raise_status(envelope)
    if envelope.schema_version != HAPTICS_EXPERIMENT_SCHEMA_VERSION:
        raise ConfigManagerError(
            f"unsupported haptics experiment schema {envelope.schema_version}; "
            "update the host tool and experiment firmware together"
        )
    if len(envelope.payload) != HAPTICS_EXPERIMENT_SIZE:
        raise ConfigManagerError("invalid haptics experiment payload size")
    counters = struct.unpack_from("<17I", envelope.payload)
    state, slot, last_error, reserved = struct.unpack_from("<4B", envelope.payload, 68)
    mode = envelope.payload[72]
    packet_frames = envelope.payload[73]
    if packet_frames not in (32, 64):
        raise ConfigManagerError("invalid haptics packet size")
    host_updates, dropped_updates = struct.unpack_from("<2I", envelope.payload, 76)
    if envelope.payload[74] not in (0, 1):
        raise ConfigManagerError("invalid haptics nonzero flag")
    if envelope.flags != 0 or reserved != 0 or envelope.payload[75] != 0:
        raise ConfigManagerError("invalid haptics experiment reserved flags")
    if state >= len(HAPTICS_EXPERIMENT_STATES):
        raise ConfigManagerError(f"invalid haptics experiment state {state}")
    if mode >= len(HAPTICS_EXPERIMENT_MODES):
        raise ConfigManagerError(f"invalid haptics experiment mode {mode}")
    if slot >= HAPTICS_EXPERIMENT_SLOT_COUNT and not (
        slot == 0xFF and HAPTICS_EXPERIMENT_STATES[state] in ("idle", "unsupported")
    ):
        raise ConfigManagerError(f"invalid haptics experiment slot {slot}")
    return HapticsExperimentDiagnostics(
        *counters,
        state=state,
        slot=None if slot == 0xFF else slot,
        last_error=last_error,
        mode=mode,
        host_updates=host_updates,
        dropped_updates=dropped_updates,
        packet_frames=packet_frames,
        last_packet_nonzero=bool(envelope.payload[74]),
    )


def read_haptics_experiment(device: UsbDevice) -> HapticsExperimentDiagnostics:
    try:
        envelope = _control_in(device, OP_HAPTICS_EXPERIMENT)
    except usb.core.USBError as exc:
        if exc.errno == 32 or exc.backend_error_code == -9:
            raise ConfigManagerError(
                "firmware does not support haptics experiment operation 0x40. "
                + HAPTICS_EXPERIMENT_ENABLE_HINT
            ) from exc
        raise
    return parse_haptics_experiment(envelope)


def parse_haptics_transport_probe(envelope: Envelope) -> HapticsTransportProbe:
    if envelope.status == STATUS_UNSUPPORTED_SCHEMA:
        raise ConfigManagerError(HAPTICS_TRANSPORT_PROBE_UNSUPPORTED_HINT)
    _raise_status(envelope)
    if envelope.schema_version != HAPTICS_TRANSPORT_PROBE_SCHEMA_VERSION:
        raise ConfigManagerError(
            f"unsupported haptics transport probe schema {envelope.schema_version}; "
            "update the host tool and experiment firmware together"
        )
    if len(envelope.payload) != HAPTICS_TRANSPORT_PROBE_SIZE:
        raise ConfigManagerError("invalid haptics transport probe payload size")
    if envelope.flags != 0:
        raise ConfigManagerError("invalid haptics transport probe reserved flags")
    fields = struct.unpack("<38Ii5I", envelope.payload)
    if fields[2] > 0xFFFF:
        raise ConfigManagerError("invalid haptics transport probe connection handle")
    if fields[25] not in (0, 1):
        raise ConfigManagerError("invalid haptics transport probe active boolean")
    if envelope.generation != fields[0]:
        raise ConfigManagerError("haptics transport probe envelope run ID mismatch")
    return HapticsTransportProbe(*fields[:25], bool(fields[25]), *fields[26:])


def read_haptics_transport_probe(device: UsbDevice) -> HapticsTransportProbe:
    try:
        envelope = _control_in(device, OP_HAPTICS_TRANSPORT_PROBE)
    except usb.core.USBError as exc:
        if exc.errno == 32 or exc.backend_error_code == -9:
            raise ConfigManagerError(HAPTICS_TRANSPORT_PROBE_UNSUPPORTED_HINT) from exc
        raise
    return parse_haptics_transport_probe(envelope)


def read_haptics_experiment_profile(
    device: UsbDevice,
) -> tuple[HapticsExperimentDiagnostics, HapticsTransportProbe]:
    before = read_haptics_experiment(device)
    transport = read_haptics_transport_probe(device)
    after = read_haptics_experiment(device)
    correlation = (before.run_id, before.connection_generation)
    if (
        (after.run_id, after.connection_generation) != correlation
        or (transport.run_id, transport.connection_generation) != correlation
        or (after.slot, after.mode) != (before.slot, before.mode)
    ):
        raise ConfigManagerError(
            "haptics experiment run, slot, mode, or connection generation changed or does not "
            "match the transport profile; cannot attribute measurements. "
            "Read profile again after the accepted run has started or finished."
        )
    return after, transport


def _print_haptics_experiment_profile(
    snapshot: HapticsExperimentDiagnostics,
    transport: HapticsTransportProbe,
    *,
    as_json: bool,
) -> None:
    if as_json:
        values = snapshot.to_json_object()
        values["transport"] = transport.to_json_object()
        values["host_monotonic_s"] = time.monotonic()
        print(json.dumps(values, sort_keys=True), flush=True)
        return
    _print_haptics_experiment(snapshot, as_json=False)
    print(
        f"Transport profile: run={transport.run_id}; "
        f"connection_generation={transport.connection_generation}; "
        f"handle=0x{transport.connection_handle:04x}; active={transport.active}"
    )
    for label, calls, maximum, total in (
        (
            "timer lateness",
            transport.timer_wakes,
            transport.max_timer_lateness_us,
            transport.total_timer_lateness_us,
        ),
        (
            "permission wait",
            transport.permission_callbacks,
            transport.max_permission_wait_us,
            transport.total_permission_wait_us,
        ),
        (
            "l2cap_send",
            transport.send_calls,
            transport.max_send_us,
            transport.total_send_us,
        ),
        (
            "HCI write",
            transport.write_calls,
            transport.max_write_us,
            transport.total_write_us,
        ),
        (
            "HCI read",
            transport.read_calls,
            transport.max_read_us,
            transport.total_read_us,
        ),
        (
            "data-source poll",
            transport.poll_calls,
            transport.max_poll_us,
            transport.total_poll_us,
        ),
    ):
        print(f"  {label}: calls={calls}, max_us={maximum}, total_us={total}")
    print(f"  read_packets: {transport.read_packets}")
    print(
        f"  completions: events={transport.completion_events}, "
        f"packets={transport.completed_packets}, "
        f"max_gap_us={transport.max_completion_gap_us}"
    )
    print(f"  max_poll_gap_us: {transport.max_poll_gap_us}")
    print(
        f"  observed ACL slots: max_outstanding={transport.max_outstanding_acl}, "
        f"min_free={transport.min_free_acl}"
    )
    print(f"  first_tone_send_return_us: {transport.first_tone_send_return_us}")
    print(
        f"  controller advertised ACL: {transport.controller_acl_packet_count} "
        f"packets of {transport.controller_acl_packet_bytes} bytes"
    )
    print(HAPTICS_TRANSPORT_PROBE_EVIDENCE_NOTE, flush=True)


def _raise_haptics_experiment_failure(
    snapshot: HapticsExperimentDiagnostics,
) -> None:
    if not snapshot.firmware_supported:
        raise ConfigManagerError(
            "haptics experiment is unsupported in this firmware. "
            + HAPTICS_EXPERIMENT_ENABLE_HINT
        )
    if snapshot.state_name in ("disconnected", "unsupported", "error"):
        raise ConfigManagerError(
            f"haptics experiment run {snapshot.run_id} "
            f"{snapshot.state_name}: {snapshot.error_name} "
            f"(last_error={snapshot.last_error}); "
            "check the selected controller connection and experiment status"
        )


def _print_haptics_experiment(
    snapshot: HapticsExperimentDiagnostics,
    *,
    as_json: bool,
) -> None:
    values = snapshot.to_json_object()
    if as_json:
        values["host_monotonic_s"] = time.monotonic()
        print(json.dumps(values, sort_keys=True), flush=True)
        return
    print(
        f"Haptics experiment: {snapshot.state_name}; run={snapshot.run_id}; "
        f"slot={snapshot.slot if snapshot.slot is not None else 'none'}; "
        f"mode={snapshot.mode_name}"
    )
    for name, value in asdict(snapshot).items():
        if name not in ("state", "slot", "last_error", "mode"):
            print(f"  {name}: {value}")
    print(f"  last_error: {snapshot.last_error} ({snapshot.error_name})")
    delay = snapshot.first_tone_submission_delay_us
    print(
        "  first_tone_submission_delay_us: "
        f"{delay if delay is not None else 'not recorded'}"
    )
    if snapshot.mode == 1:
        print(
            f"Gameplay: continuous 3 kHz, {snapshot.packet_frames} stereo frames/packet; "
            f"{values['gameplay']['lookback_us']:.3f} us lookback, 8000 us command window, "
            "50000 us host-effect watchdog; balanced 2x gameplay gain with a "
            "0.8-power response curve, jointly headroom-limited. "
            "Silence continues without commands."
        )
        print(HAPTICS_GAMEPLAY_ARMING_NOTE)
    else:
        pattern = values["pattern"]
        print(
            f"Pattern: 3 kHz, {pattern['stereo_frames_per_packet']} stereo frames/packet, "
            f"peak {pattern['peak_amplitude']}/127; "
            f"{pattern['priming_silence_packets']} packets priming silence (1.024 s), "
            f"{pattern['cycles']} cycles of "
            "left 100 Hz / silence / right 200 Hz / silence "
            f"({pattern['phases'][0]['packets']} packets = 256 ms each), "
            f"{pattern['trailing_silence_packets']} packets trailing silence (1.024 s); "
            f"{pattern['total_packets']} packets / 6.144 s total. "
            "Initial state-only mode handoff carries no PCM frames and counts as one packet."
        )
    print(
        "Timestamp fields are low 32-bit Pico uptime microseconds; "
        "differences use unsigned wraparound."
    )
    print(values["evidence_note"], flush=True)


def _watch_haptics_experiment(
    device: UsbDevice,
    snapshot: HapticsExperimentDiagnostics,
    deadline: float,
    *,
    as_json: bool,
) -> None:
    run_id = snapshot.run_id
    slot = snapshot.slot
    mode = snapshot.mode
    while True:
        _print_haptics_experiment(snapshot, as_json=as_json)
        _raise_haptics_experiment_failure(snapshot)
        if snapshot.state_name not in ("pending", "running"):
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if mode == 1:
                raise ConfigManagerError(
                    f"haptics gameplay watch reached --timeout; run {run_id} "
                    "is still armed. Use haptics-experiment stop --slot "
                    f"{slot} to disarm; watching does not stop the stream."
                )
            raise ConfigManagerError(
                f"haptics experiment run {run_id} did not reach a terminal "
                "state before --timeout; it may still be active, use status "
                "or stop --slot " + str(slot)
            )
        time.sleep(min(0.1, remaining))
        snapshot = read_haptics_experiment(device)
        if (snapshot.run_id, snapshot.slot, snapshot.mode) != (run_id, slot, mode):
            raise ConfigManagerError(
                "haptics experiment run changed while watching; "
                "cannot attribute measurements to the requested run"
            )


def _run_haptics_experiment_command(
    device: UsbDevice,
    args: argparse.Namespace,
) -> None:
    if args.haptics_command == "profile":
        snapshot, transport = read_haptics_experiment_profile(device)
        _print_haptics_experiment_profile(snapshot, transport, as_json=args.json)
        return
    before = read_haptics_experiment(device)
    deadline = time.monotonic() + args.timeout
    action = args.haptics_command
    if action == "status":
        if not before.firmware_supported:
            _print_haptics_experiment(before, as_json=args.json)
            print(HAPTICS_EXPERIMENT_ENABLE_HINT, file=sys.stderr)
        elif args.watch:
            _watch_haptics_experiment(device, before, deadline, as_json=args.json)
        else:
            _print_haptics_experiment(before, as_json=args.json)
            _raise_haptics_experiment_failure(before)
        return
    if not before.firmware_supported:
        _raise_haptics_experiment_failure(before)
    active = before.state_name in ("pending", "running")
    starting = action in ("start", "gameplay")
    if starting and active:
        raise ConfigManagerError(
            f"haptics experiment is already {before.state_name} "
            f"on slot {before.slot}; stop that run before starting another"
        )
    if action == "stop":
        if active and before.slot != args.slot:
            raise ConfigManagerError(
                f"haptics experiment is active on slot {before.slot}, "
                f"not requested slot {args.slot}"
            )
        if not active:
            _print_haptics_experiment(before, as_json=args.json)
            print(
                "No active haptics experiment to stop.",
                file=sys.stderr if args.json else sys.stdout,
            )
            return
    _control_out(
        device,
        OP_HAPTICS_EXPERIMENT,
        bytes(({"start": 1, "gameplay": 2, "stop": 0}[action], args.slot)),
    )
    print(
        f"{action.capitalize()} request accepted; pending firmware confirmation. "
        "USB ACK is not evidence of stream start, completion, or playback.",
        file=sys.stderr if args.json else sys.stdout,
        flush=True,
    )
    expected_run_id = (before.run_id + 1) & 0xFFFFFFFF if starting else before.run_id
    expected_mode = (1 if action == "gameplay" else 0) if starting else before.mode
    observed_run = False
    while True:
        snapshot = read_haptics_experiment(device)
        if snapshot.run_id == expected_run_id:
            observed_run = True
            if snapshot.slot != args.slot:
                raise ConfigManagerError(
                    "haptics experiment response belongs to another slot"
                )
            if snapshot.mode != expected_mode:
                raise ConfigManagerError(
                    "haptics experiment response belongs to another mode"
                )
            if snapshot.state_name in ("disconnected", "unsupported", "error"):
                _print_haptics_experiment(snapshot, as_json=args.json)
                _raise_haptics_experiment_failure(snapshot)
            if starting and args.watch:
                _watch_haptics_experiment(device, snapshot, deadline, as_json=args.json)
                return
            if starting and (
                snapshot.state_name == "running"
                or (action == "start" and snapshot.state_name == "completed")
            ):
                _print_haptics_experiment(snapshot, as_json=args.json)
                if snapshot.state_name == "running":
                    print(
                        "Firmware reports running; use status --watch "
                        "to capture later states and failures.",
                        file=sys.stderr if args.json else sys.stdout,
                    )
                return
            if snapshot.state_name in ("stopped", "completed") and action == "stop":
                _print_haptics_experiment(snapshot, as_json=args.json)
                return
            if snapshot.state_name not in ("pending", "running"):
                raise ConfigManagerError(
                    f"haptics experiment {action} ended in unexpected "
                    f"state {snapshot.state_name}"
                )
        elif (
            action == "stop"
            or observed_run
            or (snapshot.run_id, snapshot.slot, snapshot.mode)
            != (before.run_id, before.slot, before.mode)
        ):
            raise ConfigManagerError(
                f"haptics experiment run changed before {action} was confirmed"
            )
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if action == "gameplay":
                raise ConfigManagerError(
                    "haptics gameplay was not confirmed before --timeout; "
                    "the stream may still be armed. Use status to inspect it "
                    f"or haptics-experiment stop --slot {args.slot} to disarm "
                    "(USB ACK alone does not confirm it)"
                )
            raise ConfigManagerError(
                f"haptics experiment {action} was not confirmed before "
                "--timeout; check status (USB ACK alone does not confirm it)"
            )
        time.sleep(min(0.1, remaining))


def _canonical_native_switch_controllers(
    identities: tuple[ControllerIdentity, ...],
) -> tuple[ControllerIdentity, ...]:
    if (
        type(identities) is not tuple
        or len(identities) > NATIVE_SWITCH_CONTROLLER_CAPACITY
    ):
        raise ConfigManagerError("invalid native rumble approval list")
    for identity in identities:
        if (
            not isinstance(identity, ControllerIdentity)
            or not identity.stable
            or identity.transport != TRANSPORT_CLASSIC
            or identity.vendor_id != 0x057E
            or identity.product_id not in (0x2009, 0x2006, 0x2007)
        ):
            raise ConfigManagerError(
                "native rumble approval requires a stable Classic Nintendo "
                "Pro Controller or Joy-Con identity"
            )
    if len(set(identities)) != len(identities):
        raise ConfigManagerError("duplicate native rumble approval")
    return tuple(sorted(identities, key=ControllerIdentity.to_bytes))


def read_configuration(device: UsbDevice) -> AdapterConfiguration:
    envelope = _control_in(device, OP_CONFIGURATION_READ)
    _raise_status(envelope)
    payload = envelope.payload
    identities: tuple[ControllerIdentity, ...] = ()
    joycon_mode = JOYCON_MODE_PAIRED
    if envelope.schema_version == 1:
        if len(payload) != 4 or payload[2:] != bytes(2):
            raise ConfigManagerError("unsupported configuration object")
        requested_mode = REQUESTED_MODE_AUTO
    elif envelope.schema_version == 2:
        if len(payload) != 8 or payload[3:] != bytes(5):
            raise ConfigManagerError("unsupported configuration object")
        requested_mode = payload[2]
    elif envelope.schema_version in (3, CONFIGURATION_SCHEMA_VERSION):
        if len(payload) != CONFIGURATION_SIZE:
            raise ConfigManagerError("unsupported configuration object")
        count = payload[3]
        end = 8 + count * CONTROLLER_IDENTITY_SIZE
        if (
            count > NATIVE_SWITCH_CONTROLLER_CAPACITY
            or payload[5:8] != bytes(3)
            or (envelope.schema_version == 3 and payload[4] != 0)
            or payload[end:] != bytes(CONFIGURATION_SIZE - end)
        ):
            raise ConfigManagerError("invalid native rumble approval encoding")
        identities = tuple(
            ControllerIdentity.from_bytes(
                payload[offset : offset + CONTROLLER_IDENTITY_SIZE]
            )
            for offset in range(8, end, CONTROLLER_IDENTITY_SIZE)
        )
        if identities != _canonical_native_switch_controllers(identities):
            raise ConfigManagerError("noncanonical native rumble approval order")
        requested_mode = payload[2]
        if envelope.schema_version == CONFIGURATION_SCHEMA_VERSION:
            joycon_mode = payload[4]
    else:
        raise ConfigManagerError("unsupported configuration object")
    pairing_window_seconds = struct.unpack_from("<H", payload)[0]
    if not (
        PAIRING_WINDOW_SECONDS_MIN
        <= pairing_window_seconds
        <= PAIRING_WINDOW_SECONDS_MAX
    ):
        raise ConfigManagerError("invalid stored pairing-window duration")
    if requested_mode >= len(REQUESTED_MODE_NAMES):
        raise ConfigManagerError(f"invalid stored requested USB mode {requested_mode}")
    if joycon_mode >= len(JOYCON_MODE_NAMES):
        raise ConfigManagerError(f"invalid stored Joy-Con2 mode {joycon_mode}")
    return AdapterConfiguration(
        pairing_window_seconds=pairing_window_seconds,
        generation=envelope.generation,
        crc=envelope.payload_crc,
        requested_mode=requested_mode,
        native_switch_controllers=identities,
        schema_version=envelope.schema_version,
        joycon_mode=joycon_mode,
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
        raise ConfigManagerError("pairing window must be between 10 and 300 seconds")
    if type(
        configuration.requested_mode
    ) is not int or not 0 <= configuration.requested_mode < len(REQUESTED_MODE_NAMES):
        raise ConfigManagerError("invalid requested USB mode")
    _require_int(configuration.joycon_mode, "Joy-Con2 mode", 0, 1)
    if (
        configuration.schema_version < CONFIGURATION_SCHEMA_VERSION
        and configuration.joycon_mode != JOYCON_MODE_PAIRED
    ):
        raise ConfigManagerError("Joy-Con2 player mode requires schema 4 firmware")
    identities = _canonical_native_switch_controllers(
        configuration.native_switch_controllers
    )
    if configuration.schema_version in (3, CONFIGURATION_SCHEMA_VERSION):
        payload = (
            struct.pack(
                "<HBBB3x",
                configuration.pairing_window_seconds,
                configuration.requested_mode,
                len(identities),
                configuration.joycon_mode,
            )
            + b"".join(identity.to_bytes() for identity in identities)
            + bytes(
                (NATIVE_SWITCH_CONTROLLER_CAPACITY - len(identities))
                * CONTROLLER_IDENTITY_SIZE
            )
        )
    elif configuration.schema_version in (1, 2):
        if identities:
            raise ConfigManagerError(
                "native rumble approval requires schema 3 firmware"
            )
        if configuration.schema_version == 1:
            if configuration.requested_mode != REQUESTED_MODE_AUTO:
                raise ConfigManagerError(
                    "schema 1 does not support a requested USB mode"
                )
            payload = struct.pack("<H2x", configuration.pairing_window_seconds)
        else:
            payload = struct.pack(
                "<HB5x",
                configuration.pairing_window_seconds,
                configuration.requested_mode,
            )
    else:
        raise ConfigManagerError("unsupported configuration schema")
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_CONFIGURATION_BEGIN,
        struct.pack(
            "<IHHI",
            transaction_id,
            configuration.schema_version,
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
    _control_out(device, OP_CONFIGURATION_COMMIT, struct.pack("<I", transaction_id))
    return _wait_for_transaction(device, transaction_id, timeout)


def set_joycon_mode(device: UsbDevice, mode: int, timeout: float) -> TransactionStatus:
    """Persist the adapter-wide player mode without rebooting or changing profiles."""
    _require_int(mode, "Joy-Con2 mode", 0, 1)
    before = read_configuration(device)
    if before.schema_version < CONFIGURATION_SCHEMA_VERSION:
        raise ConfigManagerError("Joy-Con2 player mode requires schema 4 firmware")
    return write_configuration(device, replace(before, joycon_mode=mode), timeout)


def set_native_switch_rumble_approval(
    device: UsbDevice,
    identity: ControllerIdentity,
    approved: bool,
    timeout: float,
) -> TransactionStatus:
    """Persist explicit physical-controller approval without changing its profiles."""
    _require_bool(approved, "native rumble approval")
    _canonical_native_switch_controllers((identity,))
    before = read_configuration(device)
    if before.schema_version < 3:
        raise ConfigManagerError("native rumble approval requires schema 3 firmware")
    identities = tuple(
        item for item in before.native_switch_controllers if item != identity
    )
    if approved:
        identities += (identity,)
    return write_configuration(
        device, replace(before, native_switch_controllers=identities), timeout
    )


def reset_configuration(device: UsbDevice, timeout: float) -> TransactionStatus:
    transaction_id = _host_transaction_id()
    _control_out(device, OP_CONFIGURATION_RESET, struct.pack("<I", transaction_id))
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
    if read_info(device).active_mode == ACTIVE_MODE_NATIVE_HUB:
        raise ConfigManagerError(
            "Native Joy-Con hub firmware has fixed USB output; mode changes are unavailable"
        )
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_MODE_SET,
        struct.pack("<IB", transaction_id, requested_mode),
    )
    return _wait_for_transaction(device, transaction_id, timeout)


def request_reboot(device: UsbDevice, transaction_id: int) -> None:
    _require_int(transaction_id, "transaction ID", 1, HOST_TRANSACTION_ID_MASK)
    if read_info(device).active_mode == ACTIVE_MODE_NATIVE_HUB:
        raise ConfigManagerError(
            "Native Joy-Con hub firmware has fixed USB output; reboot-to-mode is unavailable"
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
        PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION,
        PROFILE_CONTROL_MAPPING_SCHEMA_VERSION,
        PROFILE_ACTION_CONTROL_SCHEMA_VERSION,
        PROFILE_SPARSE_MACRO_SCHEMA_VERSION,
        PROFILE_EXPANDED_SCHEMA_VERSION,
        PROFILE_EXTRA_CONTROL_SCHEMA_VERSION,
        PROFILE_SWING_SCHEMA_VERSION,
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
        alias_size = envelope.payload[offset + 16]
        alias_payload = envelope.payload[offset + 17 : offset + 48]
        if (
            envelope.payload[offset + 15] != 0
            or alias_size > PROFILE_METADATA_MAX_BYTES
            or any(alias_payload[alias_size:])
        ):
            raise ConfigManagerError("invalid profile-list metadata")
        try:
            alias = alias_payload[:alias_size].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ConfigManagerError("invalid profile-list alias") from exc
        if index == 0 and not identity.is_global_fallback:
            raise ConfigManagerError("profile list does not begin with global fallback")
        if index != 0 and identity.is_global_fallback:
            raise ConfigManagerError("duplicate global fallback profile entry")
        if identity in identities:
            raise ConfigManagerError("duplicate identity in profile list")
        identities.add(identity)
        entries.append(ProfileListEntry(identity, active_profile_index, alias))
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
        PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION,
        PROFILE_CONTROL_MAPPING_SCHEMA_VERSION,
        PROFILE_ACTION_CONTROL_SCHEMA_VERSION,
        PROFILE_SPARSE_MACRO_SCHEMA_VERSION,
        PROFILE_EXPANDED_SCHEMA_VERSION,
        PROFILE_EXTRA_CONTROL_SCHEMA_VERSION,
        PROFILE_SWING_SCHEMA_VERSION,
        PROFILE_SCHEMA_VERSION,
    ):
        raise ConfigManagerError("unsupported profile schema")
    if (
        len(envelope.payload) < 4
        or struct.unpack_from("<H", envelope.payload)[0] != envelope.schema_version
    ):
        raise ConfigManagerError("profile envelope schema does not match payload")
    return ControllerProfile.from_bytes(envelope.payload)


def read_profile(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
) -> ControllerProfile:
    select_profile(device, identity, profile_index)
    return read_selected_profile(device)


def _decode_profile_metadata_value(payload: bytes, offset: int, label: str) -> str:
    size = payload[offset]
    encoded = payload[offset + 1 : offset + PROFILE_METADATA_VALUE_SIZE]
    if size > PROFILE_METADATA_MAX_BYTES or any(encoded[size:]):
        raise ConfigManagerError(f"invalid {label} metadata")
    try:
        return encoded[:size].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ConfigManagerError(f"invalid {label} UTF-8") from exc


def parse_profile_metadata(envelope: Envelope) -> ProfileMetadata:
    _raise_status(envelope)
    if (
        envelope.schema_version != PROFILE_METADATA_SCHEMA_VERSION
        or len(envelope.payload) != PROFILE_METADATA_SIZE
    ):
        raise ConfigManagerError("invalid profile metadata payload")
    alias = _decode_profile_metadata_value(envelope.payload, 0, "controller alias")
    names = tuple(
        _decode_profile_metadata_value(
            envelope.payload,
            (index + 1) * PROFILE_METADATA_VALUE_SIZE,
            f"profile {index + 1} name",
        )
        for index in range(PROFILE_CAPACITY)
    )
    return ProfileMetadata(alias, names)


def read_selected_profile_metadata(device: UsbDevice) -> ProfileMetadata:
    return parse_profile_metadata(_control_in(device, OP_PROFILE_METADATA_READ))


def read_profile_metadata(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int = 0,
) -> ProfileMetadata:
    select_profile(device, identity, profile_index)
    return read_selected_profile_metadata(device)


def set_profile_metadata(
    device: UsbDevice,
    identity: ControllerIdentity,
    profile_index: int,
    value: str,
    timeout: float,
) -> TransactionStatus:
    if profile_index != PROFILE_NONE_BUTTON:
        _validate_profile_index(profile_index)
    if type(value) is not str or "\x00" in value:
        raise ConfigManagerError("profile metadata must be text")
    encoded = value.encode("utf-8")
    if len(encoded) > PROFILE_METADATA_MAX_BYTES:
        raise ConfigManagerError("profile metadata must contain at most 31 UTF-8 bytes")
    transaction_id = _host_transaction_id()
    _control_out(
        device,
        OP_PROFILE_METADATA_SET,
        struct.pack("<I", transaction_id)
        + identity.to_bytes()
        + bytes((profile_index, len(encoded)))
        + encoded,
    )
    return _wait_for_profile_transaction(device, transaction_id, timeout)


def identify_controller(device: UsbDevice, identity: ControllerIdentity) -> None:
    if identity.is_global_fallback:
        raise ConfigManagerError("default profile has no controller to identify")
    _control_out(device, OP_PROFILE_IDENTIFY, identity.to_bytes())


def set_wii_orientation(
    device: UsbDevice,
    identity: ControllerIdentity,
    connection_generation: int,
    orientation: str,
) -> None:
    if identity.is_global_fallback or not identity.stable:
        raise ConfigManagerError("select a connected Wii Remote to change orientation")
    _require_int(connection_generation, "connection generation", 0, 0xFFFFFFFF)
    mode = _require_enum(orientation, ("horizontal", "vertical"), "Wii orientation")
    _control_out(
        device,
        OP_WII_ORIENTATION,
        identity.to_bytes() + struct.pack("<IB", connection_generation, mode),
    )


def parse_profile_playtest(envelope: Envelope) -> ProfilePlaytest:
    _raise_status(envelope)
    expected_size = {
        PROFILE_PLAYTEST_LEGACY_SCHEMA_VERSION: PROFILE_PLAYTEST_LEGACY_SIZE,
        PROFILE_PLAYTEST_EXTRA_BUTTON_SCHEMA_VERSION: PROFILE_PLAYTEST_EXTRA_BUTTON_SIZE,
        PROFILE_PLAYTEST_TOPOLOGY_SCHEMA_VERSION: PROFILE_PLAYTEST_SIZE,
        PROFILE_PLAYTEST_SCHEMA_VERSION: PROFILE_PLAYTEST_SIZE,
    }.get(envelope.schema_version)
    if len(envelope.payload) != expected_size:
        raise ConfigManagerError("invalid profile playtest payload")
    payload = envelope.payload
    extra_buttons = (
        payload[54] if len(payload) >= PROFILE_PLAYTEST_EXTRA_BUTTON_SIZE else 0
    )
    layout_code = payload[55] if len(payload) >= PROFILE_PLAYTEST_SIZE else 0
    layout_count = (
        6
        if envelope.schema_version == PROFILE_PLAYTEST_TOPOLOGY_SCHEMA_VERSION
        else len(PROFILE_PLAYTEST_LAYOUTS)
    )
    if layout_code >= layout_count:
        raise ConfigManagerError("invalid playtest controller layout")
    if extra_buttons & ~0x7F:
        raise ConfigManagerError("invalid playtest extra buttons")
    flags = payload[0]
    if flags & ~0x03 or flags != envelope.flags or payload[41] != 0:
        raise ConfigManagerError("invalid profile playtest flags")
    connected = bool(flags & 0x01)
    has_motion = bool(flags & 0x02)
    motion_count = payload[38]
    if not connected:
        if flags != 0 or payload[1] != 0xFF or any(payload[2:]):
            raise ConfigManagerError("invalid disconnected playtest payload")
        return ProfilePlaytest(
            False,
            None,
            0,
            0,
            None,
            0,
            (0, 0),
            (0, 0),
            (0, 0),
            0,
            0,
            None,
        )
    if (
        payload[1] >= PROFILE_PLAYTEST_SLOT_COUNT
        or not 0 <= motion_count <= 3
        or has_motion != (motion_count != 0)
    ):
        raise ConfigManagerError("invalid connected playtest payload")
    identity = ControllerIdentity.from_bytes(payload[12:26])
    left_x, left_y, right_x, right_y, left_trigger, right_trigger = struct.unpack_from(
        "<hhhhHH", payload, 26
    )
    motion_values = struct.unpack_from("<hhhhhh", payload, 42)
    if not has_motion and any(motion_values):
        raise ConfigManagerError("playtest motion sample was not declared")
    return ProfilePlaytest(
        connected=True,
        slot_index=payload[1],
        connection_generation=struct.unpack_from("<I", payload, 4)[0],
        state_generation=struct.unpack_from("<I", payload, 8)[0],
        identity=identity,
        button_mask=struct.unpack_from("<H", payload, 2)[0],
        left_stick=(left_x, left_y),
        right_stick=(right_x, right_y),
        triggers=(left_trigger, right_trigger),
        battery=payload[39],
        capabilities=payload[40],
        motion=motion_values if has_motion else None,
        extra_buttons=extra_buttons,
        layout=PROFILE_PLAYTEST_LAYOUTS[layout_code],
    )


def read_profile_playtest(device: UsbDevice) -> ProfilePlaytest:
    return parse_profile_playtest(_control_in(device, OP_PROFILE_PLAYTEST))


def read_profile_transaction_status(device: UsbDevice) -> TransactionStatus:
    envelope = _control_in(device, OP_PROFILE_TRANSACTION_STATUS)
    _raise_status(envelope, pending_ok=True)
    if (
        not PROFILE_LEGACY_SCHEMA_VERSION
        <= envelope.schema_version
        <= PROFILE_SCHEMA_VERSION
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
            not PROFILE_LEGACY_SCHEMA_VERSION
            <= envelope.schema_version
            <= PROFILE_SCHEMA_VERSION
            or len(envelope.payload) != 20
        ):
            raise ConfigManagerError("invalid profile transaction-status payload")
        values = struct.unpack("<IHHIII", envelope.payload)
        status = TransactionStatus(*values, status=envelope.status)
        if status.transaction_id != transaction_id:
            raise ConfigManagerError("device reported a different profile transaction")
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
        struct.pack("<I", transaction_id) + identity.to_bytes() + bytes((wire_index,)),
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
    if record_count > PAIRING_RECORD_CAPACITY or len(envelope.payload) != required:
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
            usb.core.find(find_all=True, idVendor=vendor_id, idProduct=product_id),
        )
        if devices is not None:
            yield from devices


def find_pico(bus: int | None, address: int | None, timeout: float = 3.0) -> UsbDevice:
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
                previous_enumeration is not None and enumeration == previous_enumeration
            )
            or (selected_location is not None and location == selected_location)
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
            _is_previous_enumeration(device, snapshot) for device in candidates
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
            "Pico did not re-enumerate on its original physical USB port after reboot"
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
    if before_info.active_mode == ACTIVE_MODE_NATIVE_HUB:
        raise ConfigManagerError(
            "Native Joy-Con hub firmware has fixed USB output; mode changes are unavailable"
        )
    before_configuration = read_configuration(device)
    if before_configuration.requested_mode == requested_mode and _mode_is_active(
        requested_mode, before_info.active_mode
    ):
        return device, False
    reenumeration_snapshot = _capture_reenumeration_snapshot(device)

    transaction = set_mode(device, requested_mode, timeout)
    request_reboot(device, transaction.transaction_id)
    reenumerated = _wait_for_reenumeration(reenumeration_snapshot, timeout)
    after_info = read_info(reenumerated)
    after_configuration = read_configuration(reenumerated)
    if after_configuration.requested_mode != requested_mode:
        raise ConfigManagerError("requested USB mode was not stored after reboot")
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


def _print_profiles(
    entries: Sequence[ProfileListEntry], *, physical_only: bool = False
) -> None:
    for index, entry in enumerate(entries):
        identity = entry.identity
        if physical_only and (identity.is_global_fallback or identity.is_joycon_pair):
            continue
        if identity.is_global_fallback:
            description = "global fallback"
        elif identity.is_joycon_pair:
            description = (
                f"Nintendo Joy-Con 2 (L+R) · {identity.transport_text} "
                f"L {identity.address_text} address-type {identity.address_type}; "
                f"R {identity.partner_address_text} "
                f"address-type {identity.partner_address_type}"
            )
        else:
            description = (
                f"{identity.transport_text} {identity.address_text} "
                f"address-type {identity.address_type} "
                f"VID:PID {identity.vendor_id:04X}:{identity.product_id:04X}"
            )
        print(
            f"{index}: {description} (active profile {entry.active_profile_index + 1})"
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
        raise ConfigManagerError(f"could not read profile JSON {path}: {exc}") from exc
    return ControllerProfile.from_json(payload)


def _save_profile(path: Path, profile: ControllerProfile) -> None:
    try:
        path.write_text(profile.to_json(), encoding="utf-8")
    except OSError as exc:
        raise ConfigManagerError(f"could not write profile JSON {path}: {exc}") from exc


def _profile_number(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"profile must be a number from 1 to {PROFILE_CAPACITY}"
        ) from exc
    if not 1 <= number <= PROFILE_CAPACITY:
        raise argparse.ArgumentTypeError(
            f"profile must be a number from 1 to {PROFILE_CAPACITY}"
        )
    return number - 1


def _identity_index(value: str) -> int:
    try:
        index = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "identity must be a non-negative list index"
        ) from exc
    if index < 0:
        raise argparse.ArgumentTypeError("identity must be a non-negative list index")
    return index


def _tcp_port(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "port must be a number from 0 to 65535"
        ) from exc
    if not 0 <= port <= 0xFFFF:
        raise argparse.ArgumentTypeError("port must be a number from 0 to 65535")
    return port


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
    haptics = commands.add_parser(
        "haptics-experiment", help="control the opt-in DualSense PCM experiment"
    )
    haptics_commands = haptics.add_subparsers(dest="haptics_command", required=True)
    for action, help_text in (
        ("start", "run the finite PCM fixture"),
        ("gameplay", "arm continuous Nintendo HD-rumble PCM until stopped"),
        ("status", "read the current fixture or gameplay stream"),
        ("stop", "stop and disarm the selected stream"),
    ):
        experiment = haptics_commands.add_parser(action, help=help_text)
        if action != "status":
            experiment.add_argument(
                "--slot",
                type=int,
                choices=range(HAPTICS_EXPERIMENT_SLOT_COUNT),
                default=0,
                help="connected controller slot (default: 0)",
            )
        if action != "stop":
            experiment.add_argument(
                "--watch",
                action="store_true",
                help="capture 100 ms status samples until terminal or --timeout; "
                "does not stop an armed gameplay stream",
            )
        experiment.add_argument(
            "--json",
            action="store_true",
            help="emit JSON diagnostics (one object per sample with --watch)",
        )
    profile = haptics_commands.add_parser(
        "profile", help="read a run-correlated transport timing snapshot"
    )
    profile.add_argument(
        "--json",
        action="store_true",
        help="emit diagnostics with a nested transport profile object",
    )
    reboot = commands.add_parser("reboot", help="reboot into a firmware or ROM target")
    reboot.add_argument("target", choices=("bootsel",))
    mode = commands.add_parser("mode", help="select the persistent USB mode")
    mode.add_argument("mode", choices=SELECTABLE_MODE_NAMES)
    joycon_mode = commands.add_parser(
        "joycon-mode",
        help="read or apply the persistent Joy-Con2 player mode without reconnecting",
    )
    joycon_mode.add_argument("mode", nargs="?", choices=JOYCON_MODE_NAMES)
    joycon_mode.add_argument(
        "--json", action="store_true", help="emit the committed mode as JSON"
    )

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
    native_rumble = config_commands.add_parser(
        "native-rumble",
        help="manage physical-controller native Nintendo rumble approvals",
        description=NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE,
    )
    native_commands = native_rumble.add_subparsers(
        dest="native_rumble_command", required=True
    )
    native_commands.add_parser("list", help="list stored identities and approvals")
    native_status = native_commands.add_parser(
        "status", help="show per-slot native Nintendo rumble qualification counters"
    )
    native_status.add_argument(
        "--json", action="store_true", help="emit JSON diagnostics"
    )
    for action in ("approve", "revoke"):
        native_action = native_commands.add_parser(
            action,
            help=f"{action} native rumble for one stored physical controller",
            description=NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE,
        )
        native_selector = native_action.add_mutually_exclusive_group(required=True)
        native_selector.add_argument(
            "--identity",
            type=_identity_index,
            metavar="N",
            help="physical identity index from profiles list or native-rumble list",
        )
        if action == "revoke":
            native_selector.add_argument(
                "--approval",
                type=_identity_index,
                metavar="N",
                help="approval index from native-rumble list, including forgotten identities",
            )
        if action == "approve":
            native_action.add_argument(
                "--yes",
                action="store_true",
                help="confirm this is a genuine qualified Pro Controller or Joy-Con",
            )

    profiles = commands.add_parser(
        "profiles",
        help="open the editor or manage controller profiles as JSON",
    )
    profile_commands = profiles.add_subparsers(dest="profile_command", required=True)
    profile_commands.add_parser(
        "list", help="list profile identities and active profiles"
    )
    profile_edit = profile_commands.add_parser(
        "edit", help="open the local graphical profile editor"
    )
    profile_edit.add_argument(
        "--port",
        type=_tcp_port,
        default=8765,
        metavar="PORT",
        help="localhost port (default: 8765; use 0 to choose automatically)",
    )
    profile_edit.add_argument(
        "--no-browser",
        action="store_true",
        help="print the editor URL without opening a browser",
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
    pairing_commands = pairings.add_subparsers(dest="pairing_command", required=True)
    pairing_commands.add_parser("list", help="list stored pairings")
    pairing_clear = pairing_commands.add_parser("clear", help="clear pairings")
    pairing_clear.add_argument("--yes", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout <= 0:
        print("error: --timeout must be positive", file=sys.stderr)
        return 2
    if args.command == "haptics-experiment" and not math.isfinite(args.timeout):
        print("error: --timeout must be finite", file=sys.stderr)
        return 2
    if args.command == "config" and args.config_command == "reset" and not args.yes:
        print("error: config reset requires --yes", file=sys.stderr)
        return 2
    if args.command == "pairings" and args.pairing_command == "clear" and not args.yes:
        print("error: pairings clear requires --yes", file=sys.stderr)
        return 2
    if args.command == "profiles" and args.profile_command == "reset" and not args.yes:
        print("error: profiles reset requires --yes", file=sys.stderr)
        return 2
    if (
        args.command == "config"
        and args.config_command == "native-rumble"
        and args.native_rumble_command == "approve"
        and not args.yes
    ):
        print(
            "error: native-rumble approve requires --yes. "
            + NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE,
            file=sys.stderr,
        )
        return 2

    imported_profile: ControllerProfile | None = None
    try:
        if args.command == "profiles" and args.profile_command == "edit":
            profile_web = importlib.import_module(".profile_web", __package__)
            profile_web.run_profile_editor(
                bus=args.bus,
                address=args.address,
                timeout=args.timeout,
                port=args.port,
                open_browser=not args.no_browser,
            )
            return 0
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
            print(f"Pairing window: {configuration.pairing_window_seconds} seconds")
            print(
                f"Joy-Con2 player mode: {JOYCON_MODE_NAMES[configuration.joycon_mode]}"
            )
        elif args.command == "diagnostics":
            diagnostics = read_runtime_diagnostics(device)
            print(f"Initialization stage: {diagnostics.initialization_stage}")
            print(f"Rumble timer ticks: {diagnostics.rumble_timer_ticks}")
            print(f"Configuration timer ticks: {diagnostics.configuration_timer_ticks}")
            print(f"Controller reports: {diagnostics.controller_reports}")
            print(f"Host rumble requests: {diagnostics.host_rumble_requests}")
            print(f"Local feedback requests: {diagnostics.local_feedback_requests}")
            print(f"Rumble dispatches: {diagnostics.rumble_dispatches}")
            print(f"Active slots: {diagnostics.active_slots}")
            print(f"Rumble-capable slots: {diagnostics.rumble_capable_slots}")
            print(f"Feedback-pending slots: {diagnostics.feedback_pending_slots}")
            print(f"Rumble-pending slots: {diagnostics.rumble_pending_slots}")
            if diagnostics.switch2_ingress_drops is not None:
                print(f"Switch 2 ingress drops: {diagnostics.switch2_ingress_drops}")
                print(f"Switch 2 output drops: {diagnostics.switch2_output_drops}")
        elif args.command == "haptics-experiment":
            _run_haptics_experiment_command(device, args)
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
        elif args.command == "joycon-mode":
            if args.mode is not None:
                set_joycon_mode(
                    device, JOYCON_MODE_NAMES.index(args.mode), args.timeout
                )
            configuration = read_configuration(device)
            mode_name = JOYCON_MODE_NAMES[configuration.joycon_mode]
            supported = configuration.schema_version >= CONFIGURATION_SCHEMA_VERSION
            if args.json:
                print(
                    json.dumps(
                        {
                            "mode": mode_name,
                            "generation": configuration.generation,
                            "supported": supported,
                        }
                    )
                )
            else:
                print(f"Joy-Con2 player mode: {mode_name}")
                print(f"Configuration generation: {configuration.generation}")
                if not supported:
                    print("Changing Joy-Con2 player mode requires schema 4 firmware.")
        elif args.command == "config":
            if args.config_command == "show":
                configuration = read_configuration(device)
                print(f"pairing_window_seconds={configuration.pairing_window_seconds}")
                print(
                    "requested_mode="
                    f"{REQUESTED_MODE_NAMES[configuration.requested_mode]}"
                )
                print(f"joycon_mode={JOYCON_MODE_NAMES[configuration.joycon_mode]}")
                print(f"generation={configuration.generation}")
                print(f"crc={configuration.crc:08x}")
                for identity in configuration.native_switch_controllers:
                    print(
                        f"native_switch_controller={identity.address_text} "
                        f"VID:PID {identity.vendor_id:04X}:{identity.product_id:04X}"
                    )
            elif args.config_command == "set":
                before = read_configuration(device)
                status = write_configuration(
                    device,
                    replace(before, pairing_window_seconds=args.pairing_window_seconds),
                    args.timeout,
                )
                print(
                    "Stored configuration generation "
                    f"{status.stored_generation} "
                    f"(CRC {status.stored_crc:08x})."
                )
            elif args.config_command == "native-rumble":
                approval_index = getattr(args, "approval", None)
                entries = (
                    list_profiles(device)
                    if args.native_rumble_command != "status" and approval_index is None
                    else ()
                )
                if args.native_rumble_command == "status":
                    _print_native_switch_rumble_status(
                        read_native_switch_rumble(device), json_output=args.json
                    )
                elif args.native_rumble_command == "list":
                    configuration = read_configuration(device)
                    print(NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE)
                    if configuration.schema_version < 3:
                        print("Native rumble approval requires schema 3 firmware.")
                    _print_profiles(entries, physical_only=True)
                    for index, entry in enumerate(entries):
                        if entry.identity in configuration.native_switch_controllers:
                            print(
                                f"Approved identity {index}: {entry.identity.address_text}"
                            )
                    for index, identity in enumerate(
                        configuration.native_switch_controllers
                    ):
                        print(
                            f"Approval {index}: {identity.address_text} "
                            f"VID:PID {identity.vendor_id:04X}:{identity.product_id:04X} "
                            f"(revoke with --approval {index})"
                        )
                    if not configuration.native_switch_controllers:
                        print("No native rumble approvals.")
                else:
                    if approval_index is None:
                        identity = _resolve_profile_identity(entries, args.identity)
                    else:
                        configuration = read_configuration(device)
                        if approval_index >= len(
                            configuration.native_switch_controllers
                        ):
                            raise ConfigManagerError(
                                "approval index is out of range; use native-rumble list"
                            )
                        identity = configuration.native_switch_controllers[
                            approval_index
                        ]
                    approved = args.native_rumble_command == "approve"
                    status = set_native_switch_rumble_approval(
                        device, identity, approved, args.timeout
                    )
                    print(NATIVE_SWITCH_RUMBLE_APPROVAL_NOTE)
                    print(
                        f"{'Approved' if approved else 'Revoked'} native rumble for "
                        f"controller {identity.address_text} at "
                        f"generation {status.stored_generation}."
                    )
            else:
                status = reset_configuration(device, args.timeout)
                print(f"Reset configuration at generation {status.stored_generation}.")
        elif args.command == "profiles":
            entries = list_profiles(device)
            if args.profile_command == "list":
                _print_profiles(entries)
            else:
                identity = _resolve_profile_identity(entries, args.identity)
                if args.profile_command == "export":
                    profile = read_profile(device, identity, args.profile_index)
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
                    reset_profile(device, identity, args.profile_index, args.timeout)
                    target = (
                        "all profiles"
                        if args.profile_index is None
                        else f"profile {args.profile_index + 1}"
                    )
                    print(f"Reset {target} for identity {args.identity}.")
                else:
                    activate_profile(device, identity, args.profile_index, args.timeout)
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
