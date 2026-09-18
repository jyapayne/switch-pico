#!/usr/bin/env python3
"""Bounded, non-pairing qualification of the switch-pico native Joy-Con USB hub.

Requires Linux, PyUSB/libusb, and the existing sudo -n setfacl permission policy.
Live checks use already-paired R/L donors or one full gamepad per virtual pair;
they cannot wake or pair them. For two pairs, deliberately exercise independent
buttons/sticks on BOTH controllers throughout the check (including controls on
each R/L half); IMU mode also needs distinct deliberate motion on both sources.
Neutral or unassigned pairs are not live input. Shared R/L motion is expected
only within each full-gamepad pair, not across pairs. This cannot prove console
gameplay, physical source isolation, or physical latency. --neutral explicitly
selects standalone transport-only qualification with one or two pairs; it cannot
prove live input, IMU, gameplay, or rumble.
The JSON capture is created exclusively before USB access and retains failures.
Qualification sends no reset, configuration change, pairing exchange, profile
access, flash write, or HID output. Motor playback requires --rumble-sample.
--capture-trace-on-error optionally asks a TRACE-enabled root to retain its current
child EP0 context on the first child control-transfer error, before cleanup.
It never retries the failed request or establishes that its SETUP reached the child.
--reboot-bootsel is a separate, explicit root-only recovery operation: it sends
the private BOOTSEL request and confirms ROM USB enumeration at the same port,
without claiming interfaces or qualifying transport, live input, or gameplay.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from switch2_native_imu import decode_block, native_block

VID = 0x057E
ROOT_PID = 0x2068
SERIAL_PREFIX = "switch-pico-"
BOOTSEL_VID = 0x2E8A
BOOTSEL_PIDS = (0x0003, 0x000F)  # RP2040 and RP2350 ROM USB boot devices.
NATIVE_HUB_TRACE_REQUEST = 0x5E
NATIVE_HUB_TRACE_VALUE = 0x5452
NATIVE_HUB_TRACE_REPLY_SIZE = 16
SIDES = ("R", "L")
# Only protocol constants live in source. Device-specific factory/calibration
# captures stay in the private build paths configured by CMake.
MODELS = {
    "R": {"pid": 0x2066, "port": 1, "report": 0x08, "diagnostic_host": "020000000001"},
    "L": {"pid": 0x2067, "port": 2, "report": 0x07, "diagnostic_host": "020000000002"},
}
CAPTURE_PREFIXES = (
    "SWITCH2_PROBE",
    "SWITCH2_PROBE_SECOND",
    "SWITCH2_PROBE_THIRD",
    "SWITCH2_PROBE_FOURTH",
)


def child_models(pairs: int) -> dict[str, dict[str, Any]]:
    if pairs not in (1, 2):
        raise ValueError("pair count must be 1 or 2")
    models = {}
    for slot in range(pairs * 2):
        side = SIDES[slot % 2]
        pair = "AB"[slot // 2]
        child = side if pairs == 1 else f"{pair}_{side}"
        models[child] = {
            **MODELS[side],
            "side": side,
            "pair": pair,
            "port": slot + 1,
            "capture_prefix": CAPTURE_PREFIXES[slot],
            "diagnostic_host": f"0200000000{slot + 1:02x}",
        }
    return models


def neutral_calibration(factory: bytes, user: bytes) -> tuple[bytes, str]:
    # Match probe_memory_stick_calibration: each child's primary record, valid
    # user override before factory; a nominal 0x800 center is not sufficient.
    def valid(data: bytes) -> bool:
        axes = []
        for offset in (0, 3, 6):
            axes.append(
                (
                    data[offset] | ((data[offset + 1] & 15) << 8),
                    (data[offset + 1] >> 4) | (data[offset + 2] << 4),
                )
            )
        center, positive, negative = axes
        return all(
            0 < center[axis] < 4095
            and 0 < positive[axis] <= 4095 - center[axis]
            and 0 < negative[axis] <= center[axis]
            for axis in (0, 1)
        )

    if user[0x40:0x42] == b"\xb2\xa1" and valid(user[0x42:0x4B]):
        return user[0x42:0x45], "user"
    if not valid(factory[0xA8:0xB1]):
        raise ValueError("no valid captured stick calibration for neutral reports")
    return factory[0xA8:0xAB], "factory"


def model_references(
    build_dir: Path, *, require_imu: bool = True, pairs: int = 1, neutral: bool = False
) -> dict[str, dict[str, Any]]:
    cache = {}
    for line in (build_dir / "CMakeCache.txt").read_text().splitlines():
        if line.startswith("SWITCH") and ":" in line and "=" in line:
            field, value = line.split("=", 1)
            cache[field.split(":", 1)[0]] = value

    def enabled(name: str) -> bool:
        return cache.get(name, "OFF").upper() not in (
            "",
            "0",
            "OFF",
            "NO",
            "FALSE",
            "N",
            "IGNORE",
            "NOTFOUND",
        ) and not cache.get(name, "").upper().endswith("-NOTFOUND")

    if int(cache.get("SWITCH2_PROBE_PAIR_COUNT", "1")) != pairs:
        raise ValueError(
            "--pairs must match SWITCH2_PROBE_PAIR_COUNT in the build cache"
        )
    if neutral != enabled("SWITCH2_PROBE_NEUTRAL_INPUT"):
        raise ValueError(
            "--neutral must match SWITCH2_PROBE_NEUTRAL_INPUT in the build cache"
        )
    if (
        pairs == 2
        and not neutral
        and (
            not enabled("SWITCH2_PROBE_HUB")
            or not enabled("SWITCH_PICO_SWITCH2_USB_BRIDGE")
            or cache.get("SWITCH2_BRIDGE_INPUT") not in ("DUALSENSE", "GAMEPAD")
        )
    ):
        raise ValueError(
            "two-pair live references require a GAMEPAD or DUALSENSE Bluetooth bridge HUB"
        )
    if neutral and (
        not enabled("SWITCH2_PROBE_HUB")
        or not enabled("SWITCH2_PROBE_USB_INIT")
        or enabled("SWITCH_PICO_SWITCH2_USB_BRIDGE")
    ):
        raise ValueError(
            "neutral references require an initialized standalone HUB, not the Bluetooth bridge"
        )
    if (
        require_imu
        and not neutral
        and cache.get("SWITCH2_BRIDGE_INPUT") in ("DUALSENSE", "GAMEPAD")
        and cache.get("SWITCH2_BRIDGE_IMU_TARGET", "BOTH") != "BOTH"
    ):
        raise ValueError(
            "Full paired-IMU qualification requires SWITCH2_BRIDGE_IMU_TARGET=BOTH on every pair; "
            "use the USB-completion UART trace for LEFT/RIGHT routing comparisons"
        )
    models = child_models(pairs)
    for child, model in models.items():
        prefix = model["capture_prefix"]
        fields = ["IDENTITY_FILE", "VERSION_FILE", "FACTORY_FILE", "CONTROLLER_ADDRESS"]
        if neutral:
            fields.append("USER_CALIBRATION_FILE")
        missing = [
            prefix + "_" + field
            for field in fields
            if not cache.get(prefix + "_" + field)
        ]
        if missing:
            raise ValueError(f"{child} missing build references: {', '.join(missing)}")
        identity = Path(cache[prefix + "_IDENTITY_FILE"]).read_bytes()
        version = Path(cache[prefix + "_VERSION_FILE"]).read_bytes()
        factory = Path(cache[prefix + "_FACTORY_FILE"]).read_bytes()
        address = bytes.fromhex(cache[prefix + "_CONTROLLER_ADDRESS"].replace(":", ""))
        if (
            len(identity) != 64
            or len(version) != 12
            or len(factory) != 8192
            or len(address) != 6
            or address in (bytes(6), b"\xff" * 6)
        ):
            raise ValueError(
                f"{child} build references have invalid native lengths/address"
            )
        if factory[:64] != identity or struct.unpack_from("<HH", identity, 18) != (
            VID,
            model["pid"],
        ):
            raise ValueError(f"{child} factory/identity references disagree")
        if version[3] != (1 if model["side"] == "R" else 0):
            raise ValueError(f"{child} firmware reference describes the wrong side")
        model.update(
            {
                "identity": identity.hex(),
                "version": version.hex(),
                "factory_extension": factory[64 : 80 + len(models) - 1].hex(),
                "mac_wire": address[::-1].hex(),
                "source_mode": "NONE"
                if neutral
                else cache.get("SWITCH2_BRIDGE_INPUT", "JOYCON2"),
            }
        )
        if neutral:
            user = Path(cache[prefix + "_USER_CALIBRATION_FILE"]).read_bytes()
            if len(user) != 4096:
                raise ValueError(f"{child} user calibration must be 4096 bytes")
            center, source = neutral_calibration(factory, user)
            model.update(
                {
                    "stick_center": center.hex(),
                    "calibration_source": source,
                    "user_read": user[: 80 + len(models) - 1].hex(),
                }
            )
    for field in ("identity", "mac_wire"):
        if len({model[field] for model in models.values()}) != len(models):
            raise ValueError(f"every child requires a unique advertised {field}")
    return models


def expected_status(model: dict[str, Any]) -> bytes:
    version = bytes.fromhex(model["version"])
    return (
        version[:3] + bytes(3) + version[4:7] + b"\0" + bytes.fromhex(model["mac_wire"])
    )


def location(device: Any) -> dict[str, Any]:
    if device.bus is None or device.address is None or not device.port_numbers:
        raise RuntimeError("USB device has no usable physical bus/address/port path")
    return {
        "bus": int(device.bus),
        "address": int(device.address),
        "ports": list(device.port_numbers),
        "vid": int(device.idVendor),
        "pid": int(device.idProduct),
    }


def command(group: int, subcommand: int, payload: bytes = b"") -> bytes:
    # An explicit allowlist, not a general-purpose command injection interface.
    if (group, subcommand) not in {
        (0x03, 0x0D),
        (0x03, 0x0A),
        (0x0C, 0x02),
        (0x0C, 0x04),
        (0x02, 0x04),
        (0x10, 0x01),
        (0x0A, 0x02),
    }:
        raise ValueError("command is outside the qualification allowlist")
    return bytes((group, 0x91, 0, subcommand, 0, len(payload), 0, 0)) + payload


def reply(request: bytes, payload: bytes = b"") -> bytes:
    return bytes((request[0], 1, 0, request[3], 0, 0xF8, 0, 0)) + payload


class Check:
    def __init__(self, args: argparse.Namespace, capture: Any) -> None:
        self.args = args
        self.models = child_models(args.pairs)
        self.children = tuple(self.models)
        self.capture = capture
        self.started = time.monotonic()
        self.deadline = self.started + args.timeout
        self.core: Any = None
        self.util: Any = None
        self.devices: dict[str, Any] = {}
        self.claimed: list[tuple[str, int]] = []
        self.detached: list[tuple[str, int]] = []
        self.current_stage = "dependencies"
        self.last_counter: dict[str, int] = {}
        self.last_controls: dict[str, tuple[bytes, bytes]] = {}
        self.imu_evidence: dict[str, set[bytes]] = {
            side: set() for side in self.children
        }
        self.motion_evidence: dict[str, set[tuple[Any, ...]]] = {
            child: set() for child in self.children
        }
        self.control_evidence: dict[str, set[tuple[bytes, bytes]]] = {
            child: set() for child in self.children
        }
        self.result: dict[str, Any] = {
            "schema_version": 1,
            "neutral_transport_only": args.neutral,
            "pair_count": args.pairs,
            "gameplay_proven": False,
            "physical_latency_proven": False,
            "physical_source_isolation_proven": False,
            "limitations": (
                [
                    "No live controls, donor IMU, Bluetooth routing, console gameplay, or motor action is qualified.",
                    "Neutral reports cannot distinguish same-side HID cross-routing when captured centers match; distinct EP0/bulk identities are checked separately.",
                    "The build cache is a reference, not proof of which firmware is flashed.",
                ]
                if args.neutral
                else [
                    "Console gameplay, physical latency, and physical motor sensation are not qualified.",
                    "Observed distinct input/motion samples do not prove physical source isolation; independently exercise every source throughout the check.",
                    "Both virtual halves of every configured pair must show real activity; an unassigned or neutral pair cannot qualify.",
                    "The build cache is a reference, not proof of which firmware is flashed.",
                ]
            ),
            "success": False,
            "exit_code": 2,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "parameters": {
                "timeout_seconds": args.timeout,
                "duration_seconds": args.duration,
                "usb_timeout_ms": args.usb_timeout_ms,
                "rumble_sample": args.rumble_sample,
                "require_imu": not args.input_only and not args.neutral,
                "input_only": args.input_only,
                "pairs": args.pairs,
                "neutral": args.neutral,
                "capture_trace_on_error": getattr(
                    args, "capture_trace_on_error", False
                ),
            },
            "safety": {
                "pairing_writes": False,
                "profile_access": False,
                "flash_writes": False,
                "usb_reset": False,
                "motor_requested": args.rumble_sample is not None,
                "trace_marker_requested": False,
                "trace_marker_note": "Optional root vendor IN retains volatile trace only; no native initialization, pairing/profile/flash access, or failed-request retry. Device context may predate the failed SETUP; endpoint/physical acceptance is not proven.",
                "initialization_note": "03/0d sets volatile diagnostic host/initialized state only; no 15/* pairing exchange or persistent pairing write is requested.",
            },
            "scope": (
                "Standalone neutral USB transport only; no live input/IMU/gameplay qualification"
                if args.neutral
                else "USB identity/protocol and observed live input/motion evidence, not console gameplay, physical source isolation, or physical latency qualification"
            ),
            "imu_decode_note": (
                "No live IMU evidence is accepted or claimed; neutral reports must have empty IMU/mouse fields."
                if args.neutral
                else "Existing candidate codec; left uses its documented one-byte-earlier IMU boundary. Physical scales are not calibrated by this check."
            ),
            "stages": [],
            "errors": [],
            "seen_roots": [],
            "seen_children": [],
            "devices": {},
            "acl": [],
            "interfaces": [],
            "controls": [],
            "failure_trace": None,
            "bulk": [],
            "active_rounds": [],
            "cleanup": [],
            "streams": {
                side: {
                    "packets": 0,
                    "valid_native_imu": 0,
                    "valid_native_packets": 0,
                    "imu_counter_changes": 0,
                    "valid_neutral_packets": 0,
                    "transport_counter_changes": 0,
                    "last_transport_counter_change_seconds": None,
                    "wrong_side": 0,
                    "unexpected_report": 0,
                    "invalid": 0,
                    "zero_length_imu": 0,
                    "timeouts": 0,
                    "report_ids": {},
                    "imu_lengths": {},
                    "imu_formats": {},
                    "buttons_nonzero": 0,
                    "control_changes": 0,
                    "samples": [],
                    "rejected_samples": [],
                    "first_valid_seconds": None,
                    "last_valid_seconds": None,
                    "last_counter_change_seconds": None,
                    "last_control_change_seconds": None,
                }
                for side in self.children
            },
        }
        self.checkpoint()

    def elapsed(self) -> float:
        return round(time.monotonic() - self.started, 6)

    def checkpoint(self) -> None:
        self.result["elapsed_seconds"] = self.elapsed()
        self.capture.seek(0)
        json.dump(self.result, self.capture, indent=2, allow_nan=False)
        self.capture.write("\n")
        self.capture.truncate()
        self.capture.flush()

    def error(self, message: str, side: str | None = None) -> None:
        self.result["errors"].append(
            {
                "stage": self.current_stage,
                "side": side,
                "at_seconds": self.elapsed(),
                "message": message,
            }
        )

    @contextmanager
    def stage(self, name: str):
        self.current_stage = name
        entry = {"name": name, "status": "running", "started_seconds": self.elapsed()}
        self.result["stages"].append(entry)
        self.checkpoint()
        print(f"[NATIVEHUB] stage={name}", flush=True)
        previous_errors = len(self.result["errors"])
        try:
            yield
        except BaseException as error:
            entry["status"] = "failed"
            self.error(f"{type(error).__name__}: {error}")
            raise
        else:
            entry["status"] = (
                "passed" if len(self.result["errors"]) == previous_errors else "failed"
            )
        finally:
            entry["finished_seconds"] = self.elapsed()
            self.checkpoint()

    def timeout_ms(self, maximum: int | None = None, until: float | None = None) -> int:
        remaining = (
            min(self.deadline, until if until is not None else self.deadline)
            - time.monotonic()
        )
        if remaining <= 0:
            raise TimeoutError(f"bounded deadline reached during {self.current_stage}")
        return max(
            1,
            min(
                self.args.usb_timeout_ms if maximum is None else maximum,
                int(remaining * 1000),
            ),
        )

    def control(
        self,
        owner: str,
        name: str,
        request_type: int,
        request: int,
        value: int,
        index: int,
        length: int,
    ) -> bytes:
        entry = {
            "stage": self.current_stage,
            "side": owner,
            "name": name,
            "setup": [request_type, request, value, index, length],
            "at_seconds": self.elapsed(),
        }
        self.result["controls"].append(entry)
        transfer_attempted = False
        try:
            timeout = self.timeout_ms()
            device = self.devices[owner]
            transfer_attempted = True
            data = bytes(
                device.ctrl_transfer(
                    request_type,
                    request,
                    value,
                    index,
                    length,
                    timeout=timeout,
                )
            )
            entry["response_hex"] = data.hex()
            entry["length"] = len(data)
            return data
        except Exception as error:
            entry["error"] = str(error)
            entry["error_type"] = type(error).__name__
            if (
                transfer_attempted
                and isinstance(error, OSError)
                and getattr(self.args, "capture_trace_on_error", False)
                and owner != "root"
                and owner in self.models
                and self.result["failure_trace"] is None
            ):
                self.capture_failure_trace(owner, entry)
            raise

    def capture_failure_trace(self, owner: str, failed_control: dict[str, Any]) -> None:
        slot = self.models[owner]["port"]
        trace: dict[str, Any] = {
            "status": "pending",
            "request_attempted": False,
            "captured": False,
            "side": owner,
            "slot": slot,
            "failed_control": failed_control.copy(),
            "marker_setup": [
                0xC0,
                NATIVE_HUB_TRACE_REQUEST,
                NATIVE_HUB_TRACE_VALUE,
                slot,
                NATIVE_HUB_TRACE_REPLY_SIZE,
            ],
            "at_seconds": self.elapsed(),
            "response_hex": None,
            "receipt": None,
        }
        # Latch before USB access: root errors cannot recurse or cause a retry.
        self.result["failure_trace"] = trace
        self.result["parameters"]["capture_trace_on_error"] = True
        try:
            try:
                timeout = self.timeout_ms()
            except TimeoutError as error:
                trace["status"] = "deadline_expired"
                trace["error"] = str(error)
                return
            root = self.devices.get("root")
            if root is None:
                trace["status"] = "root_unavailable"
                trace["error"] = "no selected root available for the trace marker"
                return
            trace["request_attempted"] = True
            trace["timeout_ms"] = timeout
            self.result["safety"]["trace_marker_requested"] = True
            # Deliberately bypass control(): this is one diagnostic IN, not recovery.
            data = bytes(root.ctrl_transfer(*trace["marker_setup"], timeout=timeout))
            trace["response_hex"] = data.hex()
            trace["status"] = "malformed"
            if len(data) != NATIVE_HUB_TRACE_REPLY_SIZE:
                raise ValueError("trace reply must be exactly 16 bytes")
            magic, version, status, echoed_slot, reserved, time_us, generation = (
                struct.unpack("<4sBBBBII", data)
            )
            if magic != b"NHTR" or version != 1 or reserved != 0:
                raise ValueError("invalid trace reply header")
            if echoed_slot != slot or status not in (0, 1):
                raise ValueError("invalid trace reply slot or status")
            if status == 1 and (time_us != 0 or generation != 0):
                raise ValueError("busy trace reply must have zero time and generation")
            trace["receipt"] = {
                "version": version,
                "status": status,
                "slot": echoed_slot,
                "time_us": time_us,
                "control_generation": generation,
            }
            trace["captured"] = status == 0
            trace["status"] = "captured" if status == 0 else "busy"
        except (OSError, RuntimeError, ValueError, TypeError, struct.error) as error:
            if trace["status"] != "malformed":
                trace["status"] = "error"
            trace["error"] = f"{type(error).__name__}: {error}"
        finally:
            # Audit failures must not replace the original child-transfer exception.
            try:
                self.checkpoint()
            except (OSError, ValueError, TypeError) as error:
                trace["checkpoint_error"] = f"{type(error).__name__}: {error}"

    def discover(self, *, root_only: bool = False) -> None:
        until = min(self.deadline, self.started + 30)
        while time.monotonic() < until:
            devices = list(self.core.find(find_all=True) or [])
            roots = []
            for device in devices:
                if (device.idVendor, device.idProduct) != (VID, ROOT_PID):
                    continue
                seen = location(device)
                # Read kernel-cached serials before granting access, so genuine
                # Nintendo hubs and unrelated devices receive no USB requests/ACL.
                sysname = f"{seen['bus']}-" + ".".join(map(str, seen["ports"]))
                try:
                    serial = (
                        (Path("/sys/bus/usb/devices") / sysname / "serial")
                        .read_text()
                        .strip()
                    )
                except OSError as error:
                    seen["serial_error"] = str(error)
                    serial = ""
                seen["serial"] = serial
                if seen not in self.result["seen_roots"]:
                    self.result["seen_roots"].append(seen)
                if serial.startswith(SERIAL_PREFIX):
                    roots.append((device, seen))
            if len(roots) > 1:
                raise RuntimeError(
                    "multiple switch-pico 057e:2068 hubs; refusing ambiguous target"
                )
            if roots:
                root, root_info = roots[0]
                if root_only:
                    self.devices = {"root": root}
                    self.result["devices"] = {"root": root_info}
                    return
                selected = {}
                direct_children = []
                for device in devices:
                    ports = tuple(device.port_numbers or ())
                    if device.bus != root.bus or ports[:-1] != tuple(root.port_numbers):
                        continue
                    seen = location(device)
                    direct_children.append(seen)
                    if seen not in self.result["seen_children"]:
                        self.result["seen_children"].append(seen)
                    for side in self.children:
                        model = self.models[side]
                        if (device.idVendor, device.idProduct, ports[-1]) == (
                            VID,
                            model["pid"],
                            model["port"],
                        ):
                            if side in selected:
                                raise RuntimeError(
                                    f"duplicate {side} child on the expected hub port"
                                )
                            selected[side] = device
                if len(selected) == len(self.children):
                    if len(direct_children) != len(self.children):
                        raise RuntimeError(
                            "target hub has unexpected additional direct children"
                        )
                    self.devices = {"root": root, **selected}
                    if (
                        len({device.address for device in self.devices.values()})
                        != len(self.children) + 1
                    ):
                        raise RuntimeError(
                            "root and children do not have distinct USB addresses"
                        )
                    self.result["devices"] = {
                        "root": root_info,
                        **{
                            side: {
                                **location(selected[side]),
                                "side": self.models[side]["side"],
                                "pair": self.models[side]["pair"],
                                "capture_prefix": self.models[side]["capture_prefix"],
                            }
                            for side in self.children
                        },
                    }
                    return
            time.sleep(min(0.1, max(0, until - time.monotonic())))
        if root_only:
            raise TimeoutError(
                "discovery: expected one switch-pico 057e:2068 root; "
                "no child enumeration is required; inspect seen_roots"
            )
        raise TimeoutError(
            "discovery: expected one switch-pico 057e:2068 with "
            + ", ".join(
                f"{child} {model['pid']:04x} at port {model['port']}"
                for child, model in self.models.items()
            )
            + " on the same hub path; inspect seen_roots/seen_children"
        )

    def permissions(self) -> None:
        for owner, device in self.devices.items():
            node = f"/dev/bus/usb/{device.bus:03d}/{device.address:03d}"
            entry = {"owner": owner, "node": node, "granted": False}
            self.result["acl"].append(entry)
            try:
                subprocess.run(
                    ["sudo", "-n", "setfacl", "-m", f"u:{os.getuid()}:rw", node],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout_ms(3000) / 1000,
                )
                entry["granted"] = True
            except subprocess.CalledProcessError as error:
                entry["error"] = error.stderr.strip()
                raise RuntimeError(
                    f"cannot grant access to {owner} {node}: {error.stderr.strip()}"
                ) from error
        data = self.control("root", "device_descriptor", 0x80, 6, 0x0100, 0, 18)
        if (
            len(data) != 18
            or data[:2] != b"\x12\x01"
            or data[4] != 9
            or struct.unpack_from("<HH", data, 8) != (VID, ROOT_PID)
        ):
            raise RuntimeError(
                "selected root returned an incorrect hub device descriptor"
            )
        languages = self.control("root", "string_languages", 0x80, 6, 0x0300, 0, 255)
        if len(languages) < 4 or languages[1] != 3 or not data[16]:
            raise RuntimeError("root has no readable serial string language")
        serial = self.control(
            "root",
            "serial",
            0x80,
            6,
            0x0300 | data[16],
            int.from_bytes(languages[2:4], "little"),
            255,
        )
        if len(serial) < 2 or serial[0] != len(serial) or serial[1] != 3:
            raise RuntimeError("invalid root USB serial descriptor")
        if serial[2:].decode("utf-16-le") != self.result["devices"]["root"]["serial"]:
            raise RuntimeError("root USB serial differs from selected sysfs identity")
        if self.args.neutral:
            hub = self.control("root", "hub_descriptor", 0xA0, 6, 0x2900, 0, 255)
            if len(hub) != 9 or hub[:3] != bytes((9, 0x29, len(self.children))):
                raise RuntimeError(
                    "root hub descriptor does not advertise the requested child count"
                )

    def claim(self) -> None:
        for side in self.children:
            # GET_CONFIGURATION only: do not reset USB or set a configuration.
            if self.control(side, "active_configuration", 0x80, 8, 0, 0, 1) != b"\x01":
                raise RuntimeError(
                    f"{side} is not already configured as configuration 1"
                )
            device = self.devices[side]
            for interface in (0, 1):
                entry = {
                    "side": side,
                    "interface": interface,
                    "detached": False,
                    "claimed": False,
                }
                self.result["interfaces"].append(entry)
                self.timeout_ms()
                if device.is_kernel_driver_active(interface):
                    device.detach_kernel_driver(interface)
                    self.detached.append((side, interface))
                    entry["detached"] = True
                self.util.claim_interface(device, interface)
                self.claimed.append((side, interface))
                entry["claimed"] = True

    def descriptors(self, side: str, *, report: bool = True) -> None:
        model = self.models[side]
        device = self.control(side, "device_descriptor", 0x80, 6, 0x0100, 0, 18)
        if (
            len(device) != 18
            or device[:2] != b"\x12\x01"
            or device[7] != 64
            or device[17] != 1
            or struct.unpack_from("<HH", device, 8) != (VID, model["pid"])
        ):
            raise RuntimeError(f"{side} device descriptor leaked or is malformed")
        config = self.control(side, "configuration_descriptor", 0x80, 6, 0x0200, 0, 255)
        if len(config) != 80 or config[:5] != b"\x09\x02\x50\x00\x02" or config[5] != 1:
            raise RuntimeError(
                f"{side} requires its own 80-byte, two-interface standalone configuration"
            )
        interfaces: dict[int, tuple[int, int]] = {}
        endpoints: dict[int, list[tuple[int, int, int]]] = {}
        current = None
        hid_length = None
        offset = 9
        while offset < len(config):
            size = config[offset]
            if size < 2 or offset + size > len(config):
                raise RuntimeError(f"{side} malformed configuration item at {offset}")
            item = config[offset : offset + size]
            if item[1] == 4:
                if size != 9 or item[3] != 0 or item[2] in interfaces:
                    raise RuntimeError(f"{side} duplicate or non-native interface")
                current = item[2]
                interfaces[current] = (item[5], item[4])
                endpoints[current] = []
            elif item[1] == 5:
                if size != 7 or current is None:
                    raise RuntimeError(f"{side} malformed endpoint")
                endpoints[current].append(
                    (item[2], item[3], int.from_bytes(item[4:6], "little"))
                )
            elif item[1] == 0x21:
                if size != 9 or current != 0 or item[5:7] != b"\x01\x22":
                    raise RuntimeError(f"{side} malformed HID descriptor")
                hid_length = int.from_bytes(item[7:9], "little")
            offset += size
        expected_endpoints = {
            0: [(0x81, 3, 64), (0x01, 3, 64)],
            1: [(0x02, 2, 64), (0x82, 2, 64)],
        }
        if (
            interfaces != {0: (3, 2), 1: (255, 2)}
            or endpoints != expected_endpoints
            or hid_length != 100
        ):
            raise RuntimeError(f"{side} is not HID0/vendor1 with native EP81/01/82/02")
        if not report:
            return
        hid = self.control(
            side, "hid_report_descriptor", 0x81, 6, 0x2200, 0, hid_length
        )
        if len(hid) != hid_length:
            raise RuntimeError(f"{side} truncated HID report descriptor")
        inputs: dict[int, int] = {}
        outputs: dict[int, int] = {}
        report_id = report_size = report_count = 0
        offset = 0
        while offset < len(hid):
            prefix = hid[offset]
            offset += 1
            size = 4 if prefix & 3 == 3 else prefix & 3
            if prefix == 0xFE or offset + size > len(hid):
                raise RuntimeError(f"{side} malformed/unsupported HID item")
            value = int.from_bytes(hid[offset : offset + size], "little")
            offset += size
            kind = prefix & 0xFC
            if kind == 0x74:
                report_size = value
            elif kind == 0x94:
                report_count = value
            elif kind == 0x84:
                report_id = value
            elif kind in (0x80, 0x90):
                target = inputs if kind == 0x80 else outputs
                target[report_id] = (
                    target.get(report_id, 0) + report_size * report_count
                )
        if inputs != {5: 504, model["report"]: 504} or outputs != {1: 504}:
            raise RuntimeError(
                f"{side} HID advertises the wrong side/report lengths: {inputs}, {outputs}"
            )

    def identities(self, side: str, round_number: int = 0) -> None:
        # Alternate device and explicit interface recipients; both child-local
        # indexes must resolve to this USB address, never to the sibling.
        request_type, index = (0xC0, 0) if round_number % 2 == 0 else (0xC1, 1)
        identity = self.control(
            side, "vendor_identity03", request_type, 3, 0, index, 64
        )
        status = self.control(side, "vendor_version02", request_type, 2, 0, index, 64)
        self.result["devices"][side]["identity03_hex"] = identity.hex()
        self.result["devices"][side]["version02_hex"] = status.hex()
        if identity != bytes.fromhex(self.models[side]["identity"]):
            raise RuntimeError(
                f"{side} vendor 03 does not match its distinct captured factory identity"
            )
        if status != expected_status(self.models[side]):
            raise RuntimeError(
                f"{side} vendor 02 must be 16 bytes with wire MAC tail {self.models[side]['mac_wire']}"
            )
        # Full/short EP0 reads check transfer length/context teardown.
        short_length = (1, 7, 15)[round_number % 3]
        short = self.control(
            side, "vendor_version02_short", request_type, 2, 0, index, short_length
        )
        if short != status[:short_length]:
            raise RuntimeError(f"{side} short vendor read leaked/truncated incorrectly")

    def interleaved_identities(self, round_number: int) -> None:
        # Complete short reads on every address before full reads in reverse
        # order, including while all child bulk replies are pending.
        request_type, index = (0xC0, 0) if round_number % 2 == 0 else (0xC1, 1)
        order = (
            self.children if round_number % 2 == 0 else tuple(reversed(self.children))
        )
        for slot, side in enumerate(order):
            length = (1, 7, 15)[(round_number + slot) % 3]
            for request, expected in (
                (3, bytes.fromhex(self.models[side]["identity"])),
                (2, expected_status(self.models[side])),
            ):
                actual = self.control(
                    side,
                    f"vendor{request:02x}_interleaved_short",
                    request_type,
                    request,
                    0,
                    index,
                    length,
                )
                if actual != expected[:length]:
                    raise RuntimeError(
                        f"{side} interleaved short EP0 identity/status leaked"
                    )
        for side in reversed(order):
            self.identities(side, round_number)

    def exchange_children(
        self, requests: dict[str, tuple[bytes, bytes]], round_number: int = 0
    ) -> None:
        order = (
            self.children if round_number % 2 == 0 else tuple(reversed(self.children))
        )
        entries = {}
        # All devices have pending commands before any IN is consumed. Reverse
        # completion order to expose global reply-buffer reuse.
        for side in order:
            request, expected = requests[side]
            if request[:4] == b"\x0a\x91\x00\x02" and (
                self.args.neutral or self.args.rumble_sample is None
            ):
                raise RuntimeError(
                    "motor command requires --rumble-sample and live mode"
                )
            entry = {
                "stage": self.current_stage,
                "side": side,
                "at_seconds": self.elapsed(),
                "request_hex": request.hex(),
                "expected_hex": expected.hex(),
                "segments_hex": [],
                "response_hex": "",
                "matched": False,
            }
            self.result["bulk"].append(entry)
            entries[side] = entry
            written = self.devices[side].write(0x02, request, timeout=self.timeout_ms())
            entry["written"] = int(written)
            if written != len(request):
                raise RuntimeError(
                    f"{side} short native bulk OUT ({written}/{len(request)})"
                )
        if self.args.neutral:
            self.interleaved_identities(round_number)
        for side in reversed(order):
            expected = requests[side][1]
            actual = bytearray()
            entry = entries[side]
            while len(actual) < len(expected):
                segment = bytes(
                    self.devices[side].read(0x82, 64, timeout=self.timeout_ms())
                )
                entry["segments_hex"].append(segment.hex())
                remaining = len(expected) - len(actual)
                actual.extend(segment)
                entry["response_hex"] = actual.hex()
                if len(segment) != min(64, remaining):
                    raise RuntimeError(
                        f"{side} bulk segment length {len(segment)}, expected {min(64, remaining)}"
                    )
            if actual != expected:
                raise RuntimeError(
                    f"{side} bulk header/length/content mismatch; possible cross-device reply leakage"
                )
            entry["matched"] = True

    def initialize(self) -> None:
        for operation in (
            "initialize",
            "select_report",
            "feature_mask",
            "feature_enable",
        ):
            requests = {}
            for side in self.children:
                if operation == "initialize":
                    request = command(
                        3,
                        0x0D,
                        b"\x01\x00"
                        + bytes.fromhex(self.models[side]["diagnostic_host"]),
                    )
                    response = reply(request, b"\x01\x00\x00\x00")
                elif operation == "select_report":
                    request = command(
                        3, 0x0A, bytes((self.models[side]["report"], 0, 0, 0))
                    )
                    response = reply(request)
                else:
                    # 0x01 buttons, 0x02 sticks, 0x04 IMU; not vibration/mouse/NFC.
                    request = command(
                        0x0C,
                        2 if operation == "feature_mask" else 4,
                        b"\x07\x00\x00\x00",
                    )
                    response = reply(request, bytes(4))
                requests[side] = (request, response)
            self.exchange_children(requests)

    def queries(self, round_number: int) -> None:
        request = command(0x10, 1)
        self.exchange_children(
            {
                side: (
                    request,
                    reply(request, bytes.fromhex(self.models[side]["version"])),
                )
                for side in self.children
            },
            round_number,
        )
        for region in ("factory", "user") if self.args.neutral else ("factory",):
            requests = {}
            for side_index, side in enumerate(self.children):
                # Distinct offsets isolate the echoed address even when same-side
                # firmware versions or captured calibration bytes are identical.
                offset = (round_number + side_index) % len(self.children)
                address = (0x13000 if region == "factory" else 0x1FC000) + offset
                request = command(
                    2, 4, b"\x50\x7e\x00\x00" + struct.pack("<I", address)
                )
                reference = bytes.fromhex(
                    self.models[side]["identity"]
                    + self.models[side]["factory_extension"]
                    if region == "factory"
                    else self.models[side]["user_read"]
                )
                payload = (
                    b"\x50\x00\x00\x00"
                    + struct.pack("<I", address)
                    + reference[offset : offset + 80]
                )
                if len(payload) != 88:
                    raise RuntimeError(
                        "captured memory reference is not an 80-byte read"
                    )
                requests[side] = (request, reply(request, payload))
            self.exchange_children(requests, round_number)

    def neutral_report(
        self, side: str, payload: bytes, sample: dict[str, Any]
    ) -> str | None:
        stream = self.result["streams"][side]
        center = bytes.fromhex(self.models[side]["stick_center"])
        if any(payload[2:4]):
            rejection = "neutral transport emitted nonzero buttons"
        elif payload[5:8] != center:
            rejection = "neutral transport stick does not match the selected captured calibration center"
        elif any(payload[8:]):
            rejection = "neutral transport emitted nonzero reserved/mouse/IMU fields"
        else:
            rejection = None
        if rejection:
            stream["invalid"] += 1
            return rejection
        now = self.elapsed()
        counter = payload[0]
        sample["transport_counter"] = counter
        stream["valid_neutral_packets"] += 1
        stream["zero_length_imu"] += 1
        stream["last_valid_seconds"] = now
        if stream["first_valid_seconds"] is None:
            stream["first_valid_seconds"] = now
        if side in self.last_counter and self.last_counter[side] != counter:
            stream["transport_counter_changes"] += 1
            stream["last_transport_counter_change_seconds"] = now
        self.last_counter[side] = counter
        samples = stream["samples"]
        if len(samples) < 4 or (
            len(samples) < 32 and now - samples[-1]["at_seconds"] >= 0.5
        ):
            samples.append(sample)
        stream["last_sample"] = sample
        return None

    def poll(self, side: str, until: float) -> None:
        stream = self.result["streams"][side]
        remaining = min(self.deadline, until) - time.monotonic()
        if remaining <= 0:
            return
        try:
            packet = bytes(
                self.devices[side].read(
                    0x81, 64, timeout=max(1, min(10, int(remaining * 1000)))
                )
            )
        except self.core.USBTimeoutError:
            stream["timeouts"] += 1
            return
        stream["packets"] += 1
        report_id = packet[0] if packet else -1
        report_key = f"{report_id:02x}" if report_id >= 0 else "empty"
        stream["report_ids"][report_key] = stream["report_ids"].get(report_key, 0) + 1
        sample = {
            "at_seconds": self.elapsed(),
            "stage": self.current_stage,
            "packet_hex": packet.hex(),
        }
        rejection = None
        if (
            report_id
            == MODELS["L" if self.models[side]["side"] == "R" else "R"]["report"]
        ):
            stream["wrong_side"] += 1
            rejection = "wrong-side native report on this device's HID pipe"
        elif report_id != self.models[side]["report"]:
            stream["unexpected_report"] += 1
            rejection = (
                "unexpected report ID; only this side's native 07/08 is accepted"
            )
        elif len(packet) != 64:
            stream["invalid"] += 1
            rejection = f"native report is {len(packet)} bytes, expected 64"
        else:
            payload = packet[1:]
            stream["valid_native_packets"] += 1
            controls = (payload[2:4], payload[5:8])
            sample["buttons_hex"], sample["stick_hex"] = (
                part.hex() for part in controls
            )
            if any(controls[0]):
                stream["buttons_nonzero"] += 1
            if side in self.last_controls and self.last_controls[side] != controls:
                if any(controls[0]) and len(self.control_evidence[side]) < 512:
                    self.control_evidence[side].add(controls)
                stream["control_changes"] += 1
                stream["last_control_change_seconds"] = self.elapsed()
            self.last_controls[side] = controls
            length_offset = 14 if self.models[side]["side"] == "L" else 15
            length = payload[length_offset]
            key = str(length)
            stream["imu_lengths"][key] = stream["imu_lengths"].get(key, 0) + 1
            if self.args.neutral:
                rejection = self.neutral_report(side, payload, sample)
            elif length == 0:
                stream["zero_length_imu"] += 1
                if self.args.input_only:
                    if len(stream["samples"]) < 4:
                        stream["samples"].append(sample)
                    stream["last_sample"] = sample
                else:
                    rejection = "zero-length IMU: inactive donor/neutral fallback is not live motion"
            else:
                try:
                    block = (
                        native_block({"native_hex": packet.hex()})
                        if self.models[side]["side"] == "R"
                        else payload[length_offset + 1 : length_offset + 1 + length]
                    )
                    decoded = decode_block(block)
                    if not any(
                        value
                        for vector in decoded["accelerations"]
                        for value in vector["raw"]
                    ):
                        raise ValueError(
                            "all-zero acceleration is not donor IMU evidence"
                        )
                    if not all(
                        math.isfinite(value) for value in decoded["quaternion_wire"]
                    ):
                        raise ValueError("non-finite decoded quaternion")
                    sample["imu"] = decoded
                except (ValueError, AssertionError) as error:
                    stream["invalid"] += 1
                    rejection = f"native IMU decode failed: {error}"
                else:
                    now = self.elapsed()
                    stream["valid_native_imu"] += 1
                    stream["last_valid_seconds"] = now
                    if stream["first_valid_seconds"] is None:
                        stream["first_valid_seconds"] = now
                    counter = decoded["counter_ticks"]
                    if side in self.last_counter and self.last_counter[side] != counter:
                        stream["imu_counter_changes"] += 1
                        stream["last_counter_change_seconds"] = now
                    self.last_counter[side] = counter
                    if len(self.imu_evidence[side]) < 512:
                        self.imu_evidence[side].add(block)
                    if len(self.motion_evidence[side]) < 512:
                        # Counters, elapsed ticks and temperature are not motion.
                        self.motion_evidence[side].add(
                            (
                                tuple(decoded["quaternion_wire"]),
                                tuple(
                                    tuple(vector["raw"])
                                    for vector in decoded["accelerations"]
                                ),
                                tuple(
                                    tuple(vector["raw"])
                                    for vector in decoded["rotation_triplets"]
                                ),
                            )
                        )
                    format_key = f"{decoded['format']:02x}"
                    first_format = format_key not in stream["imu_formats"]
                    stream["imu_formats"][format_key] = (
                        stream["imu_formats"].get(format_key, 0) + 1
                    )
                    samples = stream["samples"]
                    if (
                        len(samples) < 4
                        or first_format
                        or (
                            len(samples) < 32 and now - samples[-1]["at_seconds"] >= 0.5
                        )
                    ):
                        samples.append(sample)
                    stream["last_sample"] = sample
        if rejection:
            sample["reason"] = rejection
            if len(stream["rejected_samples"]) < 8:
                stream["rejected_samples"].append(sample)

    def poll_children(self, until: float, reverse: bool = False) -> None:
        for side in reversed(self.children) if reverse else self.children:
            if time.monotonic() >= until:
                break
            self.poll(side, until)

    def counts(self) -> dict[str, int]:
        return {
            side: self.result["streams"][side][
                "valid_neutral_packets"
                if self.args.neutral
                else "valid_native_packets"
                if self.args.input_only
                else "valid_native_imu"
            ]
            for side in self.children
        }

    def streams_ready(self) -> None:
        until = min(self.deadline, time.monotonic() + 60)
        iteration = 0
        while time.monotonic() < until:
            self.poll_children(until, bool(iteration % 2))
            iteration += 1
            if all(
                (
                    stream["valid_neutral_packets"] >= 3
                    and stream["transport_counter_changes"] >= 2
                )
                if self.args.neutral
                else (stream["buttons_nonzero"] > 0 and stream["control_changes"] >= 2)
                if self.args.input_only
                else (
                    stream["valid_native_imu"] >= 3
                    and stream["imu_counter_changes"] >= 2
                    and (
                        self.args.pairs == 1
                        or (
                            stream["buttons_nonzero"] > 0
                            and stream["control_changes"] >= 2
                        )
                    )
                )
                for stream in self.result["streams"].values()
            ):
                return
        details = "; ".join(
            f"{side}: neutral={stream['valid_neutral_packets']}, transport_changes={stream['transport_counter_changes']}, imu={stream['valid_native_imu']}, counter_changes={stream['imu_counter_changes']}, buttons={stream['buttons_nonzero']}, control_changes={stream['control_changes']}, zero_imu={stream['zero_length_imu']}, invalid={stream['invalid']}, timeouts={stream['timeouts']}"
            for side, stream in self.result["streams"].items()
        )
        raise RuntimeError(
            f"neutral reports/counters did not become ready on every child; {details}"
            if self.args.neutral
            else f"sources did not satisfy live-input evidence on every child during the manual-input window; both pairs must be assigned and independently exercised for --pairs 2; {details}; no pairing/wake/reset was attempted"
        )

    def active(self) -> None:
        until = min(self.deadline, time.monotonic() + self.args.duration)
        initial_counts = self.counts()
        initial_controls = {
            child: self.result["streams"][child]["control_changes"]
            for child in self.children
        }
        for evidence in (
            self.imu_evidence,
            self.motion_evidence,
            self.control_evidence,
        ):
            for samples in evidence.values():
                samples.clear()
        change_key = (
            "transport_counter_changes"
            if self.args.neutral
            else "control_changes"
            if self.args.input_only
            else "imu_counter_changes"
        )
        initial_changes = {
            side: self.result["streams"][side][change_key] for side in self.children
        }
        next_query = time.monotonic()
        round_number = 0
        pending = None
        while time.monotonic() < until:
            if time.monotonic() >= next_query and until - time.monotonic() >= 0.25:
                if pending is not None:
                    pending["after"] = self.counts()
                    if self.args.neutral:
                        pending["after_changes"] = {
                            side: self.result["streams"][side][change_key]
                            for side in self.children
                        }
                pending = {
                    "round": round_number,
                    "before": self.counts(),
                    "started_seconds": self.elapsed(),
                }
                if self.args.neutral:
                    pending["before_changes"] = {
                        side: self.result["streams"][side][change_key]
                        for side in self.children
                    }
                self.result["active_rounds"].append(pending)
                for side in (
                    self.children if round_number % 2 == 0 else reversed(self.children)
                ):
                    self.descriptors(side, report=round_number % 2 == 0)
                    self.identities(side, round_number)
                self.queries(round_number)
                pending["reads_matched"] = True
                next_query = time.monotonic() + 0.25
                round_number += 1
                self.checkpoint()
            self.poll_children(until, bool(round_number % 2))
        if pending is not None:
            pending["after"] = self.counts()
            if self.args.neutral:
                pending["after_changes"] = {
                    side: self.result["streams"][side][change_key]
                    for side in self.children
                }
        if round_number < 2:
            self.error(
                "fewer than two interleaved control/bulk rounds completed during streaming"
            )
        if not any(
            all(
                entry.get("after", {}).get(side, 0) > entry["before"][side]
                and (
                    not self.args.neutral
                    or entry.get("after_changes", {}).get(side, 0)
                    > entry["before_changes"][side]
                )
                for side in self.children
            )
            for entry in self.result["active_rounds"]
            if entry.get("reads_matched")
        ):
            self.error(
                "no interleaved control/bulk round was bracketed by valid reports and advancing counters on every child"
                if self.args.neutral
                else "no interleaved control/bulk round was bracketed by valid input from every child"
            )
        if self.args.neutral:
            self.result["imu_isolation"] = {
                "policy": "not_proven_neutral_transport_only"
            }
        else:
            pair_results = {}
            self.result["imu_isolation"] = {
                "sample_limit_per_child": 512,
                "policy": "not_required_input_only"
                if self.args.input_only
                else "pair_local_source_policy",
                "pairs": pair_results,
            }
            for offset in range(0, len(self.children), 2):
                pair_children = self.children[offset : offset + 2]
                right, left = pair_children
                shared = self.imu_evidence[right] & self.imu_evidence[left]
                shared_source = self.models[right].get("source_mode") in (
                    "DUALSENSE",
                    "GAMEPAD",
                )
                pair_results[self.models[right]["pair"]] = {
                    "policy": "not_required_input_only"
                    if self.args.input_only
                    else "shared_physical_source"
                    if shared_source
                    else "independent_physical_sources",
                    "identical_blocks_seen_on_both_sides": len(shared),
                    "unique_blocks": {
                        child: len(self.imu_evidence[child]) for child in pair_children
                    },
                    "side_exclusive_blocks": {
                        child: len(self.imu_evidence[child] - shared)
                        for child in pair_children
                    },
                    "unique_motion_samples": {
                        child: len(self.motion_evidence[child])
                        for child in pair_children
                    },
                }
                if not self.args.input_only:
                    for child in pair_children:
                        evidence = self.imu_evidence[child]
                        if not shared_source:
                            evidence = evidence - shared
                        if len(evidence) < 2 or len(self.motion_evidence[child]) < 2:
                            self.error(
                                "IMU evidence is frozen or lacks deliberate motion"
                                if shared_source
                                else "donor IMU evidence is frozen or duplicated within its virtual pair",
                                child,
                            )
            if self.args.pairs == 2:
                comparison = {
                    "policy": "distinct_exercised_samples_required_not_physical_source_isolation",
                    "physical_source_isolation_proven": False,
                    "children": {},
                }
                self.result["inter_pair_evidence"] = comparison
                for child in self.children:
                    other_pair = [
                        other
                        for other in self.children
                        if self.models[other]["pair"] != self.models[child]["pair"]
                    ]
                    other_side = next(
                        other
                        for other in other_pair
                        if self.models[other]["side"] == self.models[child]["side"]
                    )
                    controls = (
                        self.control_evidence[child] - self.control_evidence[other_side]
                    )
                    motion = self.motion_evidence[child] - set().union(
                        *(self.motion_evidence[other] for other in other_pair)
                    )
                    comparison["children"][child] = {
                        "pair_exclusive_pressed_control_states": len(controls),
                        "pair_exclusive_motion_samples": len(motion),
                    }
                    if len(controls) < 2:
                        self.error(
                            "insufficient pair-distinct pressed control states; independently press buttons and vary sticks on both controllers, including every R/L half; neutral/static or mirrored input cannot qualify",
                            child,
                        )
                    if not self.args.input_only and len(motion) < 2:
                        self.error(
                            "insufficient pair-distinct motion; deliberately move both sources differently; shared/static samples or advancing counters alone cannot qualify",
                            child,
                        )
        for side in self.children:
            stream = self.result["streams"][side]
            if self.args.pairs == 2 and not self.args.neutral:
                last_control = stream["last_control_change_seconds"]
                if (
                    stream["control_changes"] - initial_controls[side] < 2
                    or last_control is None
                    or self.elapsed() - last_control > 2
                ):
                    self.error(
                        "real controls must keep changing on every child of both pairs during active reads",
                        side,
                    )
            if (
                self.counts()[side] - initial_counts[side] < 3
                or stream[change_key] - initial_changes[side] < 2
            ):
                self.error(
                    "insufficient fresh neutral reports/transport counters during active control/bulk reads"
                    if self.args.neutral
                    else "insufficient fresh controller transitions"
                    if self.args.input_only
                    else "insufficient fresh native source IMU during active control/bulk reads",
                    side,
                )
            last_change = stream[
                "last_transport_counter_change_seconds"
                if self.args.neutral
                else "last_control_change_seconds"
                if self.args.input_only
                else "last_counter_change_seconds"
            ]
            if last_change is None or self.elapsed() - last_change > 2:
                self.error(
                    "neutral transport counter stopped advancing before streaming finished"
                    if self.args.neutral
                    else "source stopped advancing before streaming finished",
                    side,
                )
            if stream["wrong_side"] or stream["unexpected_report"] or stream["invalid"]:
                self.error(
                    f"rejected wrong-side={stream['wrong_side']}, unexpected={stream['unexpected_report']}, malformed={stream['invalid']} HID packets",
                    side,
                )
        if time.monotonic() >= self.deadline:
            raise TimeoutError(
                "overall timeout interrupted the required streaming duration"
            )

    def cleanup(self) -> None:
        self.current_stage = "cleanup"
        for action, tracked in (("release", self.claimed), ("reattach", self.detached)):
            for side, interface in reversed(tracked):
                entry = {
                    "action": action,
                    "side": side,
                    "interface": interface,
                    "success": False,
                }
                self.result["cleanup"].append(entry)
                try:
                    if action == "release":
                        self.util.release_interface(self.devices[side], interface)
                    else:
                        self.devices[side].attach_kernel_driver(interface)
                    entry["success"] = True
                except (OSError, RuntimeError, NotImplementedError) as error:
                    entry["error"] = str(error)
                    self.error(f"{action} interface {interface}: {error}", side)
        if self.util is not None:
            for owner, device in self.devices.items():
                try:
                    self.util.dispose_resources(device)
                except (OSError, RuntimeError, NotImplementedError) as error:
                    self.error(f"dispose USB resources: {error}", owner)

    def run(self) -> int:
        completed = False
        try:
            with self.stage("dependencies"):
                import usb.core
                import usb.util

            with self.stage("references"):
                self.models = model_references(
                    self.args.build_dir,
                    require_imu=not self.args.input_only and not self.args.neutral,
                    pairs=self.args.pairs,
                    neutral=self.args.neutral,
                )
                self.result["references"] = {
                    "build_dir": str(self.args.build_dir.resolve()),
                    "children": {
                        side: {
                            field: model[field]
                            for field in (
                                "pair",
                                "side",
                                "port",
                                "capture_prefix",
                                "mac_wire",
                                "source_mode",
                                "stick_center",
                                "calibration_source",
                            )
                            if field in model
                        }
                        for side, model in self.models.items()
                    },
                }
                self.core, self.util = usb.core, usb.util
            with self.stage("discovery"):
                self.discover()
            with self.stage("permissions_and_root_identity"):
                self.permissions()
            with self.stage("claim_child_interfaces"):
                self.claim()
            with self.stage("descriptor_and_ep0_isolation"):
                for round_number in range(20):
                    for side in (
                        self.children
                        if round_number % 2 == 0
                        else reversed(self.children)
                    ):
                        self.descriptors(side, report=round_number in (0, 19))
                        self.identities(side, round_number)
            with self.stage("native_initialization"):
                self.initialize()
            with self.stage("bulk_isolation"):
                for round_number in range(2):
                    self.queries(round_number)
            with self.stage(
                "neutral_stream_startup" if self.args.neutral else "donor_startup"
            ):
                self.streams_ready()
            with self.stage(
                "active_neutral_transport_and_read_isolation"
                if self.args.neutral
                else "active_input_and_read_isolation"
            ):
                self.active()
            if (
                not self.args.neutral
                and self.args.rumble_sample is not None
                and not self.result["errors"]
            ):
                with self.stage("explicit_motor_sample_ack"):
                    request = command(
                        0x0A, 2, bytes((self.args.rumble_sample, 0, 0, 0))
                    )
                    self.exchange_children(
                        {side: (request, reply(request)) for side in self.children}
                    )
                    self.result["motor_result"] = (
                        "native sample ACK received for each side; physical sensation is not measured"
                    )
            else:
                self.result["stages"].append(
                    {"name": "explicit_motor_sample_ack", "status": "skipped"}
                )
                if self.args.neutral:
                    self.result["motor_result"] = (
                        "not_requested_neutral_transport_only; no motor action or ACK qualified"
                    )
            completed = True
        except KeyboardInterrupt:
            self.result["interrupted"] = True
        except (
            OSError,
            RuntimeError,
            ValueError,
            TypeError,
            ImportError,
            subprocess.SubprocessError,
            struct.error,
        ) as error:
            # The stage records partial responses; retain the terminal cause too.
            self.result["failure"] = str(error)
        finally:
            try:
                with self.stage("cleanup"):
                    self.cleanup()
            finally:
                self.result["success"] = completed and not self.result["errors"]
                self.result["exit_code"] = 0 if self.result["success"] else 2
                failed = sorted({entry["stage"] for entry in self.result["errors"]})
                streams = self.result["streams"]
                self.result["child_results"] = {
                    side: {
                        "pair": self.models[side]["pair"],
                        "side": self.models[side]["side"],
                        "port": self.models[side]["port"],
                        "qualified": self.result["success"],
                        "neutral_transport_only": self.args.neutral,
                        "live_input_proven": self.result["success"]
                        and not self.args.neutral,
                        "live_imu_proven": self.result["success"]
                        and not self.args.neutral
                        and not self.args.input_only,
                        "gameplay_proven": False,
                        "physical_latency_proven": False,
                        "physical_source_isolation_proven": False,
                        "valid_reports": self.counts()[side],
                        "transport_counter_changes": streams[side][
                            "transport_counter_changes"
                        ],
                        "control_changes": streams[side]["control_changes"],
                        "imu_counter_changes": streams[side]["imu_counter_changes"],
                        "last_sample": streams[side].get("last_sample"),
                        "errors": [
                            entry
                            for entry in self.result["errors"]
                            if entry["side"] in (side, None, "root")
                        ],
                    }
                    for side in self.children
                }
                if self.args.neutral:
                    stream_summary = (
                        f"NEUTRAL_TRANSPORT_ONLY pairs={self.args.pairs} gameplay=not_proven "
                        + " ".join(
                            f"{side}={streams[side]['valid_neutral_packets']} {side}_transport_counter_changes={streams[side]['transport_counter_changes']}"
                            for side in self.children
                        )
                        + " "
                    )
                else:
                    stream_summary = (
                        f"{'LIVE_INPUT_ONLY' if self.args.input_only else 'LIVE_INPUT_AND_IMU'} "
                        f"pairs={self.args.pairs} gameplay=not_proven physical_latency=not_proven "
                        + " ".join(
                            f"{child}={self.counts()[child]} {child}_control_changes={streams[child]['control_changes']} {child}_imu_counter_changes={streams[child]['imu_counter_changes']}"
                            for child in self.children
                        )
                        + " "
                    )
                self.result["summary"] = (
                    f"{'PASS' if self.result['success'] else 'FAIL'} "
                    f"{stream_summary}"
                    f"active_rounds={len(self.result['active_rounds'])} "
                    f"errors={len(self.result['errors'])} failed_stages={','.join(failed) or 'none'}"
                )
                self.checkpoint()
                print(f"[NATIVEHUB] {self.result['summary']}", flush=True)
                print(f"[NATIVEHUB] capture={self.args.output}", flush=True)
        return self.result["exit_code"]


class BootselRecovery(Check):
    """Reuse bounded auditing/root identity checks, never the qualification run."""

    def __init__(self, args: argparse.Namespace, capture: Any) -> None:
        self.args = args
        self.capture = capture
        self.started = time.monotonic()
        self.deadline = self.started + args.timeout
        self.core: Any = None
        self.util: Any = None
        self.devices: dict[str, Any] = {}
        self.claimed: list[tuple[str, int]] = []
        self.detached: list[tuple[str, int]] = []
        self.current_stage = "dependencies"
        self.result: dict[str, Any] = {
            "schema_version": 1,
            "operation": "bootsel_recovery",
            "scope": "Root-only ROM BOOTSEL recovery; no controller or transport qualification",
            "success": False,
            "recovery_success": False,
            "qualification_success": False,
            "live_input_proven": False,
            "live_imu_proven": False,
            "gameplay_proven": False,
            "exit_code": 2,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "parameters": {
                "timeout_seconds": args.timeout,
                "usb_timeout_ms": args.usb_timeout_ms,
                "reboot_bootsel": True,
            },
            "safety": {
                "pairing_writes": False,
                "profile_access": False,
                "flash_writes": False,
                "usb_reset": False,
                "motor_requested": False,
                "native_initialization": False,
                "interface_claims": False,
            },
            "recovery": {
                "request_attempted": False,
                "request_acknowledged": False,
                "root_disappeared": False,
                "rom_confirmed": False,
            },
            "stages": [],
            "errors": [],
            "seen_roots": [],
            "seen_bootsel": [],
            "devices": {},
            "acl": [],
            "controls": [],
            "cleanup": [],
        }
        self.checkpoint()

    def ctrl_transfer(
        self,
        request_type: int,
        request: int,
        value: int,
        index: int,
        data: bytes,
        timeout: int,
    ) -> int:
        # config_manager owns the envelope/setup encoding; this transport adds
        # the scenario deadline, audit trail, and short-write detection.
        entry = {
            "stage": self.current_stage,
            "side": "root",
            "name": "bootsel_reboot",
            "setup": [request_type, request, value, index, len(data)],
            "request_hex": data.hex(),
            "at_seconds": self.elapsed(),
        }
        self.result["controls"].append(entry)
        self.result["recovery"]["request_attempted"] = True
        self.checkpoint()
        try:
            written = self.devices["root"].ctrl_transfer(
                request_type,
                request,
                value,
                index,
                data,
                timeout=self.timeout_ms(min(timeout, self.args.usb_timeout_ms)),
            )
            entry["length"] = written
            if written != len(data):
                raise RuntimeError(
                    f"short BOOTSEL control write: {written} of {len(data)} bytes; "
                    "reboot outcome is unconfirmed"
                )
            self.result["recovery"]["request_acknowledged"] = True
            return written
        except Exception as error:
            entry["error"] = str(error)
            raise

    def confirm_bootsel(self) -> None:
        root = self.result["devices"]["root"]
        recovery = self.result["recovery"]
        while time.monotonic() < self.deadline:
            at_port = []
            for device in self.core.find(find_all=True) or []:
                same_port = device.bus == root["bus"] and tuple(
                    device.port_numbers or ()
                ) == tuple(root["ports"])
                if same_port:
                    at_port.append(device)
                if device.idVendor == BOOTSEL_VID and device.idProduct in BOOTSEL_PIDS:
                    seen = location(device)
                    if seen not in self.result["seen_bootsel"]:
                        self.result["seen_bootsel"].append(seen)
            if not any(
                (device.idVendor, device.idProduct) == (VID, ROOT_PID)
                for device in at_port
            ):
                recovery["root_disappeared"] = True
            if len(at_port) == 1:
                device = at_port[0]
                if device.idVendor == BOOTSEL_VID and device.idProduct in BOOTSEL_PIDS:
                    self.result["devices"]["bootsel"] = location(device)
                    recovery["rom_confirmed"] = True
                    return
            self.checkpoint()
            time.sleep(min(0.1, max(0, self.deadline - time.monotonic())))
        raise TimeoutError(
            "BOOTSEL request acknowledged, but ROM USB did not replace the root "
            "at the same physical bus/port before the deadline; recovery is unconfirmed"
        )

    def run(self) -> int:
        completed = False
        try:
            with self.stage("dependencies"):
                import usb.core
                import usb.util

                from switch_pico_bridge.config_manager import request_bootsel_reboot

                self.core, self.util = usb.core, usb.util
            with self.stage("root_only_discovery"):
                self.discover(root_only=True)
            with self.stage("permissions_and_root_identity"):
                self.permissions()
            with self.stage("explicit_bootsel_request"):
                request_bootsel_reboot(self)
            with self.stage("same_port_rom_enumeration"):
                self.confirm_bootsel()
            completed = True
        except KeyboardInterrupt:
            self.result["interrupted"] = True
        except (
            OSError,
            RuntimeError,
            ValueError,
            TypeError,
            ImportError,
            subprocess.SubprocessError,
            struct.error,
        ) as error:
            self.result["failure"] = str(error)
        finally:
            try:
                with self.stage("cleanup"):
                    self.cleanup()
            finally:
                success = completed and not self.result["errors"]
                self.result["success"] = self.result["recovery_success"] = success
                self.result["exit_code"] = 0 if success else 2
                recovery = self.result["recovery"]
                self.result["summary"] = (
                    f"{'RECOVERY_CONFIRMED' if success else 'RECOVERY_INCOMPLETE'} "
                    f"BOOTSEL request_acknowledged={recovery['request_acknowledged']} "
                    f"same_port_rom_confirmed={recovery['rom_confirmed']} "
                    "qualification=not_run live_input=not_proven gameplay=not_proven"
                )
                self.checkpoint()
                print(f"[NATIVEHUB] {self.result['summary']}", flush=True)
                print(f"[NATIVEHUB] capture={self.args.output}", flush=True)
        return self.result["exit_code"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "build-switch2-native-hub",
        help="configured build containing private capture reference paths",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="new JSON capture path; existing files are never overwritten",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=90,
        help="overall USB scenario deadline in seconds, at most 600 (default: 90)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10,
        help="active input/read overlap duration, 2..120 seconds (default: 10)",
    )
    parser.add_argument(
        "--usb-timeout-ms",
        type=int,
        default=500,
        help="per control/bulk transfer timeout, 20..3000 ms (default: 500)",
    )
    input_mode = parser.add_mutually_exclusive_group()
    input_mode.add_argument(
        "--input-only",
        action="store_true",
        help="qualify without IMU; requires real button presses and continued control changes on every R/L half of every pair; use distinct independent controls for two pairs",
    )
    input_mode.add_argument(
        "--neutral",
        action="store_true",
        help="OPT-IN: standalone neutral transport only; requires calibrated neutral controls and advancing USB counters, never proves live input/IMU/gameplay",
    )
    input_mode.add_argument(
        "--reboot-bootsel",
        action="store_true",
        help="OPT-IN: reboot only the identified root into ROM BOOTSEL and confirm the same physical port; no qualification, children, build references, or interface claims",
    )
    parser.add_argument(
        "--pairs",
        type=int,
        choices=(1, 2),
        default=1,
        help="hub pair count matching the build cache; two live pairs require GAMEPAD/DUALSENSE and independent activity on both controllers (default: 1)",
    )
    parser.add_argument(
        "--rumble-sample",
        type=int,
        choices=range(8),
        help="OPT-IN: play native motor sample 0..7 once on each donor and require its ACK",
    )
    parser.add_argument(
        "--capture-trace-on-error",
        action="store_true",
        help="OPT-IN: request one volatile child EP0 snapshot from a TRACE-enabled root after the first child control-transfer error, before cleanup; never retry the failed request",
    )
    args = parser.parse_args()
    if args.reboot_bootsel and args.capture_trace_on_error:
        parser.error(
            "--reboot-bootsel forbids --capture-trace-on-error; recovery never captures child traces"
        )
    if args.reboot_bootsel and args.rumble_sample is not None:
        parser.error(
            "--reboot-bootsel forbids --rumble-sample; recovery never actuates motors"
        )
    if args.neutral and args.rumble_sample is not None:
        parser.error(
            "--neutral forbids --rumble-sample; transport qualification must not actuate motors"
        )
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 600:
        parser.error("timeout must be finite and in (0,600]")
    if not args.reboot_bootsel:
        if not math.isfinite(args.duration) or not 2 <= args.duration <= 120:
            parser.error("duration must be finite and in [2,120]")
        if args.timeout < args.duration + 10:
            parser.error(
                "timeout must allow at least duration + 10 seconds for discovery/initialization"
            )
    if not 20 <= args.usb_timeout_ms <= 3000:
        parser.error("usb-timeout-ms must be in [20,3000]")
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        capture = args.output.open("x+", encoding="utf-8")
    except OSError as error:
        parser.error(f"cannot create a new capture: {error}")

    def interrupted(signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt(f"received signal {signum}")

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        with capture:
            scenario = BootselRecovery if args.reboot_bootsel else Check
            return scenario(args, capture).run()
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
