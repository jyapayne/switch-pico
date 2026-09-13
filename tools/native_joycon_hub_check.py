#!/usr/bin/env python3
"""Bounded, non-pairing qualification of the switch-pico native Joy-Con USB hub.

Requires Linux, PyUSB/libusb, and the existing sudo -n setfacl permission policy.
Uses the already-paired real R/L donors; it cannot wake or pair them. The JSON
capture is created exclusively before USB access and retains partial failures.
No reset, configuration change, pairing exchange, profile access, flash write,
or HID output is sent. Motor sample playback requires --rumble-sample explicitly.
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
SIDES = ("R", "L")
# Only protocol constants live in source. Device-specific factory/calibration
# captures stay in the private build paths configured by CMake.
MODELS = {
    "R": {"pid": 0x2066, "port": 1, "report": 0x08, "diagnostic_host": "020000000001"},
    "L": {"pid": 0x2067, "port": 2, "report": 0x07, "diagnostic_host": "020000000002"},
}


def model_references(build_dir: Path) -> dict[str, dict[str, Any]]:
    cache = {}
    for line in (build_dir / "CMakeCache.txt").read_text().splitlines():
        if line.startswith("SWITCH2_") and ":" in line and "=" in line:
            field, value = line.split("=", 1)
            cache[field.split(":", 1)[0]] = value
    if (
        cache.get("SWITCH2_BRIDGE_INPUT") == "DUALSENSE"
        and cache.get("SWITCH2_BRIDGE_IMU_TARGET", "BOTH") != "BOTH"
    ):
        raise ValueError(
            "Full dual-IMU qualification requires SWITCH2_BRIDGE_IMU_TARGET=BOTH; "
            "use the USB-completion UART trace for LEFT/RIGHT routing comparisons"
        )
    models = {}
    for side, constants in MODELS.items():
        prefix = "SWITCH2_PROBE" if side == "R" else "SWITCH2_PROBE_SECOND"
        identity = Path(cache[prefix + "_IDENTITY_FILE"]).read_bytes()
        version = Path(cache[prefix + "_VERSION_FILE"]).read_bytes()
        factory = Path(cache[prefix + "_FACTORY_FILE"]).read_bytes()
        address = bytes.fromhex(cache[prefix + "_CONTROLLER_ADDRESS"].replace(":", ""))
        if (
            len(identity) != 64
            or len(version) != 12
            or len(factory) != 8192
            or len(address) != 6
        ):
            raise ValueError(f"{side} build references have invalid native lengths")
        if factory[:64] != identity or struct.unpack_from("<HH", identity, 18) != (
            VID,
            constants["pid"],
        ):
            raise ValueError(f"{side} factory/identity references disagree")
        models[side] = {
            **constants,
            "identity": identity.hex(),
            "version": version.hex(),
            "factory_extension": factory[64:81].hex(),
            "mac_wire": address[::-1].hex(),
            "source_mode": cache.get("SWITCH2_BRIDGE_INPUT", "JOYCON2"),
        }
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
        self.models = {side: model.copy() for side, model in MODELS.items()}
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
        self.imu_evidence: dict[str, set[bytes]] = {side: set() for side in SIDES}
        self.result: dict[str, Any] = {
            "schema_version": 1,
            "success": False,
            "exit_code": 2,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "parameters": {
                "timeout_seconds": args.timeout,
                "duration_seconds": args.duration,
                "usb_timeout_ms": args.usb_timeout_ms,
                "rumble_sample": args.rumble_sample,
            },
            "safety": {
                "pairing_writes": False,
                "profile_access": False,
                "flash_writes": False,
                "usb_reset": False,
                "motor_requested": args.rumble_sample is not None,
            },
            "scope": "USB identity/protocol/input isolation, not console or motor-feel qualification",
            "imu_decode_note": "Existing candidate codec; left uses its documented one-byte-earlier IMU boundary. Physical scales are not calibrated by this check.",
            "stages": [],
            "errors": [],
            "seen_roots": [],
            "seen_children": [],
            "devices": {},
            "acl": [],
            "interfaces": [],
            "controls": [],
            "bulk": [],
            "active_rounds": [],
            "cleanup": [],
            "streams": {
                side: {
                    "packets": 0,
                    "valid_native_imu": 0,
                    "imu_counter_changes": 0,
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
                }
                for side in SIDES
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
        try:
            data = bytes(
                self.devices[owner].ctrl_transfer(
                    request_type,
                    request,
                    value,
                    index,
                    length,
                    timeout=self.timeout_ms(),
                )
            )
            entry["response_hex"] = data.hex()
            entry["length"] = len(data)
            return data
        except Exception as error:
            entry["error"] = str(error)
            raise

    def discover(self) -> None:
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
                    for side in SIDES:
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
                if len(selected) == 2:
                    if len(direct_children) != 2:
                        raise RuntimeError(
                            "target hub has unexpected additional direct children"
                        )
                    self.devices = {"root": root, **selected}
                    if len({device.address for device in self.devices.values()}) != 3:
                        raise RuntimeError(
                            "root/R/L do not have three distinct USB addresses"
                        )
                    self.result["devices"] = {
                        "root": root_info,
                        **{side: location(selected[side]) for side in SIDES},
                    }
                    return
            time.sleep(min(0.1, max(0, until - time.monotonic())))
        raise TimeoutError(
            "discovery: expected one switch-pico 057e:2068 with R 2066 at port 1 and L 2067 at port 2 on the same path; inspect seen_roots/seen_children"
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

    def claim(self) -> None:
        for side in SIDES:
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
        # Short-then-full EP0 reads also check transfer length/context teardown.
        short_length = (1, 7, 15)[round_number % 3]
        short = self.control(
            side, "vendor_version02_short", request_type, 2, 0, index, short_length
        )
        if short != status[:short_length]:
            raise RuntimeError(f"{side} short vendor read leaked/truncated incorrectly")

    def exchange_pair(
        self, requests: dict[str, tuple[bytes, bytes]], round_number: int = 0
    ) -> None:
        order = SIDES if round_number % 2 == 0 else tuple(reversed(SIDES))
        entries = {}
        # Both devices have pending, identical-form commands before either IN is
        # consumed. Reverse completion order to expose global reply-buffer reuse.
        for side in order:
            request, expected = requests[side]
            if request[:4] == b"\x0a\x91\x00\x02" and self.args.rumble_sample is None:
                raise RuntimeError("motor command requires explicit --rumble-sample")
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
            for side in SIDES:
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
            self.exchange_pair(requests)

    def queries(self, round_number: int) -> None:
        request = command(0x10, 1)
        self.exchange_pair(
            {
                side: (
                    request,
                    reply(request, bytes.fromhex(self.models[side]["version"])),
                )
                for side in SIDES
            },
            round_number,
        )
        requests = {}
        for side_index, side in enumerate(SIDES):
            offset = (round_number + side_index) % 2
            address = 0x13000 + offset
            request = command(2, 4, b"\x50\x7e\x00\x00" + struct.pack("<I", address))
            factory = bytes.fromhex(
                self.models[side]["identity"] + self.models[side]["factory_extension"]
            )
            payload = (
                b"\x50\x00\x00\x00"
                + struct.pack("<I", address)
                + factory[offset : offset + 80]
            )
            if len(payload) != 88:
                raise RuntimeError("captured memory reference is not an 80-byte read")
            requests[side] = (request, reply(request, payload))
        self.exchange_pair(requests, round_number)

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
        if report_id == self.models["L" if side == "R" else "R"]["report"]:
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
            length_offset = 14 if side == "L" else 15
            length = payload[length_offset]
            key = str(length)
            stream["imu_lengths"][key] = stream["imu_lengths"].get(key, 0) + 1
            if length == 0:
                stream["zero_length_imu"] += 1
                rejection = (
                    "zero-length IMU: inactive donor/neutral fallback is not live input"
                )
            else:
                try:
                    block = (
                        native_block({"native_hex": packet.hex()})
                        if side == "R"
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
                    controls = (payload[2:4], payload[5:8])
                    sample["buttons_hex"], sample["stick_hex"] = (
                        part.hex() for part in controls
                    )
                    if any(controls[0]):
                        stream["buttons_nonzero"] += 1
                    if (
                        side in self.last_controls
                        and self.last_controls[side] != controls
                    ):
                        stream["control_changes"] += 1
                    self.last_controls[side] = controls
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

    def poll_pair(self, until: float, reverse: bool = False) -> None:
        for side in reversed(SIDES) if reverse else SIDES:
            if time.monotonic() >= until:
                break
            self.poll(side, until)

    def counts(self) -> dict[str, int]:
        return {
            side: self.result["streams"][side]["valid_native_imu"] for side in SIDES
        }

    def donors_ready(self) -> None:
        until = min(self.deadline, time.monotonic() + 60)
        iteration = 0
        while time.monotonic() < until:
            self.poll_pair(until, bool(iteration % 2))
            iteration += 1
            if all(
                stream["valid_native_imu"] >= 3 and stream["imu_counter_changes"] >= 2
                for stream in self.result["streams"].values()
            ):
                return
        details = "; ".join(
            f"{side}: valid={stream['valid_native_imu']}, counter_changes={stream['imu_counter_changes']}, zero_imu={stream['zero_length_imu']}, invalid={stream['invalid']}, timeouts={stream['timeouts']}"
            for side, stream in self.result["streams"].items()
        )
        raise RuntimeError(
            f"donors did not produce fresh decodable native IMU during the manual-wake window; {details}; no pairing/wake/reset was attempted"
        )

    def active(self) -> None:
        until = min(self.deadline, time.monotonic() + self.args.duration)
        initial_counts = self.counts()
        initial_changes = {
            side: self.result["streams"][side]["imu_counter_changes"] for side in SIDES
        }
        next_query = time.monotonic()
        round_number = 0
        pending = None
        while time.monotonic() < until:
            if time.monotonic() >= next_query and until - time.monotonic() >= 0.25:
                if pending is not None:
                    pending["after"] = self.counts()
                pending = {
                    "round": round_number,
                    "before": self.counts(),
                    "started_seconds": self.elapsed(),
                }
                self.result["active_rounds"].append(pending)
                for side in SIDES if round_number % 2 == 0 else reversed(SIDES):
                    self.descriptors(side, report=round_number % 2 == 0)
                    self.identities(side, round_number)
                self.queries(round_number)
                pending["reads_matched"] = True
                next_query = time.monotonic() + 0.25
                round_number += 1
                self.checkpoint()
            self.poll_pair(until, bool(round_number % 2))
        if pending is not None:
            pending["after"] = self.counts()
        if round_number < 2:
            self.error(
                "fewer than two interleaved control/bulk rounds completed during streaming"
            )
        if not any(
            all(
                entry.get("after", {}).get(side, 0) > entry["before"][side]
                for side in SIDES
            )
            for entry in self.result["active_rounds"]
            if entry.get("reads_matched")
        ):
            self.error(
                "no interleaved control/bulk round was bracketed by valid input from both donors"
            )
        shared = self.imu_evidence["R"] & self.imu_evidence["L"]
        shared_source = self.models["R"].get("source_mode") == "DUALSENSE"
        self.result["imu_isolation"] = {
            "sample_limit_per_side": 512,
            "policy": "shared_physical_source"
            if shared_source
            else "independent_physical_sources",
            "identical_blocks_seen_on_both_sides": len(shared),
            "unique_blocks": {
                side: len(blocks) for side, blocks in self.imu_evidence.items()
            },
            "side_exclusive_blocks": {
                side: len(blocks - shared) for side, blocks in self.imu_evidence.items()
            },
        }
        for side in SIDES:
            evidence = (
                self.imu_evidence[side]
                if shared_source
                else self.imu_evidence[side] - shared
            )
            if len(evidence) < 2:
                self.error(
                    "IMU evidence is frozen"
                    if shared_source
                    else "donor IMU evidence is frozen or duplicated across child devices",
                    side,
                )
        for side in SIDES:
            stream = self.result["streams"][side]
            if (
                stream["valid_native_imu"] - initial_counts[side] < 3
                or stream["imu_counter_changes"] - initial_changes[side] < 2
            ):
                self.error(
                    "insufficient fresh native donor IMU during active control/bulk reads",
                    side,
                )
            last_change = stream["last_counter_change_seconds"]
            if last_change is None or self.elapsed() - last_change > 2:
                self.error(
                    "donor IMU stopped advancing before streaming finished", side
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
                self.models = model_references(self.args.build_dir)
                self.core, self.util = usb.core, usb.util
            with self.stage("discovery"):
                self.discover()
            with self.stage("permissions_and_root_identity"):
                self.permissions()
            with self.stage("claim_child_interfaces"):
                self.claim()
            with self.stage("descriptor_and_ep0_isolation"):
                for round_number in range(20):
                    for side in SIDES if round_number % 2 == 0 else reversed(SIDES):
                        self.descriptors(side, report=round_number in (0, 19))
                        self.identities(side, round_number)
            with self.stage("native_initialization"):
                self.initialize()
            with self.stage("bulk_isolation"):
                for round_number in range(2):
                    self.queries(round_number)
            with self.stage("donor_startup"):
                self.donors_ready()
            with self.stage("active_input_and_read_isolation"):
                self.active()
            if self.args.rumble_sample is not None and not self.result["errors"]:
                with self.stage("explicit_motor_sample_ack"):
                    request = command(
                        0x0A, 2, bytes((self.args.rumble_sample, 0, 0, 0))
                    )
                    self.exchange_pair(
                        {side: (request, reply(request)) for side in SIDES}
                    )
                    self.result["motor_result"] = (
                        "native sample ACK received for each side; physical sensation is not measured"
                    )
            else:
                self.result["stages"].append(
                    {"name": "explicit_motor_sample_ack", "status": "skipped"}
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
                self.result["summary"] = (
                    f"{'PASS' if self.result['success'] else 'FAIL'} "
                    f"R={streams['R']['valid_native_imu']} L={streams['L']['valid_native_imu']} "
                    f"R_counter_changes={streams['R']['imu_counter_changes']} "
                    f"L_counter_changes={streams['L']['imu_counter_changes']} "
                    f"active_rounds={len(self.result['active_rounds'])} "
                    f"errors={len(self.result['errors'])} failed_stages={','.join(failed) or 'none'}"
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
    parser.add_argument(
        "--rumble-sample",
        type=int,
        choices=range(8),
        help="OPT-IN: play native motor sample 0..7 once on each donor and require its ACK",
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 600:
        parser.error("timeout must be finite and in (0,600]")
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
            return Check(args, capture).run()
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
