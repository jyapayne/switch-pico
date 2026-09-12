#!/usr/bin/env python3
"""Exercise the RAM-only native USB address probe; never flash firmware."""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import time
from collections.abc import Iterable
from pathlib import Path
from typing import Any, cast

import usb.core
import usb.util

VID = 0x1209
HUB_PID = 0x0001
CHILD_PIDS = (0x0002, 0x0003)
FIELDS: tuple[str, ...] = (
    "magic",
    "version",
    "system_hz",
    "routing_enabled",
    "hub_address",
    "child1_address",
    "child2_address",
    "default_slot",
    "hub_setups",
    "child1_setups",
    "child2_setups",
    "bad_setup_owner",
    "observer_ready",
    "sops",
    "sync_ok",
    "valid_tokens",
    "valid_setups",
    "crc_errors",
    "late_samples",
    "retargets",
    "hub_tokens",
    "child1_tokens",
    "child2_tokens",
    "cycles_per_bit",
    "raw0",
    "raw1",
    "raw2",
    "raw_count",
    "raw_eop",
    "raw_late",
    "live_phy",
    "correlated_setups",
)


def probe_devices(product_id: int) -> list[usb.core.Device]:
    found = usb.core.find(find_all=True, idVendor=VID, idProduct=product_id)
    return list(cast(Iterable[usb.core.Device], found)) if found is not None else []


def device_location(device: usb.core.Device) -> tuple[int, int]:
    bus, address = device.bus, device.address
    if not isinstance(bus, int) or not isinstance(address, int):
        raise TypeError("USB device has no usable bus/address")
    return bus, address


def grant_access(device: usb.core.Device) -> None:
    bus, address = device_location(device)
    node = f"/dev/bus/usb/{bus:03d}/{address:03d}"
    subprocess.run(
        ["sudo", "-n", "setfacl", "-m", f"u:{os.getuid()}:rw", node],
        check=True,
        capture_output=True,
        text=True,
        timeout=3,
    )


def stats(device: usb.core.Device) -> dict[str, int]:
    packet = bytes(device.ctrl_transfer(0xC0, 0x5A, 0, 0, 128, timeout=400))
    if len(packet) != 128:
        raise RuntimeError(f"statistics length {len(packet)} != 128")
    result = {
        key: int(value)
        for key, value in zip(FIELDS, struct.unpack("<32I", packet), strict=True)
    }
    if result["magic"] != 0x42554850 or result["version"] != 3:
        raise RuntimeError("device did not return the address-probe signature")
    return result


def descriptor(device: usb.core.Device, expected_pid: int) -> dict[str, int]:
    data = bytes(device.ctrl_transfer(0x80, 6, 0x0100, 0, 18, timeout=400))
    if len(data) != 18 or data[0:2] != b"\x12\x01":
        raise RuntimeError("invalid device descriptor")
    vendor, product = struct.unpack_from("<HH", data, 8)
    if (vendor, product) != (VID, expected_pid):
        raise RuntimeError(
            f"address {device.address} returned wrong identity {vendor:04x}:{product:04x}"
        )
    bus, address = device_location(device)
    return {"bus": bus, "address": address, "vid": int(vendor), "pid": int(product)}


def delta(after: dict[str, int], before: dict[str, int], field: str) -> int:
    return (after[field] - before[field]) & 0xFFFFFFFF


def measure_phase(device: usb.core.Device, phase: int) -> dict[str, Any]:
    device.ctrl_transfer(0x40, 0x5D, phase, 0, b"", timeout=400)
    before = stats(device)
    after = before
    for _ in range(24):
        after = stats(device)
        time.sleep(0.002)
    hardware = delta(after, before, "hub_setups")
    confirmed = delta(after, before, "correlated_setups")
    hits = delta(after, before, "hub_tokens")
    # Qualify observed headers against real, CRC-accepted hardware SETUP IRQs.
    # Full software CRC capture can overrun after routing work and is diagnostic.
    credible = hardware >= 20 and hardware * 0.8 <= confirmed <= hardware * 1.5
    return {
        "phase": phase,
        "hardware_setups": hardware,
        "correlated_setups": confirmed,
        "crc_verified_setups": delta(after, before, "valid_setups"),
        "matched_hub_tokens": hits,
        "credible": credible,
        "crc_errors": delta(after, before, "crc_errors"),
        "late_samples": delta(after, before, "late_samples"),
        "before": before,
        "after": after,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wait-seconds", type=float, default=30)
    parser.add_argument(
        "--arm",
        action="store_true",
        help="attempt address routing only after credible passive capture",
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new capture filename")
    if not 0 < args.wait_seconds <= 120:
        parser.error("wait-seconds must be in (0,120]")
    result: dict[str, Any] = {
        "success": False,
        "armed": False,
        "phases": [],
        "return_request_sent": False,
    }
    hub: usb.core.Device | None = None
    print("[HOSTPROBE] waiting for RAM hub probe", flush=True)
    try:
        deadline = time.monotonic() + args.wait_seconds
        while time.monotonic() < deadline:
            devices = probe_devices(HUB_PID)
            if len(devices) > 1:
                raise RuntimeError(
                    "multiple matching hub probes; refusing ambiguous target"
                )
            if devices:
                hub = devices[0]
                break
            time.sleep(0.05)
        if hub is None:
            raise RuntimeError("RAM hub did not enumerate before timeout")
        grant_access(hub)
        result["hub"] = descriptor(hub, HUB_PID)
        result["initial_stats"] = stats(hub)
        print(
            f"[HOSTPROBE] hub address={hub.address}, observer={result['initial_stats']['observer_ready']}",
            flush=True,
        )
        if result["initial_stats"]["observer_ready"] != 1:
            result["failure"] = (
                "cycle-timed observer did not initialize; no address routing attempted"
            )
            return 2
        phases = result["initial_stats"]["cycles_per_bit"]
        if not 1 <= phases <= 64:
            raise RuntimeError(f"invalid cycles-per-bit {phases}")
        for phase in range(phases):
            measured = measure_phase(hub, phase)
            result["phases"].append(measured)
            print(
                f"[HOSTPROBE] phase={phase} confirmed={measured['correlated_setups']}/{measured['hardware_setups']} hits={measured['matched_hub_tokens']} late={measured['late_samples']} raw={measured['after']['raw0']:08x}/{measured['after']['raw1']:08x} n={measured['after']['raw_count']} eop={measured['after']['raw_eop']}",
                flush=True,
            )
        candidates = [item for item in result["phases"] if item["credible"]]
        if not candidates:
            result["failure"] = (
                "no sampling phase reliably observed native USB SETUP tokens; retargeting was not armed"
            )
            return 2
        best = min(
            candidates,
            key=lambda item: (
                abs(item["correlated_setups"] - item["hardware_setups"]),
                item["crc_errors"],
                item["late_samples"],
            ),
        )
        hub.ctrl_transfer(0x40, 0x5D, best["phase"], 0, b"", timeout=400)
        result["selected_phase"] = best["phase"]
        if not args.arm:
            result["passive_capture_verified"] = True
            return 0
        hub.ctrl_transfer(0x40, 0x5B, 1, 0, b"", timeout=400)
        result["armed"] = True
        print(
            "[HOSTPROBE] address retargeting armed; waiting for real downstream enumeration",
            flush=True,
        )
        children = {}
        deadline = time.monotonic() + 5
        # Let the kernel finish downstream enumeration without injecting root
        # control transfers into the probe's still-shared physical EP0 context.
        # Five seconds is below the ACK-fed watchdog's eight-second deadline.
        while time.monotonic() < deadline and len(children) != 2:
            for port, pid in enumerate(CHILD_PIDS, 1):
                found = probe_devices(pid)
                if (
                    len(found) == 1
                    and found[0].bus == hub.bus
                    and hub.port_numbers is not None
                    and found[0].port_numbers == (*hub.port_numbers, port)
                ):
                    children[pid] = found[0]
            time.sleep(0.05)
        if len(children) != 2:
            result["failure"] = (
                "hub did not enumerate both separately addressed children"
            )
            result["children_seen"] = [hex(pid) for pid in children]
            return 2
        ordered = [hub, children[CHILD_PIDS[0]], children[CHILD_PIDS[1]]]
        if len({device.address for device in ordered}) != 3:
            raise RuntimeError("host did not assign three distinct USB addresses")
        for child in ordered[1:]:
            grant_access(child)
        result["devices"] = []
        for _ in range(20):
            for device, pid in zip(ordered, (HUB_PID, *CHILD_PIDS), strict=True):
                identity = descriptor(device, pid)
                result["devices"].append(identity)
            result["latest_stats"] = stats(hub)
        result["success"] = True
        print(
            "[HOSTPROBE] PASS: three actual addresses, each repeatedly returned its own descriptor",
            flush=True,
        )
        return 0
    except (
        usb.core.USBError,
        RuntimeError,
        TypeError,
        subprocess.SubprocessError,
        OSError,
    ) as error:
        result["failure"] = str(error)
        print(f"[HOSTPROBE] failure: {error}", flush=True)
        return 2
    finally:
        if hub is not None:
            try:
                hub.ctrl_transfer(0x40, 0x5C, 0, 0, b"", timeout=400)
                result["return_request_sent"] = True
            except usb.core.USBError as error:
                result["return_request_error"] = str(error)
            usb.util.dispose_resources(hub)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(
            f"[HOSTPROBE] saved {args.output}; RAM probe has an 8-second watchdog fallback",
            flush=True,
        )


if __name__ == "__main__":
    raise SystemExit(main())
