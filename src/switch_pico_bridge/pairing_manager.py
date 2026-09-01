#!/usr/bin/env python3
"""Manage Pico 2 W Bluetooth pairings over vendor requests on USB EP0."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from collections.abc import Iterable, Sequence
from typing import Any, Protocol

import usb.core

USB_VENDOR_ID = 0x057E
USB_PRODUCT_ID = 0x2009
REQUEST_CLEAR = 0x50
REQUEST_GET = 0x51
REQUEST_REFRESH = 0x52
REQUEST_VALUE = 0x5350
REQUEST_INDEX = 0x4D47
PROTOCOL_VERSION = 1
RESPONSE_HEADER_SIZE = 12
RECORD_SIZE = 8
RECORD_CAPACITY = 16
MAXIMUM_RESPONSE_SIZE = RESPONSE_HEADER_SIZE + RECORD_CAPACITY * RECORD_SIZE
STATUS_READY = 0
STATUS_PENDING = 1
TRANSPORT_CLASSIC = 1
TRANSPORT_BLE = 2
USB_TIMEOUT_MS = 1000


class PairingManagerError(RuntimeError):
    """Expected discovery, USB transport, or protocol failure."""


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


class UsbDevice(Protocol):
    bus: int | None
    address: int | None

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
class PairingSnapshot:
    generation: int
    status: int
    overflow: bool
    records: tuple[PairingRecord, ...]


def parse_snapshot(payload: bytes) -> PairingSnapshot:
    if len(payload) < RESPONSE_HEADER_SIZE:
        raise PairingManagerError("short pairing-management response")
    if payload[:4] != b"SPPM":
        raise PairingManagerError("device does not implement pairing management")
    if payload[4] != PROTOCOL_VERSION:
        raise PairingManagerError(
            f"unsupported pairing protocol version {payload[4]}"
        )

    status = payload[5]
    record_count = payload[6]
    required = RESPONSE_HEADER_SIZE + record_count * RECORD_SIZE
    if record_count > RECORD_CAPACITY or len(payload) < required:
        raise PairingManagerError("invalid pairing record count")

    generation = int(struct.unpack_from("<I", payload, 8)[0])
    records: list[PairingRecord] = []
    offset = RESPONSE_HEADER_SIZE
    for _ in range(record_count):
        records.append(
            PairingRecord(
                transport=payload[offset],
                address_type=payload[offset + 1],
                address=bytes(payload[offset + 2 : offset + 8]),
            )
        )
        offset += RECORD_SIZE
    return PairingSnapshot(
        generation=generation,
        status=status,
        overflow=bool(payload[7] & 1),
        records=tuple(records),
    )


def _control_in(device: UsbDevice) -> bytes:
    payload = device.ctrl_transfer(
        0xC0,
        REQUEST_GET,
        REQUEST_VALUE,
        REQUEST_INDEX,
        MAXIMUM_RESPONSE_SIZE,
        timeout=USB_TIMEOUT_MS,
    )
    return bytes(payload)


def _control_out(device: UsbDevice, request: int) -> None:
    device.ctrl_transfer(
        0x40,
        request,
        REQUEST_VALUE,
        REQUEST_INDEX,
        None,
        timeout=USB_TIMEOUT_MS,
    )


def read_snapshot(device: UsbDevice) -> PairingSnapshot:
    return parse_snapshot(_control_in(device))


def wait_for_snapshot(
    device: UsbDevice, previous_generation: int, timeout: float
) -> PairingSnapshot:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = read_snapshot(device)
        if (
            snapshot.status == STATUS_READY
            and snapshot.generation != previous_generation
        ):
            return snapshot
        time.sleep(0.05)
    raise PairingManagerError("Pico did not finish the pairing operation")


def refresh_snapshot(device: UsbDevice, timeout: float) -> PairingSnapshot:
    initial = read_snapshot(device)
    _control_out(device, REQUEST_REFRESH)
    return wait_for_snapshot(device, initial.generation, timeout)


def clear_pairings(device: UsbDevice, timeout: float) -> PairingSnapshot:
    initial = read_snapshot(device)
    _control_out(device, REQUEST_CLEAR)
    snapshot = wait_for_snapshot(device, initial.generation, timeout)
    if snapshot.records:
        raise PairingManagerError("Pico reported pairings after clear completed")
    return snapshot


def _candidate_devices() -> Iterable[UsbDevice]:
    devices = usb.core.find(
        find_all=True,
        idVendor=USB_VENDOR_ID,
        idProduct=USB_PRODUCT_ID,
    )
    return () if devices is None else devices


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
                _ = read_snapshot(device)
            except (PairingManagerError, usb.core.USBError) as exc:
                failures.append(exc)
                continue
            matches.append(device)

        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            locations = ", ".join(
                f"{device.bus}:{device.address}" for device in matches
            )
            raise PairingManagerError(
                f"multiple switch-pico devices found ({locations}); "
                "select one with --bus and --address"
            )
        if time.monotonic() >= deadline:
            break
        time.sleep(0.05)

    if failures:
        raise PairingManagerError(
            "matching USB devices were found, but none accepted the "
            f"management request; last error: {failures[-1]}"
        ) from failures[-1]
    raise PairingManagerError("no USB-connected switch-pico AIO firmware found")


def _print_snapshot(snapshot: PairingSnapshot) -> None:
    if not snapshot.records:
        print("No stored pairings.")
        return
    for index, record in enumerate(snapshot.records, start=1):
        print(f"{index}: {record.transport_text} {record.address_text}")
    if snapshot.overflow:
        print("Warning: additional pairings did not fit in the response.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="switch-pico-pairings",
        description="List or clear switch-pico AIO Bluetooth pairings.",
    )
    parser.add_argument("--bus", type=int, help="USB bus number")
    parser.add_argument("--address", type=int, help="USB device address")
    parser.add_argument(
        "--timeout", type=float, default=3.0,
        help="operation timeout in seconds (default: 3)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list", help="list stored Classic and BLE pairings")
    clear_parser = subparsers.add_parser("clear", help="clear all pairings")
    clear_parser.add_argument(
        "--yes", action="store_true",
        help="confirm destructive clearing without prompting",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout <= 0:
        print("error: --timeout must be positive", file=sys.stderr)
        return 2
    if args.command == "clear" and not args.yes:
        print("error: clear requires --yes", file=sys.stderr)
        return 2

    try:
        device = find_pico(args.bus, args.address, args.timeout)
        if args.command == "list":
            _print_snapshot(refresh_snapshot(device, args.timeout))
        else:
            before = refresh_snapshot(device, args.timeout)
            clear_pairings(device, args.timeout)
            print(f"Cleared {len(before.records)} stored pairing(s).")
    except PairingManagerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except usb.core.USBError as exc:
        print(f"error: USB access failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
