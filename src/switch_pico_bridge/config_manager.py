#!/usr/bin/env python3
"""Manage switch-pico persistent configuration and pairings over USB EP0."""

from __future__ import annotations

import argparse
import secrets
import struct
import sys
import time
import zlib
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol, cast

import usb.core

USB_IDENTITIES = ((0x057E, 0x2009), (0xCAFE, 0x4010))
REQUEST_VALUE = 0x5350
REQUEST_INDEX = 0x0001
PROTOCOL_VERSION = 1
REQUEST_HEADER_SIZE = 16
RESPONSE_HEADER_SIZE = 20
MAXIMUM_REQUEST_SIZE = 64
MAXIMUM_RESPONSE_SIZE = 152
MAXIMUM_CHUNK_SIZE = 40
USB_TIMEOUT_MS = 1000

OP_INFO = 0x01
OP_CONFIGURATION_READ = 0x10
OP_CONFIGURATION_BEGIN = 0x11
OP_CONFIGURATION_CHUNK = 0x12
OP_CONFIGURATION_COMMIT = 0x13
OP_CONFIGURATION_RESET = 0x14
OP_TRANSACTION_STATUS = 0x15
OP_PAIRING_READ = 0x20
OP_PAIRING_REFRESH = 0x21
OP_PAIRING_CLEAR = 0x22

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

CONFIGURATION_SCHEMA_VERSION = 1
CONFIGURATION_SIZE = 4
PAIRING_WINDOW_SECONDS_MIN = 10
PAIRING_WINDOW_SECONDS_MAX = 300
PAIRING_RECORD_SIZE = 8
PAIRING_RECORD_CAPACITY = 16
TRANSPORT_CLASSIC = 1
TRANSPORT_BLE = 2


class ConfigManagerError(RuntimeError):
    """Expected discovery, USB transport, or management protocol failure."""


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
    maximum_configuration_size: int


@dataclass(frozen=True)
class AdapterConfiguration:
    pairing_window_seconds: int
    generation: int
    crc: int


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


def _crc32(payload: bytes) -> int:
    return zlib.crc32(payload) & 0xFFFFFFFF


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
    return DeviceInfo(
        firmware_version=(
            envelope.payload[0],
            envelope.payload[1],
            envelope.payload[2],
        ),
        board=envelope.payload[3],
        active_mode=envelope.payload[4],
        maximum_configuration_size=struct.unpack_from(
            "<H", envelope.payload, 6
        )[0],
    )


def read_configuration(device: UsbDevice) -> AdapterConfiguration:
    envelope = _control_in(device, OP_CONFIGURATION_READ)
    _raise_status(envelope)
    if (
        envelope.schema_version != CONFIGURATION_SCHEMA_VERSION
        or len(envelope.payload) != CONFIGURATION_SIZE
        or envelope.payload[2:] != b"\x00\x00"
    ):
        raise ConfigManagerError("unsupported configuration object")
    pairing_window_seconds = struct.unpack_from("<H", envelope.payload)[0]
    if not (
        PAIRING_WINDOW_SECONDS_MIN
        <= pairing_window_seconds
        <= PAIRING_WINDOW_SECONDS_MAX
    ):
        raise ConfigManagerError("invalid stored pairing-window duration")
    return AdapterConfiguration(
        pairing_window_seconds=pairing_window_seconds,
        generation=envelope.generation,
        crc=envelope.payload_crc,
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
    payload = struct.pack("<Hxx", configuration.pairing_window_seconds)
    transaction_id = secrets.randbits(32) or 1
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
    transaction_id = secrets.randbits(32) or 1
    _control_out(
        device, OP_CONFIGURATION_RESET, struct.pack("<I", transaction_id)
    )
    return _wait_for_transaction(device, transaction_id, timeout)


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


def _print_pairings(snapshot: PairingSnapshot) -> None:
    if not snapshot.records:
        print("No stored pairings.")
        return
    for index, record in enumerate(snapshot.records, start=1):
        print(f"{index}: {record.transport_text} {record.address_text}")
    if snapshot.overflow:
        print("Warning: additional pairings did not fit in the response.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="switch-pico-config",
        description="Manage switch-pico persistent configuration and pairings.",
    )
    parser.add_argument("--bus", type=int, help="USB bus number")
    parser.add_argument("--address", type=int, help="USB device address")
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="operation timeout in seconds (default: 3)",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="show firmware and configuration status")

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
    if args.command == "config" and args.config_command == "reset" and not args.yes:
        print("error: config reset requires --yes", file=sys.stderr)
        return 2
    if args.command == "pairings" and args.pairing_command == "clear" and not args.yes:
        print("error: pairings clear requires --yes", file=sys.stderr)
        return 2

    try:
        device = find_pico(args.bus, args.address, args.timeout)
        if args.command == "status":
            info = read_info(device)
            configuration = read_configuration(device)
            version = ".".join(str(part) for part in info.firmware_version)
            mode = "XInput" if info.active_mode else "Switch"
            print(f"Firmware: {version}")
            print(f"Board: Pico 2 W ({info.board})")
            print(f"Active USB mode: {mode}")
            print(f"Configuration generation: {configuration.generation}")
            print(f"Configuration CRC: {configuration.crc:08x}")
            print(
                "Pairing window: "
                f"{configuration.pairing_window_seconds} seconds"
            )
        elif args.command == "config":
            if args.config_command == "show":
                configuration = read_configuration(device)
                print(
                    f"pairing_window_seconds={configuration.pairing_window_seconds}"
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
