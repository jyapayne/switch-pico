"""USB CDC client for the standalone wake-only beacon (no controller transport).

A completed request confirms the beacon finished its advertising burst, not RF
reception or the console's power state. An interrupted request is never retried.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import sys
import time
from collections.abc import Sequence
from dataclasses import asdict, dataclass, fields

import serial
from serial.tools import list_ports

USB_VID = 0xCAFE
USB_PID = 0x4030
MAX_REQUEST_ID = 0x7FFFFFFF
MAX_RESPONSE_BYTES = 1024
_STATES = {
    "idle",
    "queued",
    "broadcasting",
    "complete",
    "unconfigured",
    "busy",
    "failed",
}
_ERRORS = {"", "malformed", "busy", "radio_init_failed"}


@dataclass(frozen=True)
class WakeBeaconStatus:
    protocol: int
    role: str
    firmware: str
    radio_ready: bool
    controller_hosting: bool
    request_id: int
    state: str
    configured: bool
    busy: bool
    accepted_requests: int
    completed_bursts: int
    failures: int
    error: str

    def to_dict(self) -> dict:
        return asdict(self)


class WakeBeaconError(Exception):
    """Failure with the last validated status and whether WAKE may have been sent."""

    code = "beacon_error"

    def __init__(self, message: str, status: WakeBeaconStatus | None = None):
        super().__init__(message)
        self.status = status
        self.request_id = None
        self.wake_sent = False


class ProtocolError(WakeBeaconError):
    code = "protocol_error"


class TransportError(WakeBeaconError):
    code = "transport_error"


class BeaconTimeout(WakeBeaconError):
    code = "timeout"


class CommandRejected(WakeBeaconError):
    code = "command_rejected"


class OperationFailed(WakeBeaconError):
    code = "operation_failed"


class RequestLost(WakeBeaconError):
    code = "request_lost"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("Duplicate status field")
        result[key] = value
    return result


def decode_status(line: bytes) -> WakeBeaconStatus:
    """Strictly decode one complete SPWB1 response, including its newline."""
    if len(line) > MAX_RESPONSE_BYTES or not line.endswith(b"\n"):
        raise ProtocolError("Missing newline or oversized beacon response")
    body = line[:-1]
    if body.endswith(b"\r"):
        body = body[:-1]
    if not body.startswith(b"SPWB1 ") or any(c < 32 or c > 126 for c in body):
        raise ProtocolError("Expected an ASCII SPWB1 response")
    try:
        value = json.loads(body[6:].decode("ascii"), object_pairs_hook=_unique_object)
    except (ValueError, RecursionError) as exc:
        raise ProtocolError("Invalid beacon JSON") from exc
    if type(value) is not dict or set(value) != {
        field.name for field in fields(WakeBeaconStatus)
    }:
        raise ProtocolError("Unexpected beacon status fields")
    for key in ("radio_ready", "controller_hosting", "configured", "busy"):
        if type(value[key]) is not bool:
            raise ProtocolError(f"Invalid boolean field: {key}")
    for key in (
        "protocol",
        "request_id",
        "accepted_requests",
        "completed_bursts",
        "failures",
    ):
        limit = MAX_REQUEST_ID if key == "request_id" else 0xFFFFFFFF
        if type(value[key]) is not int or not 0 <= value[key] <= limit:
            raise ProtocolError(f"Invalid integer field: {key}")
    if (
        value["protocol"] != 1
        or value["role"] != "wake-only"
        or value["controller_hosting"]
    ):
        raise ProtocolError("Device is not a supported wake-only beacon")
    if (
        type(value["firmware"]) is not str
        or not value["firmware"].strip()
        or len(value["firmware"]) > 64
    ):
        raise ProtocolError("Invalid firmware identifier")
    if type(value["state"]) is not str or value["state"] not in _STATES:
        raise ProtocolError("Unknown beacon state")
    if type(value["error"]) is not str or value["error"] not in _ERRORS:
        raise ProtocolError("Unknown beacon command error")
    return WakeBeaconStatus(**value)


def discover_beacon_port(port: str | None = None) -> str:
    """Use an explicit path or require exactly one CAFE:4030 serial interface."""
    if port is not None:
        if not port.strip():
            raise TransportError("Serial port must not be empty")
        return port
    try:
        matches = sorted(
            {
                p.device
                for p in list_ports.comports()
                if p.vid == USB_VID and p.pid == USB_PID and p.device
            }
        )
    except (OSError, serial.SerialException) as exc:
        raise TransportError(f"Cannot enumerate serial ports: {exc}") from exc
    if not matches:
        raise TransportError(
            "No CAFE:4030 wake beacon found; specify --port if necessary"
        )
    if len(matches) != 1:
        raise TransportError(
            "Multiple wake beacons found; select --port: " + ", ".join(matches)
        )
    return matches[0]


def _positive_timeout(value) -> float:
    try:
        timeout = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("timeout must be finite and positive") from exc
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    return timeout


class WakeBeaconClient:
    """Own a pyserial-compatible transport; use as a context manager to close it.

    Calls are synchronous and must not run concurrently. Each public operation
    has one deadline covering all serial reads, writes and status polls.
    """

    def __init__(self, transport, timeout: float = 15.0):
        self.timeout = _positive_timeout(timeout)
        self.transport = transport
        self._last_operation = (None, None, False)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except TransportError:
            if exc_type is None:
                raise

    def close(self) -> None:
        try:
            self.transport.close()
        except (OSError, serial.SerialException) as exc:
            error = TransportError(f"Cannot close beacon serial port: {exc}")
            error.status, error.request_id, error.wake_sent = self._last_operation
            raise error from exc

    @staticmethod
    def _remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise BeaconTimeout("Beacon deadline expired; no wake retry was attempted")
        return remaining

    def _exchange(self, command: bytes, deadline: float) -> WakeBeaconStatus:
        try:
            write_timeout = min(0.25, self._remaining(deadline))
            if self.transport.write_timeout != write_timeout:
                self.transport.write_timeout = write_timeout
            # A partial write may already have reached the device. Never resend.
            if self.transport.write(command) != len(command):
                raise TransportError("Incomplete serial write; command was not retried")
            response = bytearray()
            while True:
                read_timeout = min(0.1, self._remaining(deadline))
                # pyserial reconfigures the Windows port on every assignment,
                # even for an unchanged timeout. Avoid USB control transfers
                # per response byte, while still shortening reads at deadline.
                if self.transport.timeout != read_timeout:
                    self.transport.timeout = read_timeout
                chunk = self.transport.read(1)
                if not chunk:
                    continue
                response.extend(chunk)
                if len(response) > MAX_RESPONSE_BYTES:
                    raise ProtocolError("Oversized beacon response")
                if chunk.endswith(b"\n"):
                    self._remaining(deadline)
                    status = decode_status(bytes(response))
                    if status.error:
                        raise CommandRejected(status.error, status)
                    return status
        except (OSError, serial.SerialException) as exc:
            raise TransportError(
                f"Beacon serial I/O failed: {exc}; no wake retry was attempted"
            ) from exc

    def read_status(self) -> WakeBeaconStatus:
        """Read retained state without actuating, including failed operations."""
        status = None
        try:
            status = self._exchange(b"SPWB1 STATUS\n", time.monotonic() + self.timeout)
            return status
        except WakeBeaconError as exc:
            status = exc.status
            raise
        finally:
            self._last_operation = (status, None, False)

    def request_wake(self) -> WakeBeaconStatus:
        """Preflight, send one fresh WAKE, and await this request's completed burst.

        Disconnects, resets, command rejection and uncertain outcomes raise a
        WakeBeaconError. None causes reconnection or rebroadcast.
        """
        deadline = time.monotonic() + self.timeout
        status = None
        request_id = None
        wake_sent = False
        try:
            while True:
                status = self._exchange(b"SPWB1 STATUS\n", deadline)
                if status.state in {"queued", "broadcasting", "busy"}:
                    raise CommandRejected("busy", status)
                if status.state == "unconfigured":
                    raise OperationFailed("Beacon is unconfigured", status)
                if not status.radio_ready:
                    if status.state == "failed" or status.failures:
                        raise OperationFailed(
                            "Beacon radio initialization failed", status
                        )
                    # Configuration is not inspected until radio startup finishes.
                    time.sleep(min(0.1, self._remaining(deadline)))
                    continue
                if status.busy:
                    raise CommandRejected("busy", status)
                if not status.configured:
                    raise OperationFailed("Beacon is unconfigured", status)
                break
            # Exclude the retained ID without an unbounded random retry loop.
            request_id = secrets.randbelow(MAX_REQUEST_ID - 1) + 1
            if request_id >= status.request_id > 0:
                request_id += 1
            previous = status
            wake_sent = True  # Even a failed write may have reached the device.
            status = self._exchange(
                f"SPWB1 WAKE {request_id}\n".encode("ascii"), deadline
            )
            while True:
                if status.request_id != request_id:
                    raise RequestLost(
                        "Beacon request changed or device reset; completion is unknown",
                        status,
                    )
                # Cleanup can increment both counters: failure always takes priority.
                if status.failures != previous.failures or status.state in {
                    "failed",
                    "unconfigured",
                }:
                    raise OperationFailed("Beacon advertising burst failed", status)
                if status.state == "complete":
                    return status
                if status.state not in {"queued", "broadcasting"}:
                    raise RequestLost(
                        "Beacon no longer reports this request as active", status
                    )
                previous = status
                time.sleep(min(0.1, self._remaining(deadline)))
                status = self._exchange(b"SPWB1 STATUS\n", deadline)
        except WakeBeaconError as exc:
            if exc.status is None:
                exc.status = status
            else:
                status = exc.status
            exc.request_id = request_id
            exc.wake_sent = wake_sent
            raise
        finally:
            self._last_operation = (status, request_id, wake_sent)


def _open_client(port: str | None, timeout: float) -> WakeBeaconClient:
    timeout = _positive_timeout(timeout)
    selected = discover_beacon_port(port)
    try:
        transport = serial.Serial(
            port=selected,
            baudrate=115200,
            timeout=min(timeout, 0.1),
            write_timeout=min(timeout, 0.25),
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except (OSError, ValueError, serial.SerialException) as exc:
        raise TransportError(f"Cannot open beacon port {selected}: {exc}") from exc
    return WakeBeaconClient(transport, timeout)


def read_status(port: str | None = None, timeout: float = 15.0) -> WakeBeaconStatus:
    """Open, read status without waking, and close the selected beacon."""
    with _open_client(port, timeout) as client:
        return client.read_status()


def request_wake(port: str | None = None, timeout: float = 15.0) -> WakeBeaconStatus:
    """Open, request exactly one completed advertising burst, and close."""
    with _open_client(port, timeout) as client:
        return client.request_wake()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Request a wake-only beacon advertising burst over USB CDC."
    )
    parser.add_argument(
        "--port", help="Serial port (COM5 or /dev/ttyACM0); otherwise unique CAFE:4030"
    )
    parser.add_argument(
        "--status", action="store_true", help="Read status only; never send WAKE"
    )
    parser.add_argument(
        "--timeout",
        type=_positive_timeout,
        default=15.0,
        help="Finite positive deadline in seconds (default: 15)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit one JSON result, without human-readable stdout",
    )
    args = parser.parse_args(argv)
    action = "status" if args.status else "wake"
    try:
        status = (read_status if args.status else request_wake)(args.port, args.timeout)
    except WakeBeaconError as exc:
        result = {
            "ok": False,
            "action": action,
            "error": exc.code,
            "message": str(exc),
            "request_id": exc.request_id,
            "wake_sent": exc.wake_sent,
            "status": exc.status.to_dict() if exc.status else None,
        }
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(f"Wake beacon: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(
            json.dumps(
                {"ok": True, "action": action, "status": status.to_dict()},
                indent=2,
            )
        )
    elif args.status:
        print(json.dumps(status.to_dict(), indent=2, sort_keys=True))
    else:
        print(
            f"Advertising burst complete (request {status.request_id}); console power state is not confirmed."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
