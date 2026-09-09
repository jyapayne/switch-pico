#!/usr/bin/env python3
"""Temporarily emulate a neutral Wii Remote for a DolphinBar on Linux.

Run with sudo and press SYNC on the DolphinBar after the READY message.
Runs until Ctrl+C by default; --seconds N optionally sets a runtime limit.
No MAC spoofing, firmware changes, or persistent Bluetooth pairing keys.
The normal Bluetooth daemon is temporarily runtime-masked; settings and service
are restored on exit. --restore recovers an interrupted run from its journal.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import selectors
import signal
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from contextlib import ExitStack, contextmanager
from pathlib import Path
from typing import ClassVar, TypedDict, cast

# Keep this executable directly as well as through python -m tools....
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

AdapterInfo = TypedDict(
    "AdapterInfo",
    {
        "address": str,
        "version": int,
        "manufacturer": int,
        "supported": int,
        "settings": int,
        "class": str,
        "name": str,
    },
)


class RecoveryState(TypedDict):
    schema: int
    index: int
    info: AdapterInfo
    service_active: bool
    hci: dict[str, str]


POWERED = 1 << 0
CONNECTABLE = 1 << 1
DISCOVERABLE = 1 << 3
BONDABLE = 1 << 4
SSP = 1 << 6
BREDR = 1 << 7
ADVERTISING = 1 << 10
SECURE_CONN = 1 << 11
stop_requested = False


def log(event: str, **fields) -> None:
    print(
        json.dumps({"time": round(time.monotonic(), 3), "event": event, **fields}),
        flush=True,
    )


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, check=check, text=True, capture_output=True, timeout=20)


def address(raw: bytes) -> str:
    return ":".join(f"{b:02X}" for b in raw[::-1])


class Management:
    def __init__(self, index: int):
        self.index = index
        self.socket = socket.socket(
            socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI
        )
        self.socket.bind((0xFFFF, 3))  # HCI_CHANNEL_CONTROL
        self.socket.settimeout(5)

    def close(self):
        self.socket.close()

    def request(self, opcode: int, payload: bytes = b"") -> bytes:
        self.socket.send(
            struct.pack("<HHH", opcode, self.index, len(payload)) + payload
        )
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            packet = self.socket.recv(4096)
            if len(packet) < 9:
                continue
            event, index, size = struct.unpack_from("<HHH", packet)
            if index != self.index or size + 6 != len(packet) or event not in (1, 2):
                continue
            command, status = struct.unpack_from("<HB", packet, 6)
            if command != opcode:
                continue
            if status:
                raise RuntimeError(
                    f"MGMT command 0x{opcode:04x}: status 0x{status:02x}"
                )
            return packet[9:]
        raise TimeoutError(f"MGMT command 0x{opcode:04x} timed out")

    def info(self) -> AdapterInfo:
        data = self.request(4)
        if len(data) != 280:
            raise RuntimeError(f"Unexpected MGMT ReadInfo size {len(data)}")
        return {
            "address": address(data[:6]),
            "version": data[6],
            "manufacturer": struct.unpack_from("<H", data, 7)[0],
            "supported": struct.unpack_from("<I", data, 9)[0],
            "settings": struct.unpack_from("<I", data, 13)[0],
            "class": data[17:20].hex(),
            "name": data[20:269].split(b"\0", 1)[0].decode(errors="replace"),
        }

    def connections(self) -> int:
        data = self.request(0x15)
        if len(data) < 2:
            raise RuntimeError("Short MGMT connection list")
        count = struct.unpack_from("<H", data)[0]
        if len(data) != 2 + count * 7:
            raise RuntimeError("Malformed MGMT connection list")
        return count

    def setting(self, opcode: int, enabled: bool) -> None:
        self.request(opcode, bytes([int(enabled)]))

    def discoverable(self, mode: int, timeout: int = 0) -> None:
        self.request(6, struct.pack("<BH", mode, timeout))


class Hci:
    # read opcode -> (write opcode, expected payload length; -1 is variable)
    SETTINGS: ClassVar[dict[int, tuple[int, int]]] = {
        0x0C14: (0x0C13, 248),
        0x0C23: (0x0C24, 3),
        0x0C19: (0x0C1A, 1),
        0x0C39: (0x0C3A, -1),
        0x0C51: (0x0C52, 241),
        0x0C09: (0x0C0A, 1),
    }

    def __init__(self, index: int):
        self.socket = socket.socket(
            socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI
        )
        self.socket.bind((index,))
        self.socket.setsockopt(
            0, 2, struct.pack("<IIIH2x", 1 << 4, 0xFFFFFFFF, 0xFFFFFFFF, 0)
        )
        self.socket.settimeout(4)
        self.pending_events: deque[bytes] = deque()

    def close(self):
        self.socket.close()

    def send(self, opcode: int, payload: bytes = b"") -> None:
        if len(payload) > 255:
            raise ValueError("HCI command too large")
        self.socket.send(
            bytes([1]) + struct.pack("<HB", opcode, len(payload)) + payload
        )

    def command(self, opcode: int, payload: bytes = b"") -> bytes:
        self.send(opcode, payload)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            packet = self.socket.recv(1024)
            if len(packet) < 7 or packet[0] != 4 or len(packet) != packet[2] + 3:
                continue
            if packet[1] == 0x0E and struct.unpack_from("<H", packet, 4)[0] == opcode:
                if packet[6]:
                    raise RuntimeError(
                        f"HCI command 0x{opcode:04x}: status 0x{packet[6]:02x}"
                    )
                return packet[7:]
            if (
                packet[1] == 0x0F
                and struct.unpack_from("<H", packet, 5)[0] == opcode
                and packet[3]
            ):
                raise RuntimeError(
                    f"HCI command 0x{opcode:04x}: status 0x{packet[3]:02x}"
                )
            if packet[1] in (3, 4, 5, 6, 0x16, 0x18):
                self.pending_events.append(packet)
        raise TimeoutError(f"HCI command 0x{opcode:04x} timed out")

    def snapshot(self) -> dict[str, str]:
        result = {}
        for read_opcode, (write_opcode, size) in self.SETTINGS.items():
            data = self.command(read_opcode)
            if (size >= 0 and len(data) != size) or (
                size < 0 and (not data or len(data) != 1 + data[0] * 3)
            ):
                raise RuntimeError(
                    f"Unexpected HCI setting length for 0x{read_opcode:04x}"
                )
            result[str(write_opcode)] = data.hex()
        return result


class AdapterSession:
    def __init__(self, index: int, journal: Path):
        self.index = index
        self.journal = journal
        self.management = Management(index)
        self.hci = Hci(index)
        self.state: RecoveryState | None = None
        self.lock_resources = ExitStack()

    def acquire_lock(self) -> None:
        @contextmanager
        def held_lock():
            with open(
                f"/run/lock/switch-pico-dolphinbar-hci{self.index}.lock", "a"
            ) as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                yield

        self.lock_resources.enter_context(held_lock())

    def prepare(self) -> None:
        self.acquire_lock()
        if self.journal.exists():
            raise RuntimeError(
                f"Recovery journal exists. Run --restore {self.journal} first"
            )
        info = self.management.info()
        if not info["settings"] & POWERED or not info["settings"] & BREDR:
            raise RuntimeError(
                "Enable the adapter and Bluetooth Classic before running this tool"
            )
        if self.management.connections():
            raise RuntimeError(
                "Refusing to interrupt connected laptop Bluetooth devices"
            )
        if info["settings"] & (ADVERTISING | DISCOVERABLE):
            raise RuntimeError(
                "Stop existing laptop Bluetooth advertising/discovery before this test"
            )
        active = (
            run("systemctl", "is-active", "bluetooth.service", check=False).returncode
            == 0
        )
        masked = run(
            "systemctl", "is-enabled", "bluetooth.service", check=False
        ).stdout.strip()
        if masked not in ("enabled", "disabled", "static", "indirect", "alias"):
            raise RuntimeError(
                f"Unexpected Bluetooth service state {masked!r}; not changing it"
            )
        self.state = {
            "schema": 1,
            "index": self.index,
            "info": info,
            "service_active": active,
            "hci": self.hci.snapshot(),
        }
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.journal, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump(self.state, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        log(
            "adapter_saved",
            adapter=f"hci{self.index}",
            address=info["address"],
            journal=str(self.journal),
        )
        run("systemctl", "mask", "--runtime", "bluetooth.service")
        run("systemctl", "stop", "bluetooth.service")
        self.management.setting(5, False)
        if info["supported"] & SECURE_CONN:
            self.management.setting(0x2D, False)
        self.management.setting(0x0B, False)  # Classic legacy PIN, not SSP.
        self.management.setting(9, True)
        self.management.setting(7, True)
        self.management.setting(5, True)
        # Discovery starts only after all L2CAP listeners are bound.
        self.management.discoverable(0)
        self.hci.command(0x0C13, b"Nintendo RVL-CNT-01".ljust(248, b"\0"))
        self.hci.command(0x0C0A, b"\0")  # Variable PIN.

    def advertise(self, seconds: int | None) -> None:
        assert self.state is not None
        # Discovery is bounded independently of an established connection.
        self.management.discoverable(
            2, min(seconds, 180) if seconds is not None else 180
        )
        self.hci.command(0x0C24, bytes.fromhex("042500"))
        supported_iac = self.hci.command(0x0C38)
        laps = (
            bytes.fromhex("008b9e338b9e")
            if supported_iac and supported_iac[0] >= 2
            else bytes.fromhex("008b9e")
        )
        self.hci.command(0x0C3A, bytes([len(laps) // 3]) + laps)
        name = b"Nintendo RVL-CNT-01"
        eir = bytes([len(name) + 1, 9]) + name + bytes.fromhex("050324110012")
        self.hci.command(0x0C52, b"\0" + eir.ljust(240, b"\0"))
        self.hci.command(0x0C1A, b"\x03")
        log(
            "identity_ready",
            address=self.state["info"]["address"],
            name=name.decode(),
            device_class="002504",
            address_spoofed=False,
        )

    def restore(self) -> None:
        if self.state is None:
            return
        # Recovery can be invoked after a partial cleanup restarted BlueZ.
        # Prevent it from racing these controller commands in either path.
        run("systemctl", "mask", "--runtime", "bluetooth.service")
        run("systemctl", "stop", "bluetooth.service")
        self.hci.socket.settimeout(4)
        errors = []
        settings = self.state["info"]["settings"]

        def attempt(label, fn):
            try:
                fn()
            except (
                OSError,
                RuntimeError,
                ValueError,
                subprocess.SubprocessError,
            ) as exc:
                errors.append(f"{label}: {exc}")

        def restore_discoverable():
            wanted = bool(settings & DISCOVERABLE)
            current = bool(self.management.info()["settings"] & DISCOVERABLE)
            # Clearing connectable already clears discoverable. The kernel
            # rejects a redundant discoverable request when not connectable.
            if current != wanted:
                self.management.discoverable(int(wanted))

        attempt("power down", lambda: self.management.setting(5, False))
        attempt("SSP", lambda: self.management.setting(0x0B, bool(settings & SSP)))
        if self.state["info"]["supported"] & SECURE_CONN:
            attempt(
                "secure connections",
                lambda: self.management.setting(0x2D, bool(settings & SECURE_CONN)),
            )
        attempt(
            "bondable", lambda: self.management.setting(9, bool(settings & BONDABLE))
        )
        attempt(
            "connectable",
            lambda: self.management.setting(7, bool(settings & CONNECTABLE)),
        )
        attempt("power up", lambda: self.management.setting(5, True))
        attempt("discoverable", restore_discoverable)
        for opcode, value in self.state["hci"].items():
            attempt(
                f"HCI {opcode}",
                lambda opcode=opcode, value=value: self.hci.command(
                    int(opcode), bytes.fromhex(value)
                ),
            )
        attempt(
            "original power",
            lambda: self.management.setting(5, bool(settings & POWERED)),
        )
        attempt(
            "unmask service",
            lambda: run("systemctl", "unmask", "--runtime", "bluetooth.service"),
        )
        if self.state["service_active"]:
            attempt(
                "restart service",
                lambda: run("systemctl", "start", "bluetooth.service"),
            )
        if errors:
            log(
                "restore_failed",
                errors=errors,
                recovery=f"sudo {sys.executable} {__file__} --restore {self.journal}",
            )
            raise RuntimeError("Restoration incomplete; recovery journal retained")
        self.journal.unlink(missing_ok=True)
        log(
            "adapter_restored",
            adapter=f"hci{self.index}",
            service_active=self.state["service_active"],
        )
        self.state = None

    def close(self):
        self.hci.close()
        self.management.close()
        self.lock_resources.close()


class Peripheral:
    def __init__(self, session: AdapterSession, pin_mode: str, peer: str | None):
        from tools.dolphinbar_sdp import SdpResponder
        from tools.dolphinbar_wiimote import HID_DESCRIPTOR, Wiimote

        self.session = session
        self.pin_mode = pin_mode
        self.peer = peer.upper() if peer else None
        self.sdp_factory = lambda: SdpResponder(HID_DESCRIPTOR)
        self.sdp_sessions: dict[socket.socket, SdpResponder] = {}
        self.peer_mtus: dict[socket.socket, int] = {}
        self.wiimote = Wiimote()
        self.selector = selectors.DefaultSelector()
        self.listeners: list[socket.socket] = []
        self.clients: dict[socket.socket, int] = {}
        self.pending: dict[socket.socket, deque[bytes]] = {}
        self.waiting_interrupt: deque[bytes] = deque()
        self.interrupt: socket.socket | None = None
        self.commands = 0
        self.sent = 0
        self.pins = 0
        self.authenticated: bool | None = None
        self.generated_link_key = False
        self.last_ir = False
        assert session.state is not None
        local = session.state["info"]["address"]
        self.local_address = local
        try:
            for psm in (1, 0x11, 0x13):
                listener = socket.socket(
                    socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP
                )
                self.listeners.append(listener)
                listener.setsockopt(274, 4, bytes([1, 0]))  # BT_SECURITY_LOW
                listener.bind((local, psm))
                listener.listen(1)
                listener.setblocking(False)
                self.selector.register(listener, selectors.EVENT_READ, ("listen", psm))
            session.hci.socket.setblocking(False)
            self.selector.register(session.hci.socket, selectors.EVENT_READ, ("hci", 0))
        except BaseException:
            self.close()
            raise

    def close_client(self, client):
        psm = self.clients.pop(client, None)
        self.pending.pop(client, None)
        self.sdp_sessions.pop(client, None)
        self.peer_mtus.pop(client, None)
        try:
            self.selector.unregister(client)
        except (KeyError, ValueError):
            pass
        if client is self.interrupt:
            self.interrupt = None
        client.close()
        if psm is not None:
            log("channel_closed", psm=hex(psm))

    def close(self):
        for client in list(self.clients):
            self.close_client(client)
        for listener in self.listeners:
            listener.close()
        self.selector.close()

    def queue(self, client, packet: bytes):
        if client is None:
            if len(self.waiting_interrupt) >= 512:
                raise RuntimeError("Peer exhausted pre-interrupt response queue")
            self.waiting_interrupt.append(packet)
            return
        queue = self.pending[client]
        if len(queue) >= 512:
            raise RuntimeError("Peer exhausted bounded response queue")
        queue.append(packet)
        self.selector.modify(
            client,
            selectors.EVENT_READ | selectors.EVENT_WRITE,
            ("client", self.clients[client]),
        )

    def hci_event(self, packet: bytes):
        if len(packet) < 3 or packet[0] != 4 or len(packet) != packet[2] + 3:
            return
        event, data = packet[1], packet[3:]
        if event == 0x16 and len(data) == 6:  # PIN_CODE_REQUEST
            remote = address(data)
            if self.peer and remote != self.peer:
                self.session.management.request(0x0017, data + b"\0")
                log("pin_rejected_other_peer", peer=remote)
                return
            self.peer = remote
            pin = (
                data
                if self.pin_mode == "host"
                else bytes.fromhex(self.local_address.replace(":", ""))[::-1]
            )
            # MGMT accepts binary PINs and updates the kernel's pairing state.
            self.session.management.request(0x0016, data + b"\0\x06" + pin + bytes(10))
            self.pins += 1
            log("legacy_pin_replied", peer=remote, mode=self.pin_mode)
        elif event == 3 and len(data) >= 11:
            log(
                "acl_connection",
                status=data[0],
                peer=address(data[3:9]),
                encryption=data[10],
            )
        elif event == 6 and len(data) == 3:
            self.authenticated = data[0] == 0
            log(
                "authentication",
                status=data[0],
                handle=struct.unpack_from("<H", data, 1)[0],
            )
        elif event == 5 and len(data) == 4:
            log("acl_disconnected", reason=data[3])
        elif event == 0x18 and len(data) == 23:
            self.generated_link_key = True
            log("link_key_generated", peer=address(data[:6]), persisted=False)
        elif event == 4 and len(data) == 10:
            log(
                "connection_requested",
                peer=address(data[:6]),
                device_class=data[6:9].hex(),
            )

    def receive(self, client, psm: int, packet: bytes):
        if psm == 1:
            reply = self.sdp_sessions[client].reply(packet, self.peer_mtus[client])
            self.queue(client, reply)
            log("sdp", request=packet.hex(), response_bytes=len(reply))
            return
        if not packet:
            return
        prefix = packet[0]
        if prefix in (0xA2, 0x52):
            log("wiimote_command", psm=hex(psm), report=packet[1:].hex())
            replies = self.wiimote.handle_output(packet[1:])
            self.commands += 1
            if prefix == 0x52 and psm == 0x11:
                self.queue(client, b"\0")
            for reply in replies:
                self.queue(self.interrupt, reply)
            if self.wiimote.ir_enabled != self.last_ir:
                self.last_ir = self.wiimote.ir_enabled
                log(
                    "emulated_camera",
                    enabled=self.last_ir,
                    mode=hex(self.wiimote.report_mode),
                )
        elif psm == 0x11:
            if prefix == 0x15:  # Virtual cable unplug.
                self.close_client(client)
            elif prefix == 0x60:
                self.queue(client, b"\xa0\x01")
            elif prefix == 0x71 or prefix & 0xF0 == 0x90:
                self.queue(client, b"\0")
            elif prefix == 0x80:
                self.queue(client, b"\xa0\0")
            elif prefix == 0x41 and len(packet) >= 2 and packet[1] == 0x20:
                self.queue(client, self.wiimote.status_report())
            else:
                self.queue(client, b"\x03")
                log("unsupported_hid_control", packet=packet.hex())

    def serve(self, seconds: int | None):
        self.session.hci.socket.settimeout(4)
        self.session.advertise(seconds)
        self.session.hci.socket.setblocking(False)
        while self.session.hci.pending_events:
            self.hci_event(self.session.hci.pending_events.popleft())
        log(
            "READY",
            message="Listening; press only the DolphinBar SYNC button now",
            seconds=seconds,
            lifetime="until Ctrl+C" if seconds is None else f"{seconds} seconds",
            pin_mode=self.pin_mode,
        )
        start = time.monotonic()
        next_report = start
        next_status = start + 10
        while not stop_requested and (
            seconds is None or time.monotonic() - start < seconds
        ):
            for key, events in self.selector.select(0.01):
                category, psm = key.data
                channel_socket = cast(socket.socket, key.fileobj)
                if category == "hci":
                    try:
                        self.hci_event(channel_socket.recv(1024))
                    except BlockingIOError:
                        pass
                elif category == "listen":
                    client, remote = channel_socket.accept()
                    remote_address = remote[0].upper()
                    if self.peer and self.peer != remote_address:
                        client.close()
                        log("rejected_other_peer", peer=remote_address)
                        continue
                    if psm != 1:
                        self.peer = remote_address
                    if any(value == psm for value in self.clients.values()):
                        client.close()
                        continue
                    client.setblocking(False)
                    self.clients[client] = psm
                    self.pending[client] = deque()
                    options = client.getsockopt(6, 1, 12)  # L2CAP_OPTIONS
                    self.peer_mtus[client] = struct.unpack_from("<H", options)[0]
                    if psm == 1:
                        self.sdp_sessions[client] = self.sdp_factory()
                    self.selector.register(
                        client, selectors.EVENT_READ, ("client", psm)
                    )
                    log("channel_connected", psm=hex(psm), peer=remote_address)
                    if psm == 0x13:
                        self.interrupt = client
                        while self.waiting_interrupt:
                            self.queue(client, self.waiting_interrupt.popleft())
                        self.queue(client, self.wiimote.status_report())
                else:
                    client = channel_socket
                    try:
                        if events & selectors.EVENT_READ:
                            packet = client.recv(4096)
                            if not packet:
                                self.close_client(client)
                                continue
                            try:
                                self.receive(client, psm, packet)
                            except ValueError as exc:
                                log(
                                    "malformed_peer_request",
                                    psm=hex(psm),
                                    error=str(exc),
                                )
                                self.close_client(client)
                                continue
                        if client in self.clients and events & selectors.EVENT_WRITE:
                            queue = self.pending[client]
                            if queue:
                                packet = queue[0]
                                if client.send(packet) != len(packet):
                                    raise RuntimeError(
                                        "Partial L2CAP sequenced-packet write"
                                    )
                                queue.popleft()
                                self.sent += 1
                            if not queue:
                                self.selector.modify(
                                    client, selectors.EVENT_READ, ("client", psm)
                                )
                    except BlockingIOError:
                        pass
                    except OSError as exc:
                        log("channel_error", psm=hex(psm), error=str(exc))
                        self.close_client(client)
            now = time.monotonic()
            if now >= next_report:
                next_report = now + 0.01
                if self.interrupt and not self.pending[self.interrupt]:
                    report = self.wiimote.periodic_report()
                    if report:
                        self.queue(self.interrupt, report)
            if now >= next_status:
                next_status = now + 10
                log(
                    "status",
                    elapsed=round(now - start, 1),
                    peer=self.peer,
                    commands=self.commands,
                    sent=self.sent,
                    channels=sorted(self.clients.values()),
                    ir_enabled=self.wiimote.ir_enabled,
                )
        log(
            "finished",
            peer=self.peer,
            commands=self.commands,
            sent=self.sent,
            pin_requests=self.pins,
            authenticated=self.authenticated,
            link_key_generated=self.generated_link_key,
            ir_enabled=self.wiimote.ir_enabled,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter", default="hci0")
    parser.add_argument(
        "--seconds",
        type=int,
        help="Stop after N seconds; omitted means run until Ctrl+C",
    )
    parser.add_argument("--pin-mode", choices=("host", "device"), default="host")
    parser.add_argument(
        "--peer", help="Restrict the experiment to one Bluetooth address"
    )
    parser.add_argument("--state-file", type=Path)
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Read adapter information without changing it",
    )
    parser.add_argument(
        "--restore",
        type=Path,
        help="Restore settings from an interrupted run's journal",
    )
    args = parser.parse_args()
    if not sys.platform.startswith("linux") or os.geteuid() != 0:
        parser.error("Linux and root/CAP_NET_ADMIN access are required; run with sudo")
    if not args.adapter.startswith("hci") or not args.adapter[3:].isdigit():
        parser.error("--adapter must be hciN")
    index = int(args.adapter[3:])
    if index > 65534 or (args.seconds is not None and args.seconds < 1):
        parser.error("Invalid adapter index or duration (must be positive)")
    if args.peer:
        try:
            if len(bytes.fromhex(args.peer.replace(":", ""))) != 6:
                raise ValueError
        except ValueError:
            parser.error("--peer must be a Bluetooth address")
    journal = args.state_file or Path("build-dolphinbar") / f"hci{index}-state.json"
    state: RecoveryState | None = None
    if args.restore:
        journal = args.restore
        state = cast(RecoveryState, json.loads(journal.read_text()))
        if state.get("schema") != 1:
            parser.error("Unsupported recovery journal")
        index = state["index"]
        if type(index) is not int or not 0 <= index <= 65534:
            parser.error("Invalid adapter index in recovery journal")
    session = AdapterSession(index, journal)
    peripheral = None
    try:
        if args.restore:
            assert state is not None
            session.acquire_lock()
            if session.management.info()["address"] != state["info"]["address"]:
                raise RuntimeError(
                    "Recovery journal belongs to a different Bluetooth adapter"
                )
            session.state = state
            session.restore()
        elif args.inspect:
            log(
                "inspection",
                **session.management.info(),
                connections=session.management.connections(),
                hci=session.hci.snapshot(),
            )
        else:

            def stop(_signal, _frame):
                global stop_requested
                stop_requested = True

            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, stop)
            session.prepare()
            peripheral = Peripheral(session, args.pin_mode, args.peer)
            peripheral.serve(args.seconds)
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        log("error", error=str(exc))
        return 1
    finally:
        if peripheral:
            peripheral.close()
        try:
            session.restore()
        finally:
            session.close()


if __name__ == "__main__":
    raise SystemExit(main())
