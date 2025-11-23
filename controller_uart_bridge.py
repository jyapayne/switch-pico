#!/usr/bin/env python3
"""
Bridge multiple SDL2 controllers to switch-pico over UART and mirror rumble back.

The framing matches ``switch-pico.cpp``:
  - Host -> Pico : 0xAA, buttons (LE16), hat, lx, ly, rx, ry
  - Pico -> Host : 0xBB, 0x01, 8 rumble bytes, checksum (sum of first 10 bytes)

Features inspired by ``host/controller_bridge.py``:
  - Multiple controllers paired to multiple UART ports
  - Rich-powered interactive pairing UI
  - Adjustable send frequency, deadzone, and trigger thresholds
  - Rumble feedback delivered to SDL2 controllers
"""

from __future__ import annotations

import argparse
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import serial
from serial import SerialException
from serial.tools import list_ports
from serial.tools import list_ports_common

import sdl2
import sdl2.ext
from rich.console import Console
from rich.prompt import Prompt
from rich.table import Table

UART_HEADER = 0xAA
RUMBLE_HEADER = 0xBB
RUMBLE_TYPE_RUMBLE = 0x01
UART_BAUD = 921600
RUMBLE_IDLE_TIMEOUT = 0.25  # seconds without packets before forcing rumble off
RUMBLE_STUCK_TIMEOUT = 0.60  # continuous same-energy rumble will be stopped after this
RUMBLE_MIN_ACTIVE = 0.50  # below this, rumble is treated as off/noise
RUMBLE_SCALE = 0.8


class SwitchButton:
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


class SwitchHat:
    TOP = 0x00
    TOP_RIGHT = 0x01
    RIGHT = 0x02
    BOTTOM_RIGHT = 0x03
    BOTTOM = 0x04
    BOTTOM_LEFT = 0x05
    LEFT = 0x06
    TOP_LEFT = 0x07
    CENTER = 0x08


def parse_mapping(value: str) -> Tuple[int, str]:
    if ":" not in value:
        raise argparse.ArgumentTypeError("Mapping must look like 'index:serial_port'")
    idx_str, port = value.split(":", 1)
    try:
        idx = int(idx_str, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid controller index '{idx_str}'") from exc
    if not port:
        raise argparse.ArgumentTypeError("Serial port cannot be empty")
    return idx, port.strip()


def axis_to_stick(value: int, deadzone: int) -> int:
    if abs(value) < deadzone:
        value = 0
    scaled = int((value + 32768) * 255 / 65535)
    return max(0, min(255, scaled))


def trigger_to_button(value: int, threshold: int) -> bool:
    return value >= threshold


def set_hint(name: str, value: str) -> None:
    """Set an SDL hint safely even if the constant is missing in PySDL2."""
    try:
        sdl2.SDL_SetHint(name.encode(), value.encode())
    except Exception:
        pass


BUTTON_MAP = {
    # Direct mapping to the Switch PRO mask definitions in switch_pro_descriptors.h
    sdl2.SDL_CONTROLLER_BUTTON_A: SwitchButton.A,
    sdl2.SDL_CONTROLLER_BUTTON_B: SwitchButton.B,
    sdl2.SDL_CONTROLLER_BUTTON_X: SwitchButton.X,
    sdl2.SDL_CONTROLLER_BUTTON_Y: SwitchButton.Y,
    sdl2.SDL_CONTROLLER_BUTTON_LEFTSHOULDER: SwitchButton.L,
    sdl2.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: SwitchButton.R,
    sdl2.SDL_CONTROLLER_BUTTON_BACK: SwitchButton.MINUS,
    sdl2.SDL_CONTROLLER_BUTTON_START: SwitchButton.PLUS,
    sdl2.SDL_CONTROLLER_BUTTON_GUIDE: SwitchButton.HOME,
    sdl2.SDL_CONTROLLER_BUTTON_MISC1: SwitchButton.CAPTURE,
    sdl2.SDL_CONTROLLER_BUTTON_LEFTSTICK: SwitchButton.LCLICK,
    sdl2.SDL_CONTROLLER_BUTTON_RIGHTSTICK: SwitchButton.RCLICK,
}

DPAD_BUTTONS = {
    sdl2.SDL_CONTROLLER_BUTTON_DPAD_UP: "up",
    sdl2.SDL_CONTROLLER_BUTTON_DPAD_DOWN: "down",
    sdl2.SDL_CONTROLLER_BUTTON_DPAD_LEFT: "left",
    sdl2.SDL_CONTROLLER_BUTTON_DPAD_RIGHT: "right",
}


def dpad_to_hat(flags: Dict[str, bool]) -> int:
    up = flags["up"]
    down = flags["down"]
    left = flags["left"]
    right = flags["right"]

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


def is_usb_serial(path: str) -> bool:
    if path.startswith("/dev/tty.") and not path.startswith("/dev/tty.usb"):
        return False
    if path.startswith("/dev/cu.") and not path.startswith("/dev/cu.usb"):
        return False
    return True


def is_usb_serial_port(port: list_ports_common.ListPortInfo) -> bool:
    """Heuristic: prefer ports with USB VID/PID; fall back to path hints."""
    if getattr(port, "vid", None) is not None or getattr(port, "pid", None) is not None:
        return True
    path = port.device or ""
    manufacturer = (getattr(port, "manufacturer", "") or "").upper()
    if "USB" in manufacturer:
        return True
    return is_usb_serial(path)


def discover_ports(
    include_non_usb: bool = False,
    ignore_descriptions: Optional[List[str]] = None,
    include_descriptions: Optional[List[str]] = None,
) -> List[Dict[str, str]]:
    ignored = [d.lower() for d in ignore_descriptions or []]
    includes = [d.lower() for d in include_descriptions or []]
    results: List[Dict[str, str]] = []
    for port in list_ports.comports():
        path = port.device or ""
        if not path:
            continue
        if not include_non_usb and not is_usb_serial_port(port):
            continue
        desc_lower = (port.description or "").lower()
        if includes and not any(keep in desc_lower for keep in includes):
            continue
        if any(skip in desc_lower for skip in ignored):
            continue
        results.append(
            {
                "device": path,
                "description": port.description or "Unknown",
            }
        )
    return results


def interactive_pairing(
    console: Console, controller_info: Dict[int, str], ports: List[Dict[str, str]]
) -> List[Tuple[int, str]]:
    available = ports.copy()
    mappings: List[Tuple[int, str]] = []
    for controller_idx in controller_info:
        if not available:
            console.print("[bold red]No more UART devices available for pairing.[/bold red]")
            break

        table = Table(
            title=f"Available UART Devices for Controller {controller_idx} ({controller_info[controller_idx]})"
        )
        table.add_column("Choice", justify="center")
        table.add_column("Port")
        table.add_column("Description")
        for i, port in enumerate(available):
            table.add_row(str(i), port["device"], port["description"])
        console.print(table)
        choices = [str(i) for i in range(len(available))] + ["q"]
        selection = Prompt.ask(
            "Select UART index (or 'q' to stop pairing)",
            choices=choices,
            default=choices[0] if choices else "q",
        )
        if selection == "q":
            break
        idx = int(selection)
        port = available.pop(idx)
        mappings.append((controller_idx, port["device"]))
        console.print(f"[bold green]Paired controller {controller_idx} with {port['device']}[/bold green]")
    return mappings


@dataclass
class SwitchReport:
    buttons: int = 0
    hat: int = SwitchHat.CENTER
    lx: int = 128
    ly: int = 128
    rx: int = 128
    ry: int = 128

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<BHBBBBB", UART_HEADER, self.buttons & 0xFFFF, self.hat & 0xFF, self.lx, self.ly, self.rx, self.ry
        )


class PicoUART:
    def __init__(self, port: str, baudrate: int = UART_BAUD) -> None:
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
        # Non-blocking write; no flush to avoid sync stalls.
        self.serial.write(report.to_bytes())

    def read_rumble_payload(self) -> Optional[bytes]:
        """
        Drain all currently available UART bytes into an internal buffer,
        then try to extract a single valid rumble frame.

        Frame format:
          0: 0xBB (RUMBLE_HEADER)
          1: type (0x01 for rumble)
          2-9: 8-byte rumble payload
          10: checksum (sum of first 10 bytes) & 0xFF
        """
        # Read whatever is waiting in OS buffer
        waiting = self.serial.in_waiting
        if waiting:
            self._buffer.extend(self.serial.read(waiting))

        while True:
            if not self._buffer:
                return None

            start = self._buffer.find(bytes([RUMBLE_HEADER]))
            if start < 0:
                # No header at all, drop garbage
                self._buffer.clear()
                return None

            # Not enough data for a full frame yet
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

            # Bad frame, drop this header and resync
            del self._buffer[:start + 1]

    def close(self) -> None:
        self.serial.close()


def decode_rumble(payload: bytes) -> Tuple[float, float]:
    """Return normalized rumble amplitudes (0.0-1.0) for left/right."""
    if len(payload) < 8:
        return 0.0, 0.0
    # Neutral/idle pattern used by Switch: no rumble energy.
    if payload == b"\x00\x01\x40\x40\x00\x01\x40\x40":
        return 0.0, 0.0
    # Rumble amp is 10 bits across bytes 0/1 and 4/5.
    # Switch format is right rumble first, then left rumble (4 bytes each).
    right_raw = ((payload[1] & 0x03) << 8) | payload[0]
    left_raw = ((payload[5] & 0x03) << 8) | payload[4]
    if left_raw < 8 and right_raw < 8:
        return 0.0, 0.0
    left = min(max(left_raw / 1023.0, 0.0), 1.0)
    right = min(max(right_raw / 1023.0, 0.0), 1.0)
    return left, right


def apply_rumble(controller: sdl2.SDL_GameController, payload: bytes) -> float:
    left_norm, right_norm = decode_rumble(payload)
    max_norm = max(left_norm, right_norm)
    # Treat small rumble as "off" to avoid idle buzz.
    if max_norm < RUMBLE_MIN_ACTIVE:
        sdl2.SDL_GameControllerRumble(controller, 0, 0, 0)
        return 0.0
    # Attenuate to feel closer to a real controller; cap at ~25% strength.
    scale = RUMBLE_SCALE
    low = int(min(1.0, left_norm * scale) * 0xFFFF)   # SDL: low_frequency_rumble
    high = int(min(1.0, right_norm * scale) * 0xFFFF)  # SDL: high_frequency_rumble
    duration = 10
    sdl2.SDL_GameControllerRumble(controller, low, high, duration)
    return max_norm


@dataclass
class ControllerContext:
    controller: sdl2.SDL_GameController
    instance_id: int
    controller_index: int
    port: Optional[str]
    uart: Optional[PicoUART]
    report: SwitchReport = field(default_factory=SwitchReport)
    dpad: Dict[str, bool] = field(default_factory=lambda: {"up": False, "down": False, "left": False, "right": False})
    button_state: Dict[int, bool] = field(default_factory=dict)
    last_trigger_state: Dict[str, bool] = field(default_factory=lambda: {"left": False, "right": False})
    last_send: float = 0.0
    last_reopen_attempt: float = 0.0
    last_rumble: float = 0.0
    last_rumble_change: float = 0.0
    last_rumble_energy: float = 0.0
    rumble_active: bool = False


def open_controller(index: int) -> Tuple[sdl2.SDL_GameController, int]:
    controller = sdl2.SDL_GameControllerOpen(index)
    if not controller:
        raise RuntimeError(f"Failed to open controller {index}: {sdl2.SDL_GetError().decode()}")
    joystick = sdl2.SDL_GameControllerGetJoystick(controller)
    instance_id = sdl2.SDL_JoystickInstanceID(joystick)
    return controller, instance_id


def try_open_uart(port: str, baud: int) -> Optional[PicoUART]:
    try:
        return PicoUART(port, baud)
    except Exception:
        return None


def open_uart_or_warn(port: str, baud: int, console: Console) -> Optional[PicoUART]:
    try:
        return PicoUART(port, baud)
    except Exception as exc:
        console.print(f"[yellow]Failed to open UART {port}: {exc}[/yellow]")
        return None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bridge SDL2 controllers to switch-pico UART (with rumble)")
    parser.add_argument(
        "--map",
        action="append",
        type=parse_mapping,
        default=[],
        help="Controller mapping 'index:serial_port'. Repeat per controller.",
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        help="Serial ports to auto-pair with controllers in ascending index order.",
    )
    parser.add_argument("--interactive", action="store_true", help="Launch an interactive pairing UI using Rich.")
    parser.add_argument("--all-ports", action="store_true", help="Include non-USB serial ports when listing devices.")
    parser.add_argument(
        "--frequency",
        type=float,
        default=500.0,
        help="Report send frequency per controller (Hz, default 500)",
    )
    parser.add_argument("--deadzone", type=float, default=0.08, help="Stick deadzone (0.0-1.0, default 0.08)")
    parser.add_argument(
        "--trigger-threshold",
        type=float,
        default=0.35,
        help="Trigger threshold treated as a digital press (0.0-1.0, default 0.35)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=UART_BAUD,
        help=f"UART baud rate (default {UART_BAUD}; must match switch-pico firmware)",
    )
    parser.add_argument(
        "--ignore-port-desc",
        action="append",
        default=[],
        help="Substring filter to exclude serial ports by description (case-insensitive). Repeatable.",
    )
    parser.add_argument(
        "--include-port-desc",
        action="append",
        default=[],
        help="Only include serial ports whose description contains this substring (case-insensitive). Repeatable.",
    )
    parser.add_argument(
        "--include-controller-name",
        action="append",
        default=[],
        help="Only open controllers whose name contains this substring (case-insensitive). Repeatable.",
    )
    parser.add_argument(
        "--swap-abxy",
        action="store_true",
        help="Swap AB/XY mapping (useful if Linux reports Switch controllers as Xbox layout).",
    )
    parser.add_argument(
        "--swap-abxy-index",
        action="append",
        type=int,
        default=[],
        help="Swap AB/XY mapping for specific controller indices (repeatable).",
    )
    parser.add_argument(
        "--sdl-mapping",
        action="append",
        default=[],
        help="Path to an SDL2 controller mapping database (e.g. controllerdb.txt). Repeatable.",
    )
    return parser


def poll_controller_buttons(ctx: ControllerContext, button_map: Dict[int, int]) -> None:
    changed = False
    for sdl_button, switch_bit in button_map.items():
        pressed = bool(sdl2.SDL_GameControllerGetButton(ctx.controller, sdl_button))
        previous = ctx.button_state.get(sdl_button)
        if previous == pressed:
            continue
        ctx.button_state[sdl_button] = pressed
        if pressed:
            ctx.report.buttons |= switch_bit
        else:
            ctx.report.buttons &= ~switch_bit
        changed = True

    dpad_changed = False
    for sdl_button, name in DPAD_BUTTONS.items():
        pressed = bool(sdl2.SDL_GameControllerGetButton(ctx.controller, sdl_button))
        if ctx.dpad[name] == pressed:
            continue
        ctx.dpad[name] = pressed
        dpad_changed = True

    if dpad_changed:
        ctx.report.hat = dpad_to_hat(ctx.dpad)


@dataclass
class BridgeConfig:
    interval: float
    deadzone_raw: int
    trigger_threshold: int
    button_map_default: Dict[int, int]
    button_map_swapped: Dict[int, int]
    swap_abxy_indices: set[int]


@dataclass
class PairingState:
    mapping_by_index: Dict[int, str]
    available_ports: List[str]
    auto_assigned_indices: set[int] = field(default_factory=set)
    auto_pairing_enabled: bool = False
    auto_discover_ports: bool = False
    include_non_usb: bool = False
    ignore_port_desc: List[str] = field(default_factory=list)
    include_port_desc: List[str] = field(default_factory=list)


def load_button_maps(console: Console, args: argparse.Namespace) -> Tuple[Dict[int, int], Dict[int, int], set[int]]:
    """Load SDL controller mappings and return button map variants."""
    default_mapping = Path(__file__).parent / "controller_db" / "gamecontrollerdb.txt"
    mappings_to_load: List[str] = []
    if default_mapping.exists():
        mappings_to_load.append(str(default_mapping))
    mappings_to_load.extend(args.sdl_mapping)
    button_map_default = dict(BUTTON_MAP)
    button_map_swapped = dict(BUTTON_MAP)
    button_map_swapped[sdl2.SDL_CONTROLLER_BUTTON_A] = SwitchButton.B
    button_map_swapped[sdl2.SDL_CONTROLLER_BUTTON_B] = SwitchButton.A
    button_map_swapped[sdl2.SDL_CONTROLLER_BUTTON_X] = SwitchButton.Y
    button_map_swapped[sdl2.SDL_CONTROLLER_BUTTON_Y] = SwitchButton.X
    swap_abxy_indices = {idx for idx in args.swap_abxy_index if idx is not None and idx >= 0}
    for mapping_path in mappings_to_load:
        try:
            loaded = sdl2.SDL_GameControllerAddMappingsFromFile(mapping_path.encode())
            console.print(f"[green]Loaded {loaded} SDL mapping(s) from {mapping_path}[/green]")
        except Exception as exc:
            console.print(f"[red]Failed to load SDL mapping {mapping_path}: {exc}[/red]")
    return button_map_default, button_map_swapped, swap_abxy_indices


def build_bridge_config(console: Console, args: argparse.Namespace) -> BridgeConfig:
    interval = 1.0 / max(args.frequency, 1.0)
    deadzone_raw = int(max(0.0, min(args.deadzone, 1.0)) * 32767)
    trigger_threshold = int(max(0.0, min(args.trigger_threshold, 1.0)) * 32767)
    button_map_default, button_map_swapped, swap_abxy_indices = load_button_maps(console, args)
    return BridgeConfig(
        interval=interval,
        deadzone_raw=deadzone_raw,
        trigger_threshold=trigger_threshold,
        button_map_default=button_map_default,
        button_map_swapped=button_map_swapped,
        swap_abxy_indices=swap_abxy_indices,
    )


def initialize_sdl(parser: argparse.ArgumentParser) -> None:
    sdl2.SDL_SetHint(sdl2.SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, b"1")
    set_hint("SDL_JOYSTICK_HIDAPI", "1")
    set_hint("SDL_JOYSTICK_HIDAPI_SWITCH", "1")
    # Use controller button labels so Nintendo layouts (ABXY) map correctly on Linux.
    set_hint("SDL_GAMECONTROLLER_USE_BUTTON_LABELS", "1")
    if sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER | sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_EVERYTHING) != 0:
        parser.error(f"SDL init failed: {sdl2.SDL_GetError().decode(errors='ignore')}")


def detect_controllers(
    console: Console, args: argparse.Namespace, parser: argparse.ArgumentParser
) -> Tuple[List[int], Dict[int, str]]:
    controller_indices: List[int] = []
    controller_names: Dict[int, str] = {}
    controller_count = sdl2.SDL_NumJoysticks()
    if controller_count < 0:
        parser.error(f"SDL error: {sdl2.SDL_GetError().decode()}")
    include_controller_name = [n.lower() for n in args.include_controller_name]
    for index in range(controller_count):
        if sdl2.SDL_IsGameController(index):
            name = sdl2.SDL_GameControllerNameForIndex(index)
            name_str = name.decode() if isinstance(name, bytes) else str(name)
            if include_controller_name and all(substr not in name_str.lower() for substr in include_controller_name):
                console.print(f"[yellow]Skipping controller {index} ({name_str}) due to name filter[/yellow]")
                continue
            console.print(f"[cyan]Detected controller {index}: {name_str}[/cyan]")
            controller_indices.append(index)
            controller_names[index] = name_str
        else:
            name = sdl2.SDL_JoystickNameForIndex(index)
            name_str = name.decode() if isinstance(name, bytes) else str(name)
            if include_controller_name and all(substr not in name_str.lower() for substr in include_controller_name):
                console.print(f"[yellow]Skipping joystick {index} ({name_str}) due to name filter[/yellow]")
                continue
            console.print(f"[yellow]Found joystick {index} (not a GameController): {name_str}[/yellow]")
    return controller_indices, controller_names


def prepare_pairing_state(
    args: argparse.Namespace,
    console: Console,
    parser: argparse.ArgumentParser,
    controller_indices: List[int],
    controller_names: Dict[int, str],
) -> PairingState:
    auto_pairing_enabled = not args.map and not args.interactive
    auto_discover_ports = auto_pairing_enabled and not args.ports
    include_non_usb = args.all_ports or False
    ignore_port_desc = [d.lower() for d in args.ignore_port_desc]
    include_port_desc = [d.lower() for d in args.include_port_desc]
    available_ports: List[str] = []

    mappings = list(args.map)
    if args.interactive:
        if not controller_indices:
            parser.error("No controllers detected for interactive pairing.")
        discovered = discover_ports(
            include_non_usb=include_non_usb,
            ignore_descriptions=ignore_port_desc,
            include_descriptions=include_port_desc,
        )
        if not discovered:
            parser.error("No UART devices found for interactive pairing.")
        mappings = interactive_pairing(console, controller_names, discovered)
        if not mappings:
            parser.error("No controller-to-UART mappings were selected.")
    elif auto_pairing_enabled:
        if args.ports:
            available_ports.extend(list(args.ports))
            console.print(f"[green]Prepared {len(available_ports)} specified UART port(s) for auto-pairing.[/green]")
        else:
            discovered = discover_ports(
                include_non_usb=include_non_usb,
                ignore_descriptions=ignore_port_desc,
                include_descriptions=include_port_desc,
            )
            if discovered:
                available_ports.extend(info["device"] for info in discovered)
                console.print("[green]Auto-detected UARTs:[/green]")
                for info in discovered:
                    console.print(f"  {info['device']} ({info['description']})")
            else:
                console.print("[yellow]No UART devices detected yet; waiting for hotplug...[/yellow]")

    mapping_by_index = {index: port for index, port in mappings}
    return PairingState(
        mapping_by_index=mapping_by_index,
        available_ports=available_ports,
        auto_pairing_enabled=auto_pairing_enabled,
        auto_discover_ports=auto_discover_ports,
        include_non_usb=include_non_usb,
        ignore_port_desc=ignore_port_desc,
        include_port_desc=include_port_desc,
    )


def assign_port_for_index(pairing: PairingState, idx: int, console: Console) -> Optional[str]:
    if idx in pairing.mapping_by_index:
        return pairing.mapping_by_index[idx]
    if not pairing.auto_pairing_enabled:
        return None
    if not pairing.available_ports:
        return None
    port_choice = pairing.available_ports.pop(0)
    pairing.mapping_by_index[idx] = port_choice
    pairing.auto_assigned_indices.add(idx)
    console.print(f"[green]Auto-paired controller {idx} to {port_choice}[/green]")
    return port_choice


def ports_in_use(pairing: PairingState, contexts: Dict[int, ControllerContext]) -> set:
    used = set(pairing.mapping_by_index.values())
    used.update(ctx.port for ctx in contexts.values() if ctx.port)
    return used


def discover_new_ports(pairing: PairingState, contexts: Dict[int, ControllerContext], console: Console) -> None:
    if not pairing.auto_discover_ports:
        return
    discovered = discover_ports(
        include_non_usb=pairing.include_non_usb,
        ignore_descriptions=pairing.ignore_port_desc,
        include_descriptions=pairing.include_port_desc,
    )
    in_use = ports_in_use(pairing, contexts)
    for info in discovered:
        path = info["device"]
        if path in in_use or path in pairing.available_ports:
            continue
        pairing.available_ports.append(path)
        console.print(f"[green]Discovered UART {path} ({info['description']}); available for pairing.[/green]")


def pair_waiting_contexts(
    args: argparse.Namespace,
    pairing: PairingState,
    contexts: Dict[int, ControllerContext],
    uarts: List[PicoUART],
    console: Console,
) -> None:
    for ctx in list(contexts.values()):
        if ctx.port is not None:
            continue
        port_choice = assign_port_for_index(pairing, ctx.controller_index, console)
        if port_choice is None:
            continue
        ctx.port = port_choice
        uart = open_uart_or_warn(port_choice, args.baud, console)
        ctx.last_reopen_attempt = time.monotonic()
        if uart:
            uarts.append(uart)
            ctx.uart = uart
            console.print(f"[green]Controller {ctx.controller_index} ({ctx.instance_id}) paired to {port_choice}[/green]")
        else:
            ctx.uart = None
            console.print(f"[yellow]Controller {ctx.controller_index} ({ctx.instance_id}) waiting for UART {port_choice}[/yellow]")


def open_initial_contexts(
    args: argparse.Namespace, pairing: PairingState, controller_indices: List[int], console: Console
) -> Tuple[Dict[int, ControllerContext], List[PicoUART]]:
    contexts: Dict[int, ControllerContext] = {}
    uarts: List[PicoUART] = []
    for index in controller_indices:
        if index >= sdl2.SDL_NumJoysticks() or not sdl2.SDL_IsGameController(index):
            name = sdl2.SDL_JoystickNameForIndex(index)
            name_str = name.decode() if isinstance(name, bytes) else str(name)
            console.print(f"[yellow]Index {index} is not a GameController ({name_str}). Trying raw open failed.[/yellow]")
            continue
        port = assign_port_for_index(pairing, index, console)
        if port is None and not pairing.auto_pairing_enabled:
            continue
        try:
            controller, instance_id = open_controller(index)
        except Exception as exc:
            console.print(f"[red]Failed to open controller {index}: {exc}[/red]")
            continue
        uart = open_uart_or_warn(port, args.baud, console) if port else None
        if uart:
            uarts.append(uart)
            console.print(f"[green]Controller {index} ({instance_id}) paired to {port}[/green]")
        elif port:
            console.print(f"[yellow]Controller {index} ({instance_id}) waiting for UART {port}[/yellow]")
        else:
            console.print(f"[yellow]Controller {index} ({instance_id}) connected; waiting for an available UART[/yellow]")
        ctx = ControllerContext(
            controller=controller,
            instance_id=instance_id,
            controller_index=index,
            port=port,
            uart=uart,
        )
        contexts[instance_id] = ctx
    return contexts, uarts


def handle_axis_motion(event: sdl2.SDL_Event, contexts: Dict[int, ControllerContext], config: BridgeConfig) -> None:
    ctx = contexts.get(event.caxis.which)
    if not ctx:
        return
    axis = event.caxis.axis
    value = event.caxis.value
    if axis == sdl2.SDL_CONTROLLER_AXIS_LEFTX:
        ctx.report.lx = axis_to_stick(value, config.deadzone_raw)
    elif axis == sdl2.SDL_CONTROLLER_AXIS_LEFTY:
        ctx.report.ly = axis_to_stick(value, config.deadzone_raw)
    elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTX:
        ctx.report.rx = axis_to_stick(value, config.deadzone_raw)
    elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTY:
        ctx.report.ry = axis_to_stick(value, config.deadzone_raw)
    elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT:
        pressed = trigger_to_button(value, config.trigger_threshold)
        if pressed != ctx.last_trigger_state["left"]:
            if pressed:
                ctx.report.buttons |= SwitchButton.ZL
            else:
                ctx.report.buttons &= ~SwitchButton.ZL
            ctx.last_trigger_state["left"] = pressed
    elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
        pressed = trigger_to_button(value, config.trigger_threshold)
        if pressed != ctx.last_trigger_state["right"]:
            if pressed:
                ctx.report.buttons |= SwitchButton.ZR
            else:
                ctx.report.buttons &= ~SwitchButton.ZR
            ctx.last_trigger_state["right"] = pressed


def handle_button_event(
    event: sdl2.SDL_Event,
    args: argparse.Namespace,
    config: BridgeConfig,
    contexts: Dict[int, ControllerContext],
) -> None:
    ctx = contexts.get(event.cbutton.which)
    if not ctx:
        return
    current_button_map = (
        config.button_map_swapped
        if (args.swap_abxy or ctx.controller_index in config.swap_abxy_indices)
        else config.button_map_default
    )
    button = event.cbutton.button
    pressed = event.type == sdl2.SDL_CONTROLLERBUTTONDOWN
    if button in current_button_map:
        bit = current_button_map[button]
        if pressed:
            ctx.report.buttons |= bit
        else:
            ctx.report.buttons &= ~bit
        ctx.button_state[button] = pressed
    elif button in DPAD_BUTTONS:
        ctx.dpad[DPAD_BUTTONS[button]] = pressed
        ctx.report.hat = dpad_to_hat(ctx.dpad)


def handle_device_added(
    event: sdl2.SDL_Event,
    args: argparse.Namespace,
    pairing: PairingState,
    contexts: Dict[int, ControllerContext],
    uarts: List[PicoUART],
    console: Console,
) -> None:
    idx = event.cdevice.which
    if any(c.controller_index == idx for c in contexts.values()):
        return
    port = assign_port_for_index(pairing, idx, console)
    if port is None and not pairing.auto_pairing_enabled:
        return
    if idx >= sdl2.SDL_NumJoysticks() or not sdl2.SDL_IsGameController(idx):
        name = sdl2.SDL_JoystickNameForIndex(idx)
        name_str = name.decode() if isinstance(name, bytes) else str(name)
        console.print(f"[yellow]Index {idx} is not a GameController ({name_str}). Trying raw open failed.[/yellow]")
        return
    try:
        controller, instance_id = open_controller(idx)
    except Exception as exc:
        console.print(f"[red]Hotplug open failed for controller {idx}: {exc}[/red]")
        return
    uart = open_uart_or_warn(port, args.baud, console) if port else None
    if uart:
        uarts.append(uart)
        console.print(f"[green]Controller {idx} ({instance_id}) paired to {port}[/green]")
    elif port:
        console.print(f"[yellow]Controller {idx} ({instance_id}) waiting for UART {port}[/yellow]")
    else:
        console.print(f"[yellow]Controller {idx} ({instance_id}) connected; waiting for an available UART[/yellow]")
    ctx = ControllerContext(
        controller=controller,
        instance_id=instance_id,
        controller_index=idx,
        port=port,
        uart=uart,
    )
    contexts[instance_id] = ctx


def handle_device_removed(
    event: sdl2.SDL_Event,
    pairing: PairingState,
    contexts: Dict[int, ControllerContext],
    console: Console,
) -> None:
    instance_id = event.cdevice.which
    ctx = contexts.pop(instance_id, None)
    if not ctx:
        return
    console.print(f"[yellow]Controller {instance_id} removed[/yellow]")
    if ctx.controller_index in pairing.auto_assigned_indices:
        freed = pairing.mapping_by_index.pop(ctx.controller_index, None)
        pairing.auto_assigned_indices.discard(ctx.controller_index)
        if freed and freed not in pairing.available_ports:
            pairing.available_ports.append(freed)
            console.print(f"[cyan]Released UART {freed} back to pool[/cyan]")
    sdl2.SDL_GameControllerClose(ctx.controller)


def service_contexts(
    now: float,
    args: argparse.Namespace,
    config: BridgeConfig,
    contexts: Dict[int, ControllerContext],
    uarts: List[PicoUART],
    console: Console,
) -> None:
    for ctx in list(contexts.values()):
        current_button_map = (
            config.button_map_swapped
            if (args.swap_abxy or ctx.controller_index in config.swap_abxy_indices)
            else config.button_map_default
        )
        poll_controller_buttons(ctx, current_button_map)
        # Reconnect UART if needed.
        if ctx.port and ctx.uart is None and (now - ctx.last_reopen_attempt) > 1.0:
            ctx.last_reopen_attempt = now
            uart = open_uart_or_warn(ctx.port, args.baud, console)
            if uart:
                uarts.append(uart)
                console.print(f"[green]Reconnected UART {ctx.port} for controller {ctx.controller_index}[/green]")
                ctx.uart = uart
        if ctx.uart is None:
            continue
        try:
            if now - ctx.last_send >= config.interval:
                ctx.uart.send_report(ctx.report)
                ctx.last_send = now

            last_payload = None
            while True:
                p = ctx.uart.read_rumble_payload()
                if not p:
                    break
                last_payload = p

            if last_payload is not None:
                energy = apply_rumble(ctx.controller, last_payload)
                ctx.rumble_active = energy >= RUMBLE_MIN_ACTIVE
                if ctx.rumble_active and energy != ctx.last_rumble_energy:
                    ctx.last_rumble_change = now
                ctx.last_rumble_energy = energy
                ctx.last_rumble = now
            elif ctx.rumble_active and (now - ctx.last_rumble) > RUMBLE_IDLE_TIMEOUT:
                sdl2.SDL_GameControllerRumble(ctx.controller, 0, 0, 0)
                ctx.rumble_active = False
                ctx.last_rumble_energy = 0.0
            elif ctx.rumble_active and (now - ctx.last_rumble_change) > RUMBLE_STUCK_TIMEOUT:
                sdl2.SDL_GameControllerRumble(ctx.controller, 0, 0, 0)
                ctx.rumble_active = False
                ctx.last_rumble_energy = 0.0
        except SerialException as exc:
            console.print(f"[yellow]UART {ctx.port} disconnected: {exc}[/yellow]")
            try:
                ctx.uart.close()
            except Exception:
                pass
            sdl2.SDL_GameControllerRumble(ctx.controller, 0, 0, 0)
            ctx.uart = None
            ctx.rumble_active = False
            ctx.last_rumble_energy = 0.0
            ctx.last_reopen_attempt = now
        except Exception as exc:
            console.print(f"[red]UART error on {ctx.port}: {exc}[/red]")


def run_bridge_loop(
    args: argparse.Namespace,
    console: Console,
    config: BridgeConfig,
    pairing: PairingState,
    contexts: Dict[int, ControllerContext],
    uarts: List[PicoUART],
) -> None:
    event = sdl2.SDL_Event()
    port_scan_interval = 2.0
    last_port_scan = time.monotonic()
    running = True

    while running:
        while sdl2.SDL_PollEvent(event):
            if event.type == sdl2.SDL_QUIT:
                running = False
                break
            if event.type == sdl2.SDL_CONTROLLERAXISMOTION:
                handle_axis_motion(event, contexts, config)
            elif event.type in (sdl2.SDL_CONTROLLERBUTTONDOWN, sdl2.SDL_CONTROLLERBUTTONUP):
                handle_button_event(event, args, config, contexts)
            elif event.type == sdl2.SDL_CONTROLLERDEVICEADDED:
                handle_device_added(event, args, pairing, contexts, uarts, console)
            elif event.type == sdl2.SDL_CONTROLLERDEVICEREMOVED:
                handle_device_removed(event, pairing, contexts, console)

        now = time.monotonic()
        if now - last_port_scan > port_scan_interval:
            discover_new_ports(pairing, contexts, console)
            last_port_scan = now
            pair_waiting_contexts(args, pairing, contexts, uarts, console)
        else:
            pair_waiting_contexts(args, pairing, contexts, uarts, console)
        service_contexts(now, args, config, contexts, uarts, console)
        sdl2.SDL_Delay(1)


def cleanup(contexts: Dict[int, ControllerContext], uarts: List[PicoUART]) -> None:
    for ctx in contexts.values():
        sdl2.SDL_GameControllerClose(ctx.controller)
    for uart in uarts:
        uart.close()
    sdl2.SDL_Quit()


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    console = Console()
    config = build_bridge_config(console, args)
    initialize_sdl(parser)
    contexts: Dict[int, ControllerContext] = {}
    uarts: List[PicoUART] = []
    try:
        controller_indices, controller_names = detect_controllers(console, args, parser)
        pairing = prepare_pairing_state(args, console, parser, controller_indices, controller_names)
        contexts, uarts = open_initial_contexts(args, pairing, controller_indices, console)
        if not contexts:
            console.print("[yellow]No controllers opened; waiting for hotplug events...[/yellow]")
        run_bridge_loop(args, console, config, pairing, contexts, uarts)
    finally:
        cleanup(contexts, uarts)


if __name__ == "__main__":
    main()
