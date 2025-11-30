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
import os
import sys
import time
import urllib.request
from dataclasses import dataclass, field
from ctypes import create_string_buffer
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from serial import SerialException
from serial.tools import list_ports
from serial.tools import list_ports_common

import sdl2
import sdl2.ext
from rich.console import Console
from rich.prompt import Prompt
from rich.table import Table

from switch_pico_uart import (
    UART_BAUD,
    PicoUART,
    SwitchButton,
    SwitchHat,
    SwitchReport,
    axis_to_stick,
    dpad_to_hat,
    decode_rumble,
    trigger_to_button,
)

RUMBLE_IDLE_TIMEOUT = 0.25  # seconds without packets before forcing rumble off
RUMBLE_STUCK_TIMEOUT = 0.60  # continuous same-energy rumble will be stopped after this
RUMBLE_MIN_ACTIVE = 0.50  # below this, rumble is treated as off/noise
RUMBLE_SCALE = 0.8
CONTROLLER_DB_URL_DEFAULT = (
    "https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/refs/heads/master/gamecontrollerdb.txt"
)


def parse_mapping(value: str) -> Tuple[int, str]:
    """Parse 'index:serial_port' CLI mapping argument."""
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


def download_controller_db(console: Console, destination: Path, url: str) -> bool:
    """Download the latest SDL controller DB to the local controller_db directory."""
    console.print(f"[cyan]Fetching SDL controller database from {url}...[/cyan]")
    try:
        with urllib.request.urlopen(url, timeout=20) as response:
            status = getattr(response, "status", 200)
            if status != 200:
                raise RuntimeError(f"HTTP {status}")
            data = response.read()
    except Exception as exc:
        console.print(f"[red]Failed to download controller database: {exc}[/red]")
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        destination.write_bytes(data)
    except Exception as exc:
        console.print(f"[red]Failed to write controller database to {destination}: {exc}[/red]")
        return False
    console.print(f"[green]Updated controller database ({len(data)} bytes) at {destination}[/green]")
    return True


def parse_hotkey(value: str) -> str:
    """Validate a single-character hotkey (empty string disables)."""
    if value is None:
        return ""
    value = value.strip()
    if not value:
        return ""
    if len(value) != 1:
        raise argparse.ArgumentTypeError("Hotkeys must be a single character (or empty to disable).")
    return value


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

STICK_AXIS_LABELS = (
    (sdl2.SDL_CONTROLLER_AXIS_LEFTX, "LX"),
    (sdl2.SDL_CONTROLLER_AXIS_LEFTY, "LY"),
    (sdl2.SDL_CONTROLLER_AXIS_RIGHTX, "RX"),
    (sdl2.SDL_CONTROLLER_AXIS_RIGHTY, "RY"),
)

STICK_AXES = tuple(axis for axis, _ in STICK_AXIS_LABELS)


def is_usb_serial(path: str) -> bool:
    """
    Heuristic for USB serial path prefixes (best-effort when VID/PID are missing).

    Accepts common USB-adapter patterns; rejects generic /dev/tty* unless they
    clearly indicate USB.
    """
    lower = path.lower()
    usb_prefixes = (
        "/dev/ttyusb",   # Linux USB serial
        "/dev/ttyacm",   # Linux CDC ACM
        "/dev/cu.usb",   # macOS cu/tty USB adapters
        "/dev/tty.usb",
    )
    if lower.startswith(usb_prefixes):
        return True
    # Default to False for unknown paths; caller can include_non_usb to override.
    return False


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
    """List serial ports, optionally filtering by description and USB-ness."""
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
    """Prompt the user to pair controllers to UART ports via Rich UI."""
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


def apply_rumble(controller: sdl2.SDL_GameController, payload: bytes) -> float:
    """Apply rumble payload to SDL controller and return max normalized energy."""
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
    stable_id: str
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
    axis_offsets: Dict[int, int] = field(default_factory=dict)


def capture_stick_offsets(controller: sdl2.SDL_GameController) -> Dict[int, int]:
    """Sample the current stick axes so they can be treated as the neutral center."""
    offsets: Dict[int, int] = {}
    for axis in STICK_AXES:
        offsets[axis] = int(sdl2.SDL_GameControllerGetAxis(controller, axis))
    return offsets


def format_axis_offsets(offsets: Dict[int, int]) -> str:
    """Return a human-friendly summary of per-axis offsets (for logging)."""
    return ", ".join(f"{label}={offsets.get(axis, 0):+d}" for axis, label in STICK_AXIS_LABELS)


def calibrate_axis_value(value: int, axis: int, ctx: ControllerContext) -> int:
    """Apply any stored calibration offset to a raw axis reading."""
    if not ctx.axis_offsets:
        return value
    offset = ctx.axis_offsets.get(axis)
    if offset is None:
        return value
    return max(-32768, min(32767, value - offset))


class HotkeyMonitor:
    """Platform-aware helper that watches for configured hotkeys without blocking the main loop."""

    def __init__(self, console: Console, key_messages: Optional[Dict[str, str]] = None) -> None:
        self.console = console
        self._platform = os.name
        self._fd: Optional[int] = None
        self._orig_termios = None
        self._msvcrt = None
        self._active = False
        self._started = False
        self._keys: Dict[str, str] = {}
        if key_messages:
            for key, message in key_messages.items():
                self.register_key(key, message)

    def register_key(self, key: str, message: str) -> None:
        key = (key or "").lower()
        if not key:
            return
        self._keys[key] = message

    def has_keys(self) -> bool:
        return bool(self._keys)

    def start(self) -> bool:
        if not self._keys or self._started:
            return False
        if self._platform == "nt":
            try:
                import msvcrt  # type: ignore
            except ImportError:
                self.console.print("[yellow]Hotkeys disabled: msvcrt unavailable.[/yellow]")
                return False
            self._msvcrt = msvcrt
            self._active = True
            self._started = True
            self._print_instructions()
            return True

        if not sys.stdin.isatty():
            self.console.print("[yellow]Hotkeys disabled: stdin is not a TTY.[/yellow]")
            return False
        import termios
        import tty

        self._fd = sys.stdin.fileno()
        self._orig_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._active = True
        self._started = True
        self._print_instructions()
        return True

    def suspend(self) -> None:
        if not self._active:
            return
        if self._platform != "nt" and self._fd is not None and self._orig_termios is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._orig_termios)
        self._active = False

    def resume(self) -> None:
        if not self._started or self._active:
            return
        if self._platform == "nt":
            self._active = True
            return
        if self._fd is None:
            return
        import tty

        tty.setcbreak(self._fd)
        self._active = True

    def stop(self) -> None:
        if self._platform != "nt" and self._fd is not None and self._orig_termios is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._orig_termios)
        self._active = False
        self._started = False

    def poll_keys(self) -> List[str]:
        if not self._active:
            return []
        pressed: List[str] = []
        while True:
            key = self._read_key()
            if not key:
                break
            lowered = key.lower()
            if lowered in self._keys:
                pressed.append(lowered)
        return pressed

    def _print_instructions(self) -> None:
        if not self._keys:
            return
        instructions = " | ".join(f"'{key.upper()}' to {message}" for key, message in self._keys.items())
        self.console.print(f"[magenta]Hotkeys active: {instructions}[/magenta]")

    def _read_key(self) -> Optional[str]:
        if self._platform == "nt":
            if self._msvcrt and self._msvcrt.kbhit():
                ch = self._msvcrt.getwch()
                if ch == "\x03":
                    raise KeyboardInterrupt
                return ch
            return None
        import select

        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x03":
            raise KeyboardInterrupt
        return ch


def zero_context_sticks(ctx: ControllerContext, console: Optional[Console] = None, reason: str = "Zeroed stick centers") -> None:
    """Capture and store the current stick positions for a controller."""
    offsets = capture_stick_offsets(ctx.controller)
    ctx.axis_offsets = offsets
    if console:
        console.print(
            f"[cyan]{reason} for controller {ctx.controller_index} (inst {ctx.instance_id}): {format_axis_offsets(offsets)}[/cyan]"
        )


def zero_all_context_sticks(contexts: Dict[int, ControllerContext], console: Console) -> None:
    """Zero every connected controller's sticks."""
    if not contexts:
        console.print("[yellow]No controllers available to zero right now.[/yellow]")
        return
    for ctx in contexts.values():
        zero_context_sticks(ctx, console, reason="Re-zeroed stick centers")


def controller_display_name(ctx: ControllerContext) -> str:
    """Return a human-readable controller name."""
    name = sdl2.SDL_GameControllerName(ctx.controller)
    if not name:
        return "Unknown"
    if isinstance(name, bytes):
        return name.decode(errors="ignore")
    return str(name)


def toggle_abxy_for_context(ctx: ControllerContext, config: BridgeConfig, console: Console) -> None:
    """Toggle the ABXY layout for a single controller."""
    if config.swap_abxy_global:
        console.print("[yellow]Global --swap-abxy is enabled; disable it to use per-controller toggles.[/yellow]")
        return
    swapped = ctx.stable_id in config.swap_abxy_ids
    action = "standard" if swapped else "swapped"
    if swapped:
        config.swap_abxy_ids.discard(ctx.stable_id)
    else:
        config.swap_abxy_ids.add(ctx.stable_id)
    console.print(
        f"[cyan]Controller {ctx.controller_index} ({controller_display_name(ctx)}, inst {ctx.instance_id}) now using {action} ABXY layout.[/cyan]"
    )


def prompt_swap_abxy_controller(
    contexts: Dict[int, ControllerContext],
    config: BridgeConfig,
    console: Console,
    hotkey: Optional[HotkeyMonitor] = None,
) -> None:
    """Prompt the user to choose a controller whose ABXY layout should be toggled."""
    if not contexts:
        console.print("[yellow]No controllers connected to toggle ABXY layout.[/yellow]")
        return
    controllers = sorted(contexts.values(), key=lambda ctx: (ctx.controller_index, ctx.instance_id))
    table = Table(title="Toggle ABXY layout for a controller")
    table.add_column("Choice", justify="center")
    table.add_column("SDL Index", justify="center")
    table.add_column("Instance", justify="center")
    table.add_column("Name")
    table.add_column("GUID")
    table.add_column("Layout", justify="center")
    for idx, ctx in enumerate(controllers):
        swapped = config.swap_abxy_global or (ctx.stable_id in config.swap_abxy_ids)
        state = "Swapped" if swapped else "Standard"
        if config.swap_abxy_global:
            state += " (global)"
        table.add_row(
            str(idx),
            str(ctx.controller_index),
            str(ctx.instance_id),
            controller_display_name(ctx),
            ctx.stable_id or "unknown",
            state,
        )
    console.print(table)
    choices = [str(i) for i in range(len(controllers))] + ["q"]
    if hotkey:
        hotkey.suspend()
    try:
        selection = Prompt.ask(
            "Select controller index to toggle ABXY (or 'q' to cancel)",
            choices=choices,
            default="q",
        )
    finally:
        if hotkey:
            hotkey.resume()
    if selection == "q":
        console.print("[yellow]ABXY toggle canceled.[/yellow]")
        return
    ctx = controllers[int(selection)]
    toggle_abxy_for_context(ctx, config, console)


def open_controller(index: int) -> Tuple[sdl2.SDL_GameController, int, str]:
    """Open an SDL GameController by index and return it with instance ID and GUID string."""
    controller = sdl2.SDL_GameControllerOpen(index)
    if not controller:
        raise RuntimeError(f"Failed to open controller {index}: {sdl2.SDL_GetError().decode()}")
    joystick = sdl2.SDL_GameControllerGetJoystick(controller)
    instance_id = sdl2.SDL_JoystickInstanceID(joystick)
    guid_str = guid_string_from_joystick(joystick)
    return controller, instance_id, guid_str


def try_open_uart(port: str, baud: int) -> Optional[PicoUART]:
    """Attempt to open a UART without logging; return None on failure."""
    try:
        return PicoUART(port, baud)
    except Exception:
        return None


def guid_string_from_joystick(joystick: sdl2.SDL_Joystick) -> str:
    """Return a GUID string for an already-open joystick."""
    guid = sdl2.SDL_JoystickGetGUID(joystick)
    buf = create_string_buffer(33)
    sdl2.SDL_JoystickGetGUIDString(guid, buf, 33)
    return buf.value.decode().lower() if buf.value else ""


def guid_string_for_device_index(index: int) -> str:
    """Return a GUID string for a joystick device index without opening it."""
    guid = sdl2.SDL_JoystickGetDeviceGUID(index)
    buf = create_string_buffer(33)
    sdl2.SDL_JoystickGetGUIDString(guid, buf, 33)
    return buf.value.decode().lower() if buf.value else ""


def open_uart_or_warn(port: str, baud: int, console: Console) -> Optional[PicoUART]:
    """Open a UART and warn on failure."""
    try:
        return PicoUART(port, baud)
    except Exception as exc:
        console.print(f"[yellow]Failed to open UART {port}: {exc}[/yellow]")
        return None


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser for the bridge."""
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
    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.08,
        help="Stick deadzone (0.0-1.0, default 0.08)",
    )
    parser.add_argument(
        "--zero-sticks",
        action="store_true",
        help="Capture stick positions on connect and treat them as neutral to cancel drift.",
    )
    parser.add_argument(
        "--zero-hotkey",
        type=parse_hotkey,
        default="z",
        metavar="KEY",
        help="Press this key in the terminal to re-zero sticks at runtime (default: 'z', empty string disables).",
    )
    parser.add_argument(
        "--update-controller-db",
        action="store_true",
        help="Download the latest SDL GameController database before loading mappings.",
    )
    parser.add_argument(
        "--controller-db-url",
        default=CONTROLLER_DB_URL_DEFAULT,
        help="Override the URL used to download the SDL GameController database.",
    )
    parser.add_argument(
        "--swap-hotkey",
        type=parse_hotkey,
        default="x",
        metavar="KEY",
        help="Press this key in the terminal to toggle ABXY layout for a connected controller (default: 'x'; empty string disables).",
    )
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
        "--list-controllers",
        action="store_true",
        help="List detected controllers with GUIDs and exit.",
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
        "--swap-abxy-guid",
        action="append",
        default=[],
        help="Swap AB/XY mapping for specific controller GUIDs (see --list-controllers). Repeatable.",
    )
    parser.add_argument(
        "--sdl-mapping",
        action="append",
        default=[],
        help="Path to an SDL2 controller mapping database (e.g. controllerdb.txt). Repeatable.",
    )
    return parser


def poll_controller_buttons(ctx: ControllerContext, button_map: Dict[int, SwitchButton]) -> None:
    """Update button/hat state based on current SDL controller readings."""
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
    zero_sticks: bool
    zero_hotkey: str
    swap_hotkey: str
    button_map_default: Dict[int, SwitchButton]
    button_map_swapped: Dict[int, SwitchButton]
    swap_abxy_indices: set[int]
    swap_abxy_ids: set[str]
    swap_abxy_global: bool


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


def load_button_maps(console: Console, args: argparse.Namespace) -> Tuple[Dict[int, SwitchButton], Dict[int, SwitchButton], set[int]]:
    """Load SDL controller mappings and return button map variants."""
    default_mapping = Path(__file__).parent / "controller_db" / "gamecontrollerdb.txt"
    if args.update_controller_db or not default_mapping.exists():
        download_controller_db(console, default_mapping, args.controller_db_url)
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
    """Derive bridge runtime configuration from CLI arguments."""
    interval = 1.0 / max(args.frequency, 1.0)
    deadzone_raw = int(max(0.0, min(args.deadzone, 1.0)) * 32767)
    trigger_threshold = int(max(0.0, min(args.trigger_threshold, 1.0)) * 32767)
    button_map_default, button_map_swapped, swap_abxy_indices = load_button_maps(console, args)
    swap_abxy_guids = {g.lower() for g in args.swap_abxy_guid}
    return BridgeConfig(
        interval=interval,
        deadzone_raw=deadzone_raw,
        trigger_threshold=trigger_threshold,
        zero_sticks=bool(args.zero_sticks),
        zero_hotkey=args.zero_hotkey or "",
        swap_hotkey=args.swap_hotkey or "",
        button_map_default=button_map_default,
        button_map_swapped=button_map_swapped,
        swap_abxy_indices=swap_abxy_indices,
        swap_abxy_ids=set(swap_abxy_guids),  # filled later once stable IDs are known
        swap_abxy_global=bool(args.swap_abxy),
    )


def initialize_sdl(parser: argparse.ArgumentParser) -> None:
    """Set SDL hints and initialize subsystems needed for controllers."""
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
    """Detect available controllers and return usable indices and names."""
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


def list_controllers_with_guids(console: Console, parser: argparse.ArgumentParser) -> None:
    """Print detected controllers with their GUID strings and exit."""
    count = sdl2.SDL_NumJoysticks()
    if count < 0:
        parser.error(f"SDL error: {sdl2.SDL_GetError().decode()}")
    if count == 0:
        console.print("[yellow]No controllers detected.[/yellow]")
        return
    table = Table(title="Detected Controllers (GUIDs)")
    table.add_column("Index", justify="center")
    table.add_column("Type")
    table.add_column("Name")
    table.add_column("GUID")
    for idx in range(count):
        is_gc = sdl2.SDL_IsGameController(idx)
        name = sdl2.SDL_GameControllerNameForIndex(idx) if is_gc else sdl2.SDL_JoystickNameForIndex(idx)
        name_str = name.decode() if isinstance(name, bytes) else str(name)
        guid_str = guid_string_for_device_index(idx)
        table.add_row(str(idx), "GameController" if is_gc else "Joystick", name_str, guid_str)
    console.print(table)


def prepare_pairing_state(
    args: argparse.Namespace,
    console: Console,
    parser: argparse.ArgumentParser,
    controller_indices: List[int],
    controller_names: Dict[int, str],
) -> PairingState:
    """Prepare pairing preferences and pre-seeded mappings from CLI options."""
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
        # Interactive pairing shows the discovered ports and lets the user bind explicitly.
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
            # Passive mode: grab whatever UARTs exist now, and keep looking later.
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
    """Return the UART assigned to a controller index, auto-pairing if allowed."""
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
    """Return a set of UART paths currently reserved or mapped."""
    used = set(pairing.mapping_by_index.values())
    used.update(ctx.port for ctx in contexts.values() if ctx.port)
    return used


def handle_removed_port(path: str, pairing: PairingState, contexts: Dict[int, ControllerContext], console: Console) -> None:
    """Clear mappings/contexts for a UART path that disappeared."""
    if path in pairing.available_ports:
        pairing.available_ports.remove(path)
        console.print(f"[yellow]UART {path} removed; dropping from available pool[/yellow]")
    indices_to_clear = [idx for idx, mapped in pairing.mapping_by_index.items() if mapped == path]
    for idx in indices_to_clear:
        pairing.mapping_by_index.pop(idx, None)
        pairing.auto_assigned_indices.discard(idx)
    for ctx in list(contexts.values()):
        if ctx.port != path:
            continue
        if ctx.uart:
            try:
                ctx.uart.close()
            except Exception:
                pass
        sdl2.SDL_GameControllerRumble(ctx.controller, 0, 0, 0)
        ctx.uart = None
        ctx.port = None
        ctx.rumble_active = False
        ctx.last_rumble_energy = 0.0
        ctx.last_reopen_attempt = time.monotonic()
        console.print(f"[yellow]UART {path} removed; controller {ctx.controller_index} waiting for reassignment[/yellow]")


def discover_new_ports(pairing: PairingState, contexts: Dict[int, ControllerContext], console: Console) -> None:
    """Scan for new serial ports and add unused ones to the available pool."""
    if not pairing.auto_discover_ports:
        return
    discovered = discover_ports(
        include_non_usb=pairing.include_non_usb,
        ignore_descriptions=pairing.ignore_port_desc,
        include_descriptions=pairing.include_port_desc,
    )
    current_paths = {info["device"] for info in discovered}
    known_paths = set(pairing.available_ports)
    known_paths.update(pairing.mapping_by_index.values())
    known_paths.update(ctx.port for ctx in contexts.values() if ctx.port)
    # Drop any paths we previously knew about that are no longer present.
    removed_paths = [path for path in known_paths if path not in current_paths]
    for path in removed_paths:
        handle_removed_port(path, pairing, contexts, console)
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
    """Attach UARTs to contexts that are waiting for a port assignment/open."""
    for ctx in list(contexts.values()):
        if ctx.port is not None:
            continue
        # Try to grab a port for this controller; if none are available, leave it waiting.
        port_choice = assign_port_for_index(pairing, ctx.controller_index, console)
        if port_choice is None:
            continue
        ctx.port = port_choice
        uart = open_uart_or_warn(port_choice, args.baud, console)
        ctx.last_reopen_attempt = time.monotonic()
        if uart:
            uarts.append(uart)
            ctx.uart = uart
            console.print(
                f"[green]Controller {ctx.controller_index} (id {ctx.stable_id}, inst {ctx.instance_id}) paired to {port_choice}[/green]"
            )
        else:
            ctx.uart = None
            console.print(
                f"[yellow]Controller {ctx.controller_index} (id {ctx.stable_id}, inst {ctx.instance_id}) waiting for UART {port_choice}[/yellow]"
            )


def open_initial_contexts(
    args: argparse.Namespace,
    pairing: PairingState,
    controller_indices: List[int],
    console: Console,
    config: BridgeConfig,
) -> Tuple[Dict[int, ControllerContext], List[PicoUART]]:
    """Open initial controllers and UARTs for detected indices."""
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
            controller, instance_id, guid = open_controller(index)
        except Exception as exc:
            console.print(f"[red]Failed to open controller {index}: {exc}[/red]")
            continue
        stable_id = guid
        if index in config.swap_abxy_indices:
            config.swap_abxy_ids.add(stable_id)
        uart = open_uart_or_warn(port, args.baud, console) if port else None
        if uart:
            uarts.append(uart)
            console.print(f"[green]Controller {index} (id {stable_id}, inst {instance_id}) paired to {port}[/green]")
        elif port:
            console.print(f"[yellow]Controller {index} (id {stable_id}, inst {instance_id}) waiting for UART {port}[/yellow]")
        else:
            console.print(
                f"[yellow]Controller {index} (id {stable_id}, inst {instance_id}) connected; waiting for an available UART[/yellow]"
            )
        ctx = ControllerContext(
            controller=controller,
            instance_id=instance_id,
            controller_index=index,
            stable_id=stable_id,
            port=port,
            uart=uart,
        )
        if config.zero_sticks:
            zero_context_sticks(ctx, console)
        contexts[instance_id] = ctx
    return contexts, uarts


def handle_axis_motion(event: sdl2.SDL_Event, contexts: Dict[int, ControllerContext], config: BridgeConfig) -> None:
    """Process axis motion event into stick/trigger state."""
    ctx = contexts.get(event.caxis.which)
    if not ctx:
        return
    axis = event.caxis.axis
    value = calibrate_axis_value(event.caxis.value, axis, ctx)
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
    config: BridgeConfig,
    contexts: Dict[int, ControllerContext],
) -> None:
    """Process button events into report/dpad state."""
    ctx = contexts.get(event.cbutton.which)
    if not ctx:
        return
    current_button_map = (
        config.button_map_swapped
        if (config.swap_abxy_global or ctx.stable_id in config.swap_abxy_ids)
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
    config: BridgeConfig,
) -> None:
    """Handle controller hotplug by opening and pairing UART if possible."""
    idx = event.cdevice.which
    # If we already have a context for this logical index, ignore the duplicate event.
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
        controller, instance_id, guid = open_controller(idx)
    except Exception as exc:
        console.print(f"[red]Hotplug open failed for controller {idx}: {exc}[/red]")
        return
    stable_id = guid
    # Promote any index-based swap flags to stable IDs on first sight.
    if idx in config.swap_abxy_indices:
        config.swap_abxy_ids.add(stable_id)
    uart = open_uart_or_warn(port, args.baud, console) if port else None
    if uart:
        uarts.append(uart)
        console.print(f"[green]Controller {idx} (id {stable_id}, inst {instance_id}) paired to {port}[/green]")
    elif port:
        console.print(f"[yellow]Controller {idx} (id {stable_id}, inst {instance_id}) waiting for UART {port}[/yellow]")
    else:
        console.print(
            f"[yellow]Controller {idx} (id {stable_id}, inst {instance_id}) connected; waiting for an available UART[/yellow]"
        )
    ctx = ControllerContext(
        controller=controller,
        instance_id=instance_id,
        controller_index=idx,
        stable_id=stable_id,
        port=port,
        uart=uart,
    )
    if config.zero_sticks:
        zero_context_sticks(ctx, console)
    contexts[instance_id] = ctx


def handle_device_removed(
    event: sdl2.SDL_Event,
    pairing: PairingState,
    contexts: Dict[int, ControllerContext],
    console: Console,
) -> None:
    """Handle controller removal and release any auto-assigned UART."""
    instance_id = event.cdevice.which
    ctx = contexts.pop(instance_id, None)
    if not ctx:
        return
    console.print(f"[yellow]Controller {instance_id} (id {ctx.stable_id}) removed[/yellow]")
    if ctx.controller_index in pairing.auto_assigned_indices:
        # Return auto-paired UART back to the pool so a future device can use it.
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
    """Poll controllers, reconnect UARTs, send reports, and apply rumble."""
    for ctx in list(contexts.values()):
        current_button_map = (
            config.button_map_swapped
            if (config.swap_abxy_global or ctx.stable_id in config.swap_abxy_ids)
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
                # Apply only the freshest rumble payload seen during this tick.
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
    hotkey: Optional[HotkeyMonitor] = None,
) -> None:
    """Main event loop for bridging controllers to UART and handling rumble."""
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
                handle_button_event(event, config, contexts)
            elif event.type == sdl2.SDL_CONTROLLERDEVICEADDED:
                handle_device_added(event, args, pairing, contexts, uarts, console, config)
            elif event.type == sdl2.SDL_CONTROLLERDEVICEREMOVED:
                handle_device_removed(event, pairing, contexts, console)

        now = time.monotonic()
        if now - last_port_scan > port_scan_interval:
            # Periodically rescan for new UARTs to auto-pair hotplugged devices.
            discover_new_ports(pairing, contexts, console)
            last_port_scan = now
            pair_waiting_contexts(args, pairing, contexts, uarts, console)
        else:
            pair_waiting_contexts(args, pairing, contexts, uarts, console)
        service_contexts(now, args, config, contexts, uarts, console)
        if hotkey:
            for key in hotkey.poll_keys():
                if key == config.zero_hotkey:
                    zero_all_context_sticks(contexts, console)
                elif key == config.swap_hotkey:
                    prompt_swap_abxy_controller(contexts, config, console, hotkey)
        sdl2.SDL_Delay(1)


def cleanup(contexts: Dict[int, ControllerContext], uarts: List[PicoUART]) -> None:
    """Gracefully close controllers, UARTs, and SDL subsystems."""
    for ctx in contexts.values():
        sdl2.SDL_GameControllerClose(ctx.controller)
    for uart in uarts:
        uart.close()
    sdl2.SDL_Quit()


def main() -> None:
    """Entry point: parse args, set up SDL, and run the bridge loop."""
    parser = build_arg_parser()
    args = parser.parse_args()
    console = Console()
    config = build_bridge_config(console, args)
    initialize_sdl(parser)
    contexts: Dict[int, ControllerContext] = {}
    uarts: List[PicoUART] = []
    hotkey_monitor: Optional[HotkeyMonitor] = None
    try:
        if args.list_controllers:
            list_controllers_with_guids(console, parser)
            return
        controller_indices, controller_names = detect_controllers(console, args, parser)
        pairing = prepare_pairing_state(args, console, parser, controller_indices, controller_names)
        hotkey_messages: Dict[str, str] = {}
        if config.zero_hotkey:
            hotkey_messages[config.zero_hotkey] = "re-zero controller sticks"
        if config.swap_hotkey:
            if config.swap_hotkey in hotkey_messages:
                hotkey_messages[config.swap_hotkey] = (
                    hotkey_messages[config.swap_hotkey] + "; toggle ABXY layout"
                )
            else:
                hotkey_messages[config.swap_hotkey] = "toggle ABXY layout for a controller"
        if hotkey_messages:
            candidate = HotkeyMonitor(console, hotkey_messages)
            if candidate.start():
                hotkey_monitor = candidate
        contexts, uarts = open_initial_contexts(args, pairing, controller_indices, console, config)
        if not contexts:
            console.print("[yellow]No controllers opened; waiting for hotplug events...[/yellow]")
        run_bridge_loop(args, console, config, pairing, contexts, uarts, hotkey_monitor)
    finally:
        if hotkey_monitor:
            hotkey_monitor.stop()
        cleanup(contexts, uarts)


if __name__ == "__main__":
    main()
