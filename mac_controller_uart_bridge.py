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
from typing import Dict, List, Optional, Tuple

import serial
from serial.tools import list_ports

import sdl2
import sdl2.ext
from rich.console import Console
from rich.prompt import Prompt
from rich.table import Table

UART_HEADER = 0xAA
RUMBLE_HEADER = 0xBB
RUMBLE_TYPE_RUMBLE = 0x01
UART_BAUD = 900000


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


def discover_ports(include_non_usb: bool = False) -> List[Dict[str, str]]:
    results: List[Dict[str, str]] = []
    for port in list_ports.comports():
        path = port.device or ""
        if not path:
            continue
        if not include_non_usb and not is_usb_serial(path):
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
        )
        self._buffer = bytearray()

    def send_report(self, report: SwitchReport) -> None:
        self.serial.write(report.to_bytes())
        self.serial.flush()

    def read_rumble_payload(self) -> Optional[bytes]:
        chunk = self.serial.read(64)  # non-blocking (timeout=0)
        if chunk:
            self._buffer.extend(chunk)

        while True:
            if RUMBLE_HEADER not in self._buffer:
                self._buffer.clear()
                return None
            start = self._buffer.find(bytes([RUMBLE_HEADER]))
            if len(self._buffer) - start < 11:
                # Need more bytes.
                if start > 0:
                    del self._buffer[:start]
                return None
            frame = self._buffer[start : start + 11]
            checksum = sum(frame[:10]) & 0xFF
            if frame[1] == RUMBLE_TYPE_RUMBLE and checksum == frame[10]:
                payload = bytes(frame[2:10])
                del self._buffer[: start + 11]
                return payload
            # Bad frame, drop the header and continue.
            del self._buffer[: start + 1]

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
    left_raw = ((payload[1] & 0x03) << 8) | payload[0]
    right_raw = ((payload[5] & 0x03) << 8) | payload[4]
    if left_raw < 8 and right_raw < 8:
        return 0.0, 0.0
    left = min(max(left_raw / 1023.0, 0.0), 1.0)
    right = min(max(right_raw / 1023.0, 0.0), 1.0)
    return left, right


def apply_rumble(controller: sdl2.SDL_GameController, payload: bytes) -> None:
    left_norm, right_norm = decode_rumble(payload)
    max_norm = max(left_norm, right_norm)
    # Treat small rumble as "off" to avoid idle buzz.
    if max_norm < 0.40:
        sdl2.SDL_GameControllerRumble(controller, 0, 0, 0)
        return
    # Attenuate to feel closer to a real controller; cap at ~25% strength.
    scale = 0.40
    left = int(min(1.0, left_norm * scale) * 0xFFFF)
    right = int(min(1.0, right_norm * scale) * 0xFFFF)
    duration = 25
    sdl2.SDL_GameControllerRumble(controller, left, right, duration)


@dataclass
class ControllerContext:
    controller: sdl2.SDL_GameController
    instance_id: int
    uart: PicoUART
    report: SwitchReport = field(default_factory=SwitchReport)
    dpad: Dict[str, bool] = field(default_factory=lambda: {"up": False, "down": False, "left": False, "right": False})
    last_trigger_state: Dict[str, bool] = field(default_factory=lambda: {"left": False, "right": False})
    last_send: float = 0.0
    stop_event: threading.Event = field(default_factory=threading.Event)
    rumble_thread: Optional[threading.Thread] = None


def open_controller(index: int) -> Tuple[sdl2.SDL_GameController, int]:
    controller = sdl2.SDL_GameControllerOpen(index)
    if not controller:
        raise RuntimeError(f"Failed to open controller {index}: {sdl2.SDL_GetError().decode()}")
    joystick = sdl2.SDL_GameControllerGetJoystick(controller)
    instance_id = sdl2.SDL_JoystickInstanceID(joystick)
    return controller, instance_id


def start_rumble_listener(ctx: ControllerContext) -> threading.Thread:
    def _worker() -> None:
        while not ctx.stop_event.is_set():
            payload = ctx.uart.read_rumble_payload()
            if not payload:
                time.sleep(0.0005)  # small yield to avoid busy-spin
                continue
            apply_rumble(ctx.controller, payload)

    thread = threading.Thread(target=_worker, name=f"rumble-{ctx.instance_id}", daemon=True)
    thread.start()
    return thread


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
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    interval = 1.0 / max(args.frequency, 1.0)
    deadzone_raw = int(max(0.0, min(args.deadzone, 1.0)) * 32767)
    trigger_threshold = int(max(0.0, min(args.trigger_threshold, 1.0)) * 32767)

    sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER)
    contexts: Dict[int, ControllerContext] = {}
    uarts: List[PicoUART] = []
    console = Console()
    try:
        controller_indices: List[int] = []
        controller_names: Dict[int, str] = {}
        controller_count = sdl2.SDL_NumJoysticks()
        if controller_count < 0:
            parser.error(f"SDL error: {sdl2.SDL_GetError().decode()}")
        for index in range(controller_count):
            if sdl2.SDL_IsGameController(index):
                name = sdl2.SDL_GameControllerNameForIndex(index)
                name_str = name.decode() if isinstance(name, bytes) else str(name)
                console.print(f"[cyan]Detected controller {index}: {name_str}[/cyan]")
                controller_indices.append(index)
                controller_names[index] = name_str

        mappings = list(args.map)
        if args.interactive:
            if not controller_indices:
                parser.error("No controllers detected for interactive pairing.")
            discovered = discover_ports(include_non_usb=args.all_ports or False)
            if not discovered:
                parser.error("No UART devices found for interactive pairing.")
            mappings = interactive_pairing(console, controller_names, discovered)
            if not mappings:
                parser.error("No controller-to-UART mappings were selected.")
        elif not mappings:
            if args.ports:
                validated = list(args.ports)
                if not controller_indices:
                    parser.error("No controllers available to pair.")
                pair_count = min(len(controller_indices), len(validated))
                if pair_count == 0:
                    parser.error("No UART ports specified.")
                if len(validated) < len(controller_indices):
                    console.print(
                        f"[yellow]Warning: only {len(validated)} UART(s) provided; pairing first {pair_count} controllers.[/yellow]"
                    )
                mappings = list((controller_indices[i], validated[i]) for i in range(pair_count))
                console.print("[green]Paired controllers to specified UART ports:[/green]")
                for idx, port in mappings:
                    console.print(f"  Controller {idx} -> {port}")
            else:
                discovered = discover_ports(include_non_usb=args.all_ports or False)
                if not discovered:
                    parser.error("No UART devices detected automatically.")
                pair_count = min(len(controller_indices), len(discovered))
                if pair_count == 0:
                    parser.error("No controllers detected to pair.")
                if len(discovered) < len(controller_indices):
                    console.print(
                        f"[yellow]Warning: detected {len(discovered)} UART(s) but {len(controller_indices)} controller(s); pairing first {pair_count}.[/yellow]"
                    )
                mappings = list((controller_indices[i], discovered[i]["device"]) for i in range(pair_count))
                console.print("[green]Auto-detected UARTs:[/green]")
                for info in discovered:
                    console.print(f"  {info['device']} ({info['description']})")
                console.print("[green]Paired controllers to detected UARTs:[/green]")
                for idx, port in mappings:
                    console.print(f"  Controller {idx} -> {port}")

        for index, port in mappings:
            controller, instance_id = open_controller(index)
            uart = PicoUART(port)
            uarts.append(uart)
            ctx = ControllerContext(controller=controller, instance_id=instance_id, uart=uart)
            ctx.rumble_thread = start_rumble_listener(ctx)
            contexts[instance_id] = ctx
            console.print(f"[green]Controller {index} ({instance_id}) paired to {port}[/green]")

        if not contexts:
            parser.error("No controllers opened. Check --map/--ports/--interactive values.")

        event = sdl2.SDL_Event()
        running = True
        while running:
            while sdl2.SDL_PollEvent(event):
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    break
                if event.type == sdl2.SDL_CONTROLLERAXISMOTION:
                    ctx = contexts.get(event.caxis.which)
                    if not ctx:
                        continue
                    axis = event.caxis.axis
                    value = event.caxis.value
                    if axis == sdl2.SDL_CONTROLLER_AXIS_LEFTX:
                        ctx.report.lx = axis_to_stick(value, deadzone_raw)
                    elif axis == sdl2.SDL_CONTROLLER_AXIS_LEFTY:
                        ctx.report.ly = axis_to_stick(value, deadzone_raw)
                    elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTX:
                        ctx.report.rx = axis_to_stick(value, deadzone_raw)
                    elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTY:
                        ctx.report.ry = axis_to_stick(value, deadzone_raw)
                    elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                        pressed = trigger_to_button(value, trigger_threshold)
                        if pressed != ctx.last_trigger_state["left"]:
                            if pressed:
                                ctx.report.buttons |= SwitchButton.ZL
                            else:
                                ctx.report.buttons &= ~SwitchButton.ZL
                            ctx.last_trigger_state["left"] = pressed
                    elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                        pressed = trigger_to_button(value, trigger_threshold)
                        if pressed != ctx.last_trigger_state["right"]:
                            if pressed:
                                ctx.report.buttons |= SwitchButton.ZR
                            else:
                                ctx.report.buttons &= ~SwitchButton.ZR
                            ctx.last_trigger_state["right"] = pressed
                elif event.type in (sdl2.SDL_CONTROLLERBUTTONDOWN, sdl2.SDL_CONTROLLERBUTTONUP):
                    ctx = contexts.get(event.cbutton.which)
                    if not ctx:
                        continue
                    button = event.cbutton.button
                    pressed = event.type == sdl2.SDL_CONTROLLERBUTTONDOWN
                    if button in BUTTON_MAP:
                        bit = BUTTON_MAP[button]
                        if pressed:
                            ctx.report.buttons |= bit
                        else:
                            ctx.report.buttons &= ~bit
                    elif button in DPAD_BUTTONS:
                        ctx.dpad[DPAD_BUTTONS[button]] = pressed
                        ctx.report.hat = dpad_to_hat(ctx.dpad)

            now = time.monotonic()
            for ctx in contexts.values():
                if now - ctx.last_send >= interval:
                    ctx.uart.send_report(ctx.report)
                    ctx.last_send = now

            sdl2.SDL_Delay(1)
    finally:
        for ctx in contexts.values():
            ctx.stop_event.set()
            if ctx.rumble_thread:
                ctx.rumble_thread.join(timeout=0.2)
            sdl2.SDL_GameControllerClose(ctx.controller)
        for uart in uarts:
            uart.close()
        sdl2.SDL_Quit()


if __name__ == "__main__":
    main()
