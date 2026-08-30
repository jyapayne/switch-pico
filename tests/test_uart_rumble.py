"""Focused tests for decoded UART rumble delivery to SDL3."""

from argparse import Namespace
from io import StringIO
from typing import cast

import pytest
import sdl3
from rich.console import Console

import switch_pico_bridge.controller_uart_bridge as bridge
from switch_pico_bridge.switch_pico_uart import PicoUART, SwitchReport, UART_BAUD


class RecordingUART:
    def __init__(self) -> None:
        self.rumble: list[tuple[float, float]] = []

    def send_report(self, _report: SwitchReport) -> None:
        pass

    def read_rumble(self) -> tuple[float, float] | None:
        if not self.rumble:
            return None
        return self.rumble.pop(0)


def make_config() -> bridge.BridgeConfig:
    return bridge.BridgeConfig(
        interval=10.0,
        deadzone_raw=0,
        trigger_threshold=0,
        zero_sticks=False,
        zero_hotkey="",
        swap_hotkey="",
        button_map_default={},
        button_map_swapped={},
        swap_abxy_indices=set(),
        swap_abxy_ids=set(),
        swap_abxy_global=False,
        no_imu=True,
    )


def test_apply_rumble_maps_low_and_high_with_50ms_duration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[int, int, int]] = []
    monkeypatch.setattr(
        bridge.sdl3,
        "SDL_RumbleGamepad",
        lambda _controller, low, high, duration: calls.append((low, high, duration)),
    )
    controller = cast(sdl3.SDL_Gamepad, object())

    assert bridge.apply_rumble(controller, 1.0, 0.5)
    assert calls[-1] == (0xFFFF, 0x7FFF, 50)

    assert not bridge.apply_rumble(controller, 0.0, 0.0)
    assert calls[-1] == (0, 0, 50)


def test_repeated_constant_rumble_stays_active_until_idle_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[int, int, int]] = []
    monkeypatch.setattr(
        bridge.sdl3,
        "SDL_RumbleGamepad",
        lambda _controller, low, high, duration: calls.append((low, high, duration)),
    )
    monkeypatch.setattr(bridge, "poll_controller_buttons", lambda _ctx, _map: None)

    uart = RecordingUART()
    controller = cast(sdl3.SDL_Gamepad, object())
    ctx = bridge.ControllerContext(
        controller,
        7,
        0,
        "controller",
        "/dev/null",
        cast(PicoUART, cast(object, uart)),
    )
    contexts = {ctx.instance_id: ctx}
    args = Namespace(baud=UART_BAUD)
    console = Console(file=StringIO())

    magnitude = (64 / 255.0, 192 / 255.0)
    uart.rumble.append(magnitude)
    bridge.service_contexts(1.0, args, make_config(), contexts, [], console)
    uart.rumble.append(magnitude)
    bridge.service_contexts(1.7, args, make_config(), contexts, [], console)
    bridge.service_contexts(1.71, args, make_config(), contexts, [], console)

    assert calls == [(16448, 49344, 50), (16448, 49344, 50)]
    assert ctx.rumble_active

    bridge.service_contexts(1.96, args, make_config(), contexts, [], console)

    assert calls[-1] == (0, 0, 0)
    assert not ctx.rumble_active
