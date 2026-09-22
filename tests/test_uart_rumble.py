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
        self.rumble: list[tuple[int, float, float]] = []
        self.sent: list[tuple[int, int]] = []

    def send_report(self, report: SwitchReport, slot: int = 0) -> None:
        self.sent.append((slot, report.buttons))

    def read_rumble(self) -> tuple[int, float, float] | None:
        if not self.rumble:
            return None
        return self.rumble.pop(0)


def make_links(port: str, uart: RecordingUART) -> dict[str, bridge.UartLink]:
    return {port: bridge.UartLink(port, cast(PicoUART, cast(object, uart)))}


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

    def fake_rumble(_controller, low, high, duration):
        calls.append((low, high, duration))
        return True

    monkeypatch.setattr(bridge.sdl3, "SDL_RumbleGamepad", fake_rumble)
    controller = cast(sdl3.SDL_Gamepad, object())

    assert bridge.apply_rumble(controller, 1.0, 0.5) == (True, True)
    assert calls[-1] == (0xFFFF, 0x7FFF, 50)

    assert bridge.apply_rumble(controller, 0.0, 0.0) == (False, True)
    assert calls[-1] == (0, 0, 50)


def test_shape_rumble_curve_and_gain_boost_faint_levels_but_keep_silence() -> None:
    # Linear defaults are the identity.
    assert bridge.shape_rumble(0.3, 1.0, 1.0) == pytest.approx(0.3)
    # Zero must never become a nonzero idle buzz, whatever the shaping.
    assert bridge.shape_rumble(0.0, 4.0, 0.5) == 0.0
    # Typical Switch HD levels (~0.03) are lifted into the ERM motor's usable range.
    boosted = bridge.shape_rumble(0.031, 1.0, 0.5)
    assert 0.15 < boosted < 0.2
    # Gain saturates instead of wrapping.
    assert bridge.shape_rumble(0.467, 4.0, 1.0) == 1.0
    # Shaping is monotonic so louder input never rumbles softer.
    assert bridge.shape_rumble(0.1, 2.0, 0.5) < bridge.shape_rumble(0.3, 2.0, 0.5)


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
    ctx = bridge.ControllerContext(controller, 7, 0, "controller", "/dev/null")
    links = make_links("/dev/null", uart)
    contexts = {ctx.instance_id: ctx}
    args = Namespace(baud=UART_BAUD)
    console = Console(file=StringIO())

    magnitude = (0, 64 / 255.0, 192 / 255.0)
    uart.rumble.append(magnitude)
    bridge.service_contexts(1.0, args, make_config(), contexts, links, console)
    uart.rumble.append(magnitude)
    bridge.service_contexts(1.7, args, make_config(), contexts, links, console)
    bridge.service_contexts(1.71, args, make_config(), contexts, links, console)

    assert calls == [(16448, 49344, 50), (16448, 49344, 50)]
    assert ctx.rumble_active

    bridge.service_contexts(1.96, args, make_config(), contexts, links, console)

    assert calls[-1] == (0, 0, 0)
    assert not ctx.rumble_active


def test_shared_port_routes_reports_and_rumble_by_slot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[object, int, int]] = []
    monkeypatch.setattr(
        bridge.sdl3,
        "SDL_RumbleGamepad",
        lambda controller, low, high, _duration: calls.append((controller, low, high)) or True,
    )
    monkeypatch.setattr(bridge, "poll_controller_buttons", lambda _ctx, _map: None)

    uart = RecordingUART()
    pad_a = cast(sdl3.SDL_Gamepad, object())
    pad_b = cast(sdl3.SDL_Gamepad, object())
    ctx_a = bridge.ControllerContext(pad_a, 7, 0, "a", "COM11", slot=0)
    ctx_b = bridge.ControllerContext(pad_b, 8, 1, "b", "COM11", slot=2)
    ctx_a.report.buttons = 0x0001
    ctx_b.report.buttons = 0x0002
    contexts = {7: ctx_a, 8: ctx_b}
    links = make_links("COM11", uart)
    args = Namespace(baud=UART_BAUD)
    console = Console(file=StringIO())

    # Slot 2 rumbles, slot 0 is idle, slot 3 has no controller attached.
    uart.rumble.extend([(0, 0.0, 0.0), (2, 1.0, 0.5), (3, 1.0, 1.0)])
    bridge.service_contexts(20.0, args, make_config(), contexts, links, console)

    assert sorted(uart.sent) == [(0, 0x0001), (2, 0x0002)]
    assert calls == [(pad_a, 0, 0), (pad_b, 0xFFFF, 0x7FFF)]
    assert not ctx_a.rumble_active
    assert ctx_b.rumble_active


def test_auto_pairing_spreads_controllers_across_ports_then_fills_slots() -> None:
    pairing = bridge.PairingState(
        mapping_by_index={},
        available_ports=["COM11", "COM12"],
        slots_per_port=2,
        auto_pairing_enabled=True,
    )
    console = Console(file=StringIO())

    assignments = [bridge.assign_port_for_index(pairing, idx, console) for idx in range(5)]

    assert assignments == [("COM11", 0), ("COM12", 0), ("COM11", 1), ("COM12", 1), None]
    # Releasing a slot makes exactly that slot reusable.
    del pairing.mapping_by_index[2]
    assert bridge.assign_port_for_index(pairing, 9, console) == ("COM11", 1)


def test_explicit_mappings_fill_omitted_slots_and_reject_conflicts() -> None:
    parser = bridge.build_arg_parser()
    resolved = bridge.resolve_mapping_slots(
        [(0, "COM11", None), (1, "COM11", 3), (2, "COM11", None)], 4, parser
    )
    assert resolved == {0: ("COM11", 0), 1: ("COM11", 3), 2: ("COM11", 1)}
    with pytest.raises(SystemExit):
        bridge.resolve_mapping_slots([(0, "COM11", 1), (1, "COM11", 1)], 4, parser)
    with pytest.raises(SystemExit):
        bridge.resolve_mapping_slots([(0, "COM11", None), (1, "COM11", None)], 1, parser)
    assert bridge.parse_mapping("2:COM11:3") == (2, "COM11", 3)
    assert bridge.parse_mapping("0:/dev/ttyUSB0") == (0, "/dev/ttyUSB0", None)
