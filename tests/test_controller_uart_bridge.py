"""Regression tests for SDL sensor buffering in the UART bridge."""

from argparse import Namespace
from io import StringIO
from types import SimpleNamespace
from typing import Protocol, cast

import sdl3
from rich.console import Console

import switch_pico_bridge.controller_uart_bridge as bridge
from switch_pico_bridge.switch_pico_uart import (
    IMUSample,
    PicoUART,
    SENSOR_ACCEL,
    SENSOR_GYRO,
    SwitchButton,
    SwitchReport,
    UART_BAUD,
)


class MonkeyPatch(Protocol):
    def setattr(self, target: object, name: str, value: object) -> None: ...


class RecordingUART:
    def __init__(self) -> None:
        self.sent_imu: list[tuple[IMUSample, ...]] = []

    def send_report(self, report: SwitchReport) -> None:
        self.sent_imu.append(tuple(report.imu_samples))

    def read_rumble_payload(self) -> bytes | None:
        return None


def make_config() -> bridge.BridgeConfig:
    return bridge.BridgeConfig(
        interval=0.002,
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
    )


def test_sensor_buffer_retains_latest_three_samples() -> None:
    controller = cast(sdl3.SDL_Gamepad, object())
    ctx = bridge.ControllerContext(controller, 7, 0, "dualsense", None, None)
    ctx.sensors_enabled = True
    ctx.gyro_bias_locked = True
    contexts = {ctx.instance_id: ctx}
    config = make_config()

    accel_event = cast(
        sdl3.SDL_Event,
        cast(
            object,
            SimpleNamespace(
                gsensor=SimpleNamespace(
                    which=ctx.instance_id,
                    sensor=SENSOR_ACCEL,
                    data=(0.0, 9.80665, 0.0),
                )
            ),
        ),
    )
    bridge.handle_sensor_update(accel_event, contexts, config)

    for gyro_x in (1.0, 2.0, 3.0, 4.0):
        gyro_event = cast(
            sdl3.SDL_Event,
            cast(
                object,
                SimpleNamespace(
                    gsensor=SimpleNamespace(
                        which=ctx.instance_id,
                        sensor=SENSOR_GYRO,
                        data=(gyro_x, 0.0, 0.0),
                    )
                ),
            ),
        )
        bridge.handle_sensor_update(gyro_event, contexts, config)

    assert len(ctx.imu_samples) == 3
    assert [sample.gyro_y for sample in ctx.imu_samples] == [
        bridge.convert_gyro_to_raw(-2.0),
        bridge.convert_gyro_to_raw(-3.0),
        bridge.convert_gyro_to_raw(-4.0),
    ]


def test_service_republishes_latest_imu_window(monkeypatch: MonkeyPatch) -> None:
    uart = RecordingUART()
    controller = cast(sdl3.SDL_Gamepad, object())
    ctx = bridge.ControllerContext(
        controller, 7, 0, "dualsense", "/dev/null", cast(PicoUART, cast(object, uart))
    )
    ctx.sensors_enabled = True
    samples = [
        IMUSample(1, 2, 3, 4, 5, 6),
        IMUSample(7, 8, 9, 10, 11, 12),
        IMUSample(13, 14, 15, 16, 17, 18),
    ]
    ctx.imu_samples = samples.copy()
    contexts = {ctx.instance_id: ctx}
    config = make_config()

    def ignore_poll(
        _ctx: bridge.ControllerContext, _button_map: dict[int, SwitchButton]
    ) -> None:
        return None

    monkeypatch.setattr(bridge, "poll_controller_buttons", ignore_poll)

    args = Namespace(baud=UART_BAUD)
    console = Console(file=StringIO())
    bridge.service_contexts(1.0, args, config, contexts, [], console)
    bridge.service_contexts(2.0, args, config, contexts, [], console)

    assert uart.sent_imu == [tuple(samples), tuple(samples)]
    assert ctx.imu_samples == samples
