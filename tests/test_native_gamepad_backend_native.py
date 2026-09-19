from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("source", ("GAMEPAD", "DUALSENSE"))
@pytest.mark.parametrize("controller_count", (2, 4))
def test_native_gamepad_backend_native(
    tmp_path: Path, source: str, controller_count: int
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "native_gamepad_backend_test"
    firmware = root / "src" / "firmware"
    sources = [
        root / "tests" / "native_gamepad_backend_test.cpp",
        firmware / "profile" / "controller_profile.cpp",
        firmware / "profile" / "controller_profile_transform.cpp",
        firmware / "profile" / "controller_synthetic_input.cpp",
        firmware / "profile" / "controller_profile_runtime.cpp",
        firmware / "profile" / "profile_storage.cpp",
        firmware / "input" / "wii_swing.cpp",
        firmware / "input" / "controller_macro_capture.cpp",
        root / "bluepad32_config" / "parser" / "uni_switch2_haptics.c",
    ]
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
            "-DSWITCH_PICO_USB_OUTPUT_MODES=1",
            "-DSWITCH_PICO_ENABLE_BLE=1",
            "-DSWITCH_PICO_ENABLE_CLASSIC=1",
            "-DSWITCH2_BRIDGE_FULL_INPUT=1",
            f"-DSWITCH2_BRIDGE_{source}_INPUT=1",
            f"-DPROBE_CONTROLLER_COUNT={controller_count}",
            f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
            f"-I{firmware}",
            f"-I{root / 'bluepad32_config'}",
            *(str(source) for source in sources),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    scenarios = [
        "stable-logical-slot",
        "cue-lifetime",
        "cue-races",
        "gameplay-timeline",
        "gameplay-availability",
        "gameplay-priority",
        "gameplay-source-epochs",
        "gameplay-profile-gain",
    ]
    if controller_count == 2:
        scenarios.append("source-isolation")
    else:
        scenarios.extend(
            (
                "two-pair-sources",
                "two-pair-cues",
                "explicit-precedence",
                "gameplay-two-pairs",
            )
        )
    if source == "GAMEPAD":
        scenarios.extend(
            (
                "paired-source",
                "pair-cue-races",
                "mono-rumble",
                "gameplay-paired-revision",
                "gameplay-wii",
            )
        )
        if controller_count == 2:
            scenarios.extend(("sensorless-admission", "independent-motion"))
        else:
            scenarios.extend(
                (
                    "paired-explicit-conflict",
                    "topology-reservations",
                    "stable-ble-reservation",
                )
            )
    for scenario in scenarios:
        subprocess.run([str(executable), scenario], check=True, cwd=root)
