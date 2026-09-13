from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("imu_target", [1, 2, 3], ids=["right", "left", "both"])
def test_native_gamepad_bridge_mapping_motion_and_backpressure(
    tmp_path: Path,
    imu_target: int,
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "switch2_native_gamepad_bridge_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_SWITCH2_USB_BRIDGE=1",
            "-DSWITCH2_BRIDGE_GAMEPAD_INPUT=1",
            "-DSWITCH2_BRIDGE_FULL_INPUT=1",
            "-DSWITCH2_BRIDGE_SOURCE_AUTO=1",
            "-DSWITCH2_PROBE_HUB=1",
            f"-DSWITCH2_BRIDGE_IMU_TARGET_MASK={imu_target}",
            "-DSWITCH_PICO_BLUEPAD32=1",
            "-DSWITCH_PICO_ENABLE_CLASSIC=1",
            f"-I{root / 'tests' / 'switch2_mouse_bridge_native_stubs'}",
            f"-I{root / 'tests' / 'wii_ir_aiming_native_stubs'}",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "switch2_native_gamepad_bridge_test.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "controller_input.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "native_gamepad_input.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "native_imu.cpp"),
            str(root / "src" / "firmware" / "core" / "controller_identity.cpp"),
            str(root / "src" / "firmware" / "profile" / "controller_profile.cpp"),
            str(
                root
                / "src"
                / "firmware"
                / "profile"
                / "controller_profile_transform.cpp"
            ),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
