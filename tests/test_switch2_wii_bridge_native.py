from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_libogc_ir import prepare


def test_wii_native_bridge_sensor_pointer_and_profile_contract(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    c_compiler = shutil.which("cc")
    assert c_compiler is not None, "a host C compiler is required"
    generated = prepare(tmp_path / "libogc_ir.c")
    ir_object = tmp_path / "libogc_ir.o"
    subprocess.run(
        [
            c_compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'external'}",
            "-c",
            str(generated),
            "-o",
            str(ir_object),
        ],
        check=True,
        cwd=root,
    )
    executable = tmp_path / "switch2_wii_bridge_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-pthread",
            "-DSWITCH_PICO_SWITCH2_USB_BRIDGE=1",
            "-DSWITCH2_BRIDGE_WII_INPUT=1",
            "-DSWITCH_PICO_BLUEPAD32=1",
            "-DSWITCH_PICO_ENABLE_BLE=1",
            "-DSWITCH_PICO_ENABLE_CLASSIC=1",
            "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE=1",
            "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=1",
            "-DSWITCH_PICO_WII_IR=1",
            "-DSWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES=0x02,0,0,0,0,1",
            f"-I{root / 'tests' / 'switch2_mouse_bridge_native_stubs'}",
            f"-I{root / 'tests' / 'wii_ir_aiming_native_stubs'}",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{root / 'bluepad32_config'}",
            f"-I{root / 'external'}",
            str(root / "tests" / "switch2_wii_bridge_test.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "controller_input.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "native_imu.cpp"),
            str(root / "src" / "firmware" / "input" / "wii_ir_pointer.cpp"),
            str(ir_object),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
