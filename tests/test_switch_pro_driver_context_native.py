from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch_pro_driver_four_contexts_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "switch_pro_driver_context_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
            f"-I{root / 'tests' / 'native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "src" / "firmware" / "usb" / "switch" / "switch_pro_driver.cpp"),
            str(root / "src" / "firmware" / "usb" / "usb_output_driver.cpp"),
            str(root / "src" / "firmware" / "usb" / "switch" / "switch_haptics.cpp"),
            str(root / "tests" / "switch_pro_driver_context_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
