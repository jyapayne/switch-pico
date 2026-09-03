from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def host_compiler() -> str:
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    return compiler


def test_usb_output_driver_contracts(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = host_compiler()
    backends = (
        ("uart", []),
        ("bluepad32", ["-DSWITCH_PICO_BLUEPAD32=1"]),
    )
    for backend, backend_definitions in backends:
        for instance_count in range(1, 5):
            executable = (
                tmp_path
                / f"usb_output_driver_{backend}_{instance_count}_test"
            )
            result = subprocess.run(
                [
                    compiler,
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pedantic",
                    f"-DSWITCH_PICO_HID_INSTANCE_COUNT={instance_count}",
                    "-DSWITCH_PICO_USB_OUTPUT_MODES=1",
                    *backend_definitions,
                    f"-I{root / 'tests' / 'native_stubs'}",
                    f"-I{root}",
                    str(root / "tests" / "usb_output_driver_test.cpp"),
                    str(root / "switch_pro_driver.cpp"),
                    str(root / "usb_output_driver.cpp"),
                    str(root / "xinput_driver.cpp"),
                    str(root / "switch_haptics.cpp"),
                    "-o",
                    str(executable),
                ],
                check=False,
                cwd=root,
                text=True,
                capture_output=True,
            )
            assert result.returncode == 0, result.stderr
            _ = subprocess.run([str(executable)], check=True, cwd=root)
