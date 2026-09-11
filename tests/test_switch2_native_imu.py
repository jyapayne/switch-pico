from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch2_native_imu(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "switch2_native_imu_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            str(root / "tests" / "switch2_native_imu_test.cpp"),
            str(root / "tools" / "switch2_usb_probe" / "native_imu.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
