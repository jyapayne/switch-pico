from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_bluepad32_backend_lifecycle_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "bluepad32_backend_lifecycle_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
            f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
            f"-I{root}",
            str(root / "tests" / "bluepad32_backend_lifecycle_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )

    for scenario in (
        "ready-forward",
        "ready-reverse",
        "rejections",
        "lifecycle",
        "pairing-policy",
        "slot-lighting",
        "abxy-hotkey",
        "motion-hotkey",
        "flash-core-start",
        "flash-core-failure",
    ):
        subprocess.run([str(executable), scenario], check=True, cwd=root)
