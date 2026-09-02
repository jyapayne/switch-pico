from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_bluepad32_backend_lifecycle_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    for adapter_feasibility in (False, True):
        suffix = "_adapter" if adapter_feasibility else ""
        executable = tmp_path / f"bluepad32_backend_lifecycle_test{suffix}"
        command = [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
        ]
        if adapter_feasibility:
            command.append("-DSWITCH_PICO_ADAPTER_FEASIBILITY=1")
        command.extend(
            [
                f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
                f"-I{root}",
                str(root / "tests" / "bluepad32_backend_lifecycle_test.cpp"),
                "-o",
                str(executable),
            ]
        )
        subprocess.run(command, check=True, cwd=root)

        for scenario in (
            "ready-forward",
            "ready-reverse",
            "rejections",
            "lifecycle",
            "pairing-policy",
            "slot-lighting",
            "abxy-hotkey",
            "motion-hotkey",
            "clear-pairings",
            "flash-core-start",
            "flash-core-failure",
        ):
            subprocess.run([str(executable), scenario], check=True, cwd=root)
