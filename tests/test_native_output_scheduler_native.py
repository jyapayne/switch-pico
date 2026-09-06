from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_native_output_scheduler_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "native_output_scheduler_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tests' / 'switch_native_output_native_stubs'}",
            f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "src" / "firmware" / "input" / "native_output_scheduler.cpp"),
            str(root / "tests" / "native_output_scheduler_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    for scenario in (
        "edf",
        "urgent",
        "reservation",
        "reservation-tie",
        "reservation-roll-forward",
        "grant",
        "reuse",
        "stale-completion",
        "bounded",
        "generic",
        "error",
    ):
        subprocess.run([str(executable), scenario], check=True, cwd=root)
