from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_wii_ir_tracker_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "wii_ir_tracker_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "wii_ir_tracker_test.cpp"),
            str(root / "src" / "firmware" / "input" / "wii_ir_tracker.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
