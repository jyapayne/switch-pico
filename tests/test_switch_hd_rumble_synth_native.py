from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch_hd_rumble_synth_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "switch_hd_rumble_synth_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "src" / "firmware" / "input" / "switch_hd_rumble_synth.cpp"),
            str(root / "tests" / "switch_hd_rumble_synth_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root, timeout=15)
