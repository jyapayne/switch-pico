from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def host_compiler() -> str:
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    return compiler


def test_xinput_feasibility_contracts(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = host_compiler()
    for instance_count in range(1, 5):
        executable = tmp_path / f"xinput_feasibility_{instance_count}_test"
        result = subprocess.run(
            [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                f"-DSWITCH_PICO_HID_INSTANCE_COUNT={instance_count}",
                f"-I{root}",
                str(root / "tests" / "xinput_feasibility_test.cpp"),
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
