from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_bluepad32_imu_normalization_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "bluepad32_imu_normalization_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'external' / 'bluepad32' / 'src' / 'components' / 'bluepad32' / 'include'}",
            str(root / "tests" / "bluepad32_imu_normalization_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
