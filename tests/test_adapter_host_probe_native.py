from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_adapter_host_probe_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "adapter_host_probe_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
            f"-I{root / 'tests' / 'mode_native_stubs'}",
            f"-I{root}",
            str(root / "adapter_host_probe.cpp"),
            str(root / "tests" / "adapter_host_probe_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
