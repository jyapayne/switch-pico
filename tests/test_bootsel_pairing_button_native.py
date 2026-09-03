from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_bootsel_pairing_button_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    for platform, rp2350 in (("rp2350", 1), ("rp2040", 0)):
        executable = tmp_path / f"bootsel_pairing_button_test_{platform}"
        subprocess.run(
            [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                f"-DPICO_RP2350={rp2350}",
                f"-I{root / 'tests' / 'bootsel_native_stubs'}",
                f"-I{root / 'src' / 'firmware'}",
                str(root / "src" / "firmware" / "platform" / "pico" / "bootsel_pairing_button.cpp"),
                str(root / "tests" / "bootsel_pairing_button_test.cpp"),
                "-o",
                str(executable),
            ],
            check=True,
            cwd=root,
        )
        subprocess.run([str(executable)], check=True, cwd=root)
