from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch_haptics_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "switch_haptics_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "src" / "firmware" / "usb" / "switch" / "switch_haptics.cpp"),
            str(root / "tests" / "switch_haptics_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)


def test_switch_native_haptics_encoder(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    firmware = root / "src" / "firmware"
    executable = tmp_path / "switch_native_haptics_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{firmware}",
            str(firmware / "usb" / "switch" / "switch_haptics.cpp"),
            str(firmware / "usb" / "switch" / "switch_native_haptics.cpp"),
            str(firmware / "core" / "controller_identity.cpp"),
            str(firmware / "profile" / "controller_profile.cpp"),
            str(firmware / "profile" / "controller_profile_transform.cpp"),
            str(root / "tests" / "switch_native_haptics_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
