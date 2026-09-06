from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch_native_output_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    firmware = root / "src" / "firmware"
    executable = tmp_path / "switch_native_output_test"
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
            f"-I{firmware}",
            str(firmware / "input" / "switch_native_output.cpp"),
            str(firmware / "usb" / "switch" / "switch_native_haptics.cpp"),
            str(firmware / "usb" / "switch" / "switch_haptics.cpp"),
            str(firmware / "configuration" / "adapter_configuration.cpp"),
            str(firmware / "core" / "controller_identity.cpp"),
            str(root / "tests" / "switch_native_output_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    # A fresh process isolates firmware owner globals without exposing test-only
    # production reset APIs. The fake runloop drives the real registered handlers.
    for scenario in (
        "approval",
        "model-gate",
        "revocation",
        "revocation-expired",
        "generation",
        "overflow",
        "retry",
        "partial-replacement",
        "stalled",
        "stalled-partial",
        "feedback-resume",
        "feedback-outlives-host",
        "feedback-congestion",
        "stateful",
        "credit-driven",
        "held-state",
        "pending-hold",
    ):
        subprocess.run([str(executable), scenario], check=True, cwd=root)
