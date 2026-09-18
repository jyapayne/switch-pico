from __future__ import annotations

import shutil
import signal
import subprocess
from pathlib import Path


def test_native_logger_keeps_usb_interrupt_progress(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    (tmp_path / "probe_version.h").write_text(
        "static const uint8_t probe_version_replies[PROBE_CONTROLLER_COUNT][16] = {{0}};\n"
        "static const uint8_t probe_firmware_versions[PROBE_CONTROLLER_COUNT][12] = {{0}};\n"
    )
    executable = tmp_path / "native_hub_log_test"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-ffunction-sections",
            "-fdata-sections",
            "-DSWITCH2_PROBE_HUB=1",
            "-DPROBE_CONTROLLER_COUNT=4",
            "-DSWITCH2_PROBE_NEUTRAL_INPUT=1",
            "-DSWITCH2_PROBE_TRACE_NATIVE_INPUT=1",
            "-DSWITCH2_PROBE_USB_INIT=1",
            "-DSWITCH2_PROBE_MEMORY=1",
            "-DSWITCH2_PROBE_VERSION_REPLY=1",
            f"-I{root / 'tests' / 'native_hub_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            f"-I{tmp_path}",
            str(root / "tests" / "native_hub_log_test.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
    for caller in ("core", "irq"):
        rejected = subprocess.run(
            [str(executable), caller], capture_output=True, check=False, cwd=root
        )
        assert rejected.returncode == -signal.SIGABRT, (
            "unsafe concurrent log producer was accepted"
        )
