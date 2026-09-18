from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("left", "composite", "hub", "count", "overlap"),
    [
        (False, False, False, 1, False),
        (True, False, False, 1, False),
        (False, True, False, 2, False),
        (False, False, True, 2, False),
        (False, False, True, 4, False),
        (False, False, True, 4, True),
    ],
    ids=[
        "right",
        "left",
        "composite",
        "hub-one-pair",
        "hub-two-pairs",
        "overlap-pair-b",
    ],
)
def test_switch2_usb_probe_storage_native(
    tmp_path: Path, left: bool, composite: bool, hub: bool, count: int, overlap: bool
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    probe = root / "tools" / "switch2_usb_probe"
    # Stub geometry: 2 MiB flash, 8 KiB BTstack, 8 KiB configuration, 256 KiB
    # profiles. Link the SDK's end-of-image symbol at the exact reserved boundary,
    # or one byte into pair B while still safely below both original pair A banks.
    reserved_start = 0x1BC000 - max(2, count) * 8192
    binary_end = reserved_start + int(overlap)
    executable = tmp_path / "switch2_usb_probe_storage_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-DSWITCH2_PROBE_JOYCON_LEFT={int(left)}",
            f"-DSWITCH2_PROBE_COMPOSITE={int(composite)}",
            f"-DSWITCH2_PROBE_HUB={int(hub)}",
            f"-DSWITCH2_PROBE_NEUTRAL_INPUT={int(hub)}",
            f"-DPROBE_CONTROLLER_COUNT={count}",
            f"-DPROBE_TEST_STORAGE_OVERLAP={int(overlap)}",
            f"-I{root / 'tests' / 'switch2_usb_probe_storage_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{probe}",
            str(root / "tests" / "switch2_usb_probe_storage_test.cpp"),
            str(probe / "storage.cpp"),
            str(
                root
                / "src"
                / "firmware"
                / "configuration"
                / "configuration_storage.cpp"
            ),
            f"-Wl,--defsym=__flash_binary_end=probe_test_flash+{binary_end}",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
