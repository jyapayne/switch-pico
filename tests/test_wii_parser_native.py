from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_bluepad32 import prepare_bluepad32


@pytest.mark.parametrize("full_input", [False, True])
def test_wii_parser_native_motion_and_lifecycle(
    tmp_path: Path, full_input: bool
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    prepared = prepare_bluepad32(
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
        tmp_path / "bluepad32-src",
    )
    component = prepared / "src" / "components" / "bluepad32"
    executable = tmp_path / "wii_parser_native_test"
    subprocess.run(
        [
            compiler,
            "-std=gnu11",
            "-O1",
            "-Wall",
            "-Wextra",
            "-ffunction-sections",
            "-fdata-sections",
            "-DENABLE_BLE",
            "-DENABLE_CLASSIC",
            f"-DSWITCH2_BRIDGE_FULL_INPUT={int(full_input)}",
            f"-I{root / 'tests' / 'switch_parser_native_stubs'}",
            f"-I{root / 'bluepad32_config'}",
            f"-I{component / 'include'}",
            str(root / "tests" / "wii_parser_native_test.c"),
            str(component / "parser" / "uni_hid_parser_wii.c"),
            str(
                root / "bluepad32_config" / "parser" / "uni_hid_parser_native_motion.c"
            ),
            str(component / "controller" / "uni_gamepad.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
