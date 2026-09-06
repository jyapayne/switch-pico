from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_bluepad32 import prepare_bluepad32


def test_switch_parser_native_wire_and_lifecycle(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    prepared = prepare_bluepad32(
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
        tmp_path / "bluepad32-src",
    )
    component = prepared / "src" / "components" / "bluepad32"
    executable = tmp_path / "switch_parser_native_test"
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
            f"-I{root / 'tests' / 'switch_parser_native_stubs'}",
            f"-I{root / 'bluepad32_config'}",
            f"-I{component / 'include'}",
            str(root / "tests" / "switch_parser_native_test.c"),
            str(component / "parser" / "uni_hid_parser_switch.c"),
            str(component / "uni_hid_device.c"),
            str(component / "uni_circular_buffer.c"),
            str(component / "bt" / "uni_bt_conn.c"),
            str(component / "controller" / "uni_gamepad.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
