from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_bluepad32 import prepare_bluepad32


def test_switch2_parser_protocol_and_lifecycle(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    sdk_candidates = [
        root / "build" / "_deps" / "pico_sdk-src",
        root / "external" / "pico-sdk",
    ]
    if sdk_path := os.environ.get("PICO_SDK_PATH"):
        sdk_candidates.insert(0, Path(sdk_path))
    btstack = next(
        (sdk / "lib" / "btstack" / "src" for sdk in sdk_candidates
         if (sdk / "lib" / "btstack" / "src" / "ble" / "gatt_client.h").is_file()),
        None,
    )
    if btstack is None:
        pytest.skip("Pico SDK BTstack headers required; configure firmware or set PICO_SDK_PATH")
    prepared = prepare_bluepad32(
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
        tmp_path / "bluepad32-src",
    )
    component = prepared / "src" / "components" / "bluepad32"
    executable = tmp_path / "switch2_parser_native_test"
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
            f"-I{root / 'tests' / 'switch2_parser_native_stubs'}",
            f"-I{root / 'bluepad32_config'}",
            f"-I{component / 'include'}",
            f"-I{btstack}",
            f"-I{btstack.parent / '3rd-party' / 'bluedroid' / 'encoder' / 'include'}",
            f"-I{btstack.parent / '3rd-party' / 'bluedroid' / 'decoder' / 'include'}",
            f"-I{btstack.parent / '3rd-party' / 'yxml'}",
            str(root / "tests" / "switch2_parser_native_test.c"),
            str(root / "bluepad32_config" / "parser" / "uni_hid_parser_switch2.c"),
            str(root / "bluepad32_config" / "parser" / "uni_switch2_haptics.c"),
            str(btstack / "btstack_util.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
