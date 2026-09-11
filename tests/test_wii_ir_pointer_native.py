from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_libogc_ir import prepare


@pytest.mark.parametrize(
    "native_bridge", [False, True], ids=["legacy-mouse", "native-wii"]
)
def test_wii_ir_pointer_output_contract(tmp_path: Path, native_bridge: bool) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "wii_ir_pointer_test"
    defines = ["-DSWITCH_PICO_WII_IR=1"]
    defines.append(
        "-DSWITCH2_BRIDGE_WII_INPUT=1"
        if native_bridge
        else "-DSWITCH_PICO_WII_IR_MOUSE=1"
    )
    if native_bridge:
        c_compiler = shutil.which("cc")
        assert c_compiler is not None, "a host C compiler is required"
        generated = prepare(tmp_path / "libogc_ir.c")
        ir_object = tmp_path / "libogc_ir.o"
        subprocess.run(
            [
                c_compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                f"-I{root / 'external'}",
                "-c",
                str(generated),
                "-o",
                str(ir_object),
            ],
            check=True,
            cwd=root,
        )
        ir_sources = [str(ir_object)]
    else:
        ir_sources = [str(root / "src" / "firmware" / "input" / "wii_ir_tracker.cpp")]
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            *defines,
            f"-I{root / 'tests' / 'wii_ir_aiming_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{root / 'external'}",
            str(root / "tests" / "wii_ir_pointer_test.cpp"),
            str(root / "src" / "firmware" / "input" / "wii_ir_pointer.cpp"),
            *ir_sources,
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
