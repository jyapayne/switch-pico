import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("experiment_enabled", [False, True])
def test_usb_configuration_management_native(
    tmp_path: Path,
    experiment_enabled: bool,
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "usb_configuration_management_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            *(["-DSWITCH_PICO_HAPTICS_EXPERIMENT=1"] if experiment_enabled else []),
            f"-I{root / 'tests' / 'usb_management_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "usb_configuration_management_test.cpp"),
            str(root / "src" / "firmware" / "input" / "controller_macro_capture.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
