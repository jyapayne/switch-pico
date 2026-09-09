import shutil
import subprocess
from pathlib import Path


def test_switch2_wake_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    for mode in ("mixed", "ble", "classic"):
        executable = tmp_path / f"switch2_wake_test_{mode}"
        subprocess.run(
            [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                f"-DSWITCH_PICO_ENABLE_BLE={int(mode != 'classic')}",
                f"-DSWITCH_PICO_ENABLE_CLASSIC={int(mode != 'ble')}",
                f"-I{root / 'tests' / 'switch2_wake_native_stubs'}",
                f"-I{root / 'src' / 'firmware'}",
                f"-I{root / 'bluepad32_config'}",
                str(root / "tests" / "switch2_wake_test.cpp"),
                "-o",
                str(executable),
            ],
            check=True,
            cwd=root,
        )
        subprocess.run([str(executable)], check=True, cwd=root)
