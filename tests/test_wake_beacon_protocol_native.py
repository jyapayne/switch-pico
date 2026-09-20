import shutil
import subprocess
from pathlib import Path


def test_wake_beacon_protocol_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "wake_beacon_protocol_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tools' / 'switch2_wake_beacon'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "wake_beacon_protocol_test.cpp"),
            str(root / "tools" / "switch2_wake_beacon" / "beacon_protocol.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
