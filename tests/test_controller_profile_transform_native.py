import shutil
import subprocess
from pathlib import Path


def test_controller_profile_transform_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "controller_profile_transform_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root}",
            str(root / "tests" / "controller_profile_transform_test.cpp"),
            str(root / "controller_identity.cpp"),
            str(root / "controller_profile.cpp"),
            str(root / "controller_profile_transform.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
