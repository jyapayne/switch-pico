import shutil
import subprocess
from pathlib import Path


def test_profile_service_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "profile_service_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "profile_service_test.cpp"),
            str(root / "src" / "firmware" / "core" / "controller_identity.cpp"),
            str(root / "src" / "firmware" / "profile" / "controller_profile.cpp"),
            str(root / "src" / "firmware" / "profile" / "profile_storage.cpp"),
            str(root / "src" / "firmware" / "profile" / "profile_service.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
