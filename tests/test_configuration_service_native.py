import shutil
import subprocess
from pathlib import Path


def test_configuration_service_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "configuration_service_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
            f"-I{root}",
            str(root / "tests" / "configuration_service_test.cpp"),
            str(root / "adapter_configuration.cpp"),
            str(root / "configuration_service.cpp"),
            str(root / "configuration_storage.cpp"),
            str(root / "configuration_transaction.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
    subprocess.run(
        [str(executable), "abandoned-receive"], check=True, cwd=root
    )
