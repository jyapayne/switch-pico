import shutil
import subprocess
from pathlib import Path


def test_haptics_transport_probe_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "haptics_transport_probe_test"
    stubs = root / "tests" / "haptics_transport_probe_native_stubs"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{stubs}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "haptics_transport_probe_test.cpp"),
            str(stubs / "sdk.cpp"),
            str(root / "src" / "firmware" / "input" / "haptics_transport_probe.cpp"),
            "-Wl,--wrap=cyw43_bluetooth_hci_write",
            "-Wl,--wrap=cyw43_bluetooth_hci_read",
            "-Wl,--wrap=btstack_run_loop_base_poll_data_sources",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
