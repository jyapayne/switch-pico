import shutil
import subprocess
from pathlib import Path


def test_cyw43_packet_transport_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    executable = tmp_path / "cyw43_packet_transport_test"
    stubs = root / "tests" / "cyw43_packet_transport_native_stubs"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{stubs}",
            str(root / "tests" / "cyw43_packet_transport_test.c"),
            str(stubs / "sdk.c"),
            str(
                root
                / "src"
                / "firmware"
                / "platform"
                / "pico"
                / "cyw43_packet_transport.c"
            ),
            "-Wl,--wrap=cyw43_btbus_read",
            "-Wl,--wrap=cyw43_btbus_init",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
