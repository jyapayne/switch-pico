import shutil
import subprocess
from pathlib import Path


def test_native_hub_management_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    cc = shutil.which("cc") or shutil.which("gcc")
    cxx = shutil.which("c++") or shutil.which("g++")
    assert cc is not None and cxx is not None, "host C and C++ compilers are required"
    includes = [
        f"-I{root / 'tests' / 'native_hub_stubs'}",
        f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
        f"-I{root / 'src' / 'firmware'}",
        f"-I{root / 'tools' / 'pico_usb_address_probe'}",
        f"-I{root / 'tools' / 'switch2_usb_probe'}",
    ]
    flags = ["-Wall", "-Wextra", "-Werror", "-pedantic", "-DSWITCH2_PROBE_HUB=1"]
    transport = tmp_path / "native_hub_transport.o"
    executable = tmp_path / "native_hub_management_test"
    subprocess.run(
        [
            cc,
            "-std=c11",
            *flags,
            *includes,
            "-c",
            str(root / "tests" / "native_hub_transport_fixture.c"),
            "-o",
            str(transport),
        ],
        check=True,
        cwd=root,
    )
    sources = [
        "tests/native_hub_management_test.cpp",
        "src/firmware/usb/usb_configuration_management.cpp",
        "tools/switch2_usb_probe/bootsel.cpp",
        "src/firmware/configuration/adapter_configuration.cpp",
        "src/firmware/core/controller_identity.cpp",
        "src/firmware/profile/controller_profile.cpp",
        "src/firmware/profile/profile_storage.cpp",
        "src/firmware/profile/profile_service.cpp",
    ]
    subprocess.run(
        [
            cxx,
            "-std=c++17",
            *flags,
            *includes,
            *(str(root / path) for path in sources),
            str(transport),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
