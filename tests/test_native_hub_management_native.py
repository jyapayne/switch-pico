import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("controller_count", "neutral_input"),
    [(2, False), (2, True), (4, True)],
    ids=["native-management", "neutral-one-pair", "neutral-two-pair"],
)
def test_native_hub_management_native(
    tmp_path: Path, controller_count: int, neutral_input: bool
) -> None:
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
    flags = [
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-ffunction-sections",
        "-fdata-sections",
        "-DSWITCH2_PROBE_HUB=1",
        f"-DPROBE_CONTROLLER_COUNT={controller_count}",
    ]
    flags.append(f"-DSWITCH2_PROBE_NEUTRAL_INPUT={int(neutral_input)}")
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
            "-Wl,--gc-sections",
            *(str(root / path) for path in sources),
            str(transport),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    for reboot_slot in ("root", "child"):
        subprocess.run([str(executable), reboot_slot], check=True, cwd=root)
    router_executable = tmp_path / "native_hub_router_test"
    subprocess.run(
        [
            cc,
            "-std=c11",
            *flags,
            *includes,
            str(root / "tests" / "native_hub_router_test.c"),
            "-o",
            str(router_executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(router_executable)], check=True, cwd=root)


@pytest.mark.parametrize("controller_count", [2, 4], ids=["one-pair", "two-pair"])
@pytest.mark.parametrize("trace_enabled", [False, True], ids=["plain", "trace"])
def test_native_hub_cold_startup(
    tmp_path: Path, controller_count: int, trace_enabled: bool
) -> None:
    root = Path(__file__).resolve().parents[1]
    cc = shutil.which("cc") or shutil.which("gcc")
    assert cc is not None, "a host C compiler is required"
    executable = tmp_path / "native_hub_startup_test"
    flags = [f"-DPROBE_CONTROLLER_COUNT={controller_count}"]
    if trace_enabled:
        flags.append("-DSWITCH2_PROBE_TRACE_NATIVE_INPUT=1")
    subprocess.run(
        [
            cc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-ffunction-sections",
            "-fdata-sections",
            "-DSWITCH2_PROBE_HUB=1",
            *flags,
            f"-I{root / 'tests' / 'native_hub_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{root / 'tools' / 'pico_usb_address_probe'}",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            str(root / "tests" / "native_hub_startup_test.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    for scenario in ("ready", "delayed", "timeout"):
        subprocess.run([str(executable), scenario], check=True, cwd=root)
