from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def host_compiler() -> str:
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    return compiler


def compile_cpp(
    root: Path,
    compiler: str,
    output: Path,
    sources: list[Path],
    definitions: list[str],
    include_stubs: bool = False,
) -> subprocess.CompletedProcess[str]:
    command = [
        compiler,
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        *[f"-D{definition}" for definition in definitions],
    ]
    if include_stubs:
        command.append(f"-I{root / 'tests' / 'native_stubs'}")
    command.extend(
        [
            f"-I{root}",
            *[str(source) for source in sources],
            "-o",
            str(output),
        ]
    )
    return subprocess.run(
        command,
        check=False,
        cwd=root,
        text=True,
        capture_output=True,
    )


def test_generic_hid_descriptor_and_report_contracts(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = host_compiler()
    source = root / "tests" / "generic_hid_descriptors_test.cpp"
    for instance_count in range(1, 5):
        executable = tmp_path / f"generic_hid_descriptors_{instance_count}_test"
        result = compile_cpp(
            root,
            compiler,
            executable,
            [source],
            [
                f"EXPECTED_HID_INSTANCE_COUNT={instance_count}",
                f"SWITCH_PICO_HID_INSTANCE_COUNT={instance_count}",
            ],
        )
        assert result.returncode == 0, result.stderr
        subprocess.run([str(executable)], check=True, cwd=root)


def test_generic_hid_rejects_unsupported_interface_counts(
    tmp_path: Path,
) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = host_compiler()
    source = root / "tests" / "generic_hid_descriptors_test.cpp"
    for instance_count in (0, 5):
        executable = tmp_path / f"generic_hid_descriptors_{instance_count}_test"
        result = compile_cpp(
            root,
            compiler,
            executable,
            [source],
            [
                f"EXPECTED_HID_INSTANCE_COUNT={instance_count}",
                f"SWITCH_PICO_HID_INSTANCE_COUNT={instance_count}",
            ],
        )
        assert result.returncode != 0, (
            f"unsupported interface count {instance_count} compiled successfully"
        )


def test_generic_hid_driver_contexts(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    executable = tmp_path / "generic_hid_driver_test"
    result = compile_cpp(
        root,
        host_compiler(),
        executable,
        [
            root / "generic_hid_driver.cpp",
            root / "tests" / "generic_hid_driver_test.cpp",
        ],
        ["SWITCH_PICO_HID_INSTANCE_COUNT=4"],
        include_stubs=True,
    )
    assert result.returncode == 0, result.stderr
    subprocess.run([str(executable)], check=True, cwd=root)
