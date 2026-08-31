from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def compile_descriptor_test(
    root: Path,
    compiler: str,
    output: Path,
    expected_count: int,
    configured_count: int | None,
) -> subprocess.CompletedProcess[str]:
    command = [
        compiler,
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        f"-DEXPECTED_HID_INSTANCE_COUNT={expected_count}",
    ]
    if configured_count is not None:
        command.append(f"-DSWITCH_PICO_HID_INSTANCE_COUNT={configured_count}")
    command.extend(
        [
            f"-I{root}",
            str(root / "tests" / "switch_pro_descriptors_test.cpp"),
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


def host_compiler() -> str:
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    return compiler


def test_default_descriptor_contract_is_single_hid(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    executable = tmp_path / "switch_pro_descriptors_default_test"
    result = compile_descriptor_test(root, host_compiler(), executable, 1, None)
    assert result.returncode == 0, result.stderr
    subprocess.run([str(executable)], check=True, cwd=root)


def test_explicit_single_descriptor_contract(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    executable = tmp_path / "switch_pro_descriptors_single_test"
    result = compile_descriptor_test(root, host_compiler(), executable, 1, 1)
    assert result.returncode == 0, result.stderr
    subprocess.run([str(executable)], check=True, cwd=root)


def test_dual_descriptor_contract(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    executable = tmp_path / "switch_pro_descriptors_dual_test"
    result = compile_descriptor_test(root, host_compiler(), executable, 2, 2)
    assert result.returncode == 0, result.stderr
    subprocess.run([str(executable)], check=True, cwd=root)


def test_unsupported_hid_instance_counts_fail_to_compile(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = host_compiler()
    for unsupported_count in (0, 3):
        executable = tmp_path / f"switch_pro_descriptors_invalid_{unsupported_count}"
        result = compile_descriptor_test(
            root,
            compiler,
            executable,
            unsupported_count,
            unsupported_count,
        )
        assert result.returncode != 0, (
            f"unsupported HID instance count {unsupported_count} compiled successfully"
        )
