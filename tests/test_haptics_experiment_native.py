import shutil
import subprocess
import zlib
from pathlib import Path

import pytest


@pytest.mark.parametrize("ram", [0, 1], ids=["flash", "sram"])
def test_haptics_experiment_native(tmp_path: Path, ram: int) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path / "haptics_experiment_test"
    corpus = tmp_path / "native_reports.bin"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HAPTICS_EXPERIMENT=1",
            f"-DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM={ram}",
            f"-I{root / 'tests' / 'haptics_experiment_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "haptics_experiment_test.cpp"),
            str(root / "src" / "firmware" / "input" / "haptics_experiment.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable), str(corpus)], check=True, cwd=root)
    reports = corpus.read_bytes()
    assert len(reports) == 288 * 143
    # Independent standard-library CRC across real module-generated packets:
    # A2 is covered once, CRC itself excluded, and stored little-endian.
    for offset in range(0, len(reports), 143):
        report = reports[offset : offset + 143]
        assert int.from_bytes(report[-4:], "little") == zlib.crc32(report[:-4])
