from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_switch2_pairing_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    executable = tmp_path / "switch2_pairing"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'tests' / 'switch2_pairing_native_stubs'}",
            f"-I{root / 'bluepad32_config'}",
            str(root / "bluepad32_config" / "parser" / "uni_switch2_pairing.c"),
            str(root / "tests" / "switch2_pairing_test.c"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
