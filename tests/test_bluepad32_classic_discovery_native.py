from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


def test_bluepad32_classic_discovery_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    sdk = Path(
        os.environ.get("PICO_SDK_PATH", root / "build" / "_deps" / "pico_sdk-src")
    )
    btstack = sdk / "lib" / "btstack"
    bluepad = root / "external" / "bluepad32" / "src" / "components" / "bluepad32"
    if (
        not (btstack / "src" / "btstack.h").is_file()
        or not (bluepad / "bt" / "uni_bt_bredr.c").is_file()
    ):
        pytest.skip("requires Pico SDK and Bluepad32 checkout")
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    git = shutil.which("git")
    assert git is not None, "Git is required to prepare the project patch"
    relative = Path("src/components/bluepad32/bt/uni_bt_bredr.c")
    patched = tmp_path / relative
    patched.parent.mkdir(parents=True)
    shutil.copyfile(bluepad / "bt" / "uni_bt_bredr.c", patched)
    subprocess.run(
        [
            git,
            "apply",
            "--no-index",
            f"--include={relative.as_posix()}",
            str(root / "patches" / "bluepad32-sdl3-imu.patch"),
        ],
        cwd=tmp_path,
        check=True,
    )
    executable = tmp_path / "classic_discovery_test"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-ffunction-sections",
            "-fdata-sections",
            f"-I{root / 'tests' / 'btstack_credit_batch_native_stubs'}",
            f"-I{root / 'bluepad32_config'}",
            f"-I{bluepad / 'include'}",
            f"-I{btstack / 'src'}",
            f"-I{btstack / 'platform' / 'embedded'}",
            f"-I{btstack / '3rd-party' / 'bluedroid' / 'encoder' / 'include'}",
            f"-I{btstack / '3rd-party' / 'bluedroid' / 'decoder' / 'include'}",
            f"-I{btstack / '3rd-party' / 'yxml'}",
            str(root / "tests" / "bluepad32_classic_discovery_test.c"),
            str(patched),
            str(bluepad / "bt" / "uni_bt_conn.c"),
            str(btstack / "src" / "btstack_util.c"),
            str(btstack / "src" / "btstack_run_loop.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        cwd=root,
        check=True,
    )
    subprocess.run([str(executable)], cwd=root, check=True)
