from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


def test_switch2_usb_probe_deferred_sample(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    sdk_candidates = [
        root / "build" / "_deps" / "pico_sdk-src",
        root / "external" / "pico-sdk",
    ]
    if sdk_path := os.environ.get("PICO_SDK_PATH"):
        sdk_candidates.insert(0, Path(sdk_path))
    mbedtls = next(
        (sdk / "lib" / "mbedtls" for sdk in sdk_candidates
         if (sdk / "lib" / "mbedtls" / "library" / "aes.c").is_file()),
        None,
    )
    if mbedtls is None:
        pytest.skip("Pico SDK mbedTLS required; configure firmware or set PICO_SDK_PATH")
    probe = root / "tools" / "switch2_usb_probe"
    executable = tmp_path / "switch2_usb_probe_protocol"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f'-DMBEDTLS_CONFIG_FILE="{probe / "mbedtls_config.h"}"',
            f"-I{probe}",
            f"-I{mbedtls / 'include'}",
            str(probe / "protocol.c"),
            str(mbedtls / "library" / "aes.c"),
            str(mbedtls / "library" / "platform_util.c"),
            str(root / "tests" / "switch2_usb_probe_protocol_test.c"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
