from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("left", "composite", "hub", "count"),
    [
        (False, False, False, 1),
        (True, False, False, 1),
        (False, True, False, 2),
        (False, False, True, 2),
        (False, False, True, 4),
    ],
    ids=["right", "left", "composite", "hub-one-pair", "hub-two-pairs"],
)
@pytest.mark.parametrize(
    "imu_mode", [None, "OMIT_NATIVE_IMU", "ZERO_NATIVE_IMU_PAYLOAD"]
)
def test_switch2_usb_probe_protocol(
    tmp_path: Path,
    left: bool,
    composite: bool,
    hub: bool,
    count: int,
    imu_mode: str | None,
) -> None:
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
        (
            sdk / "lib" / "mbedtls"
            for sdk in sdk_candidates
            if (sdk / "lib" / "mbedtls" / "library" / "aes.c").is_file()
        ),
        None,
    )
    if mbedtls is None:
        pytest.skip(
            "Pico SDK mbedTLS required; configure firmware or set PICO_SDK_PATH"
        )
    probe = root / "tools" / "switch2_usb_probe"
    sides = (
        [bool(instance & 1) for instance in range(count)]
        if composite or hub
        else [left]
    )
    factory_rows = []
    user_rows = []
    for instance, is_left in enumerate(sides):
        # A and B must differ even for the same side: detect side-indexed aliases.
        pair = instance // 2
        factory_center = (
            f"{pair * 0x20}, {9 if is_left else 8}, {0x90 if is_left else 0x80}"
        )
        factory_rows.append(
            f"{{[0xa8] = {factory_center}, 0, 3, 0x30, 0, 4, 0x40,"
            f" [8191] = {(0xE2 if is_left else 0xE1) + pair * 2}}}"
        )
        # L deliberately has invalid user calibration despite valid magic.
        user_center = "0, 0, 0" if is_left else f"{0x10 + pair * 0x20}, 0x08, 0x81"
        user_rows.append(
            f"{{[0x40] = 0xb2, 0xa1, {user_center}, 0, 3, 0x30, 0, 4, 0x40,"
            f" [4095] = {(0xF2 if is_left else 0xF1) + pair * 2}}}"
        )
    (tmp_path / "probe_memory_data.h").write_text(
        '#include "model.h"\n'
        "static const uint8_t probe_factory_memories[PROBE_CONTROLLER_COUNT][8192] = {\n"
        + ",\n".join(factory_rows)
        + "\n};\n"
        "static const uint8_t probe_user_calibrations[PROBE_CONTROLLER_COUNT][4096] = {\n"
        + ",\n".join(user_rows)
        + "\n};\n"
    )
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
            f"-DSWITCH2_PROBE_JOYCON_LEFT={int(left)}",
            f"-DSWITCH2_PROBE_COMPOSITE={int(composite)}",
            f"-DSWITCH2_PROBE_HUB={int(hub)}",
            f"-DPROBE_CONTROLLER_COUNT={count}",
            f"-DSWITCH2_PROBE_NEUTRAL_INPUT={int(hub)}",
            *([f"-DSWITCH2_PROBE_{imu_mode}=1"] if imu_mode else []),
            f"-I{probe}",
            f"-I{tmp_path}",
            f"-I{mbedtls / 'include'}",
            str(probe / "protocol.c"),
            str(probe / "memory.c"),
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
