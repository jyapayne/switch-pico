"""The UART firmware's EP0 management must interoperate with config_manager."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from switch_pico_bridge import config_manager


@pytest.fixture(scope="module")
def harness(tmp_path_factory: pytest.TempPathFactory) -> Path:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"
    executable = tmp_path_factory.mktemp("uart_usb_management") / "harness"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
            f"-I{root / 'tests' / 'native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "src" / "firmware" / "usb" / "uart_usb_management.cpp"),
            str(root / "src" / "firmware" / "configuration" / "configuration_storage.cpp"),
            str(root / "tests" / "uart_usb_management_test.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    return executable


def run(harness: Path, *args: str) -> str:
    return subprocess.run(
        [str(harness), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


def test_info_response_parses_as_a_regular_pico(harness: Path) -> None:
    response = bytes.fromhex(run(harness, "info"))
    envelope = config_manager.parse_response(response, config_manager.OP_INFO)
    assert envelope.status == config_manager.STATUS_OK
    version = tuple(envelope.payload[:3])
    board, active_mode, capabilities = envelope.payload[3:6]
    assert version == (0, 2, 0)
    assert config_manager.BOARD_NAMES[board] == "Pico"
    assert config_manager.ACTIVE_MODE_NAMES[active_mode] == "Switch"
    assert capabilities & config_manager.CAPABILITY_INPUT
    assert capabilities & ~config_manager.CAPABILITY_MASK == 0


def test_bootsel_request_from_config_manager_reboots_after_status_stage(
    harness: Path,
) -> None:
    request = config_manager.encode_request(config_manager.OP_BOOTSEL_REBOOT, b"")
    assert run(harness, "bootsel", request.hex()) == "accepted=1 early=0 reboot=1"


def test_corrupted_bootsel_request_is_rejected_without_rebooting(harness: Path) -> None:
    request = bytearray(config_manager.encode_request(config_manager.OP_BOOTSEL_REBOOT, b""))
    request[-1] ^= 0x01  # break the payload CRC
    assert run(harness, "bootsel", bytes(request).hex()) == "accepted=0 early=0 reboot=0"
    wrong_operation = config_manager.encode_request(config_manager.OP_REBOOT, b"")
    assert run(harness, "bootsel", wrong_operation.hex()) == "accepted=0 early=0 reboot=0"


def test_unsupported_operations_are_stalled(harness: Path) -> None:
    assert run(harness, "other", str(config_manager.OP_CONFIGURATION_READ)) == "accepted=0"
