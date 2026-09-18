from __future__ import annotations

import json
import os
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import usb.core
import usb.util


class Root:
    idVendor = 0x057E
    idProduct = 0x2068
    bus = 2

    def __init__(self, address=7, ports=(3, 4), serial="switch-pico-test"):
        self.address = address
        self.port_numbers = ports
        self.serial = self.cached_serial = serial
        self.transfers = []
        self.bootsel_requested = False
        self.write_result = 16

    def ctrl_transfer(self, request_type, request, value, index, data, *, timeout):
        self.transfers.append((request_type, request, value, index, data))
        assert timeout > 0
        if request_type == 0x80:
            assert request == 6, "recovery must only read root identity descriptors"
            if value == 0x0100:
                assert index == 0 and data == 18
                descriptor = bytearray(18)
                descriptor[:2] = b"\x12\x01"
                descriptor[4] = 9
                descriptor[16] = 3
                struct.pack_into("<HH", descriptor, 8, self.idVendor, self.idProduct)
                return descriptor
            if value == 0x0300:
                assert index == 0 and data == 255
                return b"\x04\x03\x09\x04"
            assert value == 0x0303 and index == 0x0409 and data == 255
            serial = self.serial.encode("utf-16-le")
            return bytes((len(serial) + 2, 3)) + serial
        assert (request_type, request, value, index) == (0x40, 4, 0x5350, 1)
        assert data == b"SPMG\x01\x04\x00\x00" + bytes(8)
        assert not self.bootsel_requested, "recovery must never retry a reboot request"
        self.bootsel_requested = True
        if isinstance(self.write_result, Exception):
            raise self.write_result
        return self.write_result

    def __getattr__(self, name):
        raise AssertionError(f"forbidden USB operation during recovery: {name}")


@pytest.fixture
def recovery_rig(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    import native_joycon_hub_check as check

    root = Root()
    rom = SimpleNamespace(
        bus=root.bus,
        address=8,
        port_numbers=root.port_numbers,
        idVendor=0x2E8A,
        idProduct=0x000F,
    )
    rig = SimpleNamespace(
        check=check,
        root=root,
        rom=rom,
        devices=[root],
        after_reboot=[rom],
        acl=[],
        disposed=[],
        clock=0.0,
        capture=tmp_path / "recovery.json",
    )

    def sleep(seconds):
        rig.clock += seconds

    monkeypatch.setattr(
        check, "time", SimpleNamespace(monotonic=lambda: rig.clock, sleep=sleep)
    )

    def find(*, find_all):
        assert find_all
        assert rig.capture.exists(), "audit capture must precede USB discovery"
        return rig.after_reboot if root.bootsel_requested else rig.devices

    monkeypatch.setattr(usb.core, "find", find)
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if str(path).startswith("/sys/bus/usb/devices/"):
            for device in rig.devices:
                name = f"{device.bus}-" + ".".join(map(str, device.port_numbers))
                if path == Path("/sys/bus/usb/devices") / name / "serial":
                    return device.cached_serial
            raise FileNotFoundError(str(path))
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)

    def permissions(command, **kwargs):
        assert command == [
            "sudo",
            "-n",
            "setfacl",
            "-m",
            f"u:{os.getuid()}:rw",
            f"/dev/bus/usb/{root.bus:03d}/{root.address:03d}",
        ], "recovery may grant access only to the verified root"
        rig.acl.append(command[-1])

    monkeypatch.setattr(check.subprocess, "run", permissions)

    def forbidden(*args, **kwargs):
        raise AssertionError("recovery attempted qualification or interface management")

    monkeypatch.setattr(check, "model_references", forbidden)
    monkeypatch.setattr(check, "child_models", forbidden)
    monkeypatch.setattr(usb.util, "claim_interface", forbidden)
    monkeypatch.setattr(usb.util, "release_interface", forbidden)

    def dispose(device):
        assert device is root, "unselected devices must receive no resource operations"
        rig.disposed.append(device)

    monkeypatch.setattr(usb.util, "dispose_resources", dispose)

    def run(*options, recovery=True):
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "native_joycon_hub_check.py",
                "--output",
                str(rig.capture),
                "--timeout",
                "0.4" if recovery else "20",
                *(["--reboot-bootsel"] if recovery else []),
                *options,
            ],
        )
        status = check.main()
        return status, json.loads(rig.capture.read_text())

    rig.run = run
    return rig


@pytest.mark.parametrize("options", [(), ("--pairs", "2")])
def test_root_only_recovery_confirms_rom_without_qualification(
    recovery_rig, tmp_path, options
):
    rig = recovery_rig
    # A real Nintendo hub shares VID/PID but must receive no ACL or transfers.
    foreign = Root(address=11, ports=(3, 5), serial="Nintendo")
    rig.devices.append(foreign)
    status, audit = rig.run("--build-dir", str(tmp_path / "not-configured"), *options)

    assert status == 0 and audit["recovery_success"]
    assert audit["devices"]["root"]["address"] == rig.root.address
    assert audit["devices"]["bootsel"]["ports"] == list(rig.root.port_numbers)
    assert audit["devices"]["bootsel"]["bus"] == rig.root.bus
    assert audit["devices"]["bootsel"]["address"] == rig.rom.address
    assert audit["recovery"] == {
        "request_attempted": True,
        "request_acknowledged": True,
        "root_disappeared": True,
        "rom_confirmed": True,
    }
    assert audit["operation"] == "bootsel_recovery"
    assert not audit["qualification_success"]
    assert not audit["live_input_proven"]
    assert not audit["live_imu_proven"]
    assert not audit["gameplay_proven"]
    assert foreign.transfers == []
    assert rig.root.bootsel_requested


def test_ambiguous_picos_are_refused_before_permissions(recovery_rig):
    rig = recovery_rig
    other = Root(address=11, ports=(3, 5), serial="switch-pico-other")
    rig.devices.append(other)
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert not audit["recovery"]["request_attempted"]
    assert rig.acl == []
    assert rig.root.transfers == other.transfers == []


def test_foreign_root_is_refused_without_usb_access(recovery_rig):
    rig = recovery_rig
    rig.root.serial = rig.root.cached_serial = "Nintendo"
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert not audit["recovery"]["request_attempted"]
    assert rig.acl == []
    assert rig.root.transfers == []


def test_changed_usb_serial_cannot_receive_reboot(recovery_rig):
    rig = recovery_rig
    rig.root.serial = "Nintendo"
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert not audit["recovery"]["request_attempted"]
    assert not rig.root.bootsel_requested
    assert all(transfer[0] == 0x80 for transfer in rig.root.transfers)


@pytest.mark.parametrize("write_result", [15, usb.core.USBError("disconnected")])
def test_incomplete_control_write_is_not_acknowledged(recovery_rig, write_result):
    rig = recovery_rig
    rig.root.write_result = write_result
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert audit["recovery"]["request_attempted"]
    assert not audit["recovery"]["request_acknowledged"]
    assert not audit["recovery"]["rom_confirmed"]


def test_ack_without_rom_enumeration_is_incomplete(recovery_rig):
    rig = recovery_rig
    rig.after_reboot = []
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert audit["recovery"]["request_acknowledged"]
    assert audit["recovery"]["root_disappeared"]
    assert not audit["recovery"]["rom_confirmed"]


@pytest.mark.parametrize(("bus", "ports"), [(2, (3, 5)), (3, (3, 4))])
def test_rom_on_another_physical_path_does_not_confirm_recovery(
    recovery_rig, bus, ports
):
    rig = recovery_rig
    rig.rom.bus = bus
    rig.rom.port_numbers = ports
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert audit["recovery"]["request_acknowledged"]
    assert not audit["recovery"]["rom_confirmed"]


def test_rom_cannot_confirm_while_root_still_enumerates(recovery_rig):
    rig = recovery_rig
    rig.after_reboot = [rig.root, rig.rom]
    status, audit = rig.run()

    assert status == 2 and not audit["recovery_success"]
    assert audit["recovery"]["request_acknowledged"]
    assert not audit["recovery"]["root_disappeared"]
    assert not audit["recovery"]["rom_confirmed"]


@pytest.mark.parametrize(
    "options",
    [
        ("--input-only",),
        ("--neutral",),
        ("--rumble-sample", "0"),
        ("--capture-trace-on-error",),
    ],
)
def test_recovery_rejects_qualification_and_motor_options(recovery_rig, options):
    rig = recovery_rig
    with pytest.raises(SystemExit) as error:
        rig.run(*options)

    assert error.value.code == 2
    assert not rig.capture.exists()
    assert rig.acl == []
    assert rig.root.transfers == []


def test_qualification_failure_does_not_reboot(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    import native_joycon_hub_check as check

    capture = tmp_path / "failed-qualification.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "native_joycon_hub_check.py",
            "--output",
            str(capture),
            "--build-dir",
            str(tmp_path / "absent-build"),
        ],
    )

    def forbidden_usb(*args, **kwargs):
        raise AssertionError(
            "failed qualification must not discover or reboot a device"
        )

    monkeypatch.setattr(usb.core, "find", forbidden_usb)
    assert check.main() == 2
    audit = json.loads(capture.read_text())
    assert not audit["success"]
