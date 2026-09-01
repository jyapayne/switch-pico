from __future__ import annotations

import struct

import pytest

import switch_pico_bridge.pairing_manager as pairing_manager


def make_payload(
    generation: int,
    records: list[tuple[int, int, bytes]],
    *,
    status: int = pairing_manager.STATUS_READY,
    overflow: bool = False,
) -> bytes:
    payload = bytearray(b"SPPM")
    payload.extend(
        [
            pairing_manager.PROTOCOL_VERSION,
            status,
            len(records),
            int(overflow),
        ]
    )
    payload.extend(struct.pack("<I", generation))
    for transport, address_type, address in records:
        payload.extend([transport, address_type])
        payload.extend(address)
    return bytes(payload)


class FakeDevice:
    bus = 1
    address = 7

    def __init__(self) -> None:
        self.generation = 3
        self.records = [
            (
                pairing_manager.TRANSPORT_CLASSIC,
                0xFE,
                bytes.fromhex("010203040506"),
            ),
            (
                pairing_manager.TRANSPORT_BLE,
                2,
                bytes.fromhex("A1A2A3A4A5A6"),
            ),
        ]
        self.requests: list[int] = []

    def ctrl_transfer(
        self,
        bm_request_type: int,
        request: int,
        value: int,
        index: int,
        data_or_w_length: object,
        timeout: int,
    ) -> bytes | int:
        assert value == pairing_manager.REQUEST_VALUE
        assert index == pairing_manager.REQUEST_INDEX
        assert timeout == pairing_manager.USB_TIMEOUT_MS
        self.requests.append(request)
        if bm_request_type == 0xC0:
            assert request == pairing_manager.REQUEST_GET
            return make_payload(self.generation, self.records)
        assert bm_request_type == 0x40
        if request == pairing_manager.REQUEST_REFRESH:
            self.generation += 1
        elif request == pairing_manager.REQUEST_CLEAR:
            self.records = []
            self.generation += 1
        else:
            raise AssertionError(f"unexpected request {request}")
        return 0


def test_parse_snapshot() -> None:
    snapshot = pairing_manager.parse_snapshot(
        make_payload(
            0x78563412,
            [
                (
                    pairing_manager.TRANSPORT_CLASSIC,
                    0xFE,
                    bytes.fromhex("010203040506"),
                ),
                (
                    pairing_manager.TRANSPORT_BLE,
                    3,
                    bytes.fromhex("A1A2A3A4A5A6"),
                ),
            ],
            overflow=True,
        )
    )
    assert snapshot.generation == 0x78563412
    assert snapshot.overflow
    assert snapshot.records[0].transport_text == "Classic"
    assert snapshot.records[0].address_text == "01:02:03:04:05:06"
    assert snapshot.records[1].transport_text == "BLE (random identity)"


@pytest.mark.parametrize(
    "payload",
    [
        b"",
        b"NOPE" + bytes(8),
        b"SPPM\x02" + bytes(7),
        b"SPPM\x01\x00\x11\x00" + bytes(4),
    ],
)
def test_parse_rejects_invalid_payload(payload: bytes) -> None:
    with pytest.raises(pairing_manager.PairingManagerError):
        pairing_manager.parse_snapshot(payload)


def test_list_and_clear_commands(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(pairing_manager, "_candidate_devices", lambda: [device])

    assert pairing_manager.main(["list"]) == 0
    output = capsys.readouterr().out
    assert "Classic 01:02:03:04:05:06" in output
    assert "BLE (public identity) A1:A2:A3:A4:A5:A6" in output

    assert pairing_manager.main(["clear"]) == 2
    assert "requires --yes" in capsys.readouterr().err

    assert pairing_manager.main(["clear", "--yes"]) == 0
    assert capsys.readouterr().out == "Cleared 2 stored pairing(s).\n"
    assert device.records == []
    assert pairing_manager.REQUEST_REFRESH in device.requests
    assert pairing_manager.REQUEST_CLEAR in device.requests


def test_find_requires_selector_for_multiple_picos(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first = FakeDevice()
    second = FakeDevice()
    second.address = 8
    monkeypatch.setattr(
        pairing_manager, "_candidate_devices", lambda: [first, second]
    )
    with pytest.raises(
        pairing_manager.PairingManagerError,
        match="multiple switch-pico devices",
    ):
        pairing_manager.find_pico(None, None)
    assert pairing_manager.find_pico(1, 8) is second
