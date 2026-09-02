from __future__ import annotations

import struct
import zlib

import pytest

import switch_pico_bridge.config_manager as config_manager


def make_response(
    operation: int,
    payload: bytes = b"",
    *,
    status: int = config_manager.STATUS_OK,
    flags: int = 0,
    schema: int = 0,
    generation: int = 0,
) -> bytes:
    return struct.pack(
        "<4sBBBBHHII",
        b"SPMG",
        config_manager.PROTOCOL_VERSION,
        operation,
        status,
        flags,
        len(payload),
        schema,
        generation,
        zlib.crc32(payload) & 0xFFFFFFFF,
    ) + payload


class FakeDevice:
    bus = 1
    address = 7

    def __init__(self) -> None:
        self.configuration = struct.pack("<Hxx", 60)
        self.configuration_generation = 3
        self.transaction_id = 0
        self.transaction_payload = bytearray()
        self.transaction_expected_size = 0
        self.transaction_expected_crc = 0
        self.transaction_status = config_manager.STATUS_OK
        self.records = [
            (
                config_manager.TRANSPORT_CLASSIC,
                0xFE,
                bytes.fromhex("010203040506"),
            ),
            (
                config_manager.TRANSPORT_BLE,
                2,
                bytes.fromhex("A1A2A3A4A5A6"),
            ),
        ]
        self.pairing_generation = 4
        self.requests: list[int] = []

    def _pairing_payload(self) -> bytes:
        payload = bytearray([len(self.records), 0, 0, 0])
        for transport, address_type, address in self.records:
            payload.extend([transport, address_type])
            payload.extend(address)
        return bytes(payload)

    def _transaction_payload(self) -> bytes:
        stored_crc = zlib.crc32(self.configuration) & 0xFFFFFFFF
        return struct.pack(
            "<IHHIII",
            self.transaction_id,
            len(self.transaction_payload),
            self.transaction_expected_size,
            self.transaction_expected_crc,
            self.configuration_generation,
            stored_crc,
        )

    def ctrl_transfer(
        self,
        bm_request_type: int,
        request: int,
        value: int,
        index: int,
        data_or_w_length: object,
        timeout: int,
    ) -> bytes | int:
        assert value == config_manager.REQUEST_VALUE
        assert index == config_manager.REQUEST_INDEX
        assert timeout == config_manager.USB_TIMEOUT_MS
        self.requests.append(request)
        if bm_request_type == 0xC0:
            if request == config_manager.OP_INFO:
                return make_response(request, bytes([0, 2, 0, 2, 0, 0, 0, 2]))
            if request == config_manager.OP_CONFIGURATION_READ:
                return make_response(
                    request,
                    self.configuration,
                    schema=config_manager.CONFIGURATION_SCHEMA_VERSION,
                    generation=self.configuration_generation,
                )
            if request == config_manager.OP_TRANSACTION_STATUS:
                return make_response(
                    request,
                    self._transaction_payload(),
                    status=self.transaction_status,
                    schema=config_manager.CONFIGURATION_SCHEMA_VERSION,
                    generation=self.configuration_generation,
                )
            if request == config_manager.OP_PAIRING_READ:
                return make_response(
                    request,
                    self._pairing_payload(),
                    generation=self.pairing_generation,
                )
            raise AssertionError(f"unexpected IN request {request}")

        assert bm_request_type == 0x40
        encoded = bytes(data_or_w_length)
        assert encoded[:4] == b"SPMG"
        payload_size = struct.unpack_from("<H", encoded, 8)[0]
        payload = encoded[config_manager.REQUEST_HEADER_SIZE :]
        assert payload_size == len(payload)
        assert struct.unpack_from("<I", encoded, 12)[0] == (
            zlib.crc32(payload) & 0xFFFFFFFF
        )
        if request == config_manager.OP_CONFIGURATION_BEGIN:
            (
                self.transaction_id,
                _schema,
                self.transaction_expected_size,
                self.transaction_expected_crc,
            ) = struct.unpack("<IHHI", payload)
            self.transaction_payload = bytearray()
            self.transaction_status = config_manager.STATUS_PENDING
        elif request == config_manager.OP_CONFIGURATION_CHUNK:
            transaction_id, offset, chunk_size = struct.unpack_from(
                "<IHH", payload
            )
            assert transaction_id == self.transaction_id
            assert offset == len(self.transaction_payload)
            self.transaction_payload.extend(payload[8 : 8 + chunk_size])
        elif request == config_manager.OP_CONFIGURATION_COMMIT:
            assert struct.unpack("<I", payload)[0] == self.transaction_id
            assert len(self.transaction_payload) == self.transaction_expected_size
            assert (
                zlib.crc32(self.transaction_payload) & 0xFFFFFFFF
            ) == self.transaction_expected_crc
            self.configuration = bytes(self.transaction_payload)
            self.configuration_generation += 1
            self.transaction_status = config_manager.STATUS_OK
        elif request == config_manager.OP_CONFIGURATION_RESET:
            self.transaction_id = struct.unpack("<I", payload)[0]
            self.configuration = struct.pack("<Hxx", 60)
            self.configuration_generation += 1
            self.transaction_payload = bytearray(self.configuration)
            self.transaction_expected_size = len(self.configuration)
            self.transaction_expected_crc = (
                zlib.crc32(self.configuration) & 0xFFFFFFFF
            )
            self.transaction_status = config_manager.STATUS_OK
        elif request == config_manager.OP_PAIRING_REFRESH:
            self.pairing_generation += 1
        elif request == config_manager.OP_PAIRING_CLEAR:
            self.records = []
            self.pairing_generation += 1
        else:
            raise AssertionError(f"unexpected OUT request {request}")
        return len(encoded)


def test_response_validation() -> None:
    payload = make_response(config_manager.OP_INFO, b"12345678")
    envelope = config_manager.parse_response(payload, config_manager.OP_INFO)
    assert envelope.payload == b"12345678"

    malformed = [
        b"",
        b"NOPE" + bytes(config_manager.RESPONSE_HEADER_SIZE - 4),
        make_response(config_manager.OP_INFO, b"12345678")[:-1],
        make_response(config_manager.OP_INFO, b"12345678") + b"x",
    ]
    bad_crc = bytearray(make_response(config_manager.OP_INFO, b"12345678"))
    bad_crc[-1] ^= 1
    malformed.append(bytes(bad_crc))
    for response in malformed:
        with pytest.raises(config_manager.ConfigManagerError):
            config_manager.parse_response(response, config_manager.OP_INFO)


def test_configuration_transaction_and_reset() -> None:
    device = FakeDevice()
    before = config_manager.read_configuration(device)
    assert before.pairing_window_seconds == 60
    status = config_manager.write_configuration(
        device,
        config_manager.AdapterConfiguration(90, before.generation, before.crc),
        1.0,
    )
    assert status.stored_generation == 4
    assert config_manager.read_configuration(device).pairing_window_seconds == 90
    reset = config_manager.reset_configuration(device, 1.0)
    assert reset.stored_generation == 5
    assert config_manager.read_configuration(device).pairing_window_seconds == 60


def test_status_and_pairing_commands(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["status"]) == 0
    output = capsys.readouterr().out
    assert "Firmware: 0.2.0" in output
    assert "Pairing window: 60 seconds" in output

    assert config_manager.main(["pairings", "list"]) == 0
    output = capsys.readouterr().out
    assert "Classic 01:02:03:04:05:06" in output
    assert "BLE (public identity) A1:A2:A3:A4:A5:A6" in output

    assert config_manager.main(["pairings", "clear"]) == 2
    assert "requires --yes" in capsys.readouterr().err
    assert config_manager.main(["pairings", "clear", "--yes"]) == 0
    assert capsys.readouterr().out == "Cleared 2 stored pairing(s).\n"


def test_find_requires_selector_for_multiple_picos(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first = FakeDevice()
    second = FakeDevice()
    second.address = 8
    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: [first, second]
    )
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="multiple switch-pico devices",
    ):
        config_manager.find_pico(None, None)
    assert config_manager.find_pico(1, 8) is second
