from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

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
    port_numbers = (1,)

    def __init__(self) -> None:
        self.configuration = struct.pack(
            "<HB5x", 60, config_manager.REQUESTED_MODE_AUTO
        )
        self.configuration_generation = 3
        self.active_mode = config_manager.ACTIVE_MODE_SWITCH_PROBE
        self.capabilities = (
            config_manager.CAPABILITY_INPUT
            | config_manager.CAPABILITY_RUMBLE
            | config_manager.CAPABILITY_MOTION
        )
        self.transaction_id = 0
        self.transaction_payload = bytearray()
        self.transaction_expected_size = 0
        self.transaction_expected_crc = 0
        self.transaction_status = config_manager.STATUS_OK
        self.pending_requested_mode: int | None = None
        self.mode_pending_reads = 0
        self.fail_mode_status: int | None = None
        self.reboot_transaction_ids: list[int] = []
        self.bootsel_reboot_requested = False
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
        self.global_identity = config_manager.ControllerIdentity.global_fallback()
        self.stable_identity = config_manager.ControllerIdentity(
            True,
            config_manager.TRANSPORT_CLASSIC,
            0,
            bytes.fromhex("102030405060"),
            0x045E,
            0x02FD,
        )
        self.profile_identities = [
            self.global_identity,
            self.stable_identity,
        ]
        self.active_profiles = {
            identity.to_bytes(): index
            for identity, index in zip(self.profile_identities, (0, 1))
        }
        default_profile = config_manager.ControllerProfile.default().to_bytes()
        self.profiles = {
            (identity.to_bytes(), index): default_profile
            for identity in self.profile_identities
            for index in range(config_manager.PROFILE_CAPACITY)
        }
        self.selected_profile = (self.global_identity.to_bytes(), 0)
        self.profile_generation = 7
        self.profile_transaction_id = 0
        self.profile_transaction_identity = self.global_identity.to_bytes()
        self.profile_transaction_index = 0
        self.profile_transaction_payload = bytearray()
        self.profile_transaction_expected_size = 0
        self.profile_transaction_expected_crc = 0
        self.profile_transaction_status = config_manager.STATUS_OK
        self.fail_profile_commit_status: int | None = None
        self.bad_profile_response_crc = False
        self.requests: list[int] = []
        self.out_requests: list[tuple[int, bytes, bytes]] = []
        self.profile_chunk_sizes: list[int] = []
        self.pending_profile_mutation: tuple[int, bytes, int] | None = None
        self.profile_transaction_pending_reads = 0
        self.profile_status_responses: list[tuple[int, int]] = []

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

    def _profile_list_payload(self) -> bytes:
        payload = bytearray([len(self.profile_identities)])
        for identity in self.profile_identities:
            payload.extend(identity.to_bytes())
            payload.extend((self.active_profiles[identity.to_bytes()], 0))
        return bytes(payload)

    def _profile_transaction_payload(self) -> bytes:
        stored = self.profiles.get(
            (
                self.profile_transaction_identity,
                self.profile_transaction_index,
            ),
            bytes(config_manager.PROFILE_SIZE),
        )
        return struct.pack(
            "<IHHIII",
            self.profile_transaction_id,
            len(self.profile_transaction_payload),
            self.profile_transaction_expected_size,
            self.profile_transaction_expected_crc,
            self.profile_generation,
            zlib.crc32(stored) & 0xFFFFFFFF,
        )

    def _queue_profile_mutation(self, operation: int, payload: bytes) -> None:
        assert len(payload) == 19
        self.profile_transaction_id = struct.unpack_from("<I", payload)[0]
        assert self.profile_transaction_id != 0
        self.profile_transaction_identity = payload[4:18]
        self.profile_transaction_index = payload[18]
        self.profile_transaction_payload = bytearray()
        self.profile_transaction_expected_size = 0
        self.profile_transaction_expected_crc = 0
        self.profile_transaction_status = config_manager.STATUS_PENDING
        self.profile_transaction_pending_reads = 1
        self.pending_profile_mutation = (
            operation,
            self.profile_transaction_identity,
            self.profile_transaction_index,
        )

    def _complete_profile_mutation(self) -> None:
        assert self.pending_profile_mutation is not None
        operation, identity, profile_index = self.pending_profile_mutation
        if self.fail_profile_commit_status is not None:
            self.profile_transaction_status = self.fail_profile_commit_status
            self.pending_profile_mutation = None
            return
        if operation == config_manager.OP_PROFILE_RESET:
            indices = (
                range(config_manager.PROFILE_CAPACITY)
                if profile_index == config_manager.PROFILE_NONE_BUTTON
                else (profile_index,)
            )
            default = config_manager.ControllerProfile.default().to_bytes()
            for reset_index in indices:
                self.profiles[(identity, reset_index)] = default
        else:
            assert operation == config_manager.OP_PROFILE_ACTIVATE
            self.active_profiles[identity] = profile_index
        self.profile_generation += 1
        self.profile_transaction_status = config_manager.STATUS_OK
        self.pending_profile_mutation = None

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
                return make_response(
                    request,
                    bytes(
                        [
                            0,
                            2,
                            0,
                            2,
                            self.active_mode,
                            self.capabilities,
                            0,
                            2,
                        ]
                    ),
                )
            if request == config_manager.OP_RUNTIME_DIAGNOSTICS:
                return make_response(
                    request,
                    struct.pack("<7I4B", 6, 1200, 120, 5000, 8, 2, 10,
                                2, 2, 1, 1),
                )
            if request == config_manager.OP_CONFIGURATION_READ:
                return make_response(
                    request,
                    self.configuration,
                    schema=config_manager.CONFIGURATION_SCHEMA_VERSION,
                    generation=self.configuration_generation,
                )
            if request == config_manager.OP_TRANSACTION_STATUS:
                if (
                    self.transaction_status == config_manager.STATUS_PENDING
                    and self.pending_requested_mode is not None
                ):
                    if self.mode_pending_reads:
                        self.mode_pending_reads -= 1
                    else:
                        self.transaction_status = (
                            self.fail_mode_status
                            if self.fail_mode_status is not None
                            else config_manager.STATUS_OK
                        )
                        if self.transaction_status == config_manager.STATUS_OK:
                            pairing_window = struct.unpack_from(
                                "<H", self.configuration
                            )[0]
                            self.configuration = struct.pack(
                                "<HB5x",
                                pairing_window,
                                self.pending_requested_mode,
                            )
                            self.configuration_generation += 1
                        self.pending_requested_mode = None
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
            if request == config_manager.OP_PROFILE_LIST:
                return make_response(
                    request,
                    self._profile_list_payload(),
                    schema=config_manager.PROFILE_SCHEMA_VERSION,
                    generation=self.profile_generation,
                )
            if request == config_manager.OP_PROFILE_READ:
                response = bytearray(
                    make_response(
                        request,
                        self.profiles[self.selected_profile],
                        schema=config_manager.PROFILE_SCHEMA_VERSION,
                        generation=self.profile_generation,
                    )
                )
                if self.bad_profile_response_crc:
                    response[-1] ^= 1
                return bytes(response)
            if request == config_manager.OP_PROFILE_TRANSACTION_STATUS:
                if self.profile_transaction_status == config_manager.STATUS_PENDING:
                    if self.profile_transaction_pending_reads:
                        self.profile_transaction_pending_reads -= 1
                    elif self.pending_profile_mutation is not None:
                        self._complete_profile_mutation()
                self.profile_status_responses.append(
                    (
                        self.profile_transaction_id,
                        self.profile_transaction_status,
                    )
                )
                return make_response(
                    request,
                    self._profile_transaction_payload(),
                    status=self.profile_transaction_status,
                    schema=config_manager.PROFILE_SCHEMA_VERSION,
                    generation=self.profile_generation,
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
        self.out_requests.append((request, payload, encoded))
        if request == config_manager.OP_CONFIGURATION_BEGIN:
            (
                self.transaction_id,
                _schema,
                self.transaction_expected_size,
                self.transaction_expected_crc,
            ) = struct.unpack("<IHHI", payload)
            assert 0 < self.transaction_id <= (
                config_manager.HOST_TRANSACTION_ID_MASK
            )
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
            assert 0 < self.transaction_id <= (
                config_manager.HOST_TRANSACTION_ID_MASK
            )
            self.configuration = struct.pack(
                "<HB5x", 60, config_manager.REQUESTED_MODE_AUTO
            )
            self.configuration_generation += 1
            self.transaction_payload = bytearray(self.configuration)
            self.transaction_expected_size = len(self.configuration)
            self.transaction_expected_crc = (
                zlib.crc32(self.configuration) & 0xFFFFFFFF
            )
            self.transaction_status = config_manager.STATUS_OK
        elif request == config_manager.OP_MODE_SET:
            assert len(payload) == 5
            self.transaction_id, requested_mode = struct.unpack("<IB", payload)
            assert 0 < self.transaction_id <= 0x7FFFFFFF
            assert requested_mode in (
                config_manager.REQUESTED_MODE_AUTO,
                config_manager.REQUESTED_MODE_SWITCH,
                config_manager.REQUESTED_MODE_XINPUT,
                config_manager.REQUESTED_MODE_DINPUT,
                config_manager.REQUESTED_MODE_MAC,
            )
            self.transaction_payload = bytearray()
            self.transaction_expected_size = 0
            self.transaction_expected_crc = 0
            self.transaction_status = config_manager.STATUS_PENDING
            self.pending_requested_mode = requested_mode
            self.mode_pending_reads = 1
        elif request == config_manager.OP_REBOOT:
            assert len(payload) == 4
            reboot_transaction_id = struct.unpack("<I", payload)[0]
            assert reboot_transaction_id == self.transaction_id
            assert self.transaction_status == config_manager.STATUS_OK
            self.reboot_transaction_ids.append(reboot_transaction_id)
        elif request == config_manager.OP_BOOTSEL_REBOOT:
            assert payload == b""
            self.bootsel_reboot_requested = True
        elif request == config_manager.OP_PAIRING_REFRESH:
            self.pairing_generation += 1
        elif request == config_manager.OP_PAIRING_CLEAR:
            self.records = []
            self.pairing_generation += 1
        elif request == config_manager.OP_PROFILE_SELECT:
            assert len(payload) == 15
            self.selected_profile = (payload[:14], payload[14])
            assert self.selected_profile in self.profiles
        elif request == config_manager.OP_PROFILE_BEGIN:
            assert len(payload) == 28
            self.profile_transaction_id = struct.unpack_from("<I", payload)[0]
            self.profile_transaction_identity = payload[4:18]
            (
                self.profile_transaction_index,
                reserved,
                schema,
                self.profile_transaction_expected_size,
                self.profile_transaction_expected_crc,
            ) = struct.unpack_from("<BBHHI", payload, 18)
            assert reserved == 0
            assert schema == config_manager.PROFILE_SCHEMA_VERSION
            assert (
                self.profile_transaction_expected_size
                == config_manager.PROFILE_SIZE
            )
            self.profile_transaction_payload = bytearray()
            self.profile_transaction_status = config_manager.STATUS_PENDING
            self.profile_chunk_sizes = []
        elif request == config_manager.OP_PROFILE_CHUNK:
            transaction_id, offset, chunk_size = struct.unpack_from(
                "<IHH", payload
            )
            assert transaction_id == self.profile_transaction_id
            assert offset == len(self.profile_transaction_payload)
            chunk = payload[8 : 8 + chunk_size]
            assert len(chunk) == chunk_size
            self.profile_transaction_payload.extend(chunk)
            self.profile_chunk_sizes.append(chunk_size)
        elif request == config_manager.OP_PROFILE_COMMIT:
            assert (
                struct.unpack("<I", payload)[0]
                == self.profile_transaction_id
            )
            assert (
                len(self.profile_transaction_payload)
                == self.profile_transaction_expected_size
            )
            assert (
                zlib.crc32(self.profile_transaction_payload) & 0xFFFFFFFF
            ) == self.profile_transaction_expected_crc
            if self.fail_profile_commit_status is None:
                key = (
                    self.profile_transaction_identity,
                    self.profile_transaction_index,
                )
                self.profiles[key] = bytes(self.profile_transaction_payload)
                self.profile_generation += 1
                self.profile_transaction_status = config_manager.STATUS_OK
            else:
                self.profile_transaction_status = (
                    self.fail_profile_commit_status
                )
        elif request == config_manager.OP_PROFILE_RESET:
            self._queue_profile_mutation(request, payload)
            assert (
                self.profile_transaction_index
                == config_manager.PROFILE_NONE_BUTTON
                or 0
                <= self.profile_transaction_index
                < config_manager.PROFILE_CAPACITY
            )
        elif request == config_manager.OP_PROFILE_ACTIVATE:
            self._queue_profile_mutation(request, payload)
            assert (
                0
                <= self.profile_transaction_index
                < config_manager.PROFILE_CAPACITY
            )
        else:
            raise AssertionError(f"unexpected OUT request {request}")
        return len(encoded)


def custom_profile() -> config_manager.ControllerProfile:
    return config_manager.ControllerProfile(
        button_map=(
            1,
            0,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            15,
            config_manager.PROFILE_NONE_BUTTON,
        ),
        left_stick=config_manager.StickConfig(
            -123, 456, 1000, 30000, 384, True, False
        ),
        right_stick=config_manager.StickConfig(
            789, -321, 500, 31000, 192, False, True
        ),
        left_trigger=config_manager.TriggerConfig(100, 65000, 320, 32000),
        right_trigger=config_manager.TriggerConfig(200, 64000, 224, 33000),
        weak_rumble_scale=77,
        strong_rumble_scale=201,
        confirmation_policy=2,
        switching_chord=(1 << 6) | (1 << 7),
        macro_trigger=0,
        macro_cancel=1,
        macro_steps=(
            config_manager.MacroStep(
                0,
                config_manager.MACRO_OVERRIDE_MASK,
                config_manager.PROFILE_MAXIMUM_WAIT_MS,
                (1 << 0) | (1 << 12),
                -32768,
                32767,
                -1000,
                1000,
                12345,
                54321,
            ),
            config_manager.MacroStep.end(),
        ),
        turbo_modes=(0, 1, 2) + (0,) * 13,
    )


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
    assert before.requested_mode == config_manager.REQUESTED_MODE_AUTO
    status = config_manager.write_configuration(
        device,
        config_manager.AdapterConfiguration(
            90,
            before.generation,
            before.crc,
            config_manager.REQUESTED_MODE_XINPUT,
        ),
        1.0,
    )
    assert status.stored_generation == 4
    stored = config_manager.read_configuration(device)
    assert stored.pairing_window_seconds == 90
    assert stored.requested_mode == config_manager.REQUESTED_MODE_XINPUT
    assert device.configuration == struct.pack(
        "<HB5x", 90, config_manager.REQUESTED_MODE_XINPUT
    )
    reset = config_manager.reset_configuration(device, 1.0)
    assert reset.stored_generation == 5
    reset_configuration = config_manager.read_configuration(device)
    assert reset_configuration.pairing_window_seconds == 60
    assert (
        reset_configuration.requested_mode
        == config_manager.REQUESTED_MODE_AUTO
    )


def test_configuration_transaction_ids_stay_in_host_range(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    generated_values = iter((0xFFFFFFFF, 0x80000000, 0xFEDCBA98))
    requested_bits: list[int] = []

    def randbits(bits: int) -> int:
        requested_bits.append(bits)
        return next(generated_values)

    monkeypatch.setattr(config_manager.secrets, "randbits", randbits)
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)
    before = config_manager.read_configuration(device)
    config_manager.write_configuration(
        device,
        config_manager.AdapterConfiguration(
            90,
            before.generation,
            before.crc,
            config_manager.REQUESTED_MODE_AUTO,
        ),
        1.0,
    )
    config_manager.reset_configuration(device, 1.0)
    config_manager.set_mode(
        device, config_manager.REQUESTED_MODE_SWITCH, 1.0
    )

    transaction_ids = [
        struct.unpack_from("<I", payload)[0]
        for operation, payload, _ in device.out_requests
        if operation
        in (
            config_manager.OP_CONFIGURATION_BEGIN,
            config_manager.OP_CONFIGURATION_RESET,
            config_manager.OP_MODE_SET,
        )
    ]
    assert requested_bits == [31, 31, 31]
    assert transaction_ids == [0x7FFFFFFF, 1, 0x7EDCBA98]
    assert all(
        transaction_id & ~config_manager.HOST_TRANSACTION_ID_MASK == 0
        for transaction_id in transaction_ids
    )


def test_mode_envelopes_and_host_side_validation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        config_manager.secrets, "randbits", lambda _bits: 0x12345678
    )

    status = config_manager.set_mode(
        device, config_manager.REQUESTED_MODE_XINPUT, 1.0
    )

    mode_payload = struct.pack(
        "<IB", 0x12345678, config_manager.REQUESTED_MODE_XINPUT
    )
    operation, payload, encoded = device.out_requests[0]
    assert operation == config_manager.OP_MODE_SET
    assert payload == mode_payload
    assert encoded == config_manager.encode_request(
        config_manager.OP_MODE_SET, mode_payload
    )
    assert status.transaction_id == 0x12345678

    config_manager.request_reboot(device, status.transaction_id)
    operation, payload, encoded = device.out_requests[-1]
    reboot_payload = struct.pack("<I", 0x12345678)
    assert operation == config_manager.OP_REBOOT
    assert payload == reboot_payload
    assert encoded == config_manager.encode_request(
        config_manager.OP_REBOOT, reboot_payload
    )

    config_manager.request_bootsel_reboot(device)
    operation, payload, encoded = device.out_requests[-1]
    assert operation == config_manager.OP_BOOTSEL_REBOOT
    assert payload == b""
    assert encoded == config_manager.encode_request(
        config_manager.OP_BOOTSEL_REBOOT
    )
    assert device.bootsel_reboot_requested

    for transaction_id in (0, 0x80000000, True):
        with pytest.raises(config_manager.ConfigManagerError):
            config_manager.request_reboot(device, transaction_id)
    for mode in (0xFF, True):
        with pytest.raises(
            config_manager.ConfigManagerError, match="not available"
        ):
            config_manager.set_mode(device, mode, 1.0)

    for mode in (
        config_manager.REQUESTED_MODE_DINPUT,
        config_manager.REQUESTED_MODE_MAC,
    ):
        status = config_manager.set_mode(device, mode, 1.0)
        assert status.status == config_manager.STATUS_OK
        assert device.pending_requested_mode is None


@pytest.mark.parametrize(
    ("failure_status", "message"),
    (
        (7, "device busy"),
        (8, "storage failure"),
    ),
)
def test_mode_set_propagates_busy_and_commit_errors_without_reboot(
    monkeypatch: pytest.MonkeyPatch,
    failure_status: int,
    message: str,
) -> None:
    device = FakeDevice()
    device.fail_mode_status = failure_status
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    with pytest.raises(config_manager.ConfigManagerError, match=message):
        config_manager.configure_mode(
            device, config_manager.REQUESTED_MODE_SWITCH, 1.0
        )

    assert config_manager.OP_REBOOT not in device.requests


def test_mode_transaction_must_correlate_before_reboot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class WrongTransactionDevice(FakeDevice):
        def _transaction_payload(self) -> bytes:
            payload = bytearray(super()._transaction_payload())
            struct.pack_into("<I", payload, 0, self.transaction_id + 1)
            return bytes(payload)

    device = WrongTransactionDevice()
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    with pytest.raises(
        config_manager.ConfigManagerError,
        match="different transaction",
    ):
        config_manager.configure_mode(
            device, config_manager.REQUESTED_MODE_SWITCH, 1.0
        )

    assert config_manager.OP_REBOOT not in device.requests


@pytest.mark.parametrize(
    ("requested_mode", "active_mode"),
    (
        (
            config_manager.REQUESTED_MODE_AUTO,
            config_manager.ACTIVE_MODE_SWITCH_PROBE,
        ),
        (
            config_manager.REQUESTED_MODE_AUTO,
            config_manager.ACTIVE_MODE_XINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_SWITCH,
            config_manager.ACTIVE_MODE_SWITCH,
        ),
        (
            config_manager.REQUESTED_MODE_XINPUT,
            config_manager.ACTIVE_MODE_XINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.ACTIVE_MODE_DINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_MAC,
            config_manager.ACTIVE_MODE_MAC,
        ),
    ),
)
def test_mode_noop_accepts_only_mode_appropriate_active_state(
    requested_mode: int, active_mode: int
) -> None:
    device = FakeDevice()
    device.configuration = struct.pack("<HB5x", 60, requested_mode)
    device.active_mode = active_mode
    if active_mode in (
        config_manager.ACTIVE_MODE_DINPUT,
        config_manager.ACTIVE_MODE_MAC,
    ):
        device.capabilities = config_manager.CAPABILITY_INPUT
    elif active_mode == config_manager.ACTIVE_MODE_XINPUT:
        device.capabilities = (
            config_manager.CAPABILITY_INPUT |
            config_manager.CAPABILITY_RUMBLE
        )

    same_device, changed = config_manager.configure_mode(
        device, requested_mode, 1.0
    )

    assert same_device is device
    assert not changed
    assert device.out_requests == []


@pytest.mark.parametrize(
    ("requested_mode", "active_mode"),
    (
        (
            config_manager.REQUESTED_MODE_AUTO,
            config_manager.ACTIVE_MODE_SWITCH_PROBE,
        ),
        (
            config_manager.REQUESTED_MODE_AUTO,
            config_manager.ACTIVE_MODE_XINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_SWITCH,
            config_manager.ACTIVE_MODE_SWITCH,
        ),
        (
            config_manager.REQUESTED_MODE_XINPUT,
            config_manager.ACTIVE_MODE_XINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.ACTIVE_MODE_DINPUT,
        ),
        (
            config_manager.REQUESTED_MODE_MAC,
            config_manager.ACTIVE_MODE_MAC,
        ),
    ),
)
def test_mode_change_waits_for_disappearance_and_reenumeration(
    monkeypatch: pytest.MonkeyPatch,
    requested_mode: int,
    active_mode: int,
) -> None:
    previous = FakeDevice()
    if requested_mode == config_manager.REQUESTED_MODE_AUTO:
        previous.configuration = struct.pack(
            "<HB5x", 60, config_manager.REQUESTED_MODE_SWITCH
        )
        previous.active_mode = config_manager.ACTIVE_MODE_SWITCH
    reenumerated = FakeDevice()
    reenumerated.address = 8
    reenumerated.configuration = struct.pack("<HB5x", 60, requested_mode)
    reenumerated.active_mode = active_mode
    if active_mode in (
        config_manager.ACTIVE_MODE_DINPUT,
        config_manager.ACTIVE_MODE_MAC,
    ):
        reenumerated.capabilities = config_manager.CAPABILITY_INPUT
    elif active_mode == config_manager.ACTIVE_MODE_XINPUT:
        reenumerated.capabilities = (
            config_manager.CAPABILITY_INPUT |
            config_manager.CAPABILITY_RUMBLE
        )
    scans = iter(((previous,), (), (reenumerated,)))
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (reenumerated,)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    result, changed = config_manager.configure_mode(
        previous, requested_mode, 1.0
    )

    assert result is reenumerated
    assert changed
    assert previous.reboot_transaction_ids == [previous.transaction_id]
    assert [
        operation
        for operation, _, _ in previous.out_requests
        if operation in (config_manager.OP_MODE_SET, config_manager.OP_REBOOT)
    ] == [config_manager.OP_MODE_SET, config_manager.OP_REBOOT]


def test_mode_reenumeration_tracks_same_port_among_adapters_on_one_bus(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = FakeDevice()
    previous.port_numbers = (2, 1)
    other = FakeDevice()
    other.address = 8
    other.port_numbers = (2, 2)
    replacement = FakeDevice()
    replacement.address = 9
    replacement.port_numbers = previous.port_numbers
    replacement.configuration = struct.pack(
        "<HB5x", 60, config_manager.REQUESTED_MODE_SWITCH
    )
    replacement.active_mode = config_manager.ACTIVE_MODE_SWITCH
    scans = iter(
        (
            (previous, other),
            (other,),
            (other, replacement),
        )
    )
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (other, replacement)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    result, changed = config_manager.configure_mode(
        previous, config_manager.REQUESTED_MODE_SWITCH, 1.0
    )

    assert result is replacement
    assert changed
    assert replacement.address != previous.address
    assert replacement.port_numbers == previous.port_numbers
    assert other.requests == []


def test_mode_reboot_fails_when_missing_topology_is_ambiguous(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previous = FakeDevice()
    previous.port_numbers = None
    other = FakeDevice()
    other.address = 8
    other.port_numbers = None
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: (previous, other),
    )

    with pytest.raises(
        config_manager.ConfigManagerError,
        match="topology is unavailable.*multiple",
    ):
        config_manager.configure_mode(
            previous, config_manager.REQUESTED_MODE_SWITCH, 1.0
        )

    assert previous.out_requests == []
    assert other.out_requests == []


@pytest.mark.parametrize(
    ("requested_mode", "stored_mode", "active_mode", "message"),
    (
        (
            config_manager.REQUESTED_MODE_SWITCH,
            config_manager.REQUESTED_MODE_SWITCH,
            config_manager.ACTIVE_MODE_SWITCH_PROBE,
            "activated",
        ),
        (
            config_manager.REQUESTED_MODE_SWITCH,
            config_manager.REQUESTED_MODE_AUTO,
            config_manager.ACTIVE_MODE_SWITCH,
            "was not stored",
        ),
        (
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.ACTIVE_MODE_MAC,
            "activated",
        ),
        (
            config_manager.REQUESTED_MODE_MAC,
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.ACTIVE_MODE_MAC,
            "was not stored",
        ),
    ),
)
def test_mode_verifies_requested_and_active_state_after_reenumeration(
    monkeypatch: pytest.MonkeyPatch,
    requested_mode: int,
    stored_mode: int,
    active_mode: int,
    message: str,
) -> None:
    previous = FakeDevice()
    reenumerated = FakeDevice()
    reenumerated.address = 8
    reenumerated.configuration = struct.pack("<HB5x", 60, stored_mode)
    reenumerated.active_mode = active_mode
    scans = iter(((previous,), (), (reenumerated,)))
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (reenumerated,)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    with pytest.raises(config_manager.ConfigManagerError, match=message):
        config_manager.configure_mode(
            previous, requested_mode, 1.0
        )


def test_reenumeration_reports_missing_disappearance_and_return(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: (device,)
    )
    with pytest.raises(
        config_manager.ConfigManagerError, match="did not disappear"
    ):
        snapshot = config_manager._capture_reenumeration_snapshot(device)
        config_manager._wait_for_reenumeration(snapshot, 0)

    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: ())
    with pytest.raises(
        config_manager.ConfigManagerError, match="did not re-enumerate"
    ):
        snapshot = config_manager._capture_reenumeration_snapshot(device)
        config_manager._wait_for_reenumeration(snapshot, 0)


def test_requested_and_active_mode_response_validation() -> None:
    device = FakeDevice()
    device.configuration = struct.pack(
        "<HB5x", 60, config_manager.REQUESTED_MODE_DINPUT
    )
    assert (
        config_manager.read_configuration(device).requested_mode
        == config_manager.REQUESTED_MODE_DINPUT
    )
    assert (
        config_manager.read_info(device).active_mode
        == config_manager.ACTIVE_MODE_SWITCH_PROBE
    )
    assert config_manager.read_info(device).capability_names() == (
        "input",
        "rumble",
        "motion",
    )
    diagnostics = config_manager.read_runtime_diagnostics(device)
    assert diagnostics == config_manager.RuntimeDiagnostics(
        initialization_stage=6,
        rumble_timer_ticks=1200,
        configuration_timer_ticks=120,
        controller_reports=5000,
        host_rumble_requests=8,
        local_feedback_requests=2,
        rumble_dispatches=10,
        active_slots=2,
        rumble_capable_slots=2,
        feedback_pending_slots=1,
        rumble_pending_slots=1,
    )

    device.configuration = struct.pack("<HB5x", 60, 0xFF)
    with pytest.raises(
        config_manager.ConfigManagerError, match="invalid stored requested"
    ):
        config_manager.read_configuration(device)
    device.configuration = (
        struct.pack("<HB5x", 60, config_manager.REQUESTED_MODE_AUTO)[:-1]
        + b"\x01"
    )
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="unsupported configuration object",
    ):
        config_manager.read_configuration(device)
    device.active_mode = 0xFF
    with pytest.raises(
        config_manager.ConfigManagerError, match="unknown active USB mode"
    ):
        config_manager.read_info(device)
    device.active_mode = config_manager.ACTIVE_MODE_DINPUT
    device.capabilities = 0x80
    with pytest.raises(
        config_manager.ConfigManagerError, match="unknown device capability"
    ):
        config_manager.read_info(device)
    device.capabilities = config_manager.CAPABILITY_RUMBLE
    with pytest.raises(
        config_manager.ConfigManagerError, match="omit required input"
    ):
        config_manager.read_info(device)


def test_identity_and_profile_binary_json_round_trip() -> None:
    identity = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_BLE,
        3,
        bytes.fromhex("A1B2C3D4E5F6"),
        0x1234,
        0xABCD,
    )
    encoded_identity = identity.to_bytes()
    assert encoded_identity == bytes.fromhex(
        "01020300A1B2C3D4E5F63412CDAB"
    )
    assert config_manager.ControllerIdentity.from_bytes(encoded_identity) == identity
    assert (
        config_manager.ControllerIdentity.global_fallback().to_bytes()
        == bytes(config_manager.CONTROLLER_IDENTITY_SIZE)
    )
    malformed_identity = bytearray(encoded_identity)
    malformed_identity[3] = 1
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerIdentity.from_bytes(malformed_identity)

    default_profile = config_manager.ControllerProfile.default()
    assert default_profile.left_trigger.digital_threshold == 22934
    assert default_profile.right_trigger.digital_threshold == 22934
    default_wire = default_profile.to_bytes()
    assert struct.unpack_from("<H", default_wire, 58)[0] == 22934
    assert struct.unpack_from("<H", default_wire, 68)[0] == 22934

    profile = custom_profile()
    encoded = profile.to_bytes()
    assert len(encoded) == config_manager.PROFILE_SIZE
    assert struct.unpack_from("<HH", encoded) == (
        config_manager.PROFILE_SCHEMA_VERSION,
        config_manager.PROFILE_SIZE,
    )
    assert encoded[75] == encoded[81] == 0
    assert encoded[98:100] == b"\x00\x00"
    assert encoded[252:] == bytes(4)
    assert config_manager.ControllerProfile.from_bytes(encoded) == profile

    serialized = profile.to_json()
    assert serialized.startswith('{\n  "schema_version": 2,\n  "size": 256,')
    decoded = config_manager.ControllerProfile.from_json(serialized)
    assert decoded == profile
    assert decoded.to_json() == serialized

    legacy_default_wire = bytearray(default_wire)
    struct.pack_into(
        "<H",
        legacy_default_wire,
        0,
        config_manager.PROFILE_LEGACY_SCHEMA_VERSION,
    )
    struct.pack_into(
        "<HHHH",
        legacy_default_wire,
        52,
        30000,
        40000,
        256,
        config_manager.PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD,
    )
    struct.pack_into(
        "<HHHH",
        legacy_default_wire,
        62,
        30000,
        40000,
        256,
        config_manager.PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD,
    )
    migrated_default = config_manager.ControllerProfile.from_bytes(
        legacy_default_wire
    )
    assert (
        migrated_default.left_trigger.digital_threshold
        == config_manager.PROFILE_DEFAULT_DIGITAL_THRESHOLD
    )
    assert (
        migrated_default.right_trigger.digital_threshold
        == config_manager.PROFILE_DEFAULT_DIGITAL_THRESHOLD
    )
    assert migrated_default.left_trigger.lower_deadzone == 30000
    assert migrated_default.left_trigger.upper_saturation == 40000
    assert migrated_default.right_trigger.lower_deadzone == 30000
    assert migrated_default.right_trigger.upper_saturation == 40000
    assert migrated_default.to_bytes()[0] == config_manager.PROFILE_SCHEMA_VERSION

    current_old_value_wire = bytearray(default_wire)
    struct.pack_into(
        "<H",
        current_old_value_wire,
        58,
        config_manager.PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD,
    )
    assert (
        config_manager.ControllerProfile.from_bytes(
            current_old_value_wire
        ).left_trigger.digital_threshold
        == config_manager.PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD
    )

    legacy_custom_wire = bytearray(encoded)
    struct.pack_into(
        "<H",
        legacy_custom_wire,
        0,
        config_manager.PROFILE_LEGACY_SCHEMA_VERSION,
    )
    assert (
        config_manager.ControllerProfile.from_bytes(legacy_custom_wire)
        == profile
    )

    legacy_json_object = default_profile.to_json_object()
    legacy_json_object["schema_version"] = (
        config_manager.PROFILE_LEGACY_SCHEMA_VERSION
    )
    legacy_json_object["triggers"]["left"]["digital_threshold"] = (
        config_manager.PROFILE_LEGACY_DEFAULT_DIGITAL_THRESHOLD
    )
    legacy_json_object["triggers"]["right"]["digital_threshold"] = 33000
    legacy_json_object["triggers"]["left"]["lower_deadzone"] = 30000
    legacy_json_object["triggers"]["left"]["upper_saturation"] = 40000
    migrated_json = config_manager.ControllerProfile.from_json_object(
        legacy_json_object
    )
    assert (
        migrated_json.left_trigger.digital_threshold
        == config_manager.PROFILE_DEFAULT_DIGITAL_THRESHOLD
    )
    assert migrated_json.right_trigger.digital_threshold == 33000
    assert migrated_json.left_trigger.lower_deadzone == 30000
    assert migrated_json.left_trigger.upper_saturation == 40000


def test_trigger_threshold_uses_transformed_output_domain() -> None:
    for threshold in (0, 0xFFFF):
        trigger = config_manager.TriggerConfig(30000, 40000, 256, threshold)
        assert config_manager.TriggerConfig.from_bytes(trigger.to_bytes()) == trigger
        assert (
            config_manager.TriggerConfig.from_json_object(
                trigger.to_json_object(), "trigger"
            )
            == trigger
        )

        current_wire = bytearray(
            config_manager.ControllerProfile.default().to_bytes()
        )
        struct.pack_into(
            "<HHHH", current_wire, 52, 30000, 40000, 256, threshold
        )
        current_profile = config_manager.ControllerProfile.from_bytes(
            current_wire
        )
        assert current_profile.left_trigger.digital_threshold == threshold
        assert current_profile.to_bytes() == current_wire

    for threshold in (-1, 0x10000):
        with pytest.raises(
            config_manager.ConfigManagerError,
            match="trigger digital_threshold",
        ):
            config_manager.TriggerConfig(30000, 40000, 256, threshold)

    for lower_deadzone, upper_saturation in (
        (40000, 40000),
        (40001, 40000),
    ):
        with pytest.raises(
            config_manager.ConfigManagerError,
            match="lower_deadzone must be below upper_saturation",
        ):
            config_manager.TriggerConfig(
                lower_deadzone, upper_saturation, 256, 0
            )


def test_default_timeout_covers_batched_profile_commit() -> None:
    args = config_manager.build_parser().parse_args(["profiles", "list"])
    assert (
        args.timeout
        == config_manager.DEFAULT_OPERATION_TIMEOUT_SECONDS
        == 15.0
    )


def test_profile_list_select_read_and_chunked_commit() -> None:
    device = FakeDevice()
    entries = config_manager.list_profiles(device)
    assert entries == (
        config_manager.ProfileListEntry(device.global_identity, 0),
        config_manager.ProfileListEntry(device.stable_identity, 1),
    )
    assert (
        config_manager.read_profile(device, device.stable_identity, 1)
        == config_manager.ControllerProfile.default()
    )
    assert device.requests[-2:] == [
        config_manager.OP_PROFILE_SELECT,
        config_manager.OP_PROFILE_READ,
    ]

    profile = custom_profile()
    status = config_manager.write_profile(
        device, device.stable_identity, 2, profile, 1.0
    )
    assert status.status == config_manager.STATUS_OK
    assert status.stored_generation == 8
    assert device.profile_chunk_sizes == [40, 40, 40, 40, 40, 40, 16]
    assert config_manager.OP_PROFILE_TRANSACTION_STATUS in device.requests
    assert (
        config_manager.read_profile(device, device.stable_identity, 2)
        == profile
    )


def test_profile_reset_and_activate_wait_for_correlated_transactions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    generated_ids = iter((0, 0xFFFFFFFF))
    monkeypatch.setattr(
        config_manager.secrets, "randbits", lambda _bits: next(generated_ids)
    )
    identity = device.stable_identity
    profile_key = (identity.to_bytes(), 2)
    device.profiles[profile_key] = custom_profile().to_bytes()

    reset = config_manager.reset_profile(device, identity, 2, 1.0)


    assert reset.transaction_id == device.profile_transaction_id == 1
    assert reset.status == config_manager.STATUS_OK
    assert reset.stored_generation == 8
    assert device.profile_status_responses == [
        (1, config_manager.STATUS_PENDING),
        (1, config_manager.STATUS_OK),
    ]
    assert (
        device.profiles[profile_key]
        == config_manager.ControllerProfile.default().to_bytes()
    )

    device.profile_status_responses.clear()
    activated = config_manager.activate_profile(device, identity, 3, 1.0)

    assert (
        activated.transaction_id
        == device.profile_transaction_id
        == config_manager.HOST_TRANSACTION_ID_MASK
    )
    assert activated.status == config_manager.STATUS_OK
    assert activated.stored_generation == 9
    assert device.profile_status_responses == [
        (config_manager.HOST_TRANSACTION_ID_MASK, config_manager.STATUS_PENDING),
        (config_manager.HOST_TRANSACTION_ID_MASK, config_manager.STATUS_OK),
    ]
    assert device.active_profiles[identity.to_bytes()] == 3


def test_profile_cli_surfaces_late_storage_failure(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    device.fail_profile_commit_status = 8
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    previous_active = device.active_profiles[device.stable_identity.to_bytes()]

    assert (
        config_manager.main(
            ["profiles", "activate", "4", "--identity", "1"]
        )
        == 1
    )

    output = capsys.readouterr()
    assert output.out == ""
    assert "storage failure" in output.err
    assert (
        device.active_profiles[device.stable_identity.to_bytes()]
        == previous_active
    )
    assert device.profile_status_responses == [
        (device.profile_transaction_id, config_manager.STATUS_PENDING),
        (device.profile_transaction_id, 8),
    ]


def test_profile_cli_json_round_trip_activate_and_reset(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    export_path = tmp_path / "profile.json"

    assert config_manager.main(["profiles", "list"]) == 0
    output = capsys.readouterr().out
    assert "0: global fallback (active profile 1)" in output
    assert "1: Classic 10:20:30:40:50:60" in output

    device.profiles[(device.stable_identity.to_bytes(), 1)] = (
        custom_profile().to_bytes()
    )
    assert (
        config_manager.main(
            [
                "profiles",
                "export",
                "2",
                str(export_path),
                "--identity",
                "1",
            ]
        )
        == 0
    )
    _ = capsys.readouterr()
    exported = config_manager.ControllerProfile.from_json(
        export_path.read_text(encoding="utf-8")
    )
    assert exported == custom_profile()

    device.profiles[(device.stable_identity.to_bytes(), 1)] = (
        config_manager.ControllerProfile.default().to_bytes()
    )
    assert (
        config_manager.main(
            [
                "profiles",
                "import",
                "2",
                str(export_path),
                "--identity",
                "1",
            ]
        )
        == 0
    )
    assert (
        device.profiles[(device.stable_identity.to_bytes(), 1)]
        == custom_profile().to_bytes()
    )
    _ = capsys.readouterr()

    assert (
        config_manager.main(
            ["profiles", "activate", "4", "--identity", "1"]
        )
        == 0
    )
    assert device.active_profiles[device.stable_identity.to_bytes()] == 3
    _ = capsys.readouterr()

    before_reset_requests = len(device.requests)
    assert (
        config_manager.main(
            ["profiles", "reset", "2", "--identity", "1"]
        )
        == 2
    )
    assert "requires --yes" in capsys.readouterr().err
    assert len(device.requests) == before_reset_requests

    assert (
        config_manager.main(
            [
                "profiles",
                "reset",
                "2",
                "--identity",
                "1",
                "--yes",
            ]
        )
        == 0
    )
    assert (
        device.profiles[(device.stable_identity.to_bytes(), 1)]
        == config_manager.ControllerProfile.default().to_bytes()
    )
    _ = capsys.readouterr()
    assert (
        config_manager.main(
            [
                "profiles",
                "reset",
                "all",
                "--identity",
                "1",
                "--yes",
            ]
        )
        == 0
    )


def test_malformed_profiles_are_rejected_before_usb(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    malformed_binary = bytearray(config_manager.ControllerProfile.default().to_bytes())
    malformed_binary[75] = 1
    with pytest.raises(
        config_manager.ConfigManagerError, match="reserved fields"
    ):
        config_manager.ControllerProfile.from_bytes(malformed_binary)

    profile_object = config_manager.ControllerProfile.default().to_json_object()
    del profile_object["turbo"]
    missing_path = tmp_path / "missing.json"
    missing_path.write_text(json.dumps(profile_object), encoding="utf-8")

    profile_object = config_manager.ControllerProfile.default().to_json_object()
    profile_object["reserved"] = 0
    unknown_path = tmp_path / "unknown.json"
    unknown_path.write_text(json.dumps(profile_object), encoding="utf-8")

    profile_object = config_manager.ControllerProfile.default().to_json_object()
    profile_object["rumble"]["confirmation_policy"] = "invalid"
    enum_path = tmp_path / "enum.json"
    enum_path.write_text(json.dumps(profile_object), encoding="utf-8")

    profile_object = config_manager.ControllerProfile.default().to_json_object()
    profile_object["sticks"]["left"]["outer_saturation"] = 0
    range_path = tmp_path / "range.json"
    range_path.write_text(json.dumps(profile_object), encoding="utf-8")

    usb_lookups = 0

    def candidates() -> list[FakeDevice]:
        nonlocal usb_lookups
        usb_lookups += 1
        return [FakeDevice()]

    monkeypatch.setattr(config_manager, "_candidate_devices", candidates)
    for path in (missing_path, unknown_path, enum_path, range_path):
        assert (
            config_manager.main(["profiles", "import", "1", str(path)]) == 1
        )
        assert "error:" in capsys.readouterr().err
    assert usb_lookups == 0


def test_profile_crc_status_failures_and_identity_bounds(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    device = FakeDevice()
    device.bad_profile_response_crc = True
    with pytest.raises(
        config_manager.ConfigManagerError, match="response CRC mismatch"
    ):
        config_manager.read_profile(device, device.global_identity, 0)

    failing_device = FakeDevice()
    failing_device.fail_profile_commit_status = 6
    with pytest.raises(config_manager.ConfigManagerError, match="CRC mismatch"):
        config_manager.write_profile(
            failing_device,
            failing_device.global_identity,
            0,
            custom_profile(),
            1.0,
        )

    bounded_device = FakeDevice()
    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: [bounded_device]
    )
    assert (
        config_manager.main(
            [
                "profiles",
                "export",
                "1",
                str(tmp_path / "unused.json"),
                "--identity",
                "2",
            ]
        )
        == 1
    )
    assert "identity index 2 is out of range" in capsys.readouterr().err
    assert config_manager.OP_PROFILE_SELECT not in bounded_device.requests


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
    assert "Requested USB mode: auto" in output
    assert "Active USB mode: Switch probe" in output
    assert "Mode capabilities: input, rumble, motion" in output

    device.configuration = struct.pack(
        "<HB5x", 60, config_manager.REQUESTED_MODE_DINPUT
    )
    device.active_mode = config_manager.ACTIVE_MODE_DINPUT
    device.capabilities = config_manager.CAPABILITY_INPUT
    assert config_manager.main(["status"]) == 0
    generic_output = capsys.readouterr().out
    assert "Requested USB mode: dinput" in generic_output
    assert "Active USB mode: DInput" in generic_output
    assert "Mode capabilities: input only\n" in generic_output
    assert "rumble" not in generic_output.lower()

    assert config_manager.main(["pairings", "list"]) == 0
    output = capsys.readouterr().out
    assert "Classic 01:02:03:04:05:06" in output
    assert "BLE (public identity) A1:A2:A3:A4:A5:A6" in output

    assert config_manager.main(["pairings", "clear"]) == 2
    assert "requires --yes" in capsys.readouterr().err
    assert config_manager.main(["pairings", "clear", "--yes"]) == 0
    assert capsys.readouterr().out == "Cleared 2 stored pairing(s).\n"


def test_config_cli_preserves_requested_mode(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    device.configuration = struct.pack(
        "<HB5x", 60, config_manager.REQUESTED_MODE_XINPUT
    )
    device.active_mode = config_manager.ACTIVE_MODE_XINPUT
    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: (device,)
    )

    assert (
        config_manager.main(
            ["config", "set", "--pairing-window-seconds", "90"]
        )
        == 0
    )
    assert struct.unpack("<HB5x", device.configuration) == (
        90,
        config_manager.REQUESTED_MODE_XINPUT,
    )
    assert "Stored configuration generation" in capsys.readouterr().out
    assert config_manager.main(["config", "show"]) == 0
    output = capsys.readouterr().out
    assert "pairing_window_seconds=90" in output
    assert "requested_mode=xinput" in output


@pytest.mark.parametrize(
    ("mode_name", "requested_mode", "active_mode", "capabilities"),
    (
        (
            "xinput",
            config_manager.REQUESTED_MODE_XINPUT,
            config_manager.ACTIVE_MODE_XINPUT,
            config_manager.CAPABILITY_INPUT |
            config_manager.CAPABILITY_RUMBLE,
        ),
        (
            "dinput",
            config_manager.REQUESTED_MODE_DINPUT,
            config_manager.ACTIVE_MODE_DINPUT,
            config_manager.CAPABILITY_INPUT,
        ),
        (
            "mac",
            config_manager.REQUESTED_MODE_MAC,
            config_manager.ACTIVE_MODE_MAC,
            config_manager.CAPABILITY_INPUT,
        ),
    ),
)
def test_mode_cli_changes_then_noops(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    mode_name: str,
    requested_mode: int,
    active_mode: int,
    capabilities: int,
) -> None:
    previous = FakeDevice()
    reenumerated = FakeDevice()
    reenumerated.address = 8
    reenumerated.configuration = struct.pack(
        "<HB5x", 60, requested_mode
    )
    reenumerated.active_mode = active_mode
    reenumerated.capabilities = capabilities
    scans = iter(
        ((previous,), (previous,), (), (reenumerated,))
    )
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (reenumerated,)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    assert config_manager.main(["mode", mode_name]) == 0
    assert capsys.readouterr().out == f"USB mode changed to {mode_name}.\n"
    assert previous.reboot_transaction_ids == [previous.transaction_id]

    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: (reenumerated,)
    )
    assert config_manager.main(["mode", mode_name]) == 0
    assert capsys.readouterr().out == f"USB mode is already {mode_name}.\n"
    assert reenumerated.out_requests == []


def test_bootsel_reboot_cli(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(
        config_manager, "_candidate_devices", lambda: (device,)
    )

    assert config_manager.main(["reboot", "bootsel"]) == 0
    assert capsys.readouterr().out == "Rebooting into USB BOOTSEL mode.\n"
    assert device.bootsel_reboot_requested


def test_mode_parser_accepts_all_implemented_modes() -> None:
    for mode in config_manager.REQUESTED_MODE_NAMES:
        args = config_manager.build_parser().parse_args(["mode", mode])
        assert args.mode == mode


def test_candidate_discovery_checks_all_usb_identities(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lookups: list[tuple[int, int]] = []

    assert config_manager.USB_IDENTITIES == (
        (0x057E, 0x2009),
        (0xCAFE, 0x4010),
        (0xCAFE, 0x4020),
        (0xCAFE, 0x4021),
    )

    def find(**arguments: object) -> tuple[object, ...]:
        lookups.append(
            (
                int(arguments["idVendor"]),
                int(arguments["idProduct"]),
            )
        )
        return ()

    monkeypatch.setattr(config_manager.usb.core, "find", find)

    assert list(config_manager._candidate_devices()) == []
    assert tuple(lookups) == config_manager.USB_IDENTITIES


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
