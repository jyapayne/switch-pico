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
                    request, bytes([0, 2, 0, 2, 0, 0, 0, 2])
                )
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
    generated_ids = iter((0, 0xA5A55A5A))
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
        == 0xA5A55A5A
    )
    assert activated.status == config_manager.STATUS_OK
    assert activated.stored_generation == 9
    assert device.profile_status_responses == [
        (0xA5A55A5A, config_manager.STATUS_PENDING),
        (0xA5A55A5A, config_manager.STATUS_OK),
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
