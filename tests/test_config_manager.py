from __future__ import annotations

import json
import struct
import zlib
from dataclasses import replace
from pathlib import Path

import pytest

from switch_pico_bridge import config_manager


def make_response(
    operation: int,
    payload: bytes = b"",
    *,
    status: int = config_manager.STATUS_OK,
    flags: int = 0,
    schema: int = 0,
    generation: int = 0,
) -> bytes:
    return (
        struct.pack(
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
        )
        + payload
    )


class FakeDevice:
    bus = 1
    address = 7
    port_numbers = (1,)
    firmware_version = (0, 2, 0)

    def __init__(self) -> None:
        self.configuration = struct.pack(
            "<HB5x", 60, config_manager.REQUESTED_MODE_AUTO
        )
        self.configuration_schema = 2
        self.configuration_generation = 3
        self.native_rumble_diagnostics = b"".join(
            struct.pack("<4B19I", slot, 0, 0, 0, *([0] * 19)) for slot in range(4)
        )
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
        self.fail_configuration_commit_status: int | None = None
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
        self.profile_aliases = {
            identity.to_bytes(): "" for identity in self.profile_identities
        }
        self.profile_names = {
            (identity.to_bytes(), index): ""
            for identity in self.profile_identities
            for index in range(config_manager.PROFILE_CAPACITY)
        }
        self.identified_identities: list[bytes] = []
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
        self.playtest_battery = 251
        self.playtest_capabilities = 0x0F
        self.profile_status_responses: list[tuple[int, int]] = []
        self.playtest_connected = True
        self.playtest_slot = 1
        self.playtest_connection_generation = 17
        self.playtest_state_generation = 93
        self.playtest_button_mask = 0x9001
        self.playtest_extra_buttons = 0
        self.playtest_layout = 0
        self.playtest_sticks = (-1234, 2345, -30000, 30000)
        self.playtest_triggers = (123, 65000)
        self.playtest_motion = (1, -2, 3, -4, 5, -6)

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
            identity_bytes = identity.to_bytes()
            alias = self.profile_aliases.get(identity_bytes, "").encode("utf-8")
            payload.extend(identity_bytes)
            payload.extend((self.active_profiles[identity_bytes], 0, len(alias)))
            payload.extend(alias)
            payload.extend(
                bytes(config_manager.PROFILE_METADATA_MAX_BYTES - len(alias))
            )
        return bytes(payload)

    def _profile_metadata_payload(self) -> bytes:
        identity, _ = self.selected_profile
        values = [
            self.profile_aliases.get(identity, ""),
            *(
                self.profile_names.get((identity, index), "")
                for index in range(config_manager.PROFILE_CAPACITY)
            ),
        ]
        payload = bytearray()
        for value in values:
            encoded = value.encode("utf-8")
            payload.extend((len(encoded),))
            payload.extend(encoded)
            payload.extend(
                bytes(config_manager.PROFILE_METADATA_MAX_BYTES - len(encoded))
            )
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

    def _profile_playtest_payload(self) -> tuple[bytes, int]:
        payload = bytearray(config_manager.PROFILE_PLAYTEST_SIZE)
        payload[1] = 0xFF
        if not self.playtest_connected:
            return bytes(payload), 0
        flags = 0x03 if self.playtest_motion is not None else 0x01
        payload[0] = flags
        payload[1] = self.playtest_slot
        struct.pack_into(
            "<HII",
            payload,
            2,
            self.playtest_button_mask,
            self.playtest_connection_generation,
            self.playtest_state_generation,
        )
        payload[12:26] = self.stable_identity.to_bytes()
        struct.pack_into(
            "<hhhhHH", payload, 26, *self.playtest_sticks, *self.playtest_triggers
        )
        payload[38] = 1 if self.playtest_motion is not None else 0
        payload[39] = self.playtest_battery
        payload[40] = self.playtest_capabilities
        payload[54] = self.playtest_extra_buttons
        payload[55] = self.playtest_layout
        if self.playtest_motion is not None:
            struct.pack_into("<hhhhhh", payload, 42, *self.playtest_motion)
        return bytes(payload), flags

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
                            *self.firmware_version,
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
                    struct.pack("<7I4B", 6, 1200, 120, 5000, 8, 2, 10, 2, 2, 1, 1)
                    + getattr(self, "runtime_diagnostics_tail", b""),
                )
            if request == config_manager.OP_NATIVE_SWITCH_RUMBLE:
                return make_response(
                    request,
                    self.native_rumble_diagnostics,
                    schema=config_manager.NATIVE_SWITCH_RUMBLE_SCHEMA_VERSION,
                )
            if request == config_manager.OP_CONFIGURATION_READ:
                return make_response(
                    request,
                    self.configuration,
                    schema=self.configuration_schema,
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
                            self.configuration = (
                                self.configuration[:2]
                                + bytes((self.pending_requested_mode,))
                                + self.configuration[3:]
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
                        schema=struct.unpack_from(
                            "<H", self.profiles[self.selected_profile]
                        )[0],
                        generation=self.profile_generation,
                    )
                )
                if self.bad_profile_response_crc:
                    response[-1] ^= 1
                return bytes(response)
            if request == config_manager.OP_PROFILE_PLAYTEST:
                payload, flags = self._profile_playtest_payload()
                return make_response(
                    request,
                    payload,
                    flags=flags,
                    schema=config_manager.PROFILE_PLAYTEST_SCHEMA_VERSION,
                    generation=self.playtest_state_generation,
                )
            if request == config_manager.OP_PROFILE_METADATA_READ:
                return make_response(
                    request,
                    self._profile_metadata_payload(),
                    schema=config_manager.PROFILE_METADATA_SCHEMA_VERSION,
                    generation=self.profile_generation,
                )
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
                self.configuration_schema,
                self.transaction_expected_size,
                self.transaction_expected_crc,
            ) = struct.unpack("<IHHI", payload)
            assert 0 < self.transaction_id <= (config_manager.HOST_TRANSACTION_ID_MASK)
            self.transaction_payload = bytearray()
            self.transaction_status = config_manager.STATUS_PENDING
        elif request == config_manager.OP_CONFIGURATION_CHUNK:
            transaction_id, offset, chunk_size = struct.unpack_from("<IHH", payload)
            assert transaction_id == self.transaction_id
            assert offset == len(self.transaction_payload)
            self.transaction_payload.extend(payload[8 : 8 + chunk_size])
        elif request == config_manager.OP_CONFIGURATION_COMMIT:
            assert struct.unpack("<I", payload)[0] == self.transaction_id
            assert len(self.transaction_payload) == self.transaction_expected_size
            assert (
                zlib.crc32(self.transaction_payload) & 0xFFFFFFFF
            ) == self.transaction_expected_crc
            if self.fail_configuration_commit_status is not None:
                self.transaction_status = self.fail_configuration_commit_status
            else:
                self.configuration = bytes(self.transaction_payload)
                self.configuration_generation += 1
                self.transaction_status = config_manager.STATUS_OK
        elif request == config_manager.OP_CONFIGURATION_RESET:
            self.transaction_id = struct.unpack("<I", payload)[0]
            assert 0 < self.transaction_id <= (config_manager.HOST_TRANSACTION_ID_MASK)
            self.configuration = struct.pack(
                "<HB5x", 60, config_manager.REQUESTED_MODE_AUTO
            )
            if self.configuration_schema >= 3:
                self.configuration += bytes(config_manager.CONFIGURATION_SIZE - 8)
            self.configuration_generation += 1
            self.transaction_payload = bytearray(self.configuration)
            self.transaction_expected_size = len(self.configuration)
            self.transaction_expected_crc = zlib.crc32(self.configuration) & 0xFFFFFFFF
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
            assert self.profile_transaction_expected_size == config_manager.PROFILE_SIZE
            self.profile_transaction_payload = bytearray()
            self.profile_transaction_status = config_manager.STATUS_PENDING
            self.profile_chunk_sizes = []
        elif request == config_manager.OP_PROFILE_CHUNK:
            transaction_id, offset, chunk_size = struct.unpack_from("<IHH", payload)
            assert transaction_id == self.profile_transaction_id
            assert offset == len(self.profile_transaction_payload)
            chunk = payload[8 : 8 + chunk_size]
            assert len(chunk) == chunk_size
            self.profile_transaction_payload.extend(chunk)
            self.profile_chunk_sizes.append(chunk_size)
        elif request == config_manager.OP_PROFILE_COMMIT:
            assert struct.unpack("<I", payload)[0] == self.profile_transaction_id
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
                self.profile_transaction_status = self.fail_profile_commit_status
        elif request == config_manager.OP_PROFILE_RESET:
            self._queue_profile_mutation(request, payload)
            assert (
                self.profile_transaction_index == config_manager.PROFILE_NONE_BUTTON
                or 0 <= self.profile_transaction_index < config_manager.PROFILE_CAPACITY
            )
        elif request == config_manager.OP_PROFILE_ACTIVATE:
            self._queue_profile_mutation(request, payload)
            assert 0 <= self.profile_transaction_index < config_manager.PROFILE_CAPACITY
        elif request == config_manager.OP_PROFILE_METADATA_SET:
            self.profile_transaction_id = struct.unpack_from("<I", payload)[0]
            identity = payload[4:18]
            profile_index = payload[18]
            value_size = payload[19]
            value = payload[20 : 20 + value_size].decode("utf-8")
            assert len(payload) == 20 + value_size
            if profile_index == config_manager.PROFILE_NONE_BUTTON:
                self.profile_aliases[identity] = value
            else:
                self.profile_names[(identity, profile_index)] = value
            self.profile_transaction_identity = identity
            self.profile_transaction_index = profile_index
            self.profile_transaction_payload = bytearray()
            self.profile_transaction_expected_size = 0
            self.profile_transaction_expected_crc = 0
            self.profile_generation += 1
            self.profile_transaction_status = config_manager.STATUS_OK
        elif request == config_manager.OP_PROFILE_IDENTIFY:
            self.identified_identities.append(payload)
        else:
            raise AssertionError(f"unexpected OUT request {request}")
        return len(encoded)


def custom_profile() -> config_manager.ControllerProfile:
    return config_manager.ControllerProfile(
        button_map=(
            config_manager.LOGICAL_CONTROLS.index("left_trigger"),
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
        left_stick=config_manager.StickConfig(-123, 456, 1000, 30000, 384, True, False),
        right_stick=config_manager.StickConfig(789, -321, 500, 31000, 192, False, True),
        left_trigger=config_manager.TriggerConfig(
            100,
            65000,
            320,
            32000,
            config_manager.LOGICAL_CONTROLS.index("right_trigger"),
        ),
        right_trigger=config_manager.TriggerConfig(
            200,
            64000,
            224,
            33000,
            config_manager.LOGICAL_BUTTONS.index("north"),
        ),
        weak_rumble_scale=77,
        strong_rumble_scale=201,
        confirmation_policy=2,
        switching_chord=(1 << 6) | (1 << 16),
        motion_toggle_chord=(1 << 5) | (1 << 17),
        macros=(
            config_manager.ControllerMacro(
                (1 << 0) | (1 << 16) | (1 << 17),
                config_manager.LOGICAL_CONTROLS.index("right_trigger"),
                (
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
                ),
            ),
            config_manager.ControllerMacro(
                (1 << 1) | (1 << 2),
                config_manager.PROFILE_NONE_BUTTON,
                (config_manager.MacroStep(0, 1, 25, 1 << 3, 0, 0, 0, 0, 0, 0),),
            ),
            config_manager.ControllerMacro.empty(),
            config_manager.ControllerMacro.empty(),
        ),
        turbo_modes=(0, 1, 2) + (0,) * 13,
    )


def legacy_profile_wire(
    schema_version: int,
    macro_trigger: int = config_manager.PROFILE_NONE_BUTTON,
    macro_cancel: int = config_manager.PROFILE_NONE_BUTTON,
) -> bytearray:
    profile = config_manager.ControllerProfile.default()
    payload = bytearray(config_manager.PROFILE_LEGACY_SIZE)
    struct.pack_into(
        "<HH", payload, 0, schema_version, config_manager.PROFILE_LEGACY_SIZE
    )
    payload[4:20] = bytes(range(len(config_manager.LOGICAL_BUTTONS)))
    payload[20:36] = profile.left_stick.to_bytes()
    payload[36:52] = profile.right_stick.to_bytes()
    payload[52:62] = profile.left_trigger.to_bytes()
    payload[62:72] = profile.right_trigger.to_bytes()
    if schema_version < config_manager.PROFILE_CONTROL_MAPPING_SCHEMA_VERSION:
        payload[60:62] = b"\x00\x00"
        payload[70:72] = b"\x00\x00"
    payload[72:75] = bytes((0xFF, 0xFF, 3))
    payload[78] = macro_trigger
    payload[79] = macro_cancel
    payload[80] = 1
    if schema_version >= config_manager.PROFILE_CONTROL_MAPPING_SCHEMA_VERSION:
        payload[81] = macro_cancel
        payload[79] = 0
    for index in range(config_manager.PROFILE_LEGACY_MACRO_STEP_CAPACITY):
        payload[100 + index * config_manager.PROFILE_MACRO_STEP_SIZE] = 1
    return payload


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


def native_rumble_identity(
    address: bytes = bytes.fromhex("102030405060"), product_id: int = 0x2009
) -> config_manager.ControllerIdentity:
    return config_manager.ControllerIdentity(
        True, config_manager.TRANSPORT_CLASSIC, 0, address, 0x057E, product_id
    )


def native_rumble_configuration(
    identities: tuple[config_manager.ControllerIdentity, ...] = (),
    joycon_mode: int = config_manager.JOYCON_MODE_PAIRED,
) -> bytes:
    return (
        struct.pack(
            "<HBBB3x",
            90,
            config_manager.REQUESTED_MODE_XINPUT,
            len(identities),
            joycon_mode,
        )
        + b"".join(identity.to_bytes() for identity in identities)
        + bytes((16 - len(identities)) * 14)
    )


@pytest.mark.parametrize(
    ("schema", "payload", "mode"),
    (
        (1, struct.pack("<H2x", 75), config_manager.REQUESTED_MODE_AUTO),
        (2, struct.pack("<HB5x", 75, 3), config_manager.REQUESTED_MODE_DINPUT),
    ),
)
def test_legacy_configuration_has_no_native_rumble_approval(
    schema: int, payload: bytes, mode: int
) -> None:
    device = FakeDevice()
    device.configuration_schema = schema
    device.configuration = payload
    configuration = config_manager.read_configuration(device)
    assert configuration.pairing_window_seconds == 75
    assert configuration.requested_mode == mode
    assert configuration.joycon_mode == config_manager.JOYCON_MODE_PAIRED
    assert configuration.native_switch_controllers == ()
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.set_native_switch_rumble_approval(
            device, native_rumble_identity(), True, 1.0
        )
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.write_configuration(
            device,
            replace(
                configuration, native_switch_controllers=(native_rumble_identity(),)
            ),
            1.0,
        )
    assert not device.out_requests


@pytest.mark.parametrize("schema", (3, 4))
def test_native_rumble_configuration_canonical_wire_round_trip(schema: int) -> None:
    device = FakeDevice()
    identities = tuple(
        native_rumble_identity(
            bytes((index, 2, 3, 4, 5, 6)), (0x2009, 0x2006, 0x2007)[index % 3]
        )
        for index in range(16)
    )
    config_manager.write_configuration(
        device,
        config_manager.AdapterConfiguration(
            90,
            0,
            0,
            config_manager.REQUESTED_MODE_XINPUT,
            tuple(reversed(identities)),
            schema_version=schema,
        ),
        1.0,
    )
    assert device.configuration_schema == schema
    assert device.configuration == native_rumble_configuration(identities)
    stored = config_manager.read_configuration(device)
    assert stored.native_switch_controllers == identities
    assert stored.crc == zlib.crc32(device.configuration) & 0xFFFFFFFF
    assert stored.pairing_window_seconds == 90
    assert stored.requested_mode == config_manager.REQUESTED_MODE_XINPUT
    assert stored.joycon_mode == config_manager.JOYCON_MODE_PAIRED


@pytest.mark.parametrize(
    ("offset", "value"),
    (
        (3, 17),  # Capacity overflow.
        (4, 1),  # Header reserved byte.
        (8, 0),  # Unstable non-global identity.
        (9, config_manager.TRANSPORT_BLE),
        (11, 1),  # Identity reserved byte.
        (18, 0),  # Different vendor.
        (20, 0),  # Unqualified product.
        (22, 1),  # Unused identity slot.
    ),
)
def test_native_rumble_configuration_rejects_malformed_approvals(
    offset: int, value: int
) -> None:
    device = FakeDevice()
    device.configuration_schema = 3
    payload = bytearray(native_rumble_configuration((native_rumble_identity(),)))
    payload[offset] = value
    device.configuration = bytes(payload)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_configuration(device)


@pytest.mark.parametrize("malformation", ("duplicate", "unsorted", "global", "short"))
def test_native_rumble_configuration_rejects_invalid_lists(malformation: str) -> None:
    device = FakeDevice()
    device.configuration_schema = 3
    first = native_rumble_identity()
    second = native_rumble_identity(bytes.fromhex("A1A2A3A4A5A6"))
    identities = {
        "duplicate": (first, first),
        "unsorted": (second, first),
        "global": (config_manager.ControllerIdentity.global_fallback(),),
        "short": (first,),
    }[malformation]
    device.configuration = native_rumble_configuration(identities)
    if malformation == "short":
        device.configuration = device.configuration[:-1]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_configuration(device)


@pytest.mark.parametrize(
    "malformation",
    ("duplicate", "overflow", "global", "ble", "pair", "vendor", "product"),
)
def test_native_rumble_write_rejects_invalid_approvals_before_transaction(
    malformation: str,
) -> None:
    device = FakeDevice()
    identity = native_rumble_identity()
    identities = {
        "duplicate": (identity, identity),
        "overflow": tuple(
            native_rumble_identity(bytes((index, 2, 3, 4, 5, 6))) for index in range(17)
        ),
        "global": (config_manager.ControllerIdentity.global_fallback(),),
        "ble": (replace(identity, transport=config_manager.TRANSPORT_BLE),),
        "pair": (
            config_manager.ControllerIdentity.from_bytes(
                bytes.fromhex("0503102030405060C12233445566")
            ),
        ),
        "vendor": (replace(identity, vendor_id=0x045E),),
        "product": (replace(identity, product_id=0x2019),),
    }[malformation]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.write_configuration(
            device,
            config_manager.AdapterConfiguration(
                90, 0, 0, native_switch_controllers=identities
            ),
            1.0,
        )
    assert not device.out_requests


@pytest.mark.parametrize(
    ("schema", "joycon_mode"),
    (
        (3, config_manager.JOYCON_MODE_PAIRED),
        (4, config_manager.JOYCON_MODE_INDIVIDUAL),
    ),
)
def test_native_rumble_cli_approval_is_physical_and_preserves_other_settings(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    schema: int,
    joycon_mode: int,
) -> None:
    device = FakeDevice()
    device.configuration_schema = schema
    device.configuration = native_rumble_configuration(joycon_mode=joycon_mode)
    first = native_rumble_identity()
    second = native_rumble_identity(bytes.fromhex("A1A2A3A4A5A6"))
    for identity in (first, second):
        device.profile_identities.append(identity)
        device.active_profiles[identity.to_bytes()] = 3
    previous_profiles = dict(device.profiles)
    previous_active = dict(device.active_profiles)
    previous_pairings = list(device.records)
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))

    assert config_manager.main(["config", "native-rumble", "list"]) == 0
    assert first.address_text in capsys.readouterr().out
    assert config_manager.read_configuration(device).native_switch_controllers == ()
    assert (
        config_manager.main(["config", "native-rumble", "approve", "--identity", "2"])
        == 2
    )
    assert not device.out_requests
    capsys.readouterr()
    assert (
        config_manager.main(
            ["config", "native-rumble", "approve", "--identity", "2", "--yes"]
        )
        == 0
    )
    capsys.readouterr()
    approved = config_manager.read_configuration(device)
    assert approved.native_switch_controllers == (first,)
    assert approved.pairing_window_seconds == 90
    assert approved.requested_mode == config_manager.REQUESTED_MODE_XINPUT
    assert approved.joycon_mode == joycon_mode
    assert device.profiles == previous_profiles
    assert device.active_profiles == previous_active
    assert device.records == previous_pairings
    assert (
        config_manager.main(["config", "set", "--pairing-window-seconds", "120"]) == 0
    )
    preserved = config_manager.read_configuration(device)
    assert preserved.pairing_window_seconds == 120
    assert preserved.requested_mode == config_manager.REQUESTED_MODE_XINPUT
    assert preserved.native_switch_controllers == (first,)
    assert preserved.joycon_mode == joycon_mode
    config_manager.set_mode(device, config_manager.REQUESTED_MODE_DINPUT, 1.0)
    preserved = config_manager.read_configuration(device)
    assert preserved.requested_mode == config_manager.REQUESTED_MODE_DINPUT
    assert preserved.native_switch_controllers == (first,)
    assert preserved.joycon_mode == joycon_mode
    assert (
        config_manager.main(["config", "native-rumble", "revoke", "--identity", "2"])
        == 0
    )
    revoked = config_manager.read_configuration(device)
    assert revoked.native_switch_controllers == ()
    assert revoked.requested_mode == config_manager.REQUESTED_MODE_DINPUT
    assert revoked.pairing_window_seconds == 120
    assert revoked.joycon_mode == joycon_mode


@pytest.mark.parametrize("identity_index", ("0", "1", "99"))
def test_native_rumble_cli_rejects_unqualified_or_missing_identity(
    monkeypatch: pytest.MonkeyPatch, identity_index: str
) -> None:
    device = FakeDevice()
    device.configuration_schema = 3
    device.configuration = native_rumble_configuration()
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    assert (
        config_manager.main(
            [
                "config",
                "native-rumble",
                "approve",
                "--identity",
                identity_index,
                "--yes",
            ]
        )
        == 1
    )
    assert not device.out_requests


def test_native_rumble_cli_can_revoke_forgotten_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    first = native_rumble_identity()
    second = native_rumble_identity(bytes.fromhex("A1A2A3A4A5A6"))
    device.configuration_schema = 3
    device.configuration = native_rumble_configuration((first, second))
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    assert (
        config_manager.main(["config", "native-rumble", "revoke", "--approval", "2"])
        == 1
    )
    assert not device.out_requests
    assert (
        config_manager.main(["config", "native-rumble", "revoke", "--approval", "0"])
        == 0
    )
    assert config_manager.read_configuration(device).native_switch_controllers == (
        second,
    )
    assert config_manager.OP_PROFILE_LIST not in device.requests


@pytest.mark.parametrize(("offset", "value"), ((0, 1), (4, 32)))
def test_native_rumble_diagnostics_rejects_malformed_rows(
    offset: int, value: int
) -> None:
    device = FakeDevice()
    payload = bytearray(device.native_rumble_diagnostics)
    payload[offset] = value
    device.native_rumble_diagnostics = bytes(payload)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_native_switch_rumble(device)


def test_native_rumble_diagnostics_rejects_truncated_snapshot() -> None:
    device = FakeDevice()
    device.native_rumble_diagnostics = device.native_rumble_diagnostics[:-1]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_native_switch_rumble(device)


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
            joycon_mode=config_manager.JOYCON_MODE_INDIVIDUAL,
        ),
        1.0,
    )
    assert status.stored_generation == 4
    stored = config_manager.read_configuration(device)
    assert stored.pairing_window_seconds == 90
    assert stored.requested_mode == config_manager.REQUESTED_MODE_XINPUT
    assert stored.joycon_mode == config_manager.JOYCON_MODE_INDIVIDUAL
    reset = config_manager.reset_configuration(device, 1.0)
    assert reset.stored_generation == 5
    reset_configuration = config_manager.read_configuration(device)
    assert reset_configuration.pairing_window_seconds == 60
    assert reset_configuration.requested_mode == config_manager.REQUESTED_MODE_AUTO
    assert reset_configuration.native_switch_controllers == ()
    assert reset_configuration.joycon_mode == config_manager.JOYCON_MODE_PAIRED


@pytest.mark.parametrize(
    ("schema", "payload"),
    (
        (1, struct.pack("<H2x", 75)),
        (2, struct.pack("<HB5x", 75, config_manager.REQUESTED_MODE_DINPUT)),
        (3, native_rumble_configuration((native_rumble_identity(),))),
    ),
)
def test_joycon_mode_legacy_read_and_write_refusal(
    schema: int,
    payload: bytes,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    device.configuration_schema = schema
    device.configuration = payload
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    assert config_manager.main(["joycon-mode", "--json"]) == 0
    assert json.loads(capsys.readouterr().out) == {
        "mode": "paired",
        "generation": 3,
        "supported": False,
    }
    configuration = config_manager.read_configuration(device)
    assert configuration.joycon_mode == config_manager.JOYCON_MODE_PAIRED
    if schema == 3:
        assert configuration.native_switch_controllers == (native_rumble_identity(),)
    for mode in config_manager.JOYCON_MODE_NAMES:
        assert config_manager.main(["joycon-mode", mode, "--json"]) == 1
        assert capsys.readouterr().out == ""
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.write_configuration(
            device,
            replace(configuration, joycon_mode=config_manager.JOYCON_MODE_INDIVIDUAL),
            1.0,
        )
    assert device.configuration == payload
    assert not device.out_requests


@pytest.mark.parametrize(
    ("offset", "value"), ((4, 2), (4, 255), (5, 1), (6, 1), (7, 1))
)
def test_joycon_mode_rejects_invalid_wire_encoding(offset: int, value: int) -> None:
    device = FakeDevice()
    device.configuration_schema = 4
    payload = bytearray(native_rumble_configuration((native_rumble_identity(),)))
    payload[offset] = value
    device.configuration = bytes(payload)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_configuration(device)


@pytest.mark.parametrize("size", (8, 231, 233))
def test_joycon_mode_requires_exact_configuration_size(size: int) -> None:
    device = FakeDevice()
    device.configuration_schema = 4
    payload = native_rumble_configuration()
    device.configuration = (payload + b"\x00")[:size]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_configuration(device)


@pytest.mark.parametrize("mode", (-1, 2, True, 1.0, "individual"))
def test_joycon_mode_rejects_invalid_values_before_transaction(mode: object) -> None:
    device = FakeDevice()
    device.configuration_schema = 4
    device.configuration = native_rumble_configuration()
    before = config_manager.read_configuration(device)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.set_joycon_mode(device, mode, 1.0)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.write_configuration(
            device, replace(before, joycon_mode=mode), 1.0
        )
    assert not device.out_requests


def test_joycon_mode_cli_commits_and_reads_without_reboot_or_profile_changes(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    device.configuration_schema = 4
    identity = native_rumble_identity()
    device.configuration = native_rumble_configuration((identity,))
    previous_profiles = dict(device.profiles)
    previous_active = dict(device.active_profiles)
    previous_aliases = dict(device.profile_aliases)
    previous_names = dict(device.profile_names)
    previous_pairings = list(device.records)
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    for generation, mode in enumerate(("individual", "paired"), start=4):
        assert config_manager.main(["joycon-mode", mode, "--json"]) == 0
        expected = {"mode": mode, "generation": generation, "supported": True}
        assert json.loads(capsys.readouterr().out) == expected
        stored = config_manager.read_configuration(device)
        assert stored.joycon_mode == config_manager.JOYCON_MODE_NAMES.index(mode)
        assert stored.pairing_window_seconds == 90
        assert stored.requested_mode == config_manager.REQUESTED_MODE_XINPUT
        assert stored.native_switch_controllers == (identity,)
        assert device.configuration == native_rumble_configuration(
            (identity,), config_manager.JOYCON_MODE_NAMES.index(mode)
        )
        writes_before_read = tuple(device.out_requests)
        assert config_manager.main(["joycon-mode", "--json"]) == 0
        assert json.loads(capsys.readouterr().out) == expected
        assert tuple(device.out_requests) == writes_before_read
    assert device.profiles == previous_profiles
    assert device.active_profiles == previous_active
    assert device.profile_aliases == previous_aliases
    assert device.profile_names == previous_names
    assert device.records == previous_pairings
    assert not device.reboot_transaction_ids
    assert not device.bootsel_reboot_requested


def test_joycon_mode_cli_commit_failure_does_not_claim_new_preference(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    device.configuration_schema = 4
    device.configuration = native_rumble_configuration((native_rumble_identity(),))
    before = config_manager.read_configuration(device)
    device.fail_configuration_commit_status = 8
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    assert config_manager.main(["joycon-mode", "individual", "--json"]) == 1
    assert capsys.readouterr().out == ""
    assert config_manager.read_configuration(device) == before
    assert not device.reboot_transaction_ids


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
    config_manager.set_mode(device, config_manager.REQUESTED_MODE_SWITCH, 1.0)

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
    monkeypatch.setattr(config_manager.secrets, "randbits", lambda _bits: 0x12345678)

    status = config_manager.set_mode(device, config_manager.REQUESTED_MODE_XINPUT, 1.0)

    mode_payload = struct.pack("<IB", 0x12345678, config_manager.REQUESTED_MODE_XINPUT)
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
    assert encoded == config_manager.encode_request(config_manager.OP_BOOTSEL_REBOOT)
    assert device.bootsel_reboot_requested

    for transaction_id in (0, 0x80000000, True):
        with pytest.raises(config_manager.ConfigManagerError):
            config_manager.request_reboot(device, transaction_id)
    for mode in (0xFF, True):
        with pytest.raises(config_manager.ConfigManagerError, match="not available"):
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
        config_manager.configure_mode(device, config_manager.REQUESTED_MODE_SWITCH, 1.0)

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
        config_manager.configure_mode(device, config_manager.REQUESTED_MODE_SWITCH, 1.0)

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
            config_manager.CAPABILITY_INPUT | config_manager.CAPABILITY_RUMBLE
        )

    same_device, changed = config_manager.configure_mode(device, requested_mode, 1.0)

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
            config_manager.CAPABILITY_INPUT | config_manager.CAPABILITY_RUMBLE
        )
    scans = iter(((previous,), (), (reenumerated,)))
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (reenumerated,)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    result, changed = config_manager.configure_mode(previous, requested_mode, 1.0)

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
        config_manager.configure_mode(previous, requested_mode, 1.0)


def test_reenumeration_reports_missing_disappearance_and_return(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))
    with pytest.raises(config_manager.ConfigManagerError, match="did not disappear"):
        snapshot = config_manager._capture_reenumeration_snapshot(device)
        config_manager._wait_for_reenumeration(snapshot, 0)

    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: ())
    with pytest.raises(config_manager.ConfigManagerError, match="did not re-enumerate"):
        snapshot = config_manager._capture_reenumeration_snapshot(device)
        config_manager._wait_for_reenumeration(snapshot, 0)


def test_runtime_diagnostics_distinguishes_unreported_from_zero_drops() -> None:
    device = FakeDevice()
    legacy = config_manager.read_runtime_diagnostics(device)
    assert legacy.switch2_ingress_drops is None
    assert legacy.switch2_output_drops is None
    device.runtime_diagnostics_tail = struct.pack("<II", 0, 0xFFFFFFFF)
    current = config_manager.read_runtime_diagnostics(device)
    assert current.switch2_ingress_drops == 0
    assert current.switch2_output_drops == 0xFFFFFFFF
    device.runtime_diagnostics_tail = bytes(4)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_runtime_diagnostics(device)


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
        struct.pack("<HB5x", 60, config_manager.REQUESTED_MODE_AUTO)[:-1] + b"\x01"
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
    with pytest.raises(config_manager.ConfigManagerError, match="omit required input"):
        config_manager.read_info(device)


def test_native_hub_reports_capabilities_but_refuses_mode_changes() -> None:
    device = FakeDevice()
    device.firmware_version = (0, 72, 0)
    device.active_mode = 5
    device.capabilities = 7
    before = config_manager.read_configuration(device)

    info = config_manager.read_info(device)
    assert info.firmware_version == (0, 72, 0)
    assert info.mode_name() == "Native Joy-Con hub"
    assert info.capability_names() == ("input", "rumble", "motion")
    with pytest.raises(config_manager.ConfigManagerError, match="fixed USB output"):
        config_manager.configure_mode(device, before.requested_mode, 1.0)
    with pytest.raises(config_manager.ConfigManagerError, match="fixed USB output"):
        config_manager.set_mode(device, config_manager.REQUESTED_MODE_SWITCH, 1.0)
    with pytest.raises(config_manager.ConfigManagerError, match="fixed USB output"):
        config_manager.request_reboot(device, 1)
    assert config_manager.read_configuration(device) == before
    assert device.out_requests == []

    config_manager.request_bootsel_reboot(device)
    assert device.bootsel_reboot_requested


@pytest.mark.parametrize(
    ("left_type", "right_type", "flags"), [(0, 0, 1), (1, 0, 3), (0, 1, 5), (1, 1, 7)]
)
def test_joycon_pair_wire_round_trip_preserves_both_typed_members(
    left_type: int,
    right_type: int,
    flags: int,
) -> None:
    left = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_BLE,
        left_type,
        bytes.fromhex("C10203040506"),
        0x057E,
        0x2067,
    )
    right = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_BLE,
        right_type,
        bytes.fromhex("D11213141516"),
        0x057E,
        0x2066,
    )
    pair = config_manager.ControllerIdentity.make_joycon_pair(left, right)
    wire = bytes((flags, 3)) + left.address + right.address
    assert pair.to_bytes() == wire
    assert config_manager.ControllerIdentity.from_bytes(wire) == pair
    assert pair.joycon_pair_members() == (left, right)
    assert pair.to_bytes() not in (left.to_bytes(), right.to_bytes())
    other_right = replace(right, address=bytes.fromhex("D11213141517"))
    assert config_manager.ControllerIdentity.make_joycon_pair(left, other_right) != pair


def test_joycon_pair_distinguishes_address_types_without_normalizing_members() -> None:
    left = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_BLE,
        0,
        bytes.fromhex("C10203040506"),
        0x057E,
        0x2067,
    )
    right = replace(left, address_type=1, product_id=0x2066)
    pair = config_manager.ControllerIdentity.make_joycon_pair(left, right)
    assert pair.to_bytes() == bytes.fromhex("0503C10203040506C10203040506")
    assert pair.joycon_pair_members() == (left, right)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerIdentity.make_joycon_pair(
            left, replace(right, address_type=0)
        )
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerIdentity.make_joycon_pair(right, left)


@pytest.mark.parametrize(
    "wire",
    [
        "0003102030405060C12233445566",  # Stable flag missing.
        "0D03102030405060C12233445566",  # Reserved flag.
        "0303102030405060C12233445566",  # Left random address is not static.
        "0503C12233445566102030405060",  # Right random address is not static.
        "0103102030405060102030405060",  # Duplicate typed members.
    ],
)
def test_joycon_pair_rejects_malformed_wire(wire: str) -> None:
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerIdentity.from_bytes(bytes.fromhex(wire))


@pytest.mark.parametrize(
    "malformation",
    [
        "global",
        "classic",
        "vendor",
        "model",
        "address_type",
        "random_address",
    ],
)
def test_joycon_pair_requires_legitimate_ble_members(malformation: str) -> None:
    pair = config_manager.ControllerIdentity.from_bytes(
        bytes.fromhex("0503102030405060C12233445566")
    )
    left, right = pair.joycon_pair_members()
    invalid = {
        "global": config_manager.ControllerIdentity.global_fallback(),
        "classic": replace(right, transport=config_manager.TRANSPORT_CLASSIC),
        "vendor": replace(right, vendor_id=0x045E),
        "model": replace(right, product_id=0x2069),
        "address_type": replace(right, address_type=3),
        "random_address": replace(right, address=bytes.fromhex("412233445566")),
    }[malformation]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerIdentity.make_joycon_pair(left, invalid)


@pytest.mark.parametrize(
    "fields",
    [
        {"stable": False},
        {"vendor_id": 0x045E},
        {"product_id": 0x2066},
        {"address_type": 2},
        {"partner_address_type": 3},
    ],
)
def test_joycon_pair_rejects_inconsistent_in_memory_identity(
    fields: dict[str, object],
) -> None:
    pair = config_manager.ControllerIdentity.from_bytes(
        bytes.fromhex("0503102030405060C12233445566")
    )
    with pytest.raises(config_manager.ConfigManagerError):
        replace(pair, **fields)


@pytest.mark.parametrize(
    "fields",
    [
        {"partner_address_type": 1},
        {"partner_address": bytes.fromhex("C12233445566")},
    ],
)
def test_physical_identity_cannot_hide_pair_members(fields: dict[str, object]) -> None:
    with pytest.raises(config_manager.ConfigManagerError):
        replace(native_rumble_identity(), **fields)


def test_pairing_inventory_rejects_logical_profile_owners() -> None:
    device = FakeDevice()
    device.records = [
        (config_manager.TRANSPORT_JOYCON_PAIR, 0, bytes.fromhex("102030405060"))
    ]
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_pairings(device)


def test_profile_cli_lists_both_pair_addresses_but_native_inventory_omits_pair(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    pair = config_manager.ControllerIdentity.from_bytes(
        bytes.fromhex("0503102030405060C12233445566")
    )
    device.profile_identities.append(pair)
    device.active_profiles[pair.to_bytes()] = 4
    monkeypatch.setattr(config_manager, "find_pico", lambda *_: device)
    assert config_manager.main(["profiles", "list"]) == 0
    listing = capsys.readouterr().out
    assert pair.address_text in listing
    assert pair.partner_address_text in listing
    assert pair.transport_text in listing
    assert config_manager.main(["config", "native-rumble", "list"]) == 0
    physical = capsys.readouterr().out
    assert pair.partner_address_text not in physical
    assert device.stable_identity.address_text in physical


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
    assert encoded_identity == bytes.fromhex("01020300A1B2C3D4E5F63412CDAB")
    assert config_manager.ControllerIdentity.from_bytes(encoded_identity) == identity
    assert config_manager.ControllerIdentity.global_fallback().to_bytes() == bytes(
        config_manager.CONTROLLER_IDENTITY_SIZE
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
    assert encoded[75] == 0x21
    assert encoded[60:62] == bytes((17, 0))
    assert encoded[70:72] == bytes((3, 0))
    assert struct.unpack_from("<H", encoded, 78)[0] == (
        profile.motion_toggle_chord & 0xFFFF
    )
    assert encoded[96:102] == bytes((1, 0, 0x47, 0, 1, 17))
    assert encoded[102:108] == bytes((6, 0, 0x7C, 17, 1, 5))
    assert encoded[252:256] == bytes(4)
    assert config_manager.ControllerProfile.from_bytes(encoded) == profile

    serialized = profile.to_json()
    decoded = config_manager.ControllerProfile.from_json(serialized)
    assert decoded == profile
    assert decoded.to_json() == serialized
    legacy_default_wire = legacy_profile_wire(
        config_manager.PROFILE_LEGACY_SCHEMA_VERSION
    )
    legacy_default_wire[81] = 0
    legacy_default_wire[98:100] = b"\x00\x00"
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
    migrated_default = config_manager.ControllerProfile.from_bytes(legacy_default_wire)
    assert migrated_default.swing.button == config_manager.PROFILE_NONE_BUTTON
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

    previous_wire = legacy_profile_wire(
        config_manager.PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION,
        macro_trigger=0,
        macro_cancel=1,
    )
    migrated_previous = config_manager.ControllerProfile.from_bytes(previous_wire)
    assert migrated_previous.swing.button == config_manager.PROFILE_NONE_BUTTON
    assert migrated_previous.left_trigger.output == 16
    assert migrated_previous.right_trigger.output == 17
    assert migrated_previous.macros[0].trigger_mask == 1
    assert migrated_previous.macros[0].cancel_control == 1
    assert migrated_previous.motion_toggle_chord == 0
    assert migrated_previous.to_bytes()[0] == config_manager.PROFILE_SCHEMA_VERSION

    legacy_json_object = default_profile.to_json_object()
    legacy_json_object["schema_version"] = config_manager.PROFILE_LEGACY_SCHEMA_VERSION
    legacy_json_object["size"] = config_manager.PROFILE_LEGACY_SIZE
    for field in (
        "shortcuts",
        "shift",
        "turbo_settings",
        "extra_button_map",
        "swing",
        "nunchuk_swing",
        "combined_swing",
        "combination_window_ms",
    ):
        del legacy_json_object[field]
    del legacy_json_object["motion_toggle_chord"]
    del legacy_json_object["triggers"]["left"]["output"]
    del legacy_json_object["triggers"]["right"]["output"]
    legacy_json_object.pop("macros")
    legacy_json_object["macro"] = {
        "trigger": "south",
        "cancel": None,
        "steps": [config_manager.MacroStep.end().to_json_object()],
    }
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
    assert migrated_json.left_trigger.output == 16
    assert migrated_json.right_trigger.output == 17
    assert migrated_json.macros[0].trigger_mask == 1
    assert migrated_json.motion_toggle_chord == 0


def test_schema5_full_macro_stream_migrates_bytes_and_json(monkeypatch) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    obj["macros"][0]["trigger"] = ["south"]
    obj["macros"][0]["steps"] = [
        custom_profile().macros[0].steps[0].to_json_object()
    ] * 8
    expected = config_manager.ControllerProfile.from_json_object(obj)
    legacy_wire = bytearray(expected.to_bytes()[:256])
    struct.pack_into("<HH", legacy_wire, 0, 5, 256)
    assert legacy_wire[254:256] == b"\x31\xd4"
    migrated = config_manager.ControllerProfile.from_bytes(legacy_wire)
    assert migrated == expected
    assert migrated.to_bytes()[4:256] == legacy_wire[4:]
    envelope = config_manager.parse_response(
        make_response(config_manager.OP_PROFILE_READ, bytes(legacy_wire), schema=5),
        config_manager.OP_PROFILE_READ,
    )
    monkeypatch.setattr(
        config_manager, "_control_in", lambda device, operation: envelope
    )
    assert config_manager.read_selected_profile(FakeDevice()) == expected
    envelope = config_manager.parse_response(
        make_response(config_manager.OP_PROFILE_READ, bytes(legacy_wire), schema=6),
        config_manager.OP_PROFILE_READ,
    )
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.read_selected_profile(FakeDevice())

    obj["schema_version"] = 5
    obj["size"] = 256
    for field in (
        "shortcuts",
        "shift",
        "turbo_settings",
        "extra_button_map",
        "swing",
        "nunchuk_swing",
        "combined_swing",
        "combination_window_ms",
    ):
        del obj[field]
    for macro in obj["macros"]:
        del macro["playback"]
        del macro["repeat_count"]
    assert config_manager.ControllerProfile.from_json_object(obj) == expected
    struct.pack_into("<HH", legacy_wire, 0, 6, 384)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(legacy_wire)


def test_set_b_sparse_settings_and_macro_modes_round_trip() -> None:
    obj = custom_profile().to_json_object()
    obj["shortcuts"] = {
        "modifier": "left_trigger",
        "profiles": ["south", None, None, None, None, None, None, "dpad_right"],
    }
    obj["shift"]["mode"] = "toggle"
    obj["shift"]["modifier"] = "right_trigger"
    obj["shift"]["button_map"]["south"] = None
    obj["turbo"]["dpad_right"] = "burst"
    obj["turbo_settings"] = {
        "defaults": {"rate_hz": 30, "duty_percent": 99, "burst_count": 255},
        "overrides": {
            "dpad_right": {"rate_hz": 23, "duty_percent": 37, "burst_count": 17},
            "west": {"rate_hz": 1, "duty_percent": 1, "burst_count": 1},
        },
    }
    for macro, playback in zip(
        obj["macros"], ("repeat", "once", "while_held", "toggle")
    ):
        macro["playback"] = playback
    obj["macros"][0]["repeat_count"] = 255
    profile = config_manager.ControllerProfile.from_json_object(obj)
    encoded = profile.to_bytes()
    assert encoded[256:267] == bytes((16, 0, 255, 255, 255, 255, 255, 255, 15, 2, 17))
    assert encoded[283:294] == bytes((30, 99, 255, 4, 128, 1, 1, 1, 23, 37, 17))
    assert encoded[294:336] == bytes(42)
    assert encoded[336:344] == bytes((3, 255, 0, 1, 1, 1, 2, 1))
    assert encoded[344:358] == bytes([255]) * 14
    assert encoded[358:364] == bytes(6)
    assert config_manager.ControllerProfile.from_bytes(encoded) == profile
    assert config_manager.ControllerProfile.from_json(profile.to_json()) == profile

    legacy_wire = bytearray(encoded)
    struct.pack_into("<H", legacy_wire, 0, 6)
    legacy_wire[344:] = bytes(40)
    legacy_json = json.loads(profile.to_json())
    legacy_json["schema_version"] = 6
    del legacy_json["extra_button_map"]
    del legacy_json["shift"]["extra_button_map"]
    for field in ("swing", "nunchuk_swing", "combined_swing", "combination_window_ms"):
        del legacy_json[field]
    assert config_manager.ControllerProfile.from_bytes(legacy_wire) == profile
    assert config_manager.ControllerProfile.from_json_object(legacy_json) == profile
    device = FakeDevice()
    device.profiles[(device.stable_identity.to_bytes(), 1)] = bytes(legacy_wire)
    assert config_manager.read_profile(device, device.stable_identity, 1) == profile
    device.profile_aliases[device.stable_identity.to_bytes()] = "Custom controller"
    old_listing = config_manager.parse_profile_list(
        config_manager.parse_response(
            make_response(
                config_manager.OP_PROFILE_LIST, device._profile_list_payload(), schema=6
            ),
            config_manager.OP_PROFILE_LIST,
        )
    )
    assert old_listing[1] == config_manager.ProfileListEntry(
        device.stable_identity, 1, "Custom controller"
    )


@pytest.mark.parametrize("version", [7, 8, 9])
def test_schema7_extra_controls_keep_output_channels_and_wire_layout(
    version: int,
) -> None:
    obj = custom_profile().to_json_object()
    obj["extra_button_map"] = dict(
        zip(
            config_manager.EXTRA_BUTTONS,
            (
                "south",
                "right_trigger",
                "left_trigger",
                None,
                "dpad_left",
                "start",
                "capture",
            ),
        )
    )
    obj["shift"]["mode"] = "hold"
    obj["shift"]["modifier"] = "gl"
    obj["shift"]["extra_button_map"] = dict(
        zip(
            config_manager.EXTRA_BUTTONS,
            ("east", None, "west", "north", "system", "dpad_up", "dpad_right"),
        )
    )
    obj["shortcuts"]["modifier"] = "left_sr"
    obj["shortcuts"]["profiles"][0] = "south"
    obj["switching_chord"] = ["left_trigger", "c", "right_sr"]
    obj["motion_toggle_chord"] = ["right_trigger", "gl", "left_sl"]
    for index, names in enumerate(
        (["c", "gl"], ["gr"], ["left_sl", "left_sr"], ["right_sl", "right_sr"])
    ):
        obj["macros"][index]["trigger"] = names
        obj["macros"][index]["cancel"] = config_manager.EXTRA_BUTTONS[index + 3]
    obj["schema_version"] = version
    if version == 7:
        del obj["swing"]
    elif version == 8:
        del obj["swing"]["macro"]
    if version < 9:
        for field in ("nunchuk_swing", "combined_swing", "combination_window_ms"):
            del obj[field]
    profile = config_manager.ControllerProfile.from_json_object(obj)
    encoded = profile.to_bytes()
    assert encoded[:4] == struct.pack("<HH", config_manager.PROFILE_SCHEMA_VERSION, 384)
    assert encoded[344:351] == bytes((0, 17, 16, 255, 14, 7, 9))
    assert encoded[351:358] == bytes((1, 255, 2, 3, 8, 12, 15))
    assert encoded[358:364] == bytes((3, 4, 24, 96, 65, 10))
    assert config_manager.ControllerProfile.from_bytes(encoded) == profile
    assert config_manager.ControllerProfile.from_json(profile.to_json()) == profile
    assert (
        len(profile.button_map)
        == len(profile.shift.button_map)
        == len(profile.turbo_modes)
        == 16
    )
    assert profile.swing.button == config_manager.PROFILE_NONE_BUTTON
    legacy_wire = bytearray(encoded)
    struct.pack_into("<H", legacy_wire, 0, version)
    if version == 7:
        legacy_wire[364:] = bytes(20)
    elif version == 8:
        legacy_wire[367:] = bytes(17)
    device = FakeDevice()
    device.profiles[(device.stable_identity.to_bytes(), 1)] = bytes(legacy_wire)
    assert config_manager.read_profile(device, device.stable_identity, 1) == profile
    listing = config_manager.parse_profile_list(
        config_manager.parse_response(
            make_response(
                config_manager.OP_PROFILE_LIST,
                device._profile_list_payload(),
                schema=version,
            ),
            config_manager.OP_PROFILE_LIST,
        )
    )
    assert listing[1].identity == device.stable_identity
    assert listing[1].active_profile_index == 1


@pytest.mark.parametrize(
    "path",
    [
        ("button_map", "south"),
        ("extra_button_map", "c"),
        ("triggers", "left", "output"),
        ("shift", "extra_button_map", "c"),
    ],
)
def test_extra_controls_cannot_be_output_destinations(path: tuple[str, ...]) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    target = obj
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = "right_sr"
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)


@pytest.mark.parametrize(
    "offset,value",
    [
        (98, 18 << 2),
        (256, 18),
        (266, 24),
        (344, 1),
        (358, 1),
        (363, 1),
    ],
)
def test_schema6_rejects_schema7_controls_in_old_fields(
    offset: int, value: int
) -> None:
    payload = bytearray(config_manager.ControllerProfile.default().to_bytes())
    struct.pack_into("<H", payload, 0, 6)
    payload[344:] = bytes(40)
    payload[offset] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


@pytest.mark.parametrize(
    "offset,value",
    [
        (344, 18),
        (351, 16),
        (358, 128),
        (362, 128),
        (363, 128),
        (364, 1),
    ],
)
def test_schema7_rejects_extra_map_and_mask_overflow(offset: int, value: int) -> None:
    payload = bytearray(config_manager.ControllerProfile.default().to_bytes())
    struct.pack_into("<H", payload, 0, 7)
    payload[364:] = bytes(20)
    payload[offset] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


@pytest.mark.parametrize("version", [3, 4])
def test_legacy_control_profiles_preserve_custom_actions(version: int) -> None:
    payload = legacy_profile_wire(version, macro_trigger=3, macro_cancel=2)
    payload[4] = 17
    payload[72:75] = bytes((71, 202, 1))
    struct.pack_into("<H", payload, 76, 1 << 6)
    struct.pack_into("<H", payload, 98, 1 << 5)
    if version == 4:
        payload[75] = 0x21
        payload[81] = 17
    profile = config_manager.ControllerProfile.from_bytes(payload)
    assert profile.button_map[0] == 17
    assert profile.weak_rumble_scale == 71
    assert profile.strong_rumble_scale == 202
    assert profile.confirmation_policy == 1
    assert profile.switching_chord == (1 << 6) | ((1 << 16) if version == 4 else 0)
    assert profile.motion_toggle_chord == (1 << 5) | ((1 << 17) if version == 4 else 0)
    assert profile.macros[0].trigger_mask == 3
    assert profile.macros[0].cancel_control == (17 if version == 4 else 2)
    assert profile.extra_button_map == (255,) * 7
    assert profile.swing.button == config_manager.PROFILE_NONE_BUTTON
    obj = profile.to_json_object()
    obj["schema_version"] = version
    obj["size"] = 256
    macro = obj.pop("macros")[0]
    macro.pop("playback")
    macro.pop("repeat_count")
    macro["steps"].append(config_manager.MacroStep.end().to_json_object())
    obj["macro"] = macro
    for key in (
        "shortcuts",
        "shift",
        "turbo_settings",
        "extra_button_map",
        "swing",
        "nunchuk_swing",
        "combined_swing",
        "combination_window_ms",
    ):
        del obj[key]
    assert config_manager.ControllerProfile.from_json_object(obj) == profile
    assert config_manager.ControllerProfile.from_bytes(profile.to_bytes()) == profile


@pytest.mark.parametrize(
    ("button", "sensitivity", "modifier"),
    [
        ("south", "low", "right_sr"),
        ("dpad_right", "high", "south"),
        ("west", "medium", "right_trigger"),
        (None, "high", None),
    ],
)
def test_swing_profile_round_trip_preserves_other_settings(
    button: str | None,
    sensitivity: str,
    modifier: str | None,
) -> None:
    before = custom_profile()
    obj = before.to_json_object()
    obj["swing"] = {
        "button": button,
        "sensitivity": sensitivity,
        "modifier": modifier,
        "macro": None,
    }
    profile = config_manager.ControllerProfile.from_json_object(obj)
    assert replace(profile, swing=before.swing) == before
    assert profile.to_json_object()["swing"] == obj["swing"]
    assert config_manager.ControllerProfile.from_json(profile.to_json()) == profile
    assert config_manager.ControllerProfile.from_bytes(profile.to_bytes()) == profile
    device = FakeDevice()
    config_manager.write_profile(device, device.stable_identity, 2, profile, 1.0)
    assert config_manager.read_profile(device, device.stable_identity, 2) == profile


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("button", "left_trigger"),
        ("button", "right_sr"),
        ("button", 0),
        ("sensitivity", "extreme"),
        ("sensitivity", None),
        ("sensitivity", 1),
        ("modifier", "unknown"),
        ("modifier", False),
        ("macro", 0),
        ("macro", 5),
        ("macro", True),
        ("macro", "1"),
    ],
)
def test_swing_rejects_invalid_json_settings(field: str, value: object) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    obj["swing"][field] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("button", -1),
        ("button", 16),
        ("button", True),
        ("sensitivity", -1),
        ("sensitivity", 3),
        ("sensitivity", False),
        ("modifier", -1),
        ("modifier", 25),
        ("modifier", 256),
        ("macro", -1),
        ("macro", 4),
        ("macro", False),
    ],
)
def test_swing_rejects_invalid_in_memory_settings(field: str, value: object) -> None:
    with pytest.raises(config_manager.ConfigManagerError):
        replace(config_manager.ProfileSwing(), **{field: value})


@pytest.mark.parametrize(
    ("offset", "value"),
    [
        (364, 16),
        (365, 3),
        (366, 25),
        (367, 4),
        (368, 16),
        (369, 3),
        (370, 25),
        (371, 4),
        (372, 16),
        (373, 4),
        (374, 25),
        (375, 29),
        (375, 201),
        (376, 1),
    ],
)
def test_swing_rejects_corrupt_wire_settings(offset: int, value: int) -> None:
    payload = bytearray(config_manager.ControllerProfile.default().to_bytes())
    payload[offset] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


@pytest.mark.parametrize(
    "mutation",
    [
        "missing",
        "unknown",
        "missing_button",
        "missing_sensitivity",
        "missing_modifier",
        "missing_macro",
        "legacy_field",
    ],
)
def test_swing_json_fields_are_strict(mutation: str) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    if mutation == "missing":
        del obj["swing"]
    elif mutation == "unknown":
        obj["swing"]["enabled"] = False
    elif mutation.startswith("missing_"):
        del obj["swing"][mutation.removeprefix("missing_")]
    else:
        for field in ("nunchuk_swing", "combined_swing", "combination_window_ms"):
            del obj[field]
        obj["schema_version"] = 7
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)


def test_schema8_swing_migration_preserves_remote_binding() -> None:
    profile = replace(custom_profile(), swing=config_manager.ProfileSwing(2, 2, 24))
    legacy_wire = bytearray(profile.to_bytes())
    struct.pack_into("<H", legacy_wire, 0, 8)
    legacy_wire[367:] = bytes(17)
    legacy_json = profile.to_json_object()
    legacy_json["schema_version"] = 8
    del legacy_json["swing"]["macro"]
    for field in ("nunchuk_swing", "combined_swing", "combination_window_ms"):
        del legacy_json[field]
    assert config_manager.ControllerProfile.from_bytes(legacy_wire) == profile
    assert config_manager.ControllerProfile.from_json_object(legacy_json) == profile
    assert config_manager.ControllerProfile.from_bytes(profile.to_bytes()) == profile
    legacy_wire[367] = 1
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(legacy_wire)


@pytest.mark.parametrize("macro_number", [1, 4])
def test_gesture_macro_json_indices_and_wire_layout(macro_number: int) -> None:
    obj = custom_profile().to_json_object()
    obj["macros"][macro_number - 1] = {
        **obj["macros"][0],
        "trigger": [],
        "playback": "toggle",
    }
    obj["swing"].update(macro=macro_number, sensitivity="high", modifier="right_sr")
    obj["nunchuk_swing"].update(button="dpad_right", sensitivity="low", modifier="c")
    obj["combined_swing"].update(macro=macro_number, modifier="left_trigger")
    obj["combination_window_ms"] = 200
    profile = config_manager.ControllerProfile.from_json_object(obj)
    assert profile.swing.macro == profile.combined_swing.macro == macro_number - 1
    encoded = profile.to_bytes()
    assert encoded[364:376] == bytes(
        (
            255,
            2,
            24,
            macro_number - 1,
            15,
            0,
            18,
            255,
            255,
            macro_number - 1,
            16,
            200,
        )
    )
    assert encoded[376:] == bytes(8)
    assert config_manager.ControllerProfile.from_bytes(encoded) == profile
    assert profile.to_json_object() == obj


@pytest.mark.parametrize("gesture", ["swing", "nunchuk_swing", "combined_swing"])
def test_gesture_actions_are_exclusive(gesture: str) -> None:
    obj = custom_profile().to_json_object()
    obj[gesture].update(button="south", macro=1)
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)
    action_type = (
        config_manager.ProfileCombinedSwing
        if gesture == "combined_swing"
        else config_manager.ProfileSwing
    )
    with pytest.raises(config_manager.ConfigManagerError):
        action_type(button=0, macro=0)
    payload = bytearray(custom_profile().to_bytes())
    button_offset, macro_offset = {
        "swing": (364, 367),
        "nunchuk_swing": (368, 371),
        "combined_swing": (372, 373),
    }[gesture]
    payload[button_offset] = payload[macro_offset] = 0
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


@pytest.mark.parametrize("gesture", ["swing", "nunchuk_swing", "combined_swing"])
@pytest.mark.parametrize("empty", [False, True])
def test_gesture_macro_requires_configured_positive_duration_target(
    gesture: str,
    empty: bool,
) -> None:
    obj = custom_profile().to_json_object()
    target = obj["macros"][0]
    target["trigger"] = []
    if empty:
        target["steps"] = []
    else:
        for step in target["steps"]:
            step["duration_ms"] = 0
    unbound = config_manager.ControllerProfile.from_json_object(obj)
    obj[gesture]["macro"] = 1
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)
    with pytest.raises(config_manager.ConfigManagerError):
        replace(unbound, **{gesture: replace(getattr(unbound, gesture), macro=0)})
    payload = bytearray(unbound.to_bytes())
    payload[{"swing": 367, "nunchuk_swing": 371, "combined_swing": 373}[gesture]] = 0
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


@pytest.mark.parametrize("window", [30, 200])
def test_combination_window_inclusive_bounds_round_trip(window: int) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    obj["combination_window_ms"] = window
    profile = config_manager.ControllerProfile.from_json_object(obj)
    assert (
        config_manager.ControllerProfile.from_bytes(
            profile.to_bytes()
        ).combination_window_ms
        == window
    )


@pytest.mark.parametrize("window", [29, 201, True, 100.0, None])
def test_combination_window_rejects_out_of_range_or_noninteger_values(
    window: object,
) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    obj["combination_window_ms"] = window
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)
    with pytest.raises(config_manager.ConfigManagerError):
        replace(
            config_manager.ControllerProfile.default(), combination_window_ms=window
        )


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("shortcuts", "modifier"), None),
        (("shortcuts", "modifier"), "south"),
        (("shortcuts", "profiles", 1), "south"),
        (("shortcuts", "profiles", 0), "left_shoulder"),
        (("shift", "modifier"), None),
        (("shift", "button_map", "south"), "left_trigger"),
        (("turbo_settings", "defaults", "rate_hz"), 0),
        (("turbo_settings", "defaults", "rate_hz"), 31),
        (("turbo_settings", "defaults", "duty_percent"), 0),
        (("turbo_settings", "defaults", "duty_percent"), 100),
        (("turbo_settings", "defaults", "burst_count"), 0),
        (("turbo_settings", "defaults", "burst_count"), 256),
        (("macros", 0, "repeat_count"), 0),
        (("macros", 0, "repeat_count"), 256),
        (("macros", 0, "playback"), "forever"),
    ],
)
def test_set_b_rejects_invalid_json_settings(path: tuple, value: object) -> None:
    obj = config_manager.ControllerProfile.default().to_json_object()
    obj["shortcuts"]["modifier"] = "left_trigger"
    obj["shortcuts"]["profiles"][0] = "south"
    obj["shift"]["mode"] = "hold"
    obj["shift"]["modifier"] = "right_trigger"
    parent = obj
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_json_object(obj)


@pytest.mark.parametrize(
    ("offset", "value"),
    [
        (288, 1),
        (383, 1),
        (265, 3),
        (267, 16),
        (283, 31),
        (284, 100),
        (285, 0),
        (336, 4),
        (337, 0),
    ],
)
def test_set_b_rejects_corrupt_extension_bytes(offset: int, value: int) -> None:
    payload = bytearray(config_manager.ControllerProfile.default().to_bytes())
    payload[offset] = value
    with pytest.raises(config_manager.ConfigManagerError):
        config_manager.ControllerProfile.from_bytes(payload)


def test_looping_macro_requires_positive_total_duration() -> None:
    obj = custom_profile().to_json_object()
    obj["macros"][0]["steps"][0]["duration_ms"] = 0
    once = config_manager.ControllerProfile.from_json_object(obj)
    assert config_manager.ControllerProfile.from_bytes(once.to_bytes()) == once
    for playback in ("while_held", "toggle", "repeat"):
        obj["macros"][0]["playback"] = playback
        with pytest.raises(config_manager.ConfigManagerError):
            config_manager.ControllerProfile.from_json_object(obj)
    obj["macros"][0]["trigger"] = []
    disabled = config_manager.ControllerProfile.from_json_object(obj)
    assert config_manager.ControllerProfile.from_bytes(disabled.to_bytes()) == disabled


def test_sparse_macro_capacity_boundaries() -> None:
    full_step = custom_profile().macros[0].steps[0].to_json_object()
    button_step = custom_profile().macros[1].steps[0].to_json_object()

    exact = config_manager.ControllerProfile.default().to_json_object()
    exact["macros"][0]["trigger"] = ["south"]
    exact["macros"][0]["steps"] = [full_step] * 8
    exact_profile = config_manager.ControllerProfile.from_json_object(exact)
    assert (
        sum(
            len(step.to_sparse_bytes())
            for macro in exact_profile.macros
            for step in macro.steps
        )
        == config_manager.PROFILE_MACRO_STREAM_SIZE
    )
    assert (
        config_manager.ControllerProfile.from_bytes(exact_profile.to_bytes())
        == exact_profile
    )

    sixteen = config_manager.ControllerProfile.default().to_json_object()
    for index, macro in enumerate(sixteen["macros"]):
        macro["trigger"] = [config_manager.LOGICAL_CONTROLS[index]]
        macro["steps"] = [button_step] * 4
    sixteen_profile = config_manager.ControllerProfile.from_json_object(sixteen)
    assert sum(len(macro.steps) for macro in sixteen_profile.macros) == 16
    assert len(sixteen_profile.to_bytes()) == config_manager.PROFILE_SIZE

    overflow = config_manager.ControllerProfile.default().to_json_object()
    overflow["macros"][0]["trigger"] = ["south"]
    overflow["macros"][0]["steps"] = [full_step] * 8
    overflow["macros"][1]["trigger"] = ["east"]
    overflow["macros"][1]["steps"] = [full_step]
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="136-byte sparse stream",
    ):
        config_manager.ControllerProfile.from_json_object(overflow)


def test_trigger_threshold_uses_transformed_output_domain() -> None:
    for threshold in (0, 0xFFFF):
        trigger = config_manager.TriggerConfig(30000, 40000, 256, threshold, 16)
        assert (
            config_manager.TriggerConfig.from_bytes(
                trigger.to_bytes(),
                schema_version=config_manager.PROFILE_SCHEMA_VERSION,
                source_index=0,
            )
            == trigger
        )
        assert (
            config_manager.TriggerConfig.from_json_object(
                trigger.to_json_object(),
                "trigger",
                schema_version=config_manager.PROFILE_SCHEMA_VERSION,
                source_index=0,
            )
            == trigger
        )

        current_wire = bytearray(config_manager.ControllerProfile.default().to_bytes())
        struct.pack_into("<HHHH", current_wire, 52, 30000, 40000, 256, threshold)
        current_profile = config_manager.ControllerProfile.from_bytes(current_wire)
        assert current_profile.left_trigger.digital_threshold == threshold
        assert current_profile.to_bytes() == current_wire

    for threshold in (-1, 0x10000):
        with pytest.raises(
            config_manager.ConfigManagerError,
            match="trigger digital_threshold",
        ):
            config_manager.TriggerConfig(30000, 40000, 256, threshold, 16)

    for lower_deadzone, upper_saturation in (
        (40000, 40000),
        (40001, 40000),
    ):
        with pytest.raises(
            config_manager.ConfigManagerError,
            match="lower_deadzone must be below upper_saturation",
        ):
            config_manager.TriggerConfig(lower_deadzone, upper_saturation, 256, 0, 16)


def test_default_timeout_covers_batched_profile_commit() -> None:
    args = config_manager.build_parser().parse_args(["profiles", "list"])
    assert args.timeout == config_manager.DEFAULT_OPERATION_TIMEOUT_SECONDS == 15.0


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
    assert max(device.profile_chunk_sizes) <= config_manager.MAXIMUM_CHUNK_SIZE
    assert sum(device.profile_chunk_sizes) == config_manager.PROFILE_SIZE
    assert config_manager.OP_PROFILE_TRANSACTION_STATUS in device.requests
    assert config_manager.read_profile(device, device.stable_identity, 2) == profile


def test_profile_metadata_and_identify_round_trip() -> None:
    device = FakeDevice()
    identity = device.stable_identity
    alias_status = config_manager.set_profile_metadata(
        device,
        identity,
        config_manager.PROFILE_NONE_BUTTON,
        "Desk pad",
        1.0,
    )
    name_status = config_manager.set_profile_metadata(
        device,
        identity,
        7,
        "Desktop",
        1.0,
    )
    metadata = config_manager.read_profile_metadata(device, identity, 7)
    assert alias_status.status == config_manager.STATUS_OK
    assert name_status.stored_generation == alias_status.stored_generation + 1
    assert metadata.alias == "Desk pad"
    assert metadata.profile_names[7] == "Desktop"
    assert config_manager.list_profiles(device)[1].alias == "Desk pad"

    config_manager.identify_controller(device, identity)
    assert device.identified_identities == [identity.to_bytes()]
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="no controller to identify",
    ):
        config_manager.identify_controller(device, device.global_identity)


def test_profile_playtest_decodes_raw_controller_state() -> None:
    device = FakeDevice()
    playtest = config_manager.read_profile_playtest(device)
    assert playtest == config_manager.ProfilePlaytest(
        connected=True,
        slot_index=1,
        connection_generation=17,
        state_generation=93,
        identity=device.stable_identity,
        button_mask=0x9001,
        left_stick=(-1234, 2345),
        right_stick=(-30000, 30000),
        triggers=(123, 65000),
        battery=251,
        capabilities=0x0F,
        motion=(1, -2, 3, -4, 5, -6),
    )
    assert playtest.to_json_object()["buttons"] == [
        "south",
        "dpad_up",
        "dpad_right",
    ]

    device.playtest_connected = False
    disconnected = config_manager.read_profile_playtest(device)
    assert disconnected == config_manager.ProfilePlaytest(
        connected=False,
        slot_index=None,
        connection_generation=0,
        state_generation=0,
        identity=None,
        button_mask=0,
        left_stick=(0, 0),
        right_stick=(0, 0),
        triggers=(0, 0),
        battery=0,
        capabilities=0,
        motion=None,
    )
    device.playtest_connected = True
    payload, flags = device._profile_playtest_payload()
    malformed = bytearray(payload)
    malformed[38] = 0
    envelope = config_manager.parse_response(
        make_response(
            config_manager.OP_PROFILE_PLAYTEST,
            malformed,
            flags=flags,
            schema=config_manager.PROFILE_PLAYTEST_SCHEMA_VERSION,
        ),
        config_manager.OP_PROFILE_PLAYTEST,
    )
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="invalid connected playtest payload",
    ):
        config_manager.parse_profile_playtest(envelope)


def test_playtest_layout_and_extra_inputs_preserve_legacy_firmware() -> None:
    device = FakeDevice()
    device.stable_identity = replace(
        device.stable_identity, vendor_id=0x057E, product_id=0x2067
    )
    device.playtest_extra_buttons = 0x55
    device.playtest_layout = 3
    payload, flags = device._profile_playtest_payload()

    def parse(data: bytes, schema: int) -> config_manager.ProfilePlaytest:
        return config_manager.parse_profile_playtest(
            config_manager.parse_response(
                make_response(
                    config_manager.OP_PROFILE_PLAYTEST, data, flags=flags, schema=schema
                ),
                config_manager.OP_PROFILE_PLAYTEST,
            )
        )

    current = parse(payload, 4)
    assert current.to_json_object()["layout"] == "joycon2-pair"
    assert current.to_json_object()["extra_buttons"] == [
        "c",
        "gr",
        "left_sr",
        "right_sr",
    ]
    assert current.to_json_object()["buttons"] == ["south", "dpad_up", "dpad_right"]
    assert parse(payload[:55], 3) == replace(current, layout=None)
    assert parse(payload[:54], 2) == replace(current, extra_buttons=0, layout=None)
    assert parse(payload, 5) == current
    assert parse(payload[:55] + b"\x06", 5).layout == "wii-horizontal"
    assert parse(payload[:55] + b"\x07", 5).layout == "wii-vertical"
    assert parse(payload[:55] + b"\x00", 4).layout is None
    for data, schema in (
        (payload[:54] + b"\x80", 3),
        (payload[:54] + b"\x80\x03", 4),
        (payload[:55] + b"\x06", 4),
        (payload[:55] + b"\xff", 4),
        (payload, 2),
        (payload, 3),
        (payload[:55], 4),
        (payload + b"\x00", 4),
        (payload[:55] + b"\x08", 5),
        (payload[:55], 5),
        (payload, 6),
    ):
        with pytest.raises(config_manager.ConfigManagerError):
            parse(data, schema)

    device.playtest_connected = False
    disconnected, flags = device._profile_playtest_payload()
    assert parse(disconnected, 4).layout is None
    with pytest.raises(config_manager.ConfigManagerError):
        parse(disconnected[:55] + b"\x03", 4)


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

    assert config_manager.main(["profiles", "activate", "4", "--identity", "1"]) == 1

    output = capsys.readouterr()
    assert output.out == ""
    assert "storage failure" in output.err
    assert device.active_profiles[device.stable_identity.to_bytes()] == previous_active
    assert device.profile_status_responses == [
        (device.profile_transaction_id, config_manager.STATUS_PENDING),
        (device.profile_transaction_id, 8),
    ]


def test_profile_edit_cli_starts_local_web_editor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from switch_pico_bridge import profile_web

    calls: list[dict[str, object]] = []
    monkeypatch.setattr(
        profile_web,
        "run_profile_editor",
        lambda **kwargs: calls.append(kwargs),
    )
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: (_ for _ in ()).throw(
            AssertionError("editor startup must not require a connected Pico")
        ),
    )

    assert (
        config_manager.main(
            [
                "--bus",
                "3",
                "--address",
                "7",
                "--timeout",
                "8",
                "profiles",
                "edit",
                "--port",
                "9000",
                "--no-browser",
            ]
        )
        == 0
    )
    assert calls == [
        {
            "bus": 3,
            "address": 7,
            "timeout": 8.0,
            "port": 9000,
            "open_browser": False,
        }
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

    assert config_manager.main(["profiles", "activate", "8", "--identity", "1"]) == 0
    assert device.active_profiles[device.stable_identity.to_bytes()] == 7
    _ = capsys.readouterr()

    before_reset_requests = len(device.requests)
    assert config_manager.main(["profiles", "reset", "2", "--identity", "1"]) == 2
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
    malformed_binary[75] = 0xC0
    with pytest.raises(config_manager.ConfigManagerError, match="action flags"):
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
        assert config_manager.main(["profiles", "import", "1", str(path)]) == 1
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
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [bounded_device])
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
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))

    assert config_manager.main(["config", "set", "--pairing-window-seconds", "90"]) == 0
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
            config_manager.CAPABILITY_INPUT | config_manager.CAPABILITY_RUMBLE,
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
    reenumerated.configuration = struct.pack("<HB5x", 60, requested_mode)
    reenumerated.active_mode = active_mode
    reenumerated.capabilities = capabilities
    scans = iter(((previous,), (previous,), (), (reenumerated,)))
    monkeypatch.setattr(
        config_manager,
        "_candidate_devices",
        lambda: next(scans, (reenumerated,)),
    )
    monkeypatch.setattr(config_manager.time, "sleep", lambda _seconds: None)

    assert config_manager.main(["mode", mode_name]) == 0
    assert capsys.readouterr().out == f"USB mode changed to {mode_name}.\n"
    assert previous.reboot_transaction_ids == [previous.transaction_id]

    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (reenumerated,))
    assert config_manager.main(["mode", mode_name]) == 0
    assert capsys.readouterr().out == f"USB mode is already {mode_name}.\n"
    assert reenumerated.out_requests == []


def test_bootsel_reboot_cli(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = FakeDevice()
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: (device,))

    assert config_manager.main(["reboot", "bootsel"]) == 0
    assert capsys.readouterr().out == "Rebooting into USB BOOTSEL mode.\n"
    assert device.bootsel_reboot_requested


def test_mode_parser_accepts_all_implemented_modes() -> None:
    for mode in config_manager.REQUESTED_MODE_NAMES:
        args = config_manager.build_parser().parse_args(["mode", mode])
        assert args.mode == mode


@pytest.mark.parametrize(
    "identity",
    [
        (0x057E, 0x2009),
        (0x057E, 0x2068),
        (0xCAFE, 0x4010),
        (0xCAFE, 0x4020),
        (0xCAFE, 0x4021),
    ],
)
def test_discovery_finds_one_adapter_without_its_native_children(
    monkeypatch: pytest.MonkeyPatch,
    identity: tuple[int, int],
) -> None:
    device = FakeDevice()
    if identity == (0x057E, 0x2068):
        device.firmware_version = (0, 72, 0)
        device.active_mode = 5
    right = FakeDevice()
    right.address = 8
    left = FakeDevice()
    left.address = 9
    devices = {
        identity: (device,),
        (0x057E, 0x2066): (right,),
        (0x057E, 0x2067): (left,),
    }

    def find(**arguments: object) -> tuple[FakeDevice, ...]:
        return devices.get(
            (int(arguments["idVendor"]), int(arguments["idProduct"])), ()
        )

    monkeypatch.setattr(config_manager.usb.core, "find", find)
    assert config_manager.find_pico(None, None, timeout=0) is device
    for child in (right, left):
        with pytest.raises(config_manager.ConfigManagerError, match="no USB-connected"):
            config_manager.find_pico(child.bus, child.address, timeout=0)


@pytest.mark.parametrize(
    "response",
    [
        config_manager.usb.core.USBError("management request stalled", errno=32),
        b"Nintendo",
        make_response(config_manager.OP_INFO, b""),
        make_response(config_manager.OP_INFO, bytes((0, 72, 0, 2, 5, 0x80, 0, 2))),
    ],
)
def test_native_discovery_requires_validated_management_info(
    monkeypatch: pytest.MonkeyPatch,
    response: bytes | Exception,
) -> None:
    class NintendoDevice(FakeDevice):
        address = 8

        def ctrl_transfer(self, *args: object, **kwargs: object) -> bytes:
            if isinstance(response, Exception):
                raise response
            return response

    nintendo = NintendoDevice()
    hub = FakeDevice()
    hub.active_mode = 5

    def find(**arguments: object) -> tuple[FakeDevice, ...]:
        if (arguments["idVendor"], arguments["idProduct"]) == (0x057E, 0x2068):
            return nintendo, hub
        return ()

    monkeypatch.setattr(config_manager.usb.core, "find", find)
    assert config_manager.find_pico(None, None, timeout=0) is hub
    with pytest.raises(config_manager.ConfigManagerError, match="none accepted"):
        config_manager.find_pico(nintendo.bus, nintendo.address, timeout=0)


def test_find_requires_selector_for_multiple_picos(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first = FakeDevice()
    second = FakeDevice()
    second.address = 8
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [first, second])
    with pytest.raises(
        config_manager.ConfigManagerError,
        match="multiple switch-pico devices",
    ):
        config_manager.find_pico(None, None)
    assert config_manager.find_pico(1, 8) is second


def haptics_response(
    state: int = 0,
    *,
    run_id: int = 0,
    slot: int = 0xFF,
    last_error: int = 0,
    sent_packets: int = 0,
    first_tone_due_us: int = 0,
    first_tone_sent_us: int = 0,
    elapsed_us: int = 0,
    connection_generation: int = 9,
    mode: int = 0,
    host_updates: int = 0,
    dropped_updates: int = 0,
    packet_frames: int = 64,
    last_packet_nonzero: bool = False,
) -> bytes:
    return make_response(
        config_manager.OP_HAPTICS_EXPERIMENT,
        struct.pack(
            "<17I8B2I",
            run_id,
            connection_generation,
            100,
            105,
            sent_packets,
            2,
            3,
            109,
            4,
            123,
            22000,
            11001,
            9876,
            first_tone_due_us,
            first_tone_sent_us,
            0x76543210,
            elapsed_us,
            state,
            slot,
            last_error,
            0,
            mode,
            packet_frames,
            int(last_packet_nonzero),
            0,
            host_updates,
            dropped_updates,
        ),
        schema=5,
        generation=run_id,
    )


class HapticsDevice(FakeDevice):
    def __init__(
        self,
        responses: list[bytes | Exception],
        *,
        transport_response: bytes | Exception | None = None,
    ) -> None:
        super().__init__()
        self.haptics_responses = responses
        self.transport_response = transport_response

    def ctrl_transfer(
        self,
        bm_request_type: int,
        request: int,
        value: int,
        index: int,
        data_or_w_length: object,
        timeout: int,
    ) -> bytes | int:
        if request == config_manager.OP_HAPTICS_TRANSPORT_PROBE:
            assert bm_request_type == 0xC0
            assert value == config_manager.REQUEST_VALUE
            assert index == config_manager.REQUEST_INDEX
            self.requests.append(request)
            assert self.transport_response is not None
            if isinstance(self.transport_response, Exception):
                raise self.transport_response
            return self.transport_response
        if request != config_manager.OP_HAPTICS_EXPERIMENT:
            return super().ctrl_transfer(
                bm_request_type, request, value, index, data_or_w_length, timeout
            )
        assert value == config_manager.REQUEST_VALUE
        assert index == config_manager.REQUEST_INDEX
        self.requests.append(request)
        if bm_request_type == 0xC0:
            response = self.haptics_responses[0]
            if len(self.haptics_responses) > 1:
                self.haptics_responses.pop(0)
            if isinstance(response, Exception):
                raise response
            return response
        assert bm_request_type == 0x40
        encoded = bytes(data_or_w_length)
        magic, version, operation, flags, reserved, size, schema, crc = (
            struct.unpack_from("<4sBBBBHHI", encoded)
        )
        payload = encoded[16:]
        assert (magic, version, operation, flags, reserved, size, schema) == (
            b"SPMG",
            1,
            0x40,
            0,
            0,
            2,
            0,
        )
        assert crc == zlib.crc32(payload) & 0xFFFFFFFF
        self.out_requests.append((request, payload, encoded))
        return len(encoded)


@pytest.fixture
def haptics_clock(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    clock = [0.0]

    def sleep(seconds: float) -> None:
        clock[0] += seconds

    monkeypatch.setattr(config_manager.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(config_manager.time, "sleep", sleep)
    return clock


def test_haptics_schema_timing_and_wraparound() -> None:
    device = HapticsDevice(
        [
            haptics_response(
                2,
                run_id=17,
                slot=2,
                sent_packets=101,
                first_tone_due_us=0xFFFFFFF0,
                first_tone_sent_us=0x30,
                elapsed_us=1100000,
                mode=1,
                host_updates=0x89ABCDEF,
                dropped_updates=0x12345678,
            ),
        ]
    )
    snapshot = config_manager.read_haptics_experiment(device)
    assert snapshot.state_name == "running"
    assert snapshot.run_id == 17 and snapshot.slot == 2
    assert snapshot.sent_packets == 101
    assert snapshot.generated_packets == 105 and snapshot.skipped_packets == 2
    assert snapshot.send_failures == 3 and snapshot.can_send_requests == 109
    assert snapshot.synchronous_callbacks == 4
    assert snapshot.max_generate_us == 123
    assert snapshot.max_send_gap_us == 22000
    assert snapshot.max_lateness_us == 11001
    assert snapshot.max_request_wait_us == 9876
    assert snapshot.first_tone_submission_delay_us == 64
    assert snapshot.last_sent_us == 0x76543210
    assert snapshot.elapsed_us == 1100000
    assert snapshot.to_json_object()["first_tone_submission_delay_us"] == 64
    assert snapshot.mode_name == "gameplay"
    assert snapshot.host_updates == 0x89ABCDEF
    assert snapshot.dropped_updates == 0x12345678


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("schema", "unsupported haptics experiment schema"),
        ("short", "payload size"),
        ("long", "payload size"),
        ("old_size", "payload size"),
        ("mode_only_size", "payload size"),
        ("state", "state"),
        ("slot", "slot"),
        ("active_without_slot", "slot"),
        ("mode", "mode"),
        ("reserved71", "reserved"),
        ("packet_size", "packet size"),
        ("nonzero_flag", "nonzero flag"),
        ("reserved75", "reserved"),
        ("flags", "reserved"),
    ],
)
def test_haptics_rejects_malformed_diagnostics(mutation: str, message: str) -> None:
    payload = bytearray(haptics_response(2, slot=0)[20:])
    schema, flags = 5, 0
    if mutation == "schema":
        schema = 2
        del payload[72:]
    elif mutation == "short":
        payload.pop()
    elif mutation == "long":
        payload.append(0)
    elif mutation == "old_size":
        del payload[72:]
    elif mutation == "mode_only_size":
        del payload[76:]
    elif mutation == "state":
        payload[68] = 8
    elif mutation == "slot":
        payload[69] = 4
    elif mutation == "active_without_slot":
        payload[69] = 0xFF
    elif mutation == "mode":
        payload[72] = 2
    elif mutation == "packet_size":
        payload[73] = 1
    elif mutation == "nonzero_flag":
        payload[74] = 2
    elif mutation.startswith("reserved"):
        payload[int(mutation.removeprefix("reserved"))] = 1
    else:
        flags = 1
    response = make_response(
        0x40,
        bytes(payload),
        schema=schema,
        flags=flags,
    )
    with pytest.raises(config_manager.ConfigManagerError, match=message):
        config_manager.read_haptics_experiment(HapticsDevice([response]))


def test_haptics_disabled_firmware_is_readable_but_cannot_start(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice([haptics_response(6)])
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "status", "--json"]) == 0
    captured = capsys.readouterr()
    status = json.loads(captured.out)
    assert status["state_name"] == "unsupported"
    assert status["firmware_supported"] is False
    assert status["mode"] == 0
    assert "SWITCH_PICO_HAPTICS_EXPERIMENT=ON" in captured.err
    assert config_manager.main(["haptics-experiment", "start"]) == 1
    assert "unsupported" in capsys.readouterr().err
    assert config_manager.main(["haptics-experiment", "gameplay"]) == 1
    assert "unsupported" in capsys.readouterr().err
    assert device.out_requests == []


def test_haptics_old_firmware_stall_is_actionable_without_hiding_disconnect(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice(
        [
            config_manager.usb.core.USBError("Pipe error", error_code=-9, errno=32),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "start"]) == 1
    assert "SWITCH_PICO_HAPTICS_EXPERIMENT=ON" in capsys.readouterr().err
    assert device.out_requests == []
    disconnected = config_manager.usb.core.USBError(
        "No such device",
        error_code=-4,
        errno=19,
    )
    with pytest.raises(config_manager.usb.core.USBError) as raised:
        config_manager.read_haptics_experiment(HapticsDevice([disconnected]))
    assert raised.value is disconnected


@pytest.mark.parametrize(("action", "mode"), [("start", 0), ("gameplay", 1)])
def test_haptics_arming_waits_for_firmware_not_usb_ack(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
    action: str,
    mode: int,
) -> None:
    device = HapticsDevice(
        [
            haptics_response(3, run_id=40, slot=0),
            haptics_response(1, run_id=41, slot=0, mode=mode),
            haptics_response(2, run_id=41, slot=0, sent_packets=1, mode=mode),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", action, "--json"]) == 0
    captured = capsys.readouterr()
    row = json.loads(captured.out)
    assert row["state_name"] == "running"
    assert (row["run_id"], row["slot"], row["mode"]) == (41, 0, mode)
    assert device.out_requests[0][1] == bytes((1 if action == "start" else 2, 0))
    assert haptics_clock[0] >= 0.1
    if mode == 1:
        assert "pattern" not in row
        assert row["first_tone_submission_delay_us"] is None
    else:
        assert row["pattern"]["duration_us"] == 6144000
        assert "gameplay" not in row


def test_haptics_start_watch_captures_correlated_measurement_series(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(),
            haptics_response(1, run_id=1, slot=3),
            haptics_response(
                2,
                run_id=1,
                slot=3,
                sent_packets=101,
                first_tone_due_us=0xFFFFFFF0,
                first_tone_sent_us=0x30,
                elapsed_us=1100000,
            ),
            haptics_response(3, run_id=1, slot=3, sent_packets=574, elapsed_us=6144000),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "haptics-experiment",
                "start",
                "--slot",
                "3",
                "--watch",
                "--json",
            ]
        )
        == 0
    )
    captured = capsys.readouterr()
    series = [json.loads(line) for line in captured.out.splitlines()]
    assert [row["state_name"] for row in series] == [
        "pending",
        "running",
        "completed",
    ]
    assert [row["sent_packets"] for row in series] == [0, 101, 574]
    assert series[1]["first_tone_submission_delay_us"] == 64
    assert series[-1]["elapsed_us"] == 6144000
    assert series[-1]["pattern"]["duration_us"] == 6144000
    assert series[0]["host_monotonic_s"] < series[-1]["host_monotonic_s"]
    assert "pending firmware confirmation" in captured.err


@pytest.mark.parametrize(
    ("state", "error", "description"),
    [
        (6, 1, "unsupported"),
        (6, 2, "MTU"),
        (5, 3, "connection"),
        (7, 4, "timed out"),
        (7, 5, "send failed"),
        (7, 6, "wait for prior output to drain"),
    ],
)
def test_haptics_gameplay_reports_asynchronous_rejection(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
    state: int,
    error: int,
    description: str,
) -> None:
    device = HapticsDevice(
        [
            haptics_response(),
            haptics_response(1, run_id=1, slot=0, mode=1),
            haptics_response(state, run_id=1, slot=0, last_error=error, mode=1),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "gameplay", "--json"]) == 1
    captured = capsys.readouterr()
    assert description in captured.err
    assert f"last_error={error}" in captured.err
    row = json.loads(captured.out)
    assert (row["run_id"], row["slot"], row["mode"]) == (1, 0, 1)
    assert row["state"] == state and row["last_error"] == error
    assert "pattern" not in row


def test_haptics_status_watch_reports_connection_loss_after_running(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(2, run_id=7, slot=0, sent_packets=11),
            haptics_response(5, run_id=7, slot=0, sent_packets=13, last_error=3),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "haptics-experiment",
                "status",
                "--watch",
                "--json",
            ]
        )
        == 1
    )
    captured = capsys.readouterr()
    assert [json.loads(line)["state_name"] for line in captured.out.splitlines()] == [
        "running",
        "disconnected",
    ]
    assert "connection missing or lost" in captured.err


def test_haptics_start_does_not_mistake_stale_completion_for_new_run(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice([haptics_response(3, run_id=8, slot=0)])
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "--timeout",
                "0.2",
                "haptics-experiment",
                "start",
            ]
        )
        == 1
    )
    assert "not confirmed before --timeout" in capsys.readouterr().err
    assert haptics_clock[0] == pytest.approx(0.2)


def test_haptics_watch_is_bounded_and_rejects_run_replacement(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice([haptics_response(2, run_id=1, slot=0)])
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "--timeout",
                "0.2",
                "haptics-experiment",
                "status",
                "--watch",
            ]
        )
        == 1
    )
    assert "did not reach a terminal state" in capsys.readouterr().err
    device.haptics_responses = [
        haptics_response(2, run_id=1, slot=0),
        haptics_response(3, run_id=2, slot=0),
    ]
    assert (
        config_manager.main(
            [
                "haptics-experiment",
                "status",
                "--watch",
            ]
        )
        == 1
    )
    assert "run changed" in capsys.readouterr().err


def test_haptics_busy_start_and_wrong_slot_stop_do_not_mutate_active_run(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice([haptics_response(2, run_id=1, slot=3, mode=1)])
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "start"]) == 1
    assert "already running" in capsys.readouterr().err
    assert config_manager.main(["haptics-experiment", "gameplay"]) == 1
    assert "already running" in capsys.readouterr().err
    assert config_manager.main(["haptics-experiment", "stop"]) == 1
    assert "not requested slot 0" in capsys.readouterr().err
    assert device.out_requests == []


@pytest.mark.parametrize("mode", [0, 1])
def test_haptics_stop_waits_for_service_completion(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
    mode: int,
) -> None:
    device = HapticsDevice(
        [
            haptics_response(2, run_id=1, slot=2, mode=mode),
            haptics_response(2, run_id=1, slot=2, mode=mode),
            haptics_response(4, run_id=1, slot=2, mode=mode),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "haptics-experiment",
                "stop",
                "--slot",
                "2",
                "--json",
            ]
        )
        == 0
    )
    captured = capsys.readouterr()
    row = json.loads(captured.out)
    assert row["state_name"] == "stopped"
    assert (row["run_id"], row["slot"], row["mode"]) == (1, 2, mode)
    assert device.out_requests[0][1] == b"\x00\x02"
    assert haptics_clock[0] >= 0.1


def test_haptics_invalid_slot_is_rejected_before_discovery(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def unexpected_discovery(*args: object) -> None:
        pytest.fail("invalid haptics slot reached USB discovery")

    monkeypatch.setattr(config_manager, "find_pico", unexpected_discovery)
    with pytest.raises(SystemExit) as raised:
        config_manager.main(["haptics-experiment", "start", "--slot", "4"])
    assert raised.value.code == 2


def test_haptics_retry_after_unsupported_controller_is_not_disabled_firmware(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(6, run_id=7, slot=0, last_error=1),
            haptics_response(2, run_id=8, slot=0),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "start", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["run_id"] == 8
    assert len(device.out_requests) == 1


def test_haptics_start_correlates_rollover_and_rejects_superseded_run(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(3, run_id=0xFFFFFFFF, slot=0),
            haptics_response(2, run_id=0, slot=0),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "start", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["run_id"] == 0
    device.haptics_responses = [
        haptics_response(3, run_id=10, slot=0),
        haptics_response(3, run_id=12, slot=0),
    ]
    assert config_manager.main(["haptics-experiment", "start"]) == 1
    assert "run changed" in capsys.readouterr().err


@pytest.mark.parametrize("timeout", ["nan", "inf"])
def test_haptics_timeout_must_be_bounded_before_discovery(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    timeout: str,
) -> None:
    def unexpected_discovery(*args: object) -> None:
        pytest.fail("unbounded timeout reached USB discovery")

    monkeypatch.setattr(config_manager, "find_pico", unexpected_discovery)
    assert (
        config_manager.main(
            [
                "--timeout",
                timeout,
                "haptics-experiment",
                "status",
                "--watch",
            ]
        )
        == 2
    )
    assert "must be finite" in capsys.readouterr().err


def transport_response(
    *,
    run_id: int = 17,
    connection_generation: int = 9,
    connection_handle: int = 0x1234,
    active: int = 0,
) -> bytes:
    return make_response(
        config_manager.OP_HAPTICS_TRANSPORT_PROBE,
        struct.pack(
            "<38Ii5I",
            run_id,
            connection_generation,
            connection_handle,
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
            14,
            15,
            16,
            17,
            18,
            19,
            20,
            21,
            22,
            23,
            24,
            0xFFFFFFF0,
            active,
            27,
            0xFFFFFFFF,
            29,
            30,
            1021,
            10,
            400000,
            399998,
            48000,
            1300,
            6,
            1366,
            -1250,
            100,
            20,
            3,
            0,
            1,
        ),
        schema=3,
        generation=run_id,
    )


def test_haptics_profile_decodes_exact_wire_order_and_correlates_live_run(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(2, run_id=17, slot=0, sent_packets=10, mode=1),
            haptics_response(2, run_id=17, slot=0, sent_packets=11, mode=1),
        ],
        transport_response=transport_response(active=1),
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "profile", "--json"]) == 0
    row = json.loads(capsys.readouterr().out)
    assert row["run_id"] == 17 and row["connection_generation"] == 9
    assert row["sent_packets"] == 11
    assert row["mode_name"] == "gameplay" and "pattern" not in row
    transport = row["transport"]
    transport.pop("evidence_note")
    assert transport == {
        "schema_version": 3,
        "run_id": 17,
        "connection_generation": 9,
        "connection_handle": 0x1234,
        "timer_wakes": 4,
        "max_timer_lateness_us": 5,
        "total_timer_lateness_us": 6,
        "send_calls": 7,
        "max_send_us": 8,
        "total_send_us": 9,
        "write_calls": 10,
        "max_write_us": 11,
        "total_write_us": 12,
        "read_calls": 13,
        "read_packets": 14,
        "max_read_us": 15,
        "total_read_us": 16,
        "poll_calls": 17,
        "max_poll_us": 18,
        "total_poll_us": 19,
        "completion_events": 20,
        "completed_packets": 21,
        "max_completion_gap_us": 22,
        "max_outstanding_acl": 23,
        "min_free_acl": 24,
        "first_tone_send_return_us": 0xFFFFFFF0,
        "active": True,
        "max_permission_wait_us": 27,
        "total_permission_wait_us": 0xFFFFFFFF,
        "permission_callbacks": 29,
        "max_poll_gap_us": 30,
        "controller_acl_packet_bytes": 1021,
        "controller_acl_packet_count": 10,
        "requested_sys_khz": 400000,
        "measured_sys_khz": 399998,
        "measured_usb_khz": 48000,
        "core_voltage_mv": 1300,
        "flash_clock_divider": 6,
        "cyw43_pio_divider256": 1366,
        "temperature_millicelsius": -1250,
        "host_completed_writes": 100,
        "acl_writes": 20,
        "other_writes": 3,
        "write_failures": 0,
        "packet_read_optimized": 1,
    }
    assert transport["active"] is True
    assert device.out_requests == []


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("schema", "unsupported haptics transport probe schema"),
        ("short", "payload size"),
        ("long", "payload size"),
        ("flags", "reserved flags"),
        ("active", "active boolean"),
        ("handle", "connection handle"),
        ("envelope_run", "envelope run ID mismatch"),
        ("status", "device busy"),
        ("crc", "CRC mismatch"),
    ],
)
def test_haptics_profile_rejects_malformed_transport(
    mutation: str,
    message: str,
) -> None:
    payload = bytearray(transport_response()[20:])
    schema, flags, generation, status = 3, 0, 17, config_manager.STATUS_OK
    if mutation == "schema":
        schema = 1
    elif mutation == "short":
        payload.pop()
    elif mutation == "long":
        payload.extend(b"\0\0\0\0")
    elif mutation == "flags":
        flags = 1
    elif mutation == "active":
        struct.pack_into("<I", payload, 100, 2)
    elif mutation == "handle":
        struct.pack_into("<I", payload, 8, 0x10000)
    elif mutation == "envelope_run":
        generation = 18
    elif mutation == "status":
        status = 7
    response = make_response(
        0x41,
        bytes(payload),
        schema=schema,
        flags=flags,
        generation=generation,
        status=status,
    )
    if mutation == "crc":
        response = response[:-1] + bytes((response[-1] ^ 1,))
    device = HapticsDevice(
        [haptics_response(3, run_id=17, slot=0)],
        transport_response=response,
    )
    with pytest.raises(config_manager.ConfigManagerError, match=message):
        config_manager.read_haptics_experiment_profile(device)


def test_haptics_profile_accepts_retained_failed_run_with_invalid_handle(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    device = HapticsDevice(
        [haptics_response(5, run_id=17, slot=0, last_error=3, mode=1)],
        transport_response=transport_response(connection_handle=0xFFFF),
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "profile", "--json"]) == 0
    row = json.loads(capsys.readouterr().out)
    assert row["state_name"] == "disconnected" and row["last_error"] == 3
    assert row["mode"] == 1 and "pattern" not in row
    assert row["transport"]["connection_handle"] == 0xFFFF
    assert row["transport"]["active"] is False


@pytest.mark.parametrize("unsupported", ["status", "stall"])
def test_haptics_profile_unsupported_keeps_legacy_status_readable(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    unsupported: str,
) -> None:
    response: bytes | Exception = make_response(
        0x41,
        status=config_manager.STATUS_UNSUPPORTED_SCHEMA,
        schema=1,
    )
    if unsupported == "stall":
        response = config_manager.usb.core.USBError(
            "Pipe error",
            error_code=-9,
            errno=32,
        )
    device = HapticsDevice(
        [haptics_response(3, run_id=17, slot=0)],
        transport_response=response,
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "profile", "--json"]) == 1
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "0x41" in captured.err
    assert "SWITCH_PICO_HAPTICS_EXPERIMENT=ON" in captured.err
    assert config_manager.main(["haptics-experiment", "status", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["run_id"] == 17
    assert device.out_requests == []


def test_haptics_profile_does_not_hide_usb_disconnect_as_unsupported() -> None:
    disconnected = config_manager.usb.core.USBError(
        "No such device",
        error_code=-4,
        errno=19,
    )
    device = HapticsDevice(
        [haptics_response(3, run_id=17, slot=0)],
        transport_response=disconnected,
    )
    with pytest.raises(config_manager.usb.core.USBError) as raised:
        config_manager.read_haptics_experiment_profile(device)
    assert raised.value is disconnected


@pytest.mark.parametrize(
    (
        "after_run",
        "after_generation",
        "probe_run",
        "probe_generation",
        "after_slot",
        "after_mode",
    ),
    [
        (17, 9, 16, 9, 0, 1),  # A stale probe cannot attach to the current run.
        (18, 9, 17, 9, 0, 1),  # A new run starts after reading the probe.
        (18, 9, 18, 9, 0, 1),  # A new run starts before reading the probe.
        (17, 10, 17, 9, 0, 1),  # A connection changes after reading the probe.
        (17, 10, 17, 10, 0, 1),  # A connection changes before reading the probe.
        (17, 9, 17, 8, 0, 1),  # A matching run ID cannot mask stale connection data.
        (17, 9, 17, 9, 1, 1),  # A different slot must not inherit the profile.
        (17, 9, 17, 9, 0, 0),  # A fixture cannot impersonate gameplay.
    ],
)
def test_haptics_profile_never_publishes_cross_run_metrics(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    after_run: int,
    after_generation: int,
    probe_run: int,
    probe_generation: int,
    after_slot: int,
    after_mode: int,
) -> None:
    device = HapticsDevice(
        [
            haptics_response(2, run_id=17, slot=0, mode=1),
            haptics_response(
                2,
                run_id=after_run,
                slot=after_slot,
                connection_generation=after_generation,
                mode=after_mode,
            ),
        ],
        transport_response=transport_response(
            run_id=probe_run,
            connection_generation=probe_generation,
        ),
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", "profile", "--json"]) == 1
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "cannot attribute measurements" in captured.err


def test_haptics_gameplay_watch_timeout_leaves_stream_armed(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(3, run_id=8, slot=0),
            haptics_response(1, run_id=9, slot=3, mode=1),
            haptics_response(
                2,
                run_id=9,
                slot=3,
                mode=1,
                sent_packets=2,
                host_updates=4,
                dropped_updates=1,
            ),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "--timeout",
                "0.2",
                "haptics-experiment",
                "gameplay",
                "--slot",
                "3",
                "--watch",
                "--json",
            ]
        )
        == 1
    )
    captured = capsys.readouterr()
    series = [json.loads(line) for line in captured.out.splitlines()]
    assert [row["state_name"] for row in series] == ["pending", "running", "running"]
    assert all((row["run_id"], row["slot"], row["mode"]) == (9, 3, 1) for row in series)
    assert all("pattern" not in row for row in series)
    assert all(row["first_tone_submission_delay_us"] is None for row in series)
    assert series[-1]["host_updates"] == 4 and series[-1]["dropped_updates"] == 1
    assert haptics_clock[0] == pytest.approx(0.2)
    assert "still armed" in captured.err and "stop --slot 3" in captured.err
    assert [request[1] for request in device.out_requests] == [b"\x02\x03"]


@pytest.mark.parametrize(
    ("action", "response_run", "response_slot", "response_mode"),
    [
        ("gameplay", 42, 0, 1),
        ("gameplay", 41, 1, 1),
        ("gameplay", 41, 0, 0),
        ("start", 41, 0, 1),
        ("stop", 40, 0, 0),
    ],
)
def test_haptics_confirmation_never_attributes_another_run_slot_or_mode(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    action: str,
    response_run: int,
    response_slot: int,
    response_mode: int,
) -> None:
    stopping = action == "stop"
    device = HapticsDevice(
        [
            haptics_response(
                2 if stopping else 3, run_id=40, slot=0, mode=int(stopping)
            ),
            haptics_response(
                4 if stopping else 2,
                run_id=response_run,
                slot=response_slot,
                mode=response_mode,
            ),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert config_manager.main(["haptics-experiment", action, "--json"]) == 1
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "run changed" in captured.err or "belongs to another" in captured.err


def test_haptics_gameplay_watch_rejects_fixture_with_same_run_and_slot(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    haptics_clock: list[float],
) -> None:
    device = HapticsDevice(
        [
            haptics_response(2, run_id=7, slot=0, mode=1),
            haptics_response(3, run_id=7, slot=0, mode=0),
        ]
    )
    monkeypatch.setattr(config_manager, "_candidate_devices", lambda: [device])
    assert (
        config_manager.main(
            [
                "haptics-experiment",
                "status",
                "--watch",
                "--json",
            ]
        )
        == 1
    )
    captured = capsys.readouterr()
    row = json.loads(captured.out)
    assert row["state_name"] == "running" and row["mode"] == 1
    assert "cannot attribute measurements" in captured.err


def capture_envelope(
    events: list[tuple[int, int]],
    *,
    first: int = 0,
    total: int = 3,
    generation: int = 9,
    elapsed_us: int = 5000,
) -> config_manager.Envelope:
    payload = struct.pack(
        "<IIIBBBBHHHHIB3x",
        7,
        generation,
        elapsed_us,
        0,
        2,
        1,
        len(events),
        total,
        first,
        512,
        1024,
        10000,
        128,
    ) + b"".join(
        struct.pack("<IHhhhhHH2x", at_us, buttons, 0, 0, 0, 0, 0, 0)
        for at_us, buttons in events
    )
    return config_manager.parse_response(
        make_response(config_manager.OP_MACRO_CAPTURE, payload, schema=1, generation=7),
        config_manager.OP_MACRO_CAPTURE,
    )


def test_capture_conversion_preserves_edges_without_rounding_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(config_manager, "_control_out", lambda *args: None)
    monkeypatch.setattr(
        config_manager,
        "_control_in",
        lambda *args: capture_envelope([(0, 0), (1500, 1), (3500, 0)]),
    )
    page = config_manager.collect_macro_capture(FakeDevice(), 7)
    steps = config_manager.capture_macro_steps(page)
    assert [step.output_button_mask for step in steps] == [0, 1, 0]
    assert [step.duration_ms for step in steps] == [2, 2, 1]


def test_capture_collection_rejects_mixed_connection_pages(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pages = iter(
        [
            capture_envelope(
                [(index * 1000, 0) for index in range(32)], total=33, elapsed_us=40000
            ),
            capture_envelope(
                [(32000, 1)], first=32, total=33, generation=10, elapsed_us=40000
            ),
        ]
    )
    monkeypatch.setattr(config_manager, "_control_out", lambda *args: None)
    monkeypatch.setattr(config_manager, "_control_in", lambda *args: next(pages))
    with pytest.raises(
        config_manager.ConfigManagerError, match="changed while reading"
    ):
        config_manager.collect_macro_capture(FakeDevice(), 7)


def test_haptics_reports_actual_short_packet_lookback() -> None:
    envelope = config_manager.parse_response(
        haptics_response(2, run_id=7, slot=0, mode=1, packet_frames=32),
        config_manager.OP_HAPTICS_EXPERIMENT,
    )
    gameplay = config_manager.parse_haptics_experiment(envelope).to_json_object()[
        "gameplay"
    ]
    assert gameplay["stereo_frames_per_packet"] == 32
    assert gameplay["lookback_us"] == pytest.approx(10666.6666667)


@pytest.mark.parametrize(
    ("packet_frames", "total_packets", "silence_packets", "phase_packets"),
    [(32, 576, 96, 24), (64, 288, 48, 12)],
)
def test_haptics_fixture_metadata_follows_reported_frame_count(
    packet_frames: int,
    total_packets: int,
    silence_packets: int,
    phase_packets: int,
) -> None:
    snapshot = config_manager.read_haptics_experiment(
        HapticsDevice([haptics_response(2, slot=0, packet_frames=packet_frames)])
    )
    pattern = snapshot.to_json_object()["pattern"]
    assert pattern["stereo_frames_per_packet"] == packet_frames
    assert pattern["packet_interval_us"] == pytest.approx(
        packet_frames * 1000000 / 3000
    )
    assert pattern["total_packets"] == total_packets
    assert pattern["priming_silence_packets"] == silence_packets
    assert pattern["trailing_silence_packets"] == silence_packets
    assert pattern["phases"] == [
        {"channel": "left", "frequency_hz": 100, "packets": phase_packets},
        {"channel": "silence", "packets": phase_packets},
        {"channel": "right", "frequency_hz": 200, "packets": phase_packets},
        {"channel": "silence", "packets": phase_packets},
    ]
    assert pattern["cycles"] == 4
    assert pattern["duration_us"] == pytest.approx(
        total_packets * pattern["packet_interval_us"]
    )
    assert pattern["duration_us"] == 6144000
    assert pattern["initial_mode_packet_stereo_frames"] == 0


@pytest.mark.parametrize("mode", [0, 1])
@pytest.mark.parametrize("packet_frames", [0, 48])
def test_haptics_rejects_unadvertised_frame_sizes(
    mode: int, packet_frames: int
) -> None:
    device = HapticsDevice(
        [haptics_response(2, slot=0, mode=mode, packet_frames=packet_frames)]
    )
    with pytest.raises(config_manager.ConfigManagerError, match="packet size"):
        config_manager.read_haptics_experiment(device)
