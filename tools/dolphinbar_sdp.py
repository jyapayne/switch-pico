"""Small, in-memory SDP server for an original Wii Remote identity.

Wire-format reference: Bluetooth Core, Vol 3, Part B. HID attribute values
are factual values from the original remote record documented in
https://github.com/rnconrad/WiimoteEmulator/blob/master/sdp.c and the identity
and channel assignments at https://wiibrew.org/wiki/Wiimote . No upstream
response arrays or implementation are used here.
"""

from __future__ import annotations

import struct
from collections import OrderedDict
from typing import NamedTuple

_MAX_REQUEST = 4096
_MAX_DESCRIPTOR = 4096
_MAX_ELEMENTS = 512
_MAX_DEPTH = 8
_MAX_CONTINUATIONS = 64
_TOKEN_BYTES = 8
_UUID_BASE_SUFFIX = bytes.fromhex("00001000800000805f9b34fb")
_VALID_SIZES = (
    (0,),  # Nil
    (0, 1, 2, 3, 4),  # Unsigned integer
    (0, 1, 2, 3, 4),  # Signed integer
    (1, 2, 4),  # UUID
    (5, 6, 7),  # Text string
    (0,),  # Boolean
    (5, 6, 7),  # Sequence
    (5, 6, 7),  # Alternative
    (5, 6, 7),  # URL
)


class _SdpError(Exception):
    def __init__(self, code: int):
        self.code = code
        super().__init__(code)


def _encode(kind: int, payload: bytes) -> bytes:
    """Encode a data element, choosing a legal shortest length header."""
    size = len(payload)
    if kind == 0:
        if size:
            raise ValueError("Nil data elements have no payload")
        return b"\x00"
    if kind in (1, 2, 3, 5):
        sizes = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4}
        index = sizes.get(size, -1)
        if index not in _VALID_SIZES[kind]:
            raise ValueError("Invalid scalar data element size")
        return bytes([(kind << 3) | index]) + payload
    if kind not in (4, 6, 7, 8):
        raise ValueError("Invalid data element type")
    for width, index in ((1, 5), (2, 6), (4, 7)):
        if size < 1 << (8 * width):
            return bytes([(kind << 3) | index]) + size.to_bytes(width, "big") + payload
    raise ValueError("Data element is too large")


def _uint(value: int, width: int = 2) -> bytes:
    return _encode(1, value.to_bytes(width, "big"))


def _uuid(value: int) -> bytes:
    return _encode(3, value.to_bytes(2, "big"))


def _boolean(value: bool) -> bytes:
    return _encode(5, bytes([int(value)]))


def _text(value: str) -> bytes:
    return _encode(4, value.encode("utf-8"))


def _sequence(*elements: bytes) -> bytes:
    return _encode(6, b"".join(elements))


class _Element(NamedTuple):
    kind: int
    size: int
    value: int | bytes | tuple[_Element, ...] | None


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.position = 0
        self.elements = 0

    def integer(self, width: int, end: int | None = None) -> int:
        if end is None:
            end = len(self.data)
        if self.position + width > end:
            raise _SdpError(0x0003)
        value = int.from_bytes(self.data[self.position : self.position + width], "big")
        self.position += width
        return value

    def element(self, end: int | None = None, depth: int = 0) -> _Element:
        if end is None:
            end = len(self.data)
        self.elements += 1
        if depth > _MAX_DEPTH or self.elements > _MAX_ELEMENTS:
            raise _SdpError(0x0006)
        header = self.integer(1, end)
        kind, index = header >> 3, header & 7
        if kind >= len(_VALID_SIZES) or index not in _VALID_SIZES[kind]:
            raise _SdpError(0x0003)
        if kind == 0:
            return _Element(kind, 0, None)
        size = (1 << index) if index < 5 else self.integer(1 << (index - 5), end)
        stop = self.position + size
        if stop > end:
            raise _SdpError(0x0003)
        if kind in (6, 7):
            children = []
            while self.position < stop:
                children.append(self.element(stop, depth + 1))
            return _Element(kind, size, tuple(children))
        raw = self.data[self.position : stop]
        self.position = stop
        if kind in (1, 2, 5):
            value = int.from_bytes(raw, "big", signed=kind == 2)
            if kind == 5:
                value = int(bool(value))
            return _Element(kind, size, value)
        return _Element(kind, size, raw)

    def search_pattern(self) -> tuple[bytes, ...]:
        element = self.element()
        if element.kind != 6 or not 1 <= len(element.value) <= 12:
            raise _SdpError(0x0003)
        uuids = []
        for child in element.value:
            if child.kind != 3:
                raise _SdpError(0x0003)
            uuid = child.value
            if child.size != 16:
                uuid = uuid.rjust(4, b"\x00") + _UUID_BASE_SUFFIX
            uuids.append(uuid)
        return tuple(uuids)

    def attributes(self) -> tuple[tuple[int, int], ...]:
        element = self.element()
        if element.kind != 6:
            raise _SdpError(0x0003)
        ranges = []
        previous = -1
        for child in element.value:
            if child.kind != 1 or child.size not in (2, 4):
                raise _SdpError(0x0003)
            if child.size == 2:
                first = last = child.value
            else:
                first, last = child.value >> 16, child.value & 0xFFFF
            if first > last or first <= previous:
                raise _SdpError(0x0003)
            ranges.append((first, last))
            previous = last
        return tuple(ranges)

    def continuation(self) -> bytes:
        size = self.integer(1)
        if size > 16:
            raise _SdpError(0x0005)
        if self.position + size != len(self.data):
            raise _SdpError(0x0003)
        return self.data[self.position :]


def _records(descriptor: bytes) -> dict[int, dict[int, bytes]]:
    language = _sequence(_uint(0x656E), _uint(106), _uint(0x0100))
    browse = _sequence(_uuid(0x1002))

    def hid_protocol(psm: int) -> bytes:
        return _sequence(_sequence(_uuid(0x0100), _uint(psm)), _sequence(_uuid(0x0011)))

    hid = {
        0x0000: _uint(0x00010000, 4),
        0x0001: _sequence(_uuid(0x1124)),
        0x0004: hid_protocol(0x0011),
        0x0005: browse,
        0x0006: language,
        0x0009: _sequence(_sequence(_uuid(0x1124), _uint(0x0100))),
        0x000D: _sequence(hid_protocol(0x0013)),
        0x0100: _text("Nintendo RVL-CNT-01"),
        0x0101: _text("Nintendo RVL-CNT-01"),
        0x0102: _text("Nintendo"),
        0x0200: _uint(0x0100),  # HIDDeviceReleaseNumber
        0x0201: _uint(0x0111),  # HIDParserVersion
        0x0202: _uint(0x04, 1),  # HIDDeviceSubclass
        0x0203: _uint(0x33, 1),  # HIDCountryCode in the original record
        0x0204: _boolean(False),  # HIDVirtualCable
        0x0205: _boolean(True),  # HIDReconnectInitiate
        0x0206: _sequence(_sequence(_uint(0x22, 1), _encode(4, descriptor))),
        0x0207: _sequence(_sequence(_uint(0x0409), _uint(0x0100))),
        0x0208: _boolean(False),  # HIDSDPDisable
        0x0209: _boolean(True),  # HIDBatteryPower
        0x020A: _boolean(True),  # HIDRemoteWake
        0x020B: _uint(0x0100),  # HIDProfileVersion
        0x020C: _uint(0x0C80),  # HIDSupervisionTimeout
        0x020D: _boolean(False),  # HIDNormallyConnectable
        0x020E: _boolean(False),  # HIDBootDevice
    }
    pnp = {
        0x0000: _uint(0x00010001, 4),
        0x0001: _sequence(_uuid(0x1200)),
        0x0005: browse,
        0x0006: language,
        0x0009: _sequence(_sequence(_uuid(0x1200), _uint(0x0103))),
        0x0100: _text("Nintendo RVL-CNT-01"),
        0x0102: _text("Nintendo"),
        0x0200: _uint(0x0103),  # Device ID specification version
        0x0201: _uint(0x057E),  # USB vendor: Nintendo
        0x0202: _uint(0x0306),  # Original Wii Remote
        0x0203: _uint(0x0100),  # Device version
        0x0204: _boolean(True),  # PrimaryRecord
        0x0205: _uint(0x0002),  # VendorIDSource: USB-IF
    }
    return {0x00010000: hid, 0x00010001: pnp}


def _collect_uuids(element: _Element, result: set[bytes]) -> None:
    if element.kind == 3:
        uuid = element.value
        if element.size != 16:
            uuid = uuid.rjust(4, b"\x00") + _UUID_BASE_SUFFIX
        result.add(uuid)
    elif element.kind in (6, 7):
        for child in element.value:
            _collect_uuids(child, result)


def _attribute_list(
    record: dict[int, bytes], ranges: tuple[tuple[int, int], ...]
) -> bytes:
    return _encode(
        6,
        b"".join(
            _uint(attribute) + value
            for attribute, value in sorted(record.items())
            if any(first <= attribute <= last for first, last in ranges)
        ),
    )


class SdpResponder:
    """Respond to one complete SDP request PDU at a time, without socket IO.

    ``mtu`` is the peer's L2CAP receive MTU, including the five-byte SDP
    header, but not an L2CAP header. Keep this instance for the lifetime of
    an SDP connection. At most 64 recently issued continuation states are
    retained; evicted or mismatched states receive Invalid Continuation
    State. Transaction IDs and attribute byte limits may change between
    fragments; the actual query may not.
    """

    def __init__(self, report_descriptor: bytes):
        if not isinstance(report_descriptor, bytes):
            raise TypeError("report_descriptor must be bytes")
        if not 1 <= len(report_descriptor) <= _MAX_DESCRIPTOR:
            raise ValueError("report_descriptor must contain 1 to 4096 bytes")
        self._records = _records(report_descriptor)
        self._record_uuids = {}
        for handle, record in self._records.items():
            uuids = set()
            for value in record.values():
                _collect_uuids(_Reader(value).element(), uuids)
            self._record_uuids[handle] = uuids
        self._continuations: OrderedDict[bytes, tuple[tuple, bytes, int]] = (
            OrderedDict()
        )
        self._serial = 0

    def _save(self, query: tuple, data: bytes, offset: int) -> bytes:
        if self._serial == (1 << (8 * _TOKEN_BYTES)) - 1:
            raise _SdpError(0x0006)
        self._serial += 1
        token = self._serial.to_bytes(_TOKEN_BYTES, "big")
        self._continuations[token] = (query, data, offset)
        if len(self._continuations) > _MAX_CONTINUATIONS:
            self._continuations.popitem(last=False)
        return token

    def _data(
        self,
        pdu: int,
        target: int | tuple[bytes, ...],
        ranges: tuple[tuple[int, int], ...],
        maximum: int,
    ) -> bytes:
        if pdu == 0x04:
            record = self._records.get(target)
            if record is None:
                raise _SdpError(0x0002)
            return _attribute_list(record, ranges)
        handles = [
            handle
            for handle, uuids in self._record_uuids.items()
            if all(uuid in uuids for uuid in target)
        ]
        if pdu == 0x02:
            return b"".join(handle.to_bytes(4, "big") for handle in handles[:maximum])
        return _sequence(
            *(_attribute_list(self._records[handle], ranges) for handle in handles)
        )

    def _response(self, pdu: int, reader: _Reader, mtu: int) -> bytes:
        if pdu not in (0x02, 0x04, 0x06):
            raise _SdpError(0x0003)
        target = reader.integer(4) if pdu == 0x04 else reader.search_pattern()
        maximum = reader.integer(2)
        if maximum < (1 if pdu == 0x02 else 7):
            raise _SdpError(0x0003)
        ranges = () if pdu == 0x02 else reader.attributes()
        token = reader.continuation()
        query = (pdu, target, ranges, maximum if pdu == 0x02 else None)
        if token:
            state = self._continuations.get(token)
            if state is None or state[0] != query:
                raise _SdpError(0x0005)
            _, data, offset = state
            self._continuations.move_to_end(token)
        else:
            data = self._data(pdu, target, ranges, maximum)
            offset = 0

        # Attribute fragments may split any data element, but must each
        # contain at least two bytes; search fragments contain whole handles.
        search = pdu == 0x02
        overhead = 10 if search else 8
        limit = 0xFFFF if search else maximum
        remaining = len(data) - offset
        final_capacity = min(limit, mtu - overhead)
        if remaining <= final_capacity:
            count = remaining
            continuation = b"\x00"
        else:
            count = min(limit, mtu - overhead - _TOKEN_BYTES)
            if search:
                count -= count % 4
            elif remaining - count == 1:
                count -= 1
            if count < (4 if search else 2):
                raise _SdpError(0x0006)
            next_token = self._save(query, data, offset + count)
            continuation = bytes([len(next_token)]) + next_token
        fragment = data[offset : offset + count]
        if search:
            prefix = struct.pack(">HH", len(data) // 4, count // 4)
        else:
            prefix = struct.pack(">H", count)
        return prefix + fragment + continuation

    def reply(self, packet: bytes, mtu: int = 672) -> bytes:
        """Return the matching response or an SDP ErrorResponse.

        Invalid local MTUs below seven bytes raise ValueError: even an SDP
        error cannot fit. Requests over 4096 bytes or overly complex data
        elements receive Insufficient Resources. A truncated header lacking
        a complete transaction ID is answered with transaction ID zero.
        """
        if mtu < 7:
            raise ValueError("MTU cannot fit an SDP ErrorResponse")
        mtu = min(mtu, 0xFFFF + 5)
        transaction = int.from_bytes(packet[1:3], "big") if len(packet) >= 3 else 0
        try:
            if len(packet) < 5 or int.from_bytes(packet[3:5], "big") != len(packet) - 5:
                raise _SdpError(0x0004)
            if len(packet) > _MAX_REQUEST:
                raise _SdpError(0x0006)
            response = self._response(packet[0], _Reader(packet[5:]), mtu)
            response_pdu = packet[0] + 1
        except _SdpError as error:
            response_pdu = 0x01
            response = struct.pack(">H", error.code)
        return struct.pack(">BHH", response_pdu, transaction, len(response)) + response
