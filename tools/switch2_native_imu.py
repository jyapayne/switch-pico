#!/usr/bin/env python3
"""Offline candidate codec for native Joy-Con 2 (R) IMU blocks.

No device access or firmware writes. Accepts exported native_hex packet JSON,
unfragmented decrypted BLE PCAPng (ATT handle 0x000e), or --block HEX.

Recovered from native captures, not copied from original-Switch report 0x30.
The quaternion is ratio-coded: select the largest component, make it positive,
and store the next three components in cyclic order divided by that component.
Labelled right-Joy-Con captures support wire order (w, x, y, z), with +X toward
the outer edge away from the rail, +Y toward R/ZR, and +Z out of the button face.
Positive gyro rotation follows the right-hand rule about those body axes.
Acceleration units are a g-scale candidate supported by gravity magnitudes.
For observed packed tag 3, labelled turns support gyro degrees/second =
raw * 500 / 2**(width - 1), correcting the initial 512-degree-range hypothesis.
Subsample timing and cross-device tick units remain unconfirmed. The trailing
temperature-like word is raw, not degrees; format 0x0f has no such word.

Reference for native report boundaries:
https://github.com/ndeadly/switch2_controller_research/blob/master/hid_reports.md
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from collections import Counter
from collections.abc import Iterable, Iterator
from pathlib import Path
from typing import TypedDict


class AccelerationVector(TypedDict):
    bit_offset: int
    width: int
    fraction_bits_candidate: int
    raw: list[int]
    g_candidate: list[float]


class RotationTriplet(TypedDict):
    bit_offset: int
    width: int
    dps_per_count_candidate: float
    raw: list[int]
    dps_candidate: list[float]
    interpretation: str


class DecodedBlock(TypedDict):
    format: int
    length: int
    counter_ticks: int
    elapsed_ticks: int
    range_tag: int | None
    largest_component: int
    quaternion_width: int
    quaternion_ratios: list[float]
    quaternion_wire: list[float]
    accelerations: list[AccelerationVector]
    rotation_triplets: list[RotationTriplet]
    temperature_raw_candidate: int | None


class InputRecord(TypedDict, total=False):
    native_hex: str
    block: str
    t_us: int | None
    stream: str


# Quaternion width; alternating acceleration / rotation triplet widths.
# All offsets are LSB-first within the IMU block, not the containing HID report.
LAYOUTS = {
    0: (31, (32,)),
    1: (23, (22, 22, 22)),
    2: (21, (14, 13, 13, 14, 14)),
    3: (21, (14, 16, 13, 16, 14)),
}


def bits(word: int, start: int, width: int) -> int:
    return (word >> start) & ((1 << width) - 1)


def signed(word: int, start: int, width: int) -> int:
    value = bits(word, start, width)
    return value - (1 << width) if value & (1 << (width - 1)) else value


def quaternion_from_ratios(index: int, ratios: list[float]) -> list[float]:
    largest = 1.0 / math.sqrt(1.0 + sum(value * value for value in ratios))
    result = [0.0] * 4
    result[index] = largest
    for i, ratio in enumerate(ratios):
        result[(index + i + 1) & 3] = ratio * largest
    return result


def decode_block(block: bytes) -> DecodedBlock:
    if len(block) not in (30, 40):
        raise ValueError(f"expected 30 or 40 IMU bytes, received {len(block)}")
    if block[3] not in (0x0C, 0x0D, 0x0E, 0x0F):
        raise ValueError(f"unsupported internal format byte 0x{block[3]:02x}")
    variant = block[3] & 3
    if len(block) != (30 if variant == 0 else 40):
        raise ValueError("internal format does not match declared IMU length")
    word = int.from_bytes(block, "little")
    quaternion_width, widths = LAYOUTS[variant]
    range_tag = None if variant == 0 else bits(word, 32, 2)
    if range_tag not in (None, 3):
        raise ValueError(f"unobserved packed range tag {range_tag}; scale unresolved")
    index_start = 32 if variant == 0 else 34
    index_tag = bits(word, index_start, 3)
    if index_tag > 3:
        raise ValueError(f"unobserved quaternion index tag {index_tag}")
    position = index_start + 3
    ratio_codes = [bits(word, position + i * quaternion_width, quaternion_width)
                   for i in range(3)]
    ratios = [value / (1 << (quaternion_width - 1)) - 1.0 for value in ratio_codes]
    position += 3 * quaternion_width
    accelerations: list[AccelerationVector] = []
    rotation_triplets: list[RotationTriplet] = []
    for vector_index, width in enumerate(widths):
        values = [signed(word, position + i * width, width) for i in range(3)]
        if vector_index % 2 == 0:
            fraction_bits = 28 if variant == 0 else width - 2
            accelerations.append({
                "bit_offset": position,
                "width": width,
                "fraction_bits_candidate": fraction_bits,
                "raw": values,
                "g_candidate": [value / (1 << fraction_bits) for value in values],
            })
        else:
            dps_per_count = 500.0 / (1 << (width - 1))
            rotation_triplets.append({
                "bit_offset": position,
                "width": width,
                "dps_per_count_candidate": dps_per_count,
                "raw": values,
                "dps_candidate": [value * dps_per_count for value in values],
                "interpretation": "body-local angular velocity candidate",
            })
        position += 3 * width
    if variant == 3:
        assert position == 319
        if bits(word, 319, 1):
            raise ValueError("unobserved nonzero format-0x0f terminal bit")
        temperature = None
    else:
        assert position == len(block) * 8 - 16
        temperature = signed(word, position, 16)
    return {
        "format": block[3],
        "length": len(block),
        "counter_ticks": bits(word, 0, 12),
        "elapsed_ticks": bits(word, 12, 12),
        "range_tag": range_tag,
        "largest_component": index_tag,
        "quaternion_width": quaternion_width,
        "quaternion_ratios": ratios,
        "quaternion_wire": quaternion_from_ratios(index_tag, ratios),
        "accelerations": accelerations,
        "rotation_triplets": rotation_triplets,
        "temperature_raw_candidate": temperature,
    }


def put_bits(word: int, start: int, width: int, value: int, *, is_signed: bool = False) -> int:
    minimum = -(1 << (width - 1)) if is_signed else 0
    maximum = (1 << (width - int(is_signed))) - 1
    if not minimum <= value <= maximum:
        raise ValueError(f"value {value} does not fit {'signed ' if is_signed else ''}{width} bits")
    return word | ((value & ((1 << width) - 1)) << start)


def pack_quaternion(word: int, start: int, width: int,
                    quaternion: list[float], index: int) -> int:
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("quaternion must contain four finite components")
    if index not in range(4) or quaternion[index] == 0:
        raise ValueError("selected quaternion component must be nonzero")
    word = put_bits(word, start, 3, index)
    start += 3
    for i in range(3):
        ratio = quaternion[(index + i + 1) & 3] / quaternion[index]
        if abs(ratio) > 1.000000001:
            raise ValueError("selected quaternion component is not largest")
        # +1 is the top quantization boundary; do not wrap it into -1.
        code = min((1 << width) - 1,
                   max(0, round((ratio + 1.0) * (1 << (width - 1)))))
        word = put_bits(word, start + i * width, width, code)
    return word


def encode_mode0(counter_ticks: int, elapsed_ticks: int,
                 quaternion_wire: list[float], acceleration_g: list[float],
                 temperature_raw: int, *, largest_component: int | None = None) -> bytes:
    """Encode the recovered one-sample layout; not yet a console-qualified encoder.

    Physical axis mapping and temperature conversion are caller responsibilities.
    This deliberately takes real orientation/acceleration, not canned motion data.
    """
    if len(quaternion_wire) != 4:
        raise ValueError("quaternion must have four components")
    if largest_component is None:
        largest_component = max(range(4), key=lambda i: abs(quaternion_wire[i]))
    if len(acceleration_g) != 3 or not all(math.isfinite(v) for v in acceleration_g):
        raise ValueError("acceleration must contain three finite components")
    word = put_bits(0, 0, 12, counter_ticks)
    word = put_bits(word, 12, 12, elapsed_ticks)
    word = put_bits(word, 24, 8, 0x0C)
    word = pack_quaternion(word, 32, 31, quaternion_wire, largest_component)
    for i, value in enumerate(acceleration_g):
        word = put_bits(word, 128 + 32 * i, 32, round(value * (1 << 28)), is_signed=True)
    word = put_bits(word, 224, 16, temperature_raw, is_signed=True)
    return word.to_bytes(30, "little")


def repack_decoded(decoded: DecodedBlock) -> bytes:
    """Rebuild fields independently; do not copy original bytes or ratio codes.

    Rebuild gyro fields from candidate degree-per-second values as well.
    """
    variant = decoded["format"] & 3
    quaternion_width, widths = LAYOUTS[variant]
    word = put_bits(0, 0, 12, decoded["counter_ticks"])
    word = put_bits(word, 12, 12, decoded["elapsed_ticks"])
    word = put_bits(word, 24, 8, decoded["format"])
    start = 32
    if variant:
        range_tag = decoded["range_tag"]
        if range_tag is None:
            raise ValueError("packed frame requires a range tag")
        word = put_bits(word, start, 2, range_tag)
        start += 2
    word = pack_quaternion(word, start, quaternion_width,
                          decoded["quaternion_wire"], decoded["largest_component"])
    position = start + 3 + 3 * quaternion_width
    accel_index = rotation_index = 0
    for vector_index, width in enumerate(widths):
        if vector_index % 2 == 0:
            fraction_bits = 28 if variant == 0 else width - 2
            values = [round(v * (1 << fraction_bits))
                      for v in decoded["accelerations"][accel_index]["g_candidate"]]
            accel_index += 1
        else:
            counts_per_dps = (1 << (width - 1)) / 500.0
            values = [round(v * counts_per_dps)
                      for v in decoded["rotation_triplets"][rotation_index]["dps_candidate"]]
            rotation_index += 1
        for i, value in enumerate(values):
            word = put_bits(word, position + i * width, width, value, is_signed=True)
        position += 3 * width
    if variant != 3:
        temperature = decoded["temperature_raw_candidate"]
        if temperature is None:
            raise ValueError("this format requires the trailing temperature-like word")
        word = put_bits(word, position, 16, temperature, is_signed=True)
    return word.to_bytes(decoded["length"], "little")


def pcap_native_records(path: Path) -> Iterator[InputRecord]:
    """Read complete decrypted native-right notifications, not BLE reassembly."""
    data = path.read_bytes()
    position = 0
    endian = "<"
    interfaces: list[tuple[int, int, int]] = []
    while position + 12 <= len(data):
        if data[position:position + 4] == b"\x0a\x0d\x0d\x0a":
            magic = data[position + 8:position + 12]
            if magic not in (b"\x4d\x3c\x2b\x1a", b"\x1a\x2b\x3c\x4d"):
                raise ValueError("invalid PCAPng byte-order magic")
            endian = "<" if magic[0] == 0x4D else ">"
            interfaces = []
        kind, size = struct.unpack_from(endian + "II", data, position)
        if (size < 12 or size % 4 or position + size > len(data) or
                struct.unpack_from(endian + "I", data, position + size - 4)[0] != size):
            raise ValueError(f"invalid PCAPng block at {position}")
        if kind == 1:
            if size < 20:
                raise ValueError("truncated interface description")
            link_type = struct.unpack_from(endian + "H", data, position + 8)[0]
            resolution, offset_seconds = 1_000_000, 0
            option = position + 16
            while option + 4 <= position + size - 4:
                code, length = struct.unpack_from(endian + "HH", data, option)
                option += 4
                if option + length > position + size - 4:
                    raise ValueError("truncated interface option")
                value = data[option:option + length]
                if code == 9 and length == 1:
                    resolution = (2 if value[0] & 128 else 10) ** (value[0] & 127)
                elif code == 14 and length == 8:
                    offset_seconds = struct.unpack(endian + "q", value)[0]
                option += (length + 3) & ~3
                if code == 0:
                    break
            interfaces.append((link_type, resolution, offset_seconds))
        elif kind == 6:
            if size < 32:
                raise ValueError("truncated enhanced packet block")
            interface, high, low, captured, _ = struct.unpack_from(endian + "IIIII", data, position + 8)
            if interface >= len(interfaces) or captured > size - 32:
                raise ValueError("invalid enhanced packet metadata")
            link_type, resolution, offset_seconds = interfaces[interface]
            packet = data[position + 28:position + 28 + captured]
            ll_start = {256: 10, 251: 0}.get(link_type)
            if ll_start is not None and len(packet) >= ll_start + 6:
                ll_payload = ll_start + 6
                # LLID=2: complete first L2CAP fragment. This reader does not
                # reconstruct fragmented or encrypted notifications.
                if (packet[ll_start + 4] & 3 == 2 and packet[ll_start + 5] == 70 and
                        packet[ll_payload:ll_payload + 7] == b"\x42\x00\x04\x00\x1b\x0e\x00" and
                        len(packet) >= ll_payload + 70):
                    timestamp = (((high << 32) | low) * 1_000_000 // resolution +
                                 offset_seconds * 1_000_000)
                    yield {"t_us": timestamp,
                           "stream": f"{interface}:{packet[ll_start:ll_start + 4].hex()}",
                           "native_hex": packet[ll_payload + 7:ll_payload + 70].hex()}
        position += size
    if position != len(data):
        raise ValueError("trailing incomplete PCAPng block")


def records(path: Path) -> Iterator[InputRecord]:
    if path.suffix.lower() == ".pcapng":
        yield from pcap_native_records(path)
    else:
        data = json.loads(path.read_text())
        yield from data["packets"] if isinstance(data, dict) else data


def native_block(record: InputRecord) -> bytes:
    native_hex = record.get("native_hex")
    if native_hex is None:
        raise ValueError("input record requires native_hex")
    packet = bytes.fromhex(native_hex)
    if len(packet) == 64 and packet[0] == 8:
        packet = packet[1:]
    if len(packet) != 63:
        raise ValueError("expected 63 native08 payload bytes, optionally prefixed by report ID")
    length = packet[15]
    if length not in (0, 30, 40):
        raise ValueError(f"unrecognized native08 IMU length {length}")
    return packet[16:16 + length]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="*", type=Path)
    parser.add_argument("--block", help="decode one 30/40-byte IMU block in hexadecimal")
    parser.add_argument("--summary", action="store_true", help="aggregate instead of JSON-lines output")
    parser.add_argument("--verify-roundtrip", action="store_true", help="re-encode all fields and compare bytes")
    args = parser.parse_args()
    if bool(args.block) == bool(args.inputs):
        parser.error("provide input files or --block, but not both")
    counts: Counter[str] = Counter()
    norms: list[float] = []
    decoded_count = 0
    empty_count = 0
    mismatch_count = 0
    timing_gaps = 0
    previous: dict[tuple[str, str], int] = {}
    streams: list[tuple[str, Iterable[InputRecord]]] = [("hex", [{"block": args.block}])] if args.block else [
        (str(path), records(path)) for path in args.inputs]
    try:
        for source, source_records in streams:
            for record in source_records:
                block = bytes.fromhex(record["block"]) if "block" in record else native_block(record)
                if not block:
                    empty_count += 1
                    continue
                decoded = decode_block(block)
                decoded_count += 1
                counts[f"0x{decoded['format']:02x}"] += 1
                for acceleration in decoded["accelerations"]:
                    norms.append(math.sqrt(sum(v * v for v in acceleration["g_candidate"])))
                key = (source, record.get("stream", "default"))
                if key in previous:
                    delta = (decoded["counter_ticks"] - previous[key]) & 0xFFF
                    if delta != decoded["elapsed_ticks"]:
                        timing_gaps += 1
                previous[key] = decoded["counter_ticks"]
                if args.verify_roundtrip:
                    rebuilt = repack_decoded(decoded)
                    if decoded["format"] == 0x0C:
                        temperature = decoded["temperature_raw_candidate"]
                        if temperature is None:
                            raise ValueError("mode0 requires the trailing temperature-like word")
                        rebuilt_mode0 = encode_mode0(
                            decoded["counter_ticks"], decoded["elapsed_ticks"],
                            decoded["quaternion_wire"], decoded["accelerations"][0]["g_candidate"],
                            temperature,
                            largest_component=decoded["largest_component"])
                        if rebuilt_mode0 != block:
                            raise ValueError("mode0 semantic encoder round trip failed")
                    mismatch_count += rebuilt != block
                if not args.summary:
                    print(json.dumps({"source": source, "t_us": record.get("t_us"), **decoded}))
        if decoded_count == 0:
            raise ValueError("no supported native IMU blocks found")
        if args.summary:
            norms.sort()
            print(json.dumps({
                "decoded_blocks": decoded_count,
                "empty_blocks": empty_count,
                "formats": dict(sorted(counts.items())),
                "acceleration_vectors": len(norms),
                "acceleration_norm_g_candidate": {
                    name: norms[round((len(norms) - 1) * fraction)]
                    for name, fraction in (("min", 0), ("p10", .1), ("median", .5), ("p90", .9), ("max", 1))
                },
                "counter_interval_mismatches": timing_gaps,
                "roundtrip_mismatches": mismatch_count if args.verify_roundtrip else None,
                "limitations": ["axis and gyro-scale qualification limited to labelled right-Joy-Con captures",
                                "subsample timing and cross-device tick units unconfirmed",
                                "temperature conversion unconfirmed",
                                "encoder not yet console-qualified"],
            }, indent=2))
    except (ValueError, KeyError, TypeError, OSError, struct.error) as error:
        print(f"decode error: {error}", file=sys.stderr)
        return 1
    return int(mismatch_count != 0)


if __name__ == "__main__":
    raise SystemExit(main())
