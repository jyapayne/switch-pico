from __future__ import annotations

import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def live_rig(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    import native_joycon_hub_check as check
    from switch2_native_imu import encode_mode0

    rig = SimpleNamespace(check=check, clock=0.0, activity="independent")
    monkeypatch.setattr(check, "time", SimpleNamespace(monotonic=lambda: rig.clock))

    def configure(pairs=2, mode="GAMEPAD", target="BOTH", neutral=False):
        cache = [
            f"SWITCH2_PROBE_PAIR_COUNT:STRING={pairs}",
            "SWITCH2_PROBE_HUB:BOOL=ON",
            "SWITCH2_PROBE_USB_INIT:BOOL=ON",
            f"SWITCH2_PROBE_NEUTRAL_INPUT:BOOL={'ON' if neutral else 'OFF'}",
            f"SWITCH_PICO_SWITCH2_USB_BRIDGE:BOOL={'OFF' if neutral else 'ON'}",
            f"SWITCH2_BRIDGE_INPUT:STRING={mode}",
            f"SWITCH2_BRIDGE_IMU_TARGET:STRING={target}",
        ]
        for index, (child, model) in enumerate(check.child_models(pairs).items()):
            identity = bytearray(64)
            identity[0] = index + 1
            struct.pack_into("<HH", identity, 18, check.VID, model["pid"])
            version = bytearray(12)
            version[3] = int(model["side"] == "R")
            factory = identity + bytearray(8192 - len(identity))
            for field, contents in (
                ("IDENTITY_FILE", identity),
                ("VERSION_FILE", version),
                ("FACTORY_FILE", factory),
            ):
                path = tmp_path / f"{child}-{field}.bin"
                path.write_bytes(contents)
                cache.append(f"{model['capture_prefix']}_{field}:FILEPATH={path}")
            cache.append(
                f"{model['capture_prefix']}_CONTROLLER_ADDRESS:STRING=02:00:00:00:00:{index + 1:02x}"
            )
        (tmp_path / "CMakeCache.txt").write_text("\n".join(cache))
        return tmp_path

    class InputPipe:
        def __init__(self, scenario, child):
            self.scenario = scenario
            self.model = scenario.models[child]
            self.frame = 0

        def read(self, endpoint, length, *, timeout):
            rig.clock += 0.02
            self.frame += 1
            pair = int(self.model["pair"] == "B")
            cycle = self.frame % 4
            missing = pair and (
                rig.activity == "missing"
                or (
                    rig.activity == "disconnect"
                    and self.scenario.current_stage == "active_input_and_read_isolation"
                )
            )
            payload = bytearray(63)
            payload[0] = self.frame % 256
            payload[5:8] = b"\x00\x08\x80"
            if not missing:
                controls_pair = 0 if rig.activity == "mirrored" else pair
                payload[2] = (1 << (cycle + controls_pair * 4)) if cycle else 0
                payload[5] = controls_pair * 32 + cycle
                if not self.scenario.args.input_only:
                    motion_pair = 0 if rig.activity == "mirrored" else pair
                    motion_cycle = 0 if rig.activity == "static" else cycle
                    block = encode_mode0(
                        self.frame % 4096,
                        4,
                        [1.0, 0.0, 0.0, 0.0],
                        [motion_pair + motion_cycle / 8, 0.0, 1.0],
                        25,
                    )
                    offset = 14 if self.model["side"] == "L" else 15
                    payload[offset] = len(block)
                    payload[offset + 1 : offset + 1 + len(block)] = block
            return bytes((self.model["report"],)) + payload

    def discover(scenario):
        scenario.devices = {
            child: InputPipe(scenario, child) for child in scenario.children
        }

    monkeypatch.setattr(check.Check, "discover", discover)
    # Only the physical USB boundary is replaced; reference parsing, packet
    # decoding, readiness, active evidence, failure handling and JSON all run.
    for method in (
        "permissions",
        "claim",
        "descriptors",
        "identities",
        "initialize",
        "queries",
        "cleanup",
    ):
        monkeypatch.setattr(check.Check, method, lambda *args, **kwargs: None)

    def run(*, pairs=2, mode="GAMEPAD", input_only=False, activity="independent"):
        rig.clock = 0.0
        rig.activity = activity
        capture = tmp_path / "qualification.json"
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "native_joycon_hub_check.py",
                "--build-dir",
                str(configure(pairs, mode)),
                "--output",
                str(capture),
                "--duration",
                "2",
                *(["--pairs", str(pairs)] if pairs != 1 else []),
                *(["--input-only"] if input_only else []),
            ],
        )
        status = check.main()
        return status, json.loads(capture.read_text())

    rig.configure = configure
    rig.run = run
    return rig


@pytest.mark.parametrize(
    ("mode", "input_only"), [("GAMEPAD", True), ("DUALSENSE", False)]
)
def test_two_live_pairs_qualify_distinct_activity_on_all_children(
    live_rig, mode, input_only
):
    status, audit = live_rig.run(mode=mode, input_only=input_only)

    assert status == 0, audit.get("failure", audit["errors"])
    children = ("A_R", "A_L", "B_R", "B_L")
    assert tuple(audit["child_results"]) == children
    assert not audit["gameplay_proven"]
    assert not audit["physical_latency_proven"]
    assert not audit["physical_source_isolation_proven"]
    for child in children:
        result = audit["child_results"][child]
        assert result["qualified"] and result["live_input_proven"]
        assert result["live_imu_proven"] is not input_only
        assert result["last_sample"]["packet_hex"].startswith(
            "08" if child.endswith("R") else "07"
        )
        assert f"{child}=" in audit["summary"]
    if not input_only:
        for pair in ("A", "B"):
            evidence = audit["imu_isolation"]["pairs"][pair]
            assert evidence["policy"] == "shared_physical_source"
            assert evidence["identical_blocks_seen_on_both_sides"] >= 2


@pytest.mark.parametrize(
    ("input_only", "activity"), [(True, "missing"), (False, "disconnect")]
)
def test_unassigned_or_disconnected_second_pair_cannot_qualify(
    live_rig, input_only, activity
):
    status, audit = live_rig.run(input_only=input_only, activity=activity)

    assert status == 2
    assert not audit["success"]
    assert all(
        not child["live_input_proven"] for child in audit["child_results"].values()
    )
    if activity == "missing":
        assert audit["streams"]["B_R"]["buttons_nonzero"] == 0
        assert audit["streams"]["B_L"]["control_changes"] == 0
    else:
        assert {error["side"] for error in audit["errors"]} >= {"B_R", "B_L"}


@pytest.mark.parametrize(
    ("input_only", "activity"),
    [(True, "mirrored"), (False, "mirrored"), (False, "static")],
)
def test_equal_or_static_pair_evidence_is_not_independent_activity(
    live_rig, input_only, activity
):
    status, audit = live_rig.run(input_only=input_only, activity=activity)

    assert status == 2
    assert not audit["physical_source_isolation_proven"]
    assert {error["side"] for error in audit["errors"]} >= {"A_R", "A_L", "B_R", "B_L"}


def test_default_one_pair_keeps_right_left_capture_ids(live_rig):
    status, audit = live_rig.run(pairs=1, input_only=True)

    assert status == 0, audit["errors"]
    assert tuple(audit["child_results"]) == ("R", "L")
    assert [child["port"] for child in audit["child_results"].values()] == [1, 2]


def test_two_pair_input_only_accepts_side_target_but_imu_requires_both(live_rig):
    build = live_rig.configure(target="LEFT")
    assert tuple(
        live_rig.check.model_references(build, pairs=2, require_imu=False)
    ) == ("A_R", "A_L", "B_R", "B_L")
    with pytest.raises(ValueError):
        live_rig.check.model_references(build, pairs=2, require_imu=True)


def test_two_pair_live_rejects_donor_only_and_neutral_builds(live_rig):
    for settings in ({"mode": "JOYCON2"}, {"neutral": True}):
        build = live_rig.configure(**settings)
        with pytest.raises(ValueError):
            live_rig.check.model_references(build, pairs=2, require_imu=False)


def test_neutral_and_input_only_are_exclusive_before_capture(
    live_rig, monkeypatch, tmp_path
):
    capture = tmp_path / "incompatible.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "native_joycon_hub_check.py",
            "--pairs",
            "2",
            "--neutral",
            "--input-only",
            "--output",
            str(capture),
        ],
    )
    with pytest.raises(SystemExit) as error:
        live_rig.check.main()
    assert error.value.code == 2
    assert not capture.exists()
