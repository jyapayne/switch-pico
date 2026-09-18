from __future__ import annotations

import io
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import usb.core


def trace_reply(
    *,
    magic=b"NHTR",
    version=1,
    status=0,
    slot=4,
    reserved=0,
    time_us=123456,
    generation=17,
):
    return struct.pack(
        "<4sBBBBII", magic, version, status, slot, reserved, time_us, generation
    )


@pytest.fixture
def trace_rig(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    import native_joycon_hub_check as check

    rig = SimpleNamespace(
        check=check,
        clock=0.0,
        child_duration=0.25,
        events=[],
        child_error=usb.core.USBError("child descriptor timed out"),
        capture=io.StringIO(),
    )
    monkeypatch.setattr(check, "time", SimpleNamespace(monotonic=lambda: rig.clock))
    args = SimpleNamespace(
        pairs=2,
        timeout=10.0,
        duration=2.0,
        usb_timeout_ms=500,
        rumble_sample=None,
        input_only=True,
        neutral=False,
        output=tmp_path / "trace.json",
        build_dir=tmp_path,
    )
    # Intentionally omit the new option: existing Namespace callers remain valid.
    rig.scenario = check.Check(args, rig.capture)

    class ControlPipe:
        def __init__(self, owner, response):
            self.owner = owner
            self.response = response
            self.transfers = []

        def ctrl_transfer(
            self, request_type, request, value, index, length, *, timeout
        ):
            assert request_type & 0x80, "trace tests must never issue USB OUT transfers"
            self.transfers.append(
                (request_type, request, value, index, length, timeout)
            )
            rig.events.append(self.owner)
            if self.owner != "root":
                rig.clock += rig.child_duration
            if isinstance(self.response, Exception):
                raise self.response
            return self.response

        def attach_kernel_driver(self, interface):
            rig.events.append("reattach")

    rig.root = ControlPipe("root", trace_reply())
    rig.child = ControlPipe("B_L", rig.child_error)
    rig.scenario.devices = {"root": rig.root, "B_L": rig.child}
    rig.scenario.util = SimpleNamespace(
        release_interface=lambda device, interface: rig.events.append("release"),
        dispose_resources=lambda device: rig.events.append("dispose"),
    )
    rig.scenario.claimed = [("B_L", 0)]
    rig.scenario.detached = [("B_L", 0)]

    def control(owner="B_L", name="device_descriptor"):
        return rig.scenario.control(owner, name, 0x80, 6, 0x0100, 0, 18)

    rig.control = control
    return rig


@pytest.mark.parametrize("explicit_default", [False, True])
def test_child_error_does_not_mark_without_opt_in(trace_rig, explicit_default):
    rig = trace_rig
    if explicit_default:
        rig.scenario.args.capture_trace_on_error = False

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    assert rig.root.transfers == []
    assert len(rig.child.transfers) == 1
    assert rig.scenario.result["failure_trace"] is None


def test_opt_in_captures_before_error_propagation_and_cleanup(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True

    with pytest.raises(usb.core.USBError) as caught:
        try:
            rig.control()
        except usb.core.USBError:
            rig.events.append("propagated")
            # The receipt is durable before the caller begins resource cleanup.
            audit = json.loads(rig.capture.getvalue())
            raise
        finally:
            rig.scenario.cleanup()

    assert caught.value is rig.child_error
    assert rig.events == [
        "B_L",
        "root",
        "propagated",
        "release",
        "reattach",
        "dispose",
        "dispose",
    ]
    assert len(rig.child.transfers) == 1
    assert rig.root.transfers == [(0xC0, 0x5E, 0x5452, 4, 16, 500)]
    trace = audit["failure_trace"]
    assert trace["status"] == "captured" and trace["captured"]
    assert trace["request_attempted"]
    assert trace["response_hex"] == trace_reply().hex()
    assert trace["receipt"] == {
        "version": 1,
        "status": 0,
        "slot": 4,
        "time_us": 123456,
        "control_generation": 17,
    }
    assert trace["failed_control"]["setup"] == [0x80, 6, 0x0100, 0, 18]
    assert trace["failed_control"]["side"] == "B_L"
    assert trace["failed_control"]["error"] == str(rig.child_error)


def test_root_error_does_not_consume_the_single_child_marker(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    root_error = usb.core.USBError("root descriptor failed")
    rig.root.response = root_error
    with pytest.raises(usb.core.USBError) as caught:
        rig.control("root")
    assert caught.value is root_error
    assert len(rig.root.transfers) == 1
    assert rig.scenario.result["failure_trace"] is None

    rig.root.response = trace_reply()
    for name in ("first_child_error", "later_child_error"):
        with pytest.raises(usb.core.USBError) as caught:
            rig.control(name=name)
        assert caught.value is rig.child_error

    assert [transfer[1] for transfer in rig.root.transfers] == [6, 0x5E]
    audit = json.loads(rig.capture.getvalue())
    assert audit["failure_trace"]["failed_control"]["name"] == "first_child_error"


@pytest.mark.parametrize(
    "marker_error",
    [usb.core.USBError("trace request stalled"), OSError("root disconnected")],
)
def test_failed_marker_preserves_original_error_and_is_not_retried(
    trace_rig, marker_error
):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.root.response = marker_error

    for name in ("first_child_error", "later_child_error"):
        with pytest.raises(usb.core.USBError) as caught:
            rig.control(name=name)
        assert caught.value is rig.child_error

    assert len(rig.root.transfers) == 1
    trace = json.loads(rig.capture.getvalue())["failure_trace"]
    assert trace["status"] == "error" and not trace["captured"]
    assert trace["response_hex"] is None and trace["receipt"] is None
    assert str(marker_error) in trace["error"]
    assert trace["failed_control"]["name"] == "first_child_error"


def test_busy_marker_is_an_explicit_refusal_not_capture(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.root.response = trace_reply(status=1, time_us=0, generation=0)

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    trace = json.loads(rig.capture.getvalue())["failure_trace"]
    assert trace["status"] == "busy" and not trace["captured"]
    assert trace["response_hex"] == rig.root.response.hex()
    assert trace["receipt"]["status"] == 1
    assert trace["receipt"]["time_us"] == trace["receipt"]["control_generation"] == 0


@pytest.mark.parametrize(
    "response",
    [
        trace_reply()[:-1],
        trace_reply() + b"\x00",
        trace_reply(magic=b"NOPE"),
        trace_reply(version=2),
        trace_reply(slot=3),
        trace_reply(reserved=1),
        trace_reply(status=2),
        trace_reply(status=1, time_us=1, generation=0),
        trace_reply(status=1, time_us=0, generation=1),
    ],
    ids=[
        "short",
        "long",
        "magic",
        "version",
        "slot",
        "reserved",
        "status",
        "busy-time",
        "busy-generation",
    ],
)
def test_malformed_marker_never_qualifies_as_a_capture(trace_rig, response):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.root.response = response

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    trace = json.loads(rig.capture.getvalue())["failure_trace"]
    assert trace["status"] == "malformed" and not trace["captured"]
    assert trace["response_hex"] == response.hex()
    assert trace["receipt"] is None
    assert "error" in trace


def test_marker_timeout_uses_only_remaining_deadline(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.child_duration = rig.scenario.args.timeout - 0.125

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    assert rig.root.transfers == [(0xC0, 0x5E, 0x5452, 4, 16, 125)]


def test_expired_deadline_after_child_error_records_without_request(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.child_duration = rig.scenario.args.timeout

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    assert rig.root.transfers == []
    trace = json.loads(rig.capture.getvalue())["failure_trace"]
    assert trace["status"] == "deadline_expired" and not trace["captured"]
    assert not trace["request_attempted"]
    assert trace["failed_control"]["error"] == str(rig.child_error)


def test_pretransfer_deadline_does_not_count_as_a_child_usb_failure(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.clock = rig.scenario.deadline

    with pytest.raises(TimeoutError):
        rig.control()

    assert rig.child.transfers == rig.root.transfers == []
    assert rig.scenario.result["failure_trace"] is None


def test_capture_file_error_cannot_replace_child_transfer_error(trace_rig):
    rig = trace_rig
    rig.scenario.args.capture_trace_on_error = True
    rig.capture.close()

    with pytest.raises(usb.core.USBError) as caught:
        rig.control()

    assert caught.value is rig.child_error
    trace = rig.scenario.result["failure_trace"]
    assert trace["status"] == "captured"
    assert "ValueError" in trace["checkpoint_error"]


def test_capture_option_marks_cli_failure_without_recovering(
    trace_rig, monkeypatch, tmp_path
):
    rig = trace_rig
    capture = tmp_path / "option.json"
    rig.child.owner = "R"
    rig.root.response = trace_reply(slot=1)

    def discover(scenario):
        scenario.devices = {"root": rig.root, "R": rig.child, "L": rig.child}
        scenario.util = rig.scenario.util

    def claim(scenario):
        scenario.claimed = [("R", 0)]
        scenario.detached = [("R", 0)]

    # Replace only private build references and the physical USB boundary. The
    # CLI, descriptor control, failure path, audit and cleanup execute normally.
    monkeypatch.setattr(
        rig.check, "model_references", lambda *args, **kwargs: rig.check.child_models(1)
    )
    monkeypatch.setattr(rig.check.Check, "discover", discover)
    monkeypatch.setattr(rig.check.Check, "permissions", lambda scenario: None)
    monkeypatch.setattr(rig.check.Check, "claim", claim)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "native_joycon_hub_check.py",
            "--capture-trace-on-error",
            "--output",
            str(capture),
        ],
    )

    assert rig.check.main() == 2
    audit = json.loads(capture.read_text())
    assert not audit["success"]
    assert audit["failure"] == str(rig.child_error)
    assert audit["failure_trace"]["status"] == "captured"
    assert audit["failure_trace"]["failed_control"]["side"] == "R"
    assert audit["parameters"]["capture_trace_on_error"]
    assert audit["safety"]["trace_marker_requested"]
    assert len(rig.child.transfers) == 1
    assert rig.root.transfers == [(0xC0, 0x5E, 0x5452, 1, 16, 500)]
    assert rig.events[:4] == ["R", "root", "release", "reattach"]
