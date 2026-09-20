"""Exercise host safety against a stateful, deterministic serial beacon."""

import json
from types import SimpleNamespace

import pytest
import serial

from switch_pico_bridge import wake_beacon as wake


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SimulatedBeacon:
    """Serial endpoint whose burst advances on time, not on echoed requests."""

    def __init__(self, clock):
        self.clock = clock
        self.timeout = 0.1
        self.write_timeout = 0.25
        self.closed = False
        self.fail_close = False
        self.input = bytearray()
        self.output = bytearray()
        self.commands = []
        self.bursts = 0
        self.started = None
        self.disconnect_on_wake = False
        self.silent = False
        self.partial_write = False
        self.finish = "complete"
        self.preflight_change = None
        self.race_busy = False
        self.ready_at = None
        self.status = {
            "protocol": 1,
            "role": "wake-only",
            "firmware": "1.0.0",
            "radio_ready": True,
            "controller_hosting": False,
            "request_id": 0,
            "state": "idle",
            "configured": True,
            "busy": False,
            "accepted_requests": 0,
            "completed_bursts": 0,
            "failures": 0,
            "error": "",
        }

    def _advance(self):
        if self.ready_at is not None and self.clock.now >= self.ready_at:
            self.status.update(radio_ready=True, configured=True, busy=False)
            self.ready_at = None
        if self.started is None:
            return
        elapsed = self.clock.now - self.started
        if elapsed < 0.05:
            return
        if elapsed < 0.3 or self.finish == "never":
            self.status["state"] = "broadcasting"
            return
        self.started = None
        if self.finish == "reset":
            self.status.update(
                request_id=0,
                state="idle",
                busy=False,
                accepted_requests=0,
                completed_bursts=0,
                failures=0,
            )
        elif self.finish == "replaced":
            self.status.update(
                request_id=self.status["request_id"] + 1,
                state="complete",
                busy=False,
                completed_bursts=1,
            )
        else:
            self.status.update(state="complete", busy=False, completed_bursts=1)
            if self.finish == "cleanup_failure":
                # A stop-cleanup failure can also increment completed_bursts.
                self.status.update(failures=1, state="failed")
            elif self.finish == "counter_failure":
                self.status["failures"] = 1

    def write(self, data):
        if self.closed:
            raise serial.SerialException("disconnected")
        if self.partial_write:
            self.input.extend(data[:3])
            return 3
        self.input.extend(data)
        while b"\n" in self.input:
            command, _, rest = self.input.partition(b"\n")
            self.input = bytearray(rest)
            self.commands.append(bytes(command))
            self._advance()
            response = dict(self.status)
            if command.startswith(b"SPWB1 WAKE "):
                request_id = int(command.split()[-1])
                if self.race_busy:
                    self.status.update(request_id=77, state="broadcasting", busy=True)
                    response = dict(self.status, error="busy")
                else:
                    self.status.update(
                        request_id=request_id,
                        state="queued",
                        busy=True,
                        accepted_requests=self.status["accepted_requests"] + 1,
                    )
                    self.started = self.clock.now
                    self.bursts += 1
                    response = dict(self.status)
                if self.disconnect_on_wake:
                    self.closed = True
                    raise serial.SerialException(
                        "USB disconnected after command delivery"
                    )
            elif command != b"SPWB1 STATUS":
                response["error"] = "malformed"
            elif self.preflight_change:
                response.update(self.preflight_change)
            if not self.silent:
                self.output.extend(
                    b"SPWB1 " + json.dumps(response).encode("ascii") + b"\r\n"
                )
        return len(data)

    def read(self, size):
        if self.closed:
            raise serial.SerialException("USB disconnected")
        if self.output:
            # Byte-at-a-time delivery proves no reliance on complete read packets.
            result = bytes(self.output[:1])
            del self.output[:1]
            return result
        self.clock.sleep(self.timeout)
        return b""

    def close(self):
        self.closed = True
        if self.fail_close:
            raise serial.SerialException("USB disconnected during close")


@pytest.fixture
def beacon(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(wake.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(wake.time, "sleep", clock.sleep)
    monkeypatch.setattr(wake.secrets, "randbelow", lambda maximum: 41)
    device = SimulatedBeacon(clock)
    monkeypatch.setattr(wake.serial, "Serial", lambda **kwargs: device)
    return device


def test_fragmented_burst_completes_once_and_excludes_retained_id(beacon):
    beacon.status.update(request_id=42, state="complete")
    result = wake.request_wake("COM5", timeout=1)
    assert result.state == "complete"
    assert result.request_id == 43
    assert result.completed_bursts == 1
    assert beacon.bursts == 1
    assert beacon.commands[0] == b"SPWB1 STATUS"
    assert [c for c in beacon.commands if c.startswith(b"SPWB1 WAKE")] == [
        b"SPWB1 WAKE 43"
    ]
    assert beacon.closed


@pytest.mark.parametrize(
    "change",
    [
        {"role": "controller"},
        {"firmware": 100},
        {"protocol": True},
        {"radio_ready": 1},
        {"controller_hosting": True},
        {"request_id": -1},
        {"completed_bursts": 0x100000000},
        {"busy": None},
        {"state": ["idle"]},
        {"unexpected": 1},
    ],
)
def test_unsafe_preflight_never_wakes(beacon, change):
    beacon.preflight_change = change
    with pytest.raises(wake.ProtocolError):
        wake.request_wake("/dev/ttyACM0", timeout=1)
    assert beacon.bursts == 0
    assert beacon.commands == [b"SPWB1 STATUS"]
    assert beacon.closed


@pytest.mark.parametrize(
    "raw",
    [
        b'SPWB1 {"protocol":1,"protocol":1}\n',
        b"SPWB1 {}\n",
        b"diagnostic output\n",
        b"SPWB1 {}",
        b"SPWB1 \xff\n",
        b"SPWB1 " + b" " * wake.MAX_RESPONSE_BYTES + b"\n",
    ],
)
def test_invalid_framing_and_ambiguous_json_rejected(raw):
    with pytest.raises(wake.ProtocolError):
        wake.decode_status(raw)


def test_busy_race_is_command_rejection_not_our_operation(beacon):
    beacon.race_busy = True
    with pytest.raises(wake.CommandRejected) as error:
        wake.request_wake("COM5", timeout=1)
    assert error.value.status.error == "busy"
    assert error.value.status.request_id == 77
    assert error.value.request_id == 42
    assert beacon.bursts == 0
    assert len(beacon.commands) == 2
    assert beacon.closed


@pytest.mark.parametrize("finish", ["cleanup_failure", "counter_failure"])
def test_failure_wins_over_completed_counter(beacon, finish):
    beacon.finish = finish
    with pytest.raises(wake.OperationFailed) as error:
        wake.request_wake("COM5", timeout=1)
    assert error.value.status.failures == 1
    assert error.value.status.completed_bursts == 1
    assert error.value.status.error == ""
    assert beacon.bursts == 1
    assert beacon.closed


@pytest.mark.parametrize("finish", ["reset", "replaced"])
def test_reset_or_unrelated_completion_never_rebroadcasts(beacon, finish):
    beacon.finish = finish
    with pytest.raises(wake.RequestLost) as error:
        wake.request_wake("COM5", timeout=1)
    assert error.value.request_id == 42
    assert error.value.status.request_id != 42
    assert error.value.wake_sent
    assert beacon.bursts == 1
    assert beacon.closed


def test_disconnect_after_delivery_has_uncertain_outcome_without_retry(beacon):
    beacon.disconnect_on_wake = True
    with pytest.raises(wake.TransportError) as error:
        wake.request_wake("COM5", timeout=1)
    assert error.value.wake_sent
    assert error.value.request_id == 42
    assert beacon.bursts == 1
    assert beacon.commands == [b"SPWB1 STATUS", b"SPWB1 WAKE 42"]
    assert beacon.closed


def test_partial_write_never_retries_or_wakes(beacon):
    beacon.partial_write = True
    with pytest.raises(wake.TransportError):
        wake.request_wake("COM5", timeout=1)
    assert beacon.input == b"SPW"
    assert beacon.bursts == 0
    assert beacon.closed


@pytest.mark.parametrize("silent", [False, True])
def test_deadline_bounds_silence_and_endless_broadcast(beacon, silent):
    beacon.silent = silent
    beacon.finish = "never"
    with pytest.raises(wake.BeaconTimeout):
        wake.request_wake("COM5", timeout=0.5)
    assert beacon.clock.now == pytest.approx(0.5)
    assert beacon.bursts == (0 if silent else 1)
    assert beacon.closed


def test_unterminated_response_is_bounded_without_wake(beacon):
    beacon.silent = True
    beacon.output.extend(b"x" * (wake.MAX_RESPONSE_BYTES + 1))
    with pytest.raises(wake.ProtocolError):
        wake.request_wake("COM5", timeout=1)
    assert beacon.bursts == 0
    assert beacon.closed


def test_status_reports_retained_failure_without_actuating(beacon, capsys):
    beacon.status.update(request_id=12, state="failed", failures=1)
    assert wake.main(["--port", "COM5", "--status", "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["status"]["state"] == "failed"
    assert result["action"] == "status"
    assert beacon.commands == [b"SPWB1 STATUS"]
    assert beacon.bursts == 0
    assert beacon.closed


def test_json_operation_failure_preserves_device_status(beacon, capsys):
    beacon.finish = "cleanup_failure"
    assert wake.main(["--port", "COM5", "--json"]) == 1
    captured = capsys.readouterr()
    result = json.loads(captured.out)
    assert captured.err == ""
    assert result["ok"] is False
    assert result["error"] == "operation_failed"
    assert result["status"]["error"] == ""
    assert result["status"]["failures"] == 1
    assert result["request_id"] == 42
    assert result["wake_sent"] is True


def test_discovery_selects_only_unique_beacon_pid(monkeypatch):
    ports = [
        SimpleNamespace(device="COM3", vid=0xCAFE, pid=0x4000),
        SimpleNamespace(device="COM4", vid=0x1234, pid=0x4030),
        SimpleNamespace(device="COM5", vid=0xCAFE, pid=0x4030),
    ]
    monkeypatch.setattr(wake.list_ports, "comports", lambda: ports)
    assert wake.discover_beacon_port() == "COM5"
    ports.pop()
    with pytest.raises(wake.TransportError):
        wake.discover_beacon_port()


def test_multiple_beacons_fail_before_port_open_or_wake(beacon, monkeypatch):
    monkeypatch.setattr(
        wake.list_ports,
        "comports",
        lambda: [
            SimpleNamespace(device=path, vid=0xCAFE, pid=0x4030)
            for path in ("COM5", "COM6")
        ],
    )
    with pytest.raises(wake.TransportError):
        wake.request_wake(timeout=1)
    assert not beacon.closed
    assert beacon.commands == []
    assert beacon.bursts == 0


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_invalid_deadline_never_opens_transport(beacon, timeout):
    with pytest.raises(ValueError):
        wake.request_wake("COM5", timeout=timeout)
    assert not beacon.closed
    assert beacon.commands == []


@pytest.mark.parametrize(
    "initial, error_type",
    [
        (
            {"busy": True, "state": "broadcasting", "request_id": 9},
            wake.CommandRejected,
        ),
        ({"configured": False, "state": "unconfigured"}, wake.OperationFailed),
    ],
)
def test_preflight_busy_or_unconfigured_never_sends_wake(beacon, initial, error_type):
    beacon.status.update(initial)
    with pytest.raises(error_type) as error:
        wake.request_wake("COM5", timeout=1)
    assert error.value.status.state == initial["state"]
    assert error.value.wake_sent is False
    assert beacon.commands == [b"SPWB1 STATUS"]
    assert beacon.bursts == 0
    assert beacon.closed


@pytest.mark.parametrize("startup_busy", [False, True])
def test_startup_waits_for_configuration_before_one_wake(beacon, startup_busy):
    beacon.status.update(radio_ready=False, configured=False, busy=startup_busy)
    beacon.ready_at = 0.2
    result = wake.request_wake("COM5", timeout=1)
    assert result.state == "complete"
    assert beacon.commands[:3] == [b"SPWB1 STATUS"] * 3
    assert beacon.commands[3] == b"SPWB1 WAKE 42"
    assert beacon.bursts == 1
    assert beacon.closed


def test_stalled_startup_times_out_without_wake(beacon):
    beacon.status.update(radio_ready=False, configured=False)
    with pytest.raises(wake.BeaconTimeout) as error:
        wake.request_wake("COM5", timeout=0.5)
    assert error.value.wake_sent is False
    assert beacon.bursts == 0
    assert set(beacon.commands) == {b"SPWB1 STATUS"}
    assert beacon.clock.now == pytest.approx(0.5)
    assert beacon.closed


def test_protocol_compatible_patch_firmware_can_wake(beacon):
    beacon.status["firmware"] = "1.0.1"
    result = wake.request_wake("COM5", timeout=1)
    assert result.state == "complete"
    assert result.firmware == "1.0.1"
    assert beacon.bursts == 1


@pytest.mark.parametrize("firmware", ["", " " * 3, "v" * 65])
def test_invalid_firmware_identifier_blocks_wake(beacon, firmware):
    beacon.status["firmware"] = firmware
    with pytest.raises(wake.ProtocolError):
        wake.request_wake("COM5", timeout=1)
    assert beacon.commands == [b"SPWB1 STATUS"]
    assert beacon.bursts == 0


def test_close_failure_preserves_completed_wake_context(beacon, capsys):
    beacon.fail_close = True
    assert wake.main(["--port", "COM5", "--json"]) == 1
    captured = capsys.readouterr()
    result = json.loads(captured.out)
    assert captured.err == ""
    assert result["error"] == "transport_error"
    assert result["request_id"] == 42
    assert result["wake_sent"] is True
    assert result["status"]["state"] == "complete"
    assert result["status"]["completed_bursts"] == 1
    assert beacon.bursts == 1
