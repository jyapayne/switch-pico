import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "configure_switch2_wake",
    ROOT / "tools" / "configure_switch2_wake.py",
)
configure = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(configure)

RAW_HEX = "0201061BFF53050100037E056620000181BDD6F7EBF1480F00000000000000"
RECORD = {
    "version": 1,
    "advertiser": "98:E2:55:07:DF:00",
    "address_type": 0,
    "event_type": 0,
    "rssi": -67,
    "pid": "2066",
    "console": "48:F1:EB:F7:D6:BD",
    "raw_hex": RAW_HEX,
    "esphome_payload_hex": RAW_HEX[14:],
}


def test_valid_capture_generates_build_configuration(tmp_path: Path) -> None:
    capture_log = tmp_path / "capture.log"
    capture_log.write_text(
        "scanner startup\nSWITCH2_WAKE_CAPTURE " + json.dumps(RECORD) + "\n",
        encoding="utf-8",
    )
    output = tmp_path / "switch2_wake_config.h"

    assert configure.main(
        ["--input", str(capture_log), "--output", str(output)]
    ) == 0
    generated = output.read_text(encoding="utf-8")
    assert "#define SWITCH2_WAKE_CONFIGURED 1" in generated
    assert "0x98, 0xE2, 0x55, 0x07, 0xDF, 0x00" in generated
    assert "0x02, 0x01, 0x06, 0x1B, 0xFF" in generated
    assert "Target Switch 2: 48:F1:EB:F7:D6:BD" in generated


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("address_type", 1, "public advertiser"),
        ("event_type", 3, "ADV_IND"),
        ("console", "48:F1:EB:F7:D6:BE", "console address"),
        ("pid", "2067", "PID"),
    ],
)
def test_inconsistent_capture_is_rejected(field, value, message) -> None:
    record = dict(RECORD)
    record[field] = value
    with pytest.raises(configure.CaptureError, match=message):
        configure.validate_capture(record)


def test_non_wake_packet_is_rejected() -> None:
    record = dict(RECORD)
    raw = bytearray.fromhex(RAW_HEX)
    raw[16] = 0
    record["raw_hex"] = raw.hex()
    with pytest.raises(configure.CaptureError, match="wake flag"):
        configure.validate_capture(record)
