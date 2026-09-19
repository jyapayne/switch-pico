from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("controller_count", [2, 4], ids=["one-pair", "two-pair"])
def test_native_hub_trace_lifecycle(tmp_path: Path, controller_count: int) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    executable = tmp_path / "native_hub_trace_test"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-ffunction-sections",
            "-fdata-sections",
            "-DSWITCH2_PROBE_HUB=1",
            "-DSWITCH2_PROBE_TRACE_NATIVE_INPUT=1",
            f"-DPROBE_CONTROLLER_COUNT={controller_count}",
            f"-I{root / 'tests' / 'native_hub_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            f"-I{root / 'tools' / 'pico_usb_address_probe'}",
            f"-I{root / 'tools' / 'switch2_usb_probe'}",
            str(root / "tests" / "native_hub_trace_test.c"),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    for scenario in (
        "live-wrap",
        "root-idle",
        "queue-pressure",
        "pending",
        "superseded",
        "immediate-supersession",
        "poll-retention",
        "selection-history",
        "frozen-selection-history",
        "delayed-publication",
        "publication-isolation",
        "publication-wrap-supersession",
        "ep0-handover",
        "coherent-publication",
        "bulk-commit-pids",
        "approved-status-handoff",
        "approved-status-superseded",
        "approved-status-reset",
        "approved-status-reset-during-completion",
        "approved-status-port-reset-ready",
        "approved-status-port-reset-queued",
        "port-reset-interrupt-progress",
        "port-reset-interrupt-reset",
        "approved-status-invalid-length",
        "approved-status-watch",
        "status-out-rejected-data",
        "status-out",
        "status-out-superseded",
        "status-out-stale",
        "status-out-reset",
        "status-out-reset-watch",
        "status-out-port-reset-watch",
        "marker-zero",
        "marker-active",
        "marker-rejected",
    ):
        subprocess.run([str(executable), scenario], check=True, cwd=root)

    for mode in ("waiting", "partial"):
        subprocess.run([str(executable), "marker-priority", mode], check=True, cwd=root)

    # Identical snapshots must reach the consumer in identical order even when
    # every individual header, record, and END is rejected twice by the logger.
    outputs = []
    for mode in ("open", "full"):
        result = subprocess.run(
            [str(executable), "backpressure", mode],
            capture_output=True,
            text=True,
            check=True,
            cwd=root,
        )
        outputs.append(result.stdout)
    assert outputs[0] == outputs[1], (
        "logger backpressure skipped, reordered, or changed dump lines"
    )
