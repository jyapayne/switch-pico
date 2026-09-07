from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_bluepad32_backend_lifecycle_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    for adapter_feasibility, native, short_packets in (
        (False, False, False),
        (True, False, False),
        (True, True, False),
        (True, True, True),
    ):
        suffix = (
            "_native32"
            if short_packets
            else "_native64"
            if native
            else "_adapter"
            if adapter_feasibility
            else ""
        )
        executable = tmp_path / f"bluepad32_backend_lifecycle_test{suffix}"
        command = [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-DSWITCH_PICO_HID_INSTANCE_COUNT=4",
        ]
        if adapter_feasibility:
            command.append("-DSWITCH_PICO_USB_OUTPUT_MODES=1")
        if native:
            command.extend(
                [
                    "-DSWITCH_PICO_HAPTICS_EXPERIMENT=1",
                    "-DSWITCH_PICO_HD_RUMBLE=1",
                    "-DSWITCH_PICO_HAPTICS_EXPERIMENT_RAM=0",
                    str(root / "src" / "firmware" / "input" / "haptics_experiment.cpp"),
                    str(
                        root
                        / "src"
                        / "firmware"
                        / "input"
                        / "native_output_scheduler.cpp"
                    ),
                    str(
                        root
                        / "src"
                        / "firmware"
                        / "input"
                        / "switch_hd_rumble_synth.cpp"
                    ),
                    str(
                        root
                        / "src"
                        / "firmware"
                        / "usb"
                        / "switch"
                        / "switch_haptics.cpp"
                    ),
                ]
            )
        if short_packets:
            command.extend(
                [
                    "-DSWITCH_PICO_CYW43_PACKET_READ=1",
                    "-DSWITCH_PICO_HCI_CREDIT_BATCH=1",
                    "-DSWITCH_PICO_SYS_CLOCK_MHZ=300",
                    "-DSWITCH_PICO_HD_PACKET_FRAMES=32",
                ]
            )
        command.extend(
            [
                f"-I{root / 'tests' / 'bluepad32_native_stubs'}",
                f"-I{root / 'src' / 'firmware'}",
                f"-I{root / 'bluepad32_config'}",
                str(root / "tests" / "bluepad32_backend_lifecycle_test.cpp"),
                str(root / "bluepad32_config" / "parser" / "uni_switch2_haptics.c"),
                str(
                    root / "src" / "firmware" / "input" / "controller_macro_capture.cpp"
                ),
                "-o",
                str(executable),
            ]
        )
        subprocess.run(command, check=True, cwd=root)
        subprocess.run([str(executable), "xbox-rumble"], check=True, cwd=root)
        for scenario in (
            "switch2-forward",
            "switch2-reverse",
            "switch2-multiple-pairs",
            "switch2-admission",
            "switch2-radio-policy",
            "switch2-radio-settling",
            "switch2-mate-reconnect",
            "switch2-mate-pending",
            "switch2-mate-pairing-window",
            "switch2-pairing-inventory",
            "switch2-hd-pro",
            "switch2-hd-solo-left",
            "switch2-hd-solo-right",
            "switch2-hd-pair",
            "switch2-hd-overflow",
            "switch2-hd-epochs",
            "switch2-hd-feedback",
        ):
            subprocess.run([str(executable), scenario], check=True, cwd=root)
        if native:
            subprocess.run([str(executable), "native-stateful"], check=True, cwd=root)
            subprocess.run(
                [str(executable), "native-second-slot"], check=True, cwd=root
            )
            continue

        for scenario in (
            "ready-forward",
            "ready-reverse",
            "rejections",
            "lifecycle",
            "pairing-policy",
            "slot-lighting",
            "profile-chord-raw",
            "profile-feedback",
            "stateful-rumble",
            "motion-hotkey",
            "analog-state",
            "rumble-mode",
            "clear-pairings",
            "configuration-timer",
            "flash-core-start",
            "wake-identity-gate",
            "system-wake",
            "flash-core-failure",
        ):
            subprocess.run([str(executable), scenario], check=True, cwd=root)
