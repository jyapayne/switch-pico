import shutil
import subprocess
from pathlib import Path
from dataclasses import replace

from switch_pico_bridge import config_manager


def test_controller_profile_native(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler is not None, "a host C++ compiler is required"

    executable = tmp_path / "controller_profile_test"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{root / 'src' / 'firmware'}",
            str(root / "tests" / "controller_profile_test.cpp"),
            str(root / "src" / "firmware" / "core" / "controller_identity.cpp"),
            str(root / "src" / "firmware" / "profile" / "controller_profile.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)

    legacy = subprocess.run(
        [str(executable), "--legacy9"], check=True, cwd=root, capture_output=True
    ).stdout
    migrated = config_manager.ControllerProfile.from_bytes(legacy)
    assert migrated.to_bytes()[2:] == legacy[2:]
    assert migrated.native_joycon_layout == 0
    assert migrated.swap_sticks is False
    assert migrated.swing.macro == 0
    assert migrated.nunchuk_swing.button == 15
    assert migrated.combined_swing.macro == 3
    upgraded = subprocess.run(
        [str(executable), "--codec"],
        input=legacy, check=True, cwd=root, capture_output=True,
    ).stdout
    assert upgraded == migrated.to_bytes()

    legacy10 = subprocess.run(
        [str(executable), "--legacy10"], check=True, cwd=root, capture_output=True
    ).stdout
    migrated10 = config_manager.ControllerProfile.from_bytes(legacy10)
    assert legacy10[:4] == bytes((10, 0, 128, 1))
    assert migrated10.native_joycon_layout == 2
    assert migrated10.swap_sticks is True
    assert migrated10.button_map[0] == 18
    assert migrated10.extra_button_map[0] == 19
    assert migrated10.shift.button_map[0] == 20
    assert migrated10.shift.extra_button_map[0] == 21
    assert migrated10.left_trigger.output == migrated10.right_trigger.output == 21
    assert migrated10.to_bytes()[2:] == legacy10[2:]
    upgraded10 = subprocess.run(
        [str(executable), "--codec"],
        input=legacy10, check=True, cwd=root, capture_output=True,
    ).stdout
    assert upgraded10 == migrated10.to_bytes()

    directions = replace(
        migrated10,
        button_map=(*migrated10.button_map[:12], 22, 23, 24, 25),
        extra_button_map=(24, *migrated10.extra_button_map[1:]),
        shift=replace(
            migrated10.shift,
            button_map=(23, *migrated10.shift.button_map[1:]),
            extra_button_map=(25, *migrated10.shift.extra_button_map[1:]),
        ),
        left_trigger=replace(migrated10.left_trigger, output=22),
        right_trigger=replace(migrated10.right_trigger, output=22),
    )
    direction_wire = directions.to_bytes()
    native_directions = subprocess.run(
        [str(executable), "--codec"],
        input=direction_wire, check=True, cwd=root, capture_output=True,
    ).stdout
    assert native_directions == direction_wire
    assert config_manager.ControllerProfile.from_bytes(native_directions) == directions
    assert config_manager.ControllerProfile.from_json(directions.to_json()) == directions

    for layout in range(len(config_manager.NATIVE_JOYCON_LAYOUTS)):
        for swap in (False, True):
            profile = replace(
                migrated,
                native_joycon_layout=layout,
                swap_sticks=swap,
                button_map=(18, *migrated.button_map[1:]),
                extra_button_map=(19, *migrated.extra_button_map[1:]),
                shift=replace(
                    migrated.shift,
                    button_map=(20, *migrated.shift.button_map[1:]),
                    extra_button_map=(21, *migrated.shift.extra_button_map[1:]),
                ),
                left_trigger=replace(migrated.left_trigger, output=21),
                right_trigger=replace(migrated.right_trigger, output=21),
            )
            encoded = profile.to_bytes()
            native = subprocess.run(
                [str(executable), "--codec"],
                input=encoded, check=True, cwd=root, capture_output=True,
            ).stdout
            assert native == encoded
            assert config_manager.ControllerProfile.from_bytes(native) == profile
