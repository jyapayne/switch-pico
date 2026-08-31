#!/usr/bin/env python3
"""Build and flash the project with optional grip color overrides."""
import argparse
import os
import random
import re
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_FILE = SCRIPT_DIR / "controller_color_config.h"
BUILD_DIR = SCRIPT_DIR / "build"
AIO_BUILD_DIR = SCRIPT_DIR / "build-aio"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
FIRMWARE_ELF_PATH = FIRMWARE_DIR / "switch-pico.elf"
FIRMWARE_UF2_PATH = FIRMWARE_DIR / "switch-pico.uf2"
AIO_FIRMWARE_ELF_PATH = FIRMWARE_DIR / "switch-pico-aio.elf"
AIO_FIRMWARE_UF2_PATH = FIRMWARE_DIR / "switch-pico-aio.uf2"

ELF_PATH = Path(os.environ.get("ELF_PATH", BUILD_DIR / "switch-pico.elf")).expanduser()
UF2_PATH = Path(os.environ.get("UF2_PATH", BUILD_DIR / "switch-pico.uf2")).expanduser()

MACROS = tuple(
    f"SWITCH_COLOR_SLOT_{slot}_{component}"
    for slot in range(1, 5)
    for component in ("R", "G", "B")
)

def parse_args():
    parser = argparse.ArgumentParser(
        description="Build and flash the project, optionally setting grip colors.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Default behavior leaves controller_color_config.h unchanged.",
    )
    parser.add_argument(
        "--aio",
        action="store_true",
        help="Build and flash the Pico 2 W Bluepad32 all-in-one firmware.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--random-grip-color",
        action="store_true",
        help="Assign one random color to every emulated controller slot.",
    )
    group.add_argument(
        "--grip-color",
        metavar="RRGGBB",
        help="Set every emulated controller slot to the provided hex color.",
    )
    return parser.parse_args()

def random_hex_color():
    return "".join(f"{random.randrange(256):02X}" for _ in range(3))

def validate_custom_color(value):
    if not re.fullmatch(r"[0-9A-Fa-f]{6}", value):
        raise ValueError("Color must be a 6-digit hex value like FF8800.")
    return value

def update_grip_colors(rgb_hex):
    if not CONFIG_FILE.exists():
        sys.stderr.write(f"Error: Cannot find {CONFIG_FILE}\n")
        sys.exit(1)

    r, g, b = rgb_hex[:2], rgb_hex[2:4], rgb_hex[4:6]

    try:
        text = CONFIG_FILE.read_text(encoding="utf-8")
    except OSError as exc:
        sys.stderr.write(f"Error reading {CONFIG_FILE}: {exc}\n")
        sys.exit(1)

    def replace(name, val, data):
        pattern = rf"(?m)^(#define\s+{name}\s+)0x[0-9A-Fa-f]{{2}}"
        updated, count = re.subn(pattern, rf"\g<1>0x{val.upper()}", data)
        if count == 0:
            sys.stderr.write(f"Error: Could not find {name} in {CONFIG_FILE}\n")
            sys.exit(1)
        return updated

    values = (r, g, b) * 4
    for macro, val in zip(MACROS, values):
        text = replace(macro, val, text)

    try:
        CONFIG_FILE.write_text(text, encoding="utf-8")
    except OSError as exc:
        sys.stderr.write(f"Error writing {CONFIG_FILE}: {exc}\n")
        sys.exit(1)

def run_cmd(command):
    try:
        subprocess.run(command, cwd=SCRIPT_DIR, check=True)
    except FileNotFoundError as exc:
        sys.stderr.write(f"Error running {command[0]}: {exc}\n")
        sys.exit(1)
    except subprocess.CalledProcessError as exc:
        sys.exit(exc.returncode)

def resolve_picotool():
    env_val = os.environ.get("PICOTOOL_PATH")
    if env_val:
        env_path = Path(env_val).expanduser()
        if not env_path.exists():
            sys.stderr.write(f"Error: PICOTOOL_PATH set to {env_path}, but it does not exist.\n")
            sys.exit(1)
        return env_path

    found = shutil.which("picotool")
    if found:
        return Path(found)

    sys.stderr.write("Error: picotool not found. Put it on your PATH or set PICOTOOL_PATH.\n")
    sys.exit(1)

def build(
    aio,
    build_dir,
    elf_path,
    uf2_path,
    firmware_elf_path,
    firmware_uf2_path,
):
    if aio:
        run_cmd([sys.executable, str(SCRIPT_DIR / "tools" / "prepare_bluepad32.py")])
        definitions = [
            "-DSWITCH_PICO_LOG=OFF",
            "-DPICO_BOARD=pico2_w",
            "-DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32",
        ]
    else:
        definitions = [
            "-DSWITCH_PICO_LOG=OFF",
            "-DPICO_BOARD=pico",
            "-DSWITCH_PICO_INPUT_BACKEND=UART",
        ]

    run_cmd(
        [
            "cmake",
            "-S",
            str(SCRIPT_DIR),
            "-B",
            str(build_dir),
            *definitions,
        ]
    )
    run_cmd(["cmake", "--build", str(build_dir)])

    missing_artifacts = [
        path for path in (elf_path, uf2_path) if not path.is_file()
    ]
    if missing_artifacts:
        missing = ", ".join(str(path) for path in missing_artifacts)
        sys.stderr.write(f"Error: Build did not produce required artifact(s): {missing}\n")
        sys.exit(1)
    FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy2(elf_path, firmware_elf_path)
    shutil.copy2(uf2_path, firmware_uf2_path)

    print(f"Built ELF: {elf_path}")
    print(f"Built UF2: {uf2_path}")
    print(f"Copied ELF: {firmware_elf_path}")
    print(f"Copied UF2: {firmware_uf2_path}")


def flash(elf_path, allow_elf_override):
    picotool = resolve_picotool()
    if not elf_path.exists():
        if allow_elf_override:
            sys.stderr.write(
                f"Error: Cannot find ELF at {elf_path}. Set ELF_PATH to override.\n"
            )
        else:
            sys.stderr.write(f"Error: Cannot find ELF at {elf_path}.\n")
        sys.exit(1)
    run_cmd([str(picotool), "load", str(elf_path), "-fx"])

def main():
    args = parse_args()
    color = None

    if args.random_grip_color:
        color = random_hex_color()
    elif args.grip_color:
        try:
            color = validate_custom_color(args.grip_color)
        except ValueError as exc:
            sys.stderr.write(f"Error: {exc}\n")
            sys.exit(1)

    if color:
        update_grip_colors(color)
        print(f"Grip color set to #{color} in {CONFIG_FILE.name}")

    if args.aio:
        build_dir = AIO_BUILD_DIR
        elf_path = AIO_BUILD_DIR / "switch-pico.elf"
        uf2_path = AIO_BUILD_DIR / "switch-pico.uf2"
        firmware_elf_path = AIO_FIRMWARE_ELF_PATH
        firmware_uf2_path = AIO_FIRMWARE_UF2_PATH
    else:
        build_dir = BUILD_DIR
        elf_path = ELF_PATH
        uf2_path = UF2_PATH
        firmware_elf_path = FIRMWARE_ELF_PATH
        firmware_uf2_path = FIRMWARE_UF2_PATH

    build(
        args.aio,
        build_dir,
        elf_path,
        uf2_path,
        firmware_elf_path,
        firmware_uf2_path,
    )
    flash(elf_path, allow_elf_override=not args.aio)

if __name__ == "__main__":
    main()
