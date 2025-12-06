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

ELF_PATH = Path(os.environ.get("ELF_PATH", BUILD_DIR / "switch-pico.elf")).expanduser()

MACROS = (
    "SWITCH_COLOR_LEFT_GRIP_R",
    "SWITCH_COLOR_LEFT_GRIP_G",
    "SWITCH_COLOR_LEFT_GRIP_B",
    "SWITCH_COLOR_RIGHT_GRIP_R",
    "SWITCH_COLOR_RIGHT_GRIP_G",
    "SWITCH_COLOR_RIGHT_GRIP_B",
)

def parse_args():
    parser = argparse.ArgumentParser(
        description="Build and flash the project, optionally setting grip colors.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Default behavior leaves controller_color_config.h unchanged.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--random-grip-color",
        action="store_true",
        help="Randomize both grip colors before building.",
    )
    group.add_argument(
        "--grip-color",
        metavar="RRGGBB",
        help="Set both grip colors to the provided hex value.",
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

    values = (r, g, b, r, g, b)
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

def build():
    run_cmd(
        [
            "cmake",
            "-S",
            str(SCRIPT_DIR),
            "-B",
            str(BUILD_DIR),
            "-DSWITCH_PICO_AUTOTEST=OFF",
            "-DSWITCH_PICO_LOG=OFF",
        ]
    )
    run_cmd(["cmake", "--build", str(BUILD_DIR)])

def flash():
    picotool = resolve_picotool()
    if not ELF_PATH.exists():
        sys.stderr.write(
            f"Error: Cannot find ELF at {ELF_PATH}. Set ELF_PATH to override.\n"
        )
        sys.exit(1)
    run_cmd([str(picotool), "load", str(ELF_PATH), "-fx"])

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

    build()
    flash()

if __name__ == "__main__":
    main()
