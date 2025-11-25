#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

CONFIG_FILE="$SCRIPT_DIR/controller_color_config.h"

usage() {
  cat <<'EOF'
Usage: build.sh [--random-grip-color | --grip-color RRGGBB]
  --random-grip-color    Randomize both grip colors before building.
  --grip-color RRGGBB    Set both grip colors to the provided hex value.
  --help                 Show this help message.

Default behavior leaves controller_color_config.h unchanged.
EOF
}

ensure_single_color_mode() {
  local current_mode="$1"
  local new_mode="$2"
  if [[ "$current_mode" != "none" && "$current_mode" != "$new_mode" ]]; then
    echo "Error: Choose either --random-grip-color or --grip-color, not both." >&2
    exit 1
  fi
}

update_grip_colors() {
  local rgb_hex="$1"
  local r="${rgb_hex:0:2}"
  local g="${rgb_hex:2:2}"
  local b="${rgb_hex:4:2}"

  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Error: Cannot find $CONFIG_FILE" >&2
    exit 1
  fi

  python - "$CONFIG_FILE" "$r" "$g" "$b" <<'PY'
import re
import sys
path, r, g, b = sys.argv[1:5]

try:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
except OSError as exc:
    sys.stderr.write(f"Error reading {path}: {exc}\n")
    sys.exit(1)

def replace(name, val, data):
    pattern = rf"(?m)^(#define\s+{name}\s+)0x[0-9A-Fa-f]{{2}}"
    updated, count = re.subn(pattern, rf"\g<1>0x{val.upper()}", data)
    if count == 0:
        sys.stderr.write(f"Error: Could not find {name} in {path}\n")
        sys.exit(1)
    return updated

for macro, val in [
    ("SWITCH_COLOR_LEFT_GRIP_R", r),
    ("SWITCH_COLOR_LEFT_GRIP_G", g),
    ("SWITCH_COLOR_LEFT_GRIP_B", b),
    ("SWITCH_COLOR_RIGHT_GRIP_R", r),
    ("SWITCH_COLOR_RIGHT_GRIP_G", g),
    ("SWITCH_COLOR_RIGHT_GRIP_B", b),
]:
    text = replace(macro, val, text)

try:
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
except OSError as exc:
    sys.stderr.write(f"Error writing {path}: {exc}\n")
    sys.exit(1)
PY
}

GRIP_MODE="none"
CUSTOM_COLOR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --random-grip-color)
      ensure_single_color_mode "$GRIP_MODE" "random"
      GRIP_MODE="random"
      shift
      ;;
    --grip-color)
      ensure_single_color_mode "$GRIP_MODE" "custom"
      GRIP_MODE="custom"
      CUSTOM_COLOR="${2-}"
      if [[ -z "$CUSTOM_COLOR" ]]; then
        echo "Error: --grip-color requires a hex value like FF8800." >&2
        exit 1
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "$GRIP_MODE" == "random" ]]; then
  CUSTOM_COLOR="$(printf "%02X%02X%02X" $((RANDOM % 256)) $((RANDOM % 256)) $((RANDOM % 256)))"
elif [[ "$GRIP_MODE" == "custom" ]]; then
  if ! [[ "$CUSTOM_COLOR" =~ ^[0-9A-Fa-f]{6}$ ]]; then
    echo "Error: Color must be a 6-digit hex value like FF8800." >&2
    exit 1
  fi
fi

if [[ "$GRIP_MODE" != "none" ]]; then
  update_grip_colors "$CUSTOM_COLOR"
  echo "Grip color set to #$CUSTOM_COLOR in controller_color_config.h"
fi

cmake -S . -B build -DSWITCH_PICO_AUTOTEST=OFF -DSWITCH_PICO_LOG=OFF && cmake --build build
/Users/joey/.pico-sdk/picotool/2.2.0-a4/picotool/picotool load /Users/joey/Projects/switch-pico/build/switch-pico.elf -fx
