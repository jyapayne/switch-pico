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
FIRMWARE_SOURCE_DIR = SCRIPT_DIR / "src" / "firmware"
CONFIG_FILE = FIRMWARE_SOURCE_DIR / "platform" / "pico" / "controller_color_config.h"
BUILD_DIR = SCRIPT_DIR / "build"
AIO_BUILD_DIR = SCRIPT_DIR / "build-aio"
FEASIBILITY_BUILD_DIR = SCRIPT_DIR / "build-feasibility"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
FIRMWARE_ELF_PATH = FIRMWARE_DIR / "switch-pico.elf"
FIRMWARE_UF2_PATH = FIRMWARE_DIR / "switch-pico.uf2"
AIO_FIRMWARE_ELF_PATH = FIRMWARE_DIR / "switch-pico-aio.elf"
AIO_FIRMWARE_UF2_PATH = FIRMWARE_DIR / "switch-pico-aio.uf2"
FEASIBILITY_FIRMWARE_ELF_PATH = (
    FIRMWARE_DIR / "switch-pico-adapter-feasibility.elf"
)
FEASIBILITY_FIRMWARE_UF2_PATH = (
    FIRMWARE_DIR / "switch-pico-adapter-feasibility.uf2"
)

ELF_PATH = Path(os.environ.get("ELF_PATH", BUILD_DIR / "switch-pico.elf")).expanduser()
UF2_PATH = Path(os.environ.get("UF2_PATH", BUILD_DIR / "switch-pico.uf2")).expanduser()

MACROS = tuple(
    f"SWITCH_COLOR_SLOT_{slot}_{component}"
    for slot in range(1, 5)
    for component in ("R", "G", "B")
)

CMAKE_CACHE_PATHS = tuple(
    build_dir / "CMakeCache.txt"
    for build_dir in (BUILD_DIR, AIO_BUILD_DIR, FEASIBILITY_BUILD_DIR)
)
TOOLCHAIN_COMPILER = (
    "arm-none-eabi-gcc.exe" if os.name == "nt" else "arm-none-eabi-gcc"
)


class BuildEnvironmentError(RuntimeError):
    """A required Pico build dependency could not be resolved."""


def parse_cmake_cache(cache_path):
    """Return the simple key/value entries from an existing CMake cache."""
    cache_path = Path(cache_path)
    try:
        lines = cache_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError):
        return {}

    entries = {}
    for line in lines:
        if not line or line.startswith(("//", "#")) or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        if key:
            entries[key] = value
    return entries


def _cache_path(value, cache_path):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = cache_path.parent / path
    return path


def _versioned_candidates(parent):
    try:
        return sorted(
            (path for path in parent.iterdir() if path.is_dir()),
            key=lambda path: path.name,
            reverse=True,
        )
    except OSError:
        return []


def _sdk_fallback_candidates():
    yield ("project-local install", BUILD_DIR / "_deps" / "pico_sdk-src")
    pico_sdk_home = Path.home() / ".pico-sdk" / "sdk"
    for path in _versioned_candidates(pico_sdk_home):
        yield ("user Pico SDK install", path)
    yield ("user Pico SDK install", Path.home() / "pico" / "pico-sdk")
    yield ("user Pico SDK install", Path.home() / "pico-sdk")
    yield ("system Pico SDK install", Path("/opt/pico-sdk"))
    yield ("system Pico SDK install", Path("/usr/local/pico-sdk"))
    yield ("system Pico SDK install", Path("/usr/share/pico-sdk"))


def _toolchain_fallback_candidates():
    yield ("project-local install", BUILD_DIR / "toolchain")
    pico_toolchain_home = Path.home() / ".pico-sdk" / "toolchain"
    for path in _versioned_candidates(pico_toolchain_home):
        yield ("user Pico toolchain install", path)
    yield (
        "user Pico toolchain install",
        Path.home() / "pico" / "arm-none-eabi-gcc",
    )
    yield ("user Pico toolchain install", Path.home() / "arm-none-eabi-gcc")
    yield ("system Pico toolchain install", Path("/opt/arm-none-eabi-gcc"))
    yield (
        "system Pico toolchain install",
        Path("/usr/local/arm-none-eabi-gcc"),
    )


def _valid_sdk(path):
    return (path / "pico_sdk_init.cmake").is_file()


def _valid_toolchain(path):
    return (path / "bin" / TOOLCHAIN_COMPILER).is_file()


def _first_valid_candidate(candidates, validator):
    for source, candidate in candidates:
        path = Path(candidate).expanduser()
        if validator(path):
            return path, source
    return None, None


def _cache_candidates(cache_paths, variables):
    for cache_path in cache_paths:
        cache_path = Path(cache_path)
        entries = parse_cmake_cache(cache_path)
        for variable, compiler_path in variables:
            value = entries.get(variable)
            if not value:
                continue
            path = _cache_path(value, cache_path)
            if compiler_path:
                path = path.parent.parent
            yield (f"cache {cache_path}", path)


def _explicit_path(environ, variable, validator, expected):
    if variable not in environ:
        return None
    value = environ[variable]
    if not value:
        raise BuildEnvironmentError(f"{variable} is set but empty.")
    path = Path(value).expanduser()
    if not validator(path):
        raise BuildEnvironmentError(
            f"{variable} is set to {path}, but {expected} was not found."
        )
    return path


def configure_pico_environment(
    *,
    environ=None,
    cache_paths=None,
    sdk_candidates=None,
    toolchain_candidates=None,
    which=None,
):
    """Resolve Pico dependencies and apply auto-detected environment values."""
    environ = os.environ if environ is None else environ
    cache_paths = CMAKE_CACHE_PATHS if cache_paths is None else cache_paths

    which = shutil.which if which is None else which
    explicit_sdk = _explicit_path(
        environ,
        "PICO_SDK_PATH",
        _valid_sdk,
        "pico_sdk_init.cmake",
    )
    explicit_toolchain = _explicit_path(
        environ,
        "PICO_TOOLCHAIN_PATH",
        _valid_toolchain,
        f"bin/{TOOLCHAIN_COMPILER}",
    )

    sdk_path = explicit_sdk
    sdk_source = None
    if sdk_path is None:
        sdk_path, sdk_source = _first_valid_candidate(
            _cache_candidates(
                cache_paths,
                (("PICO_SDK_PATH", False),),
            ),
            _valid_sdk,
        )
        if sdk_path is None:
            sdk_path, sdk_source = _first_valid_candidate(
                (
                    _sdk_fallback_candidates()
                    if sdk_candidates is None
                    else sdk_candidates
                ),
                _valid_sdk,
            )

    compiler_on_path = which(
        TOOLCHAIN_COMPILER,
        path=environ.get("PATH", ""),
    )
    toolchain_path = explicit_toolchain
    toolchain_source = None
    if toolchain_path is None and compiler_on_path is None:
        toolchain_path, toolchain_source = _first_valid_candidate(
            _cache_candidates(
                cache_paths,
                (
                    ("PICO_TOOLCHAIN_PATH", False),
                    ("CMAKE_C_COMPILER", True),
                    ("PICO_COMPILER_CC", True),
                ),
            ),
            _valid_toolchain,
        )
        if toolchain_path is None:
            toolchain_path, toolchain_source = _first_valid_candidate(
                (
                    _toolchain_fallback_candidates()
                    if toolchain_candidates is None
                    else toolchain_candidates
                ),
                _valid_toolchain,
            )

    missing = []
    if sdk_path is None:
        missing.append("Pico SDK (set PICO_SDK_PATH)")
    if toolchain_path is None and compiler_on_path is None:
        missing.append(
            "Arm GNU toolchain "
            "(set PICO_TOOLCHAIN_PATH or add arm-none-eabi-gcc to PATH)"
        )
    if missing:
        raise BuildEnvironmentError(
            "Missing build prerequisite(s): " + "; ".join(missing) + "."
        )

    updates = {}
    detected = []
    if explicit_sdk is None:
        updates["PICO_SDK_PATH"] = str(sdk_path)
        detected.append(("PICO_SDK_PATH", sdk_path, sdk_source))
    if explicit_toolchain is None and compiler_on_path is None:
        updates["PICO_TOOLCHAIN_PATH"] = str(toolchain_path)
        detected.append(
            ("PICO_TOOLCHAIN_PATH", toolchain_path, toolchain_source)
        )

    environ.update(updates)
    for variable, path, source in detected:
        print(f"Auto-detected {variable}={path} ({source})")
    return updates

def parse_args():
    parser = argparse.ArgumentParser(
        description="Build and flash the project, optionally setting grip colors.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"Default behavior leaves {CONFIG_FILE.relative_to(SCRIPT_DIR)} unchanged.",
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--aio",
        action="store_true",
        help="Build and flash the Pico 2 W Bluepad32 all-in-one firmware.",
    )
    mode_group.add_argument(
        "--adapter-feasibility",
        action="store_true",
        help="Build and flash the Pico 2 W automatic Switch/XInput prototype.",
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
    adapter_feasibility,
    build_dir,
    elf_path,
    uf2_path,
    firmware_elf_path,
    firmware_uf2_path,
):
    if aio or adapter_feasibility:
        run_cmd([sys.executable, str(SCRIPT_DIR / "tools" / "prepare_bluepad32.py")])
        definitions = [
            "-DSWITCH_PICO_LOG=OFF",
            "-DPICO_BOARD=pico2_w",
            "-DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32",
        ]
        if adapter_feasibility:
            definitions.append("-DSWITCH_PICO_ADAPTER_FEASIBILITY=ON")
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
    try:
        configure_pico_environment()
    except BuildEnvironmentError as exc:
        sys.stderr.write(f"Error: {exc}\n")
        sys.exit(1)

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

    if args.adapter_feasibility:
        build_dir = FEASIBILITY_BUILD_DIR
        elf_path = FEASIBILITY_BUILD_DIR / "switch-pico.elf"
        uf2_path = FEASIBILITY_BUILD_DIR / "switch-pico.uf2"
        firmware_elf_path = FEASIBILITY_FIRMWARE_ELF_PATH
        firmware_uf2_path = FEASIBILITY_FIRMWARE_UF2_PATH
    elif args.aio:
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
        args.adapter_feasibility,
        build_dir,
        elf_path,
        uf2_path,
        firmware_elf_path,
        firmware_uf2_path,
    )
    flash(
        elf_path,
        allow_elf_override=not args.aio and not args.adapter_feasibility,
    )

if __name__ == "__main__":
    main()
