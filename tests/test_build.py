import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("switch_pico_build", ROOT / "build.py")
build_script = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build_script)


def make_sdk(path):
    path.mkdir(parents=True)
    (path / "pico_sdk_init.cmake").touch()
    return path


def make_toolchain(path):
    bin_dir = path / "bin"
    bin_dir.mkdir(parents=True)
    (bin_dir / build_script.TOOLCHAIN_COMPILER).touch()
    return path


def no_compiler(_name, *, path):
    return None


def test_parse_cmake_cache_ignores_comments_and_malformed_lines(tmp_path):
    cache = tmp_path / "CMakeCache.txt"
    cache.write_text(
        "// comment\n"
        "# internal comment\n"
        "PICO_SDK_PATH:PATH=/sdk\n"
        "VALUE_WITH_EQUALS:STRING=left=right\n"
        "malformed\n",
        encoding="utf-8",
    )

    assert build_script.parse_cmake_cache(cache) == {
        "PICO_SDK_PATH": "/sdk",
        "VALUE_WITH_EQUALS": "left=right",
    }


def test_explicit_environment_wins_over_cache_and_fallback(tmp_path):
    explicit_sdk = make_sdk(tmp_path / "explicit-sdk")
    explicit_toolchain = make_toolchain(tmp_path / "explicit-toolchain")
    cached_sdk = make_sdk(tmp_path / "cached-sdk")
    cached_toolchain = make_toolchain(tmp_path / "cached-toolchain")
    fallback_sdk = make_sdk(tmp_path / "fallback-sdk")
    fallback_toolchain = make_toolchain(tmp_path / "fallback-toolchain")
    cache = tmp_path / "build" / "CMakeCache.txt"
    cache.parent.mkdir()
    cache.write_text(
        f"PICO_SDK_PATH:PATH={cached_sdk}\n"
        f"PICO_TOOLCHAIN_PATH:PATH={cached_toolchain}\n",
        encoding="utf-8",
    )
    environ = {
        "PATH": "",
        "PICO_SDK_PATH": str(explicit_sdk),
        "PICO_TOOLCHAIN_PATH": str(explicit_toolchain),
    }

    updates = build_script.configure_pico_environment(
        environ=environ,
        cache_paths=[cache],
        sdk_candidates=[("fallback", fallback_sdk)],
        toolchain_candidates=[("fallback", fallback_toolchain)],
        which=no_compiler,
    )

    assert updates == {}
    assert environ["PICO_SDK_PATH"] == str(explicit_sdk)
    assert environ["PICO_TOOLCHAIN_PATH"] == str(explicit_toolchain)


@pytest.mark.parametrize(
    ("variable", "invalid", "expected"),
    [
        ("PICO_SDK_PATH", "stale-sdk", "pico_sdk_init.cmake"),
        (
            "PICO_TOOLCHAIN_PATH",
            "stale-toolchain",
            f"bin/{build_script.TOOLCHAIN_COMPILER}",
        ),
    ],
)
def test_stale_explicit_environment_is_an_error(tmp_path, variable, invalid, expected):
    sdk = make_sdk(tmp_path / "sdk")
    toolchain = make_toolchain(tmp_path / "toolchain")
    environ = {
        "PATH": "",
        "PICO_SDK_PATH": str(sdk),
        "PICO_TOOLCHAIN_PATH": str(toolchain),
        variable: str(tmp_path / invalid),
    }

    with pytest.raises(build_script.BuildEnvironmentError) as error:
        build_script.configure_pico_environment(
            environ=environ,
            cache_paths=[],
            sdk_candidates=[],
            toolchain_candidates=[],
            which=no_compiler,
        )

    assert variable in str(error.value)
    assert expected in str(error.value)


def test_cache_paths_win_over_fallbacks(tmp_path, capsys):
    cached_sdk = make_sdk(tmp_path / "cached-sdk")
    cached_toolchain = make_toolchain(tmp_path / "cached-toolchain")
    fallback_sdk = make_sdk(tmp_path / "fallback-sdk")
    fallback_toolchain = make_toolchain(tmp_path / "fallback-toolchain")
    cache = tmp_path / "build" / "CMakeCache.txt"
    cache.parent.mkdir()
    cache.write_text(
        f"PICO_SDK_PATH:PATH={cached_sdk}\n"
        f"CMAKE_C_COMPILER:FILEPATH="
        f"{cached_toolchain / 'bin' / build_script.TOOLCHAIN_COMPILER}\n",
        encoding="utf-8",
    )
    environ = {"PATH": ""}

    updates = build_script.configure_pico_environment(
        environ=environ,
        cache_paths=[cache],
        sdk_candidates=[("fallback", fallback_sdk)],
        toolchain_candidates=[("fallback", fallback_toolchain)],
        which=no_compiler,
    )

    assert updates == {
        "PICO_SDK_PATH": str(cached_sdk),
        "PICO_TOOLCHAIN_PATH": str(cached_toolchain),
    }
    output = capsys.readouterr().out
    assert f"Auto-detected PICO_SDK_PATH={cached_sdk}" in output
    assert f"Auto-detected PICO_TOOLCHAIN_PATH={cached_toolchain}" in output
    assert str(cache) in output


def test_project_local_fallback_ignores_stale_cache(tmp_path, monkeypatch):
    project_build = tmp_path / "build"
    local_sdk = make_sdk(project_build / "_deps" / "pico_sdk-src")
    local_toolchain = make_toolchain(project_build / "toolchain")
    cache = tmp_path / "old-build" / "CMakeCache.txt"
    cache.parent.mkdir()
    cache.write_text(
        f"PICO_SDK_PATH:PATH={tmp_path / 'missing-sdk'}\n"
        f"PICO_TOOLCHAIN_PATH:PATH={tmp_path / 'missing-toolchain'}\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(build_script, "BUILD_DIR", project_build)
    environ = {"PATH": ""}

    updates = build_script.configure_pico_environment(
        environ=environ,
        cache_paths=[cache],
        which=no_compiler,
    )

    assert updates == {
        "PICO_SDK_PATH": str(local_sdk),
        "PICO_TOOLCHAIN_PATH": str(local_toolchain),
    }


def test_compiler_on_path_avoids_toolchain_override(tmp_path):
    sdk = make_sdk(tmp_path / "sdk")
    compiler = tmp_path / "path-bin" / build_script.TOOLCHAIN_COMPILER
    environ = {"PATH": str(compiler.parent), "PICO_SDK_PATH": str(sdk)}

    updates = build_script.configure_pico_environment(
        environ=environ,
        cache_paths=[],
        sdk_candidates=[],
        toolchain_candidates=[],
        which=lambda name, *, path: str(compiler),
    )

    assert updates == {}
    assert "PICO_TOOLCHAIN_PATH" not in environ


def test_missing_dependencies_name_only_actionable_overrides():
    environ = {"PATH": ""}

    with pytest.raises(build_script.BuildEnvironmentError) as error:
        build_script.configure_pico_environment(
            environ=environ,
            cache_paths=[],
            sdk_candidates=[],
            toolchain_candidates=[],
            which=no_compiler,
        )

    assert "PICO_SDK_PATH" in str(error.value)
    assert "PICO_TOOLCHAIN_PATH" in str(error.value)


@pytest.fixture
def build_cli(tmp_path, monkeypatch):
    for name in (
        "SCRIPT_DIR",
        "CONFIG_FILE",
        "BUILD_DIR",
        "AIO_BUILD_DIR",
        "WAKE_CAPTURE_SOURCE_DIR",
        "WAKE_CAPTURE_BUILD_DIR",
        "WAKE_ONLY_SOURCE_DIR",
        "WAKE_ONLY_BUILD_DIR",
        "FIRMWARE_DIR",
        "FIRMWARE_ELF_PATH",
        "FIRMWARE_UF2_PATH",
        "AIO_FIRMWARE_ELF_PATH",
        "AIO_FIRMWARE_UF2_PATH",
        "WAKE_CAPTURE_FIRMWARE_ELF_PATH",
        "WAKE_CAPTURE_FIRMWARE_UF2_PATH",
        "WAKE_ONLY_FIRMWARE_ELF_PATH",
        "WAKE_ONLY_FIRMWARE_UF2_PATH",
    ):
        original = getattr(build_script, name)
        monkeypatch.setattr(build_script, name, tmp_path / original.relative_to(ROOT))
    monkeypatch.setattr(
        build_script,
        "CMAKE_CACHE_PATHS",
        tuple(
            tmp_path / path.relative_to(ROOT) for path in build_script.CMAKE_CACHE_PATHS
        ),
    )
    monkeypatch.setattr(
        build_script, "ELF_PATH", tmp_path / "build" / "switch-pico.elf"
    )
    monkeypatch.setattr(
        build_script, "UF2_PATH", tmp_path / "build" / "switch-pico.uf2"
    )
    sdk = make_sdk(tmp_path / "sdk")
    toolchain = make_toolchain(tmp_path / "toolchain")
    monkeypatch.setenv("PICO_SDK_PATH", str(sdk))
    monkeypatch.setenv("PICO_TOOLCHAIN_PATH", str(toolchain))
    monkeypatch.setattr(build_script, "resolve_picotool", lambda: Path("picotool"))
    commands = []

    def run_cmd(command):
        commands.append(command)
        if command[:2] == ["cmake", "--build"]:
            build_dir = Path(command[2])
            build_dir.mkdir(parents=True, exist_ok=True)
            stem = {
                "build-wake-capture": "switch2-wake-capture",
                "build-wake-only": "switch2-wake-beacon",
            }.get(build_dir.name, "switch-pico")
            for extension in ("elf", "uf2"):
                (build_dir / f"{stem}.{extension}").write_bytes(
                    f"{build_dir.name}:{extension}".encode()
                )

    monkeypatch.setattr(build_script, "run_cmd", run_cmd)
    return commands


def test_bluetooth_modes_configure_and_publish_isolated_artifacts(
    tmp_path, monkeypatch, build_cli
):
    # Switch away from mixed and back, leaving every other image untouched.
    published = {}
    for mode in (None, "ble", "classic", "mixed"):
        arguments = ["build.py", "--aio"]
        if mode is not None:
            arguments.extend(["--bluetooth-mode", mode])
        monkeypatch.setattr(build_script.sys, "argv", arguments)
        build_script.main()

        selected = mode or "mixed"
        suffix = "" if selected == "mixed" else f"-{selected}"
        build_dir = tmp_path / f"build-aio{suffix}"
        configure, compile_command, flash_command = build_cli[-3:]
        assert configure[:5] == ["cmake", "-S", str(tmp_path), "-B", str(build_dir)]
        assert f"-DSWITCH_PICO_BLUETOOTH_MODE={selected.upper()}" in configure
        assert "-DSWITCH_PICO_INPUT_BACKEND=BLUEPAD32" in configure
        assert compile_command == ["cmake", "--build", str(build_dir)]
        assert flash_command == [
            "picotool",
            "load",
            str(build_dir / "switch-pico.elf"),
            "-fx",
        ]
        for extension in ("elf", "uf2"):
            destination = tmp_path / "firmware" / f"switch-pico-aio{suffix}.{extension}"
            published[destination] = f"build-aio{suffix}:{extension}".encode()
        for destination, expected in published.items():
            assert destination.read_bytes() == expected


def test_uart_default_preserves_paths_and_explicit_mixed_configuration(
    tmp_path, monkeypatch, build_cli
):
    monkeypatch.setattr(build_script.sys, "argv", ["build.py"])
    build_script.main()

    configure, compile_command, flash_command = build_cli
    assert configure[:5] == [
        "cmake",
        "-S",
        str(tmp_path),
        "-B",
        str(tmp_path / "build"),
    ]
    assert "-DSWITCH_PICO_INPUT_BACKEND=UART" in configure
    assert "-DSWITCH_PICO_BLUETOOTH_MODE=MIXED" in configure
    assert compile_command == ["cmake", "--build", str(tmp_path / "build")]
    assert flash_command == [
        "picotool",
        "load",
        str(tmp_path / "build" / "switch-pico.elf"),
        "-fx",
    ]
    for extension in ("elf", "uf2"):
        assert (tmp_path / "firmware" / f"switch-pico.{extension}").read_bytes() == (
            f"build:{extension}".encode()
        )


@pytest.mark.parametrize(
    "arguments",
    [
        ["--bluetooth-mode", "ble"],
        ["--bluetooth-mode", "classic"],
        ["--wake-capture", "--bluetooth-mode", "ble"],
        ["--wake-capture", "--bluetooth-mode", "classic"],
        ["--aio", "--bluetooth-mode", "invalid"],
    ],
)
def test_invalid_bluetooth_selection_exits_before_build_setup(monkeypatch, arguments):
    monkeypatch.setattr(build_script.sys, "argv", ["build.py", *arguments])
    monkeypatch.setattr(
        build_script,
        "configure_pico_environment",
        lambda: pytest.fail("invalid selection reached build setup"),
    )
    with pytest.raises(SystemExit) as error:
        build_script.main()
    assert error.value.code == 2


def test_uart_artifact_overrides_remain_effective(tmp_path, monkeypatch, build_cli):
    elf_path = tmp_path / "custom.elf"
    uf2_path = tmp_path / "custom.uf2"
    elf_path.write_bytes(b"custom ELF")
    uf2_path.write_bytes(b"custom UF2")
    monkeypatch.setenv("ELF_PATH", str(elf_path))
    monkeypatch.setenv("UF2_PATH", str(uf2_path))
    overridden = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(overridden)
    monkeypatch.setattr(build_script, "ELF_PATH", overridden.ELF_PATH)
    monkeypatch.setattr(build_script, "UF2_PATH", overridden.UF2_PATH)
    monkeypatch.setattr(build_script.sys, "argv", ["build.py"])

    build_script.main()

    assert (tmp_path / "firmware" / "switch-pico.elf").read_bytes() == b"custom ELF"
    assert (tmp_path / "firmware" / "switch-pico.uf2").read_bytes() == b"custom UF2"
    assert build_cli[-1] == ["picotool", "load", str(elf_path), "-fx"]


@pytest.mark.parametrize("color_option", ["--grip-color", "--random-grip-color"])
def test_ble_build_preserves_grip_color_options(
    tmp_path, monkeypatch, build_cli, color_option
):
    config = build_script.CONFIG_FILE
    config.parent.mkdir(parents=True)
    config.write_text(
        "".join(f"#define {macro} 0x00\n" for macro in build_script.MACROS),
        encoding="utf-8",
    )
    arguments = ["build.py", "--aio", "--bluetooth-mode", "ble", color_option]
    if color_option == "--grip-color":
        arguments.append("A1B2C3")
    else:
        monkeypatch.setattr(build_script, "random_hex_color", lambda: "A1B2C3")
    monkeypatch.setattr(build_script.sys, "argv", arguments)

    build_script.main()

    assert config.read_text(encoding="utf-8") == "".join(
        f"#define SWITCH_COLOR_SLOT_{slot}_{component} 0x{value}\n"
        for slot in range(1, 5)
        for component, value in zip(("R", "G", "B"), ("A1", "B2", "C3"))
    )
    assert (tmp_path / "firmware" / "switch-pico-aio-ble.uf2").read_bytes() == (
        b"build-aio-ble:uf2"
    )


def test_wake_only_build_publishes_and_flashes_only_beacon_artifacts(
    tmp_path, monkeypatch, build_cli
):
    firmware = tmp_path / "firmware"
    firmware.mkdir()
    unchanged = {}
    for stem in ("switch-pico", "switch-pico-aio", "switch-pico-wake-capture"):
        for extension in ("elf", "uf2"):
            path = firmware / f"{stem}.{extension}"
            path.write_bytes(f"existing {path.name}".encode())
            unchanged[path] = path.read_bytes()
    config = build_script.CONFIG_FILE
    config.parent.mkdir(parents=True)
    config.write_bytes(b"controller colors must remain untouched")
    unchanged[config] = config.read_bytes()
    override = tmp_path / "custom.elf"
    override.write_bytes(b"not the wake beacon")
    monkeypatch.setattr(build_script, "ELF_PATH", override)
    monkeypatch.setattr(build_script, "UF2_PATH", tmp_path / "custom.uf2")
    monkeypatch.setattr(build_script.sys, "argv", ["build.py", "--wake-only"])

    build_script.main()

    build_dir = tmp_path / "build-wake-only"
    assert build_cli == [
        [
            "cmake",
            "-S",
            str(tmp_path / "tools" / "switch2_wake_beacon"),
            "-B",
            str(build_dir),
            "-DPICO_BOARD=pico2_w",
        ],
        ["cmake", "--build", str(build_dir)],
        ["picotool", "load", str(build_dir / "switch2-wake-beacon.elf"), "-fx"],
    ]
    for extension in ("elf", "uf2"):
        assert (firmware / f"switch-pico-wake-only.{extension}").read_bytes() == (
            f"build-wake-only:{extension}".encode()
        )
    for path, expected in unchanged.items():
        assert path.read_bytes() == expected


def test_wake_only_build_discovers_dependencies_from_its_cache(
    tmp_path, monkeypatch, build_cli
):
    monkeypatch.delenv("PICO_SDK_PATH")
    monkeypatch.delenv("PICO_TOOLCHAIN_PATH")
    monkeypatch.setattr(build_script.shutil, "which", no_compiler)
    monkeypatch.setattr(build_script, "_sdk_fallback_candidates", lambda: ())
    monkeypatch.setattr(build_script, "_toolchain_fallback_candidates", lambda: ())
    cache = tmp_path / "build-wake-only" / "CMakeCache.txt"
    cache.parent.mkdir()
    cache.write_text(
        f"PICO_SDK_PATH:PATH={tmp_path / 'sdk'}\n"
        f"PICO_TOOLCHAIN_PATH:PATH={tmp_path / 'toolchain'}\n",
        encoding="utf-8",
    )
    commands = build_script.run_cmd

    def run_with_dependencies(command):
        assert build_script.os.environ["PICO_SDK_PATH"] == str(tmp_path / "sdk")
        assert build_script.os.environ["PICO_TOOLCHAIN_PATH"] == str(
            tmp_path / "toolchain"
        )
        commands(command)

    monkeypatch.setattr(build_script, "run_cmd", run_with_dependencies)
    monkeypatch.setattr(build_script.sys, "argv", ["build.py", "--wake-only"])

    build_script.main()

    assert (tmp_path / "firmware" / "switch-pico-wake-only.uf2").read_bytes() == (
        b"build-wake-only:uf2"
    )


@pytest.mark.parametrize("missing_extension", ["elf", "uf2"])
def test_wake_only_missing_artifact_does_not_publish_or_flash(
    tmp_path, monkeypatch, build_cli, missing_extension
):
    build_dir = tmp_path / "build-wake-only"
    build_dir.mkdir()
    for extension in ("elf", "uf2"):
        (build_dir / f"switch-pico.{extension}").write_bytes(b"wrong target")
        if extension != missing_extension:
            (build_dir / f"switch2-wake-beacon.{extension}").write_bytes(b"beacon")
    firmware = tmp_path / "firmware"
    firmware.mkdir()
    for extension in ("elf", "uf2"):
        (firmware / f"switch-pico-wake-only.{extension}").write_bytes(b"old beacon")
    monkeypatch.setattr(build_script, "run_cmd", build_cli.append)
    monkeypatch.setattr(build_script.sys, "argv", ["build.py", "--wake-only"])

    with pytest.raises(SystemExit) as error:
        build_script.main()

    assert error.value.code == 1
    assert [command[0] for command in build_cli] == ["cmake", "cmake"]
    for extension in ("elf", "uf2"):
        assert (firmware / f"switch-pico-wake-only.{extension}").read_bytes() == (
            b"old beacon"
        )


@pytest.mark.parametrize(
    "conflict",
    [
        ["--aio"],
        ["--wake-capture"],
        ["--grip-color", "A1B2C3"],
        ["--grip-color", ""],
        ["--random-grip-color"],
        ["--bluetooth-mode", "mixed"],
        ["--bluetooth-mode", "ble"],
        ["--bluetooth-mode", "classic"],
        ["--input-backend", "BLUEPAD32"],
        ["--hd-rumble"],
        ["--native"],
    ],
)
def test_wake_only_conflicts_fail_before_dependencies_or_mutations(
    monkeypatch, conflict
):
    monkeypatch.setattr(
        build_script.sys, "argv", ["build.py", "--wake-only", *conflict]
    )
    monkeypatch.setattr(
        build_script,
        "configure_pico_environment",
        lambda: pytest.fail("invalid wake-only selection reached build setup"),
    )

    with pytest.raises(SystemExit) as error:
        build_script.main()

    assert error.value.code == 2
