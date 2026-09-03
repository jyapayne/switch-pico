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
def test_stale_explicit_environment_is_an_error(
    tmp_path, variable, invalid, expected
):
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

    assert str(error.value) == (
        "Missing build prerequisite(s): Pico SDK (set PICO_SDK_PATH); "
        "Arm GNU toolchain (set PICO_TOOLCHAIN_PATH or add "
        "arm-none-eabi-gcc to PATH)."
    )
