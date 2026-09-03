from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from prepare_bluepad32 import (  # noqa: E402
    PatchError,
    check_paths,
    main,
    prepare_bluepad32,
    resolve_paths,
)


def run_git(repository: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(repository), *args],
        capture_output=True,
        text=True,
        check=True,
    )


@pytest.fixture
def bluepad32_fixture(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    source = tmp_path / "external" / "bluepad32"
    source.mkdir(parents=True)
    run_git(source, "init")
    run_git(source, "config", "user.email", "test@example.com")
    run_git(source, "config", "user.name", "Test User")
    (source / "test.txt").write_text("line 1\n", encoding="utf-8")
    run_git(source, "add", "test.txt")
    run_git(source, "commit", "-m", "initial")

    patches = tmp_path / "patches"
    patches.mkdir()
    patch = patches / "bluepad32-sdl3-imu.patch"
    original = (source / "test.txt").read_text(encoding="utf-8")
    (source / "test.txt").write_text(f"{original}line 2\n", encoding="utf-8")
    patch.write_text(run_git(source, "diff", "--", "test.txt").stdout, encoding="utf-8")
    run_git(source, "restore", "test.txt")

    output = tmp_path / "build-aio" / "_deps" / "bluepad32-src"
    return tmp_path, source, patch, output


def source_status(source: Path) -> str:
    return run_git(source, "status", "--porcelain", "--untracked-files=all").stdout


def test_resolve_paths_uses_project_build_tree(tmp_path: Path) -> None:
    source, patch, output = resolve_paths(tmp_path)

    assert source == tmp_path / "external" / "bluepad32"
    assert patch == tmp_path / "patches" / "bluepad32-sdl3-imu.patch"
    assert output == tmp_path / "build" / "_deps" / "bluepad32-src"


def test_check_paths_rejects_missing_source(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    shutil.rmtree(source)

    with pytest.raises(PatchError, match="bluepad32 directory does not exist"):
        check_paths(source, patch, output)


def test_check_paths_rejects_missing_patch(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    patch.unlink()

    with pytest.raises(PatchError, match="patch file does not exist"):
        check_paths(source, patch, output)


def test_check_paths_rejects_non_repository(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    shutil.rmtree(source / ".git")

    with pytest.raises(PatchError, match="not a git repository"):
        check_paths(source, patch, output)


def test_check_paths_rejects_output_inside_source(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, _ = bluepad32_fixture

    with pytest.raises(PatchError, match="outside the Bluepad32 source tree"):
        check_paths(source, patch, source / "patched")


def test_prepare_patches_copy_and_preserves_pristine_source(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture

    result = prepare_bluepad32(source, patch, output)

    assert result == output
    assert (source / "test.txt").read_text(encoding="utf-8") == "line 1\n"
    assert (output / "test.txt").read_text(encoding="utf-8") == "line 1\nline 2\n"
    assert not (output / ".git").exists()
    assert source_status(source) == ""


def test_prepare_replaces_existing_output_idempotently(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    prepare_bluepad32(source, patch, output)
    (output / "stale.txt").write_text("stale", encoding="utf-8")

    prepare_bluepad32(source, patch, output)

    assert not (output / "stale.txt").exists()
    assert (output / "test.txt").read_text(encoding="utf-8") == "line 1\nline 2\n"
    assert source_status(source) == ""


def test_prepare_rejects_dirty_source_without_replacing_output(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    output.mkdir(parents=True)
    sentinel = output / "sentinel.txt"
    sentinel.write_text("keep", encoding="utf-8")
    (source / "test.txt").write_text("local edit\n", encoding="utf-8")

    with pytest.raises(PatchError, match="must be pristine"):
        prepare_bluepad32(source, patch, output)

    assert sentinel.read_text(encoding="utf-8") == "keep"


def test_prepare_rejects_diverged_source_revision(
    bluepad32_fixture: tuple[Path, Path, Path, Path],
) -> None:
    _, source, patch, output = bluepad32_fixture
    (source / "test.txt").write_text("different base\n", encoding="utf-8")
    run_git(source, "add", "test.txt")
    run_git(source, "commit", "-m", "diverge")

    with pytest.raises(PatchError, match="Patch validation failed"):
        prepare_bluepad32(source, patch, output)

    assert not output.exists()


def test_prepare_uses_default_paths(
    bluepad32_fixture: tuple[Path, Path, Path, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    root, source, _, _ = bluepad32_fixture
    monkeypatch.chdir(root)

    output = prepare_bluepad32()

    assert output == root / "build" / "_deps" / "bluepad32-src"
    assert (output / "test.txt").read_text(encoding="utf-8") == "line 1\nline 2\n"
    assert source_status(source) == ""


def test_cli_accepts_explicit_output(
    bluepad32_fixture: tuple[Path, Path, Path, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    _, source, patch, output = bluepad32_fixture
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "prepare_bluepad32.py",
            "--bluepad32",
            str(source),
            "--patch",
            str(patch),
            "--output",
            str(output),
        ],
    )

    assert main() == 0
    assert (output / "test.txt").read_text(encoding="utf-8") == "line 1\nline 2\n"
    assert source_status(source) == ""
