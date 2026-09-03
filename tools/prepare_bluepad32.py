#!/usr/bin/env python3
"""Create a patched build-local copy of the pinned Bluepad32 source."""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


class PatchError(RuntimeError):
    pass


def resolve_paths(repo_root: Path | None = None) -> tuple[Path, Path, Path]:
    root = Path.cwd() if repo_root is None else Path(repo_root)
    return (
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
        root / "build" / "_deps" / "bluepad32-src",
    )


def _run(command: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
    )


def check_paths(source_path: Path, patch_path: Path, output_path: Path) -> None:
    if not source_path.is_dir():
        raise PatchError(f"bluepad32 directory does not exist: {source_path}")
    if not patch_path.is_file():
        raise PatchError(f"patch file does not exist: {patch_path}")

    repository_check = _run(
        ["git", "-C", str(source_path), "rev-parse", "--is-inside-work-tree"]
    )
    if repository_check.returncode != 0:
        raise PatchError(f"Bluepad32 is not a git repository: {source_path}")

    source = source_path.resolve()
    output = output_path.resolve()
    if output == source or source in output.parents:
        raise PatchError("patched output must be outside the Bluepad32 source tree")


def validate_pristine_source(source_path: Path, patch_path: Path) -> None:
    status = _run(
        [
            "git",
            "-C",
            str(source_path),
            "status",
            "--porcelain",
            "--untracked-files=all",
        ]
    )
    if status.returncode != 0:
        detail = status.stderr.strip() or "git status failed"
        raise PatchError(f"Could not inspect Bluepad32 source: {detail}")
    if status.stdout.strip():
        raise PatchError(
            "Bluepad32 source must be pristine; patches are applied only to the build-local copy"
        )

    check = _run(
        ["git", "-C", str(source_path), "apply", "--check", str(patch_path)]
    )
    if check.returncode != 0:
        detail = check.stderr.strip() or "patch does not apply"
        raise PatchError(f"Patch validation failed (source revision may have diverged):\n{detail}")


def _copy_and_patch(source_path: Path, patch_path: Path, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    staging_path = output_path.with_name(f".{output_path.name}.tmp")

    if staging_path.exists():
        shutil.rmtree(staging_path)

    try:
        shutil.copytree(
            source_path,
            staging_path,
            symlinks=True,
            ignore=shutil.ignore_patterns(".git"),
        )
        result = _run(
            ["git", "apply", "--no-index", str(patch_path.resolve())],
            cwd=staging_path,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or "git apply failed"
            raise PatchError(f"Could not patch build-local Bluepad32 copy: {detail}")

        if output_path.exists():
            shutil.rmtree(output_path)
        staging_path.replace(output_path)
    except Exception:
        if staging_path.exists():
            shutil.rmtree(staging_path)
        raise


def prepare_bluepad32(
    source_path: Path | None = None,
    patch_path: Path | None = None,
    output_path: Path | None = None,
) -> Path:
    default_source, default_patch, default_output = resolve_paths()
    source = Path(source_path or default_source)
    patch = Path(patch_path or default_patch)
    output = Path(output_path or default_output)

    check_paths(source, patch, output)
    validate_pristine_source(source, patch)
    _copy_and_patch(source, patch, output)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bluepad32", type=Path)
    parser.add_argument("--patch", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        prepare_bluepad32(args.bluepad32, args.patch, args.output)
    except PatchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
