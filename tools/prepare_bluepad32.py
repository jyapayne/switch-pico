#!/usr/bin/env python3
"""Apply the project Bluepad32 patch exactly once."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


class PatchError(RuntimeError):
    pass


def resolve_paths(repo_root: Path | None = None) -> tuple[Path, Path]:
    root = Path.cwd() if repo_root is None else Path(repo_root)
    return (
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
    )


def check_paths(bluepad32_path: Path, patch_path: Path) -> None:
    if not bluepad32_path.is_dir():
        raise PatchError(f"bluepad32 directory does not exist: {bluepad32_path}")
    if not (bluepad32_path / ".git").exists():
        raise PatchError(f"Bluepad32 is not a git repository: {bluepad32_path}")
    if not patch_path.is_file():
        raise PatchError(f"patch file does not exist: {patch_path}")


def git_apply(bluepad32_path: Path, patch_path: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(bluepad32_path), "apply", *args, str(patch_path)],
        capture_output=True,
        text=True,
        check=False,
    )


def is_patch_applied(bluepad32_path: Path, patch_path: Path) -> bool:
    return git_apply(bluepad32_path, patch_path, "--reverse", "--check").returncode == 0


def apply_patch(bluepad32_path: Path, patch_path: Path) -> None:
    if is_patch_applied(bluepad32_path, patch_path):
        return

    check = git_apply(bluepad32_path, patch_path, "--check")
    if check.returncode != 0:
        detail = check.stderr.strip() or "patch does not apply"
        raise PatchError(f"Patch validation failed (repository may be diverged):\n{detail}")

    result = git_apply(bluepad32_path, patch_path)
    if result.returncode != 0:
        detail = result.stderr.strip() or "git apply failed"
        raise PatchError(f"Could not patch Bluepad32: {detail}")


def prepare_bluepad32(
    bluepad32_path: Path | None = None,
    patch_path: Path | None = None,
) -> None:
    default_bluepad32, default_patch = resolve_paths()
    dependency = Path(bluepad32_path or default_bluepad32)
    patch = Path(patch_path or default_patch)
    check_paths(dependency, patch)
    apply_patch(dependency, patch)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bluepad32", type=Path)
    parser.add_argument("--patch", type=Path)
    args = parser.parse_args()
    try:
        prepare_bluepad32(args.bluepad32, args.patch)
    except PatchError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
