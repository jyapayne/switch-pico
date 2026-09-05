from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def test_profile_playtest_math() -> None:
    root = Path(__file__).resolve().parents[1]
    node = shutil.which("node")
    assert node is not None, "Node.js is required for Profile Studio tests"
    _ = subprocess.run(
        [node, str(root / "tests" / "profile_playtest_test.js")],
        check=True,
        cwd=root,
    )
