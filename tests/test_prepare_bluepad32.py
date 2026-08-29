"""
Tests for prepare_bluepad32.py patch preparation tool.

Tests cover:
- Fresh patch application
- Idempotence (second invocation succeeds without changing content)
- Missing paths validation
- Diverged/ambiguous repository states
"""

import subprocess
import tempfile
import pytest
from pathlib import Path
from unittest.mock import patch as mock_patch

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))

from prepare_bluepad32 import (
    PatchError,
    resolve_paths,
    check_paths,
    is_patch_applied,
    apply_patch,
    prepare_bluepad32,
)


@pytest.fixture
def temp_repo_structure():
    """Create a temporary directory structure with git repositories."""
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        
        # Create bluepad32 repo
        bp_dir = root / "external" / "bluepad32"
        bp_dir.mkdir(parents=True)
        subprocess.run(["git", "init"], cwd=bp_dir, check=True, capture_output=True)
        subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=bp_dir, check=True, capture_output=True)
        subprocess.run(["git", "config", "user.name", "Test User"], cwd=bp_dir, check=True, capture_output=True)
        
        # Create a file to patch
        test_file = bp_dir / "test.txt"
        test_file.write_text("line 1\n")
        subprocess.run(["git", "add", "test.txt"], cwd=bp_dir, check=True, capture_output=True)
        subprocess.run(["git", "commit", "-m", "initial"], cwd=bp_dir, check=True, capture_output=True)
        
        # Create patches dir
        patches_dir = root / "patches"
        patches_dir.mkdir()
        
        yield root, bp_dir, patches_dir


def create_simple_patch(repo_path: Path, patch_path: Path, file_to_patch: str = "test.txt") -> str:
    """
    Create a simple patch file that modifies a file in the repository.
    
    Returns the patch content as a string.
    """
    # Create the modification
    test_file = repo_path / file_to_patch
    original_content = test_file.read_text()
    modified_content = original_content + "line 2\n"
    
    # Generate patch using git diff
    test_file.write_text(modified_content)
    result = subprocess.run(
        ["git", "diff", file_to_patch],
        cwd=repo_path,
        capture_output=True,
        text=True,
        check=True,
    )
    patch_content = result.stdout
    
    # Reset the file to original state
    test_file.write_text(original_content)
    
    # Write patch to file
    patch_path.write_text(patch_content)
    return patch_content


def test_resolve_paths_with_defaults():
    """Test that resolve_paths returns expected default paths."""
    with tempfile.TemporaryDirectory() as tmpdir:
        bp_path, patch_path = resolve_paths(Path(tmpdir))
        assert bp_path == Path(tmpdir) / "external" / "bluepad32"
        assert patch_path == Path(tmpdir) / "patches" / "bluepad32-sdl3-imu.patch"


def test_resolve_paths_no_root():
    """Test resolve_paths with no root uses current directory."""
    bp_path, patch_path = resolve_paths()
    assert bp_path.is_absolute()
    assert patch_path.is_absolute()


def test_check_paths_missing_bluepad32(temp_repo_structure):
    """Test that check_paths fails if bluepad32 dir is missing."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    # Remove bluepad32
    import shutil
    shutil.rmtree(bp_dir)
    
    patch_file = patches_dir / "test.patch"
    patch_file.write_text("dummy")
    
    with pytest.raises(PatchError, match="bluepad32 directory does not exist"):
        check_paths(bp_dir, patch_file)


def test_check_paths_missing_patch(temp_repo_structure):
    """Test that check_paths fails if patch file is missing."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "nonexistent.patch"
    
    with pytest.raises(PatchError, match="patch file does not exist"):
        check_paths(bp_dir, patch_file)


def test_check_paths_bluepad32_not_git_repo(temp_repo_structure):
    """Test that check_paths fails if bluepad32 is not a git repo."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    # Remove .git to make it not a git repo
    import shutil
    shutil.rmtree(bp_dir / ".git")
    
    patch_file = patches_dir / "test.patch"
    patch_file.write_text("dummy")
    
    with pytest.raises(PatchError, match="not a git repository"):
        check_paths(bp_dir, patch_file)


def test_fresh_patch_application(temp_repo_structure):
    """Test applying a fresh patch to a clean repository."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Verify test.txt before patch
    test_file = bp_dir / "test.txt"
    original = test_file.read_text()
    assert "line 2" not in original
    
    # Apply patch
    apply_patch(bp_dir, patch_file)
    
    # Verify test.txt after patch
    patched = test_file.read_text()
    assert "line 2" in patched


def test_idempotent_patch_application(temp_repo_structure):
    """Test that applying the same patch twice succeeds (idempotence)."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # First application
    apply_patch(bp_dir, patch_file)
    test_file = bp_dir / "test.txt"
    after_first = test_file.read_text()
    
    # Second application should succeed without changing content
    apply_patch(bp_dir, patch_file)
    after_second = test_file.read_text()
    
    assert after_first == after_second


def test_is_patch_applied_not_applied(temp_repo_structure):
    """Test is_patch_applied returns False for unapplied patch."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Patch not applied yet
    assert is_patch_applied(bp_dir, patch_file) is False


def test_is_patch_applied_already_applied(temp_repo_structure):
    """Test is_patch_applied returns True for already applied patch."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Apply patch first
    apply_patch(bp_dir, patch_file)
    
    # Now check should detect it's applied
    assert is_patch_applied(bp_dir, patch_file) is True


def test_diverged_repository_state(temp_repo_structure):
    """Test that diverged repository (patch doesn't apply cleanly) is rejected."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Diverge the repository by modifying the file such that the patch conflicts
    test_file = bp_dir / "test.txt"
    test_file.write_text("completely different line 1\n")
    subprocess.run(["git", "add", "test.txt"], cwd=bp_dir, check=True, capture_output=True)
    subprocess.run(["git", "commit", "-m", "divergence"], cwd=bp_dir, check=True, capture_output=True)
    
    # Try to apply patch - should fail because file content doesn't match
    with pytest.raises(PatchError, match="Patch validation failed"):
        apply_patch(bp_dir, patch_file)


def test_missing_bluepad32_path(temp_repo_structure):
    """Test prepare_bluepad32 fails gracefully with missing bluepad32."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    import shutil
    shutil.rmtree(bp_dir)
    
    patch_file = patches_dir / "test.patch"
    patch_file.write_text("dummy")
    
    with pytest.raises(PatchError, match="bluepad32 directory does not exist"):
        prepare_bluepad32(bp_dir, patch_file)


def test_missing_patch_file(temp_repo_structure):
    """Test prepare_bluepad32 fails gracefully with missing patch file."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "nonexistent.patch"
    
    with pytest.raises(PatchError, match="patch file does not exist"):
        prepare_bluepad32(bp_dir, patch_file)


def test_prepare_bluepad32_full_workflow(temp_repo_structure):
    """Test complete prepare_bluepad32 workflow: apply then idempotent re-apply."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    test_file = bp_dir / "test.txt"
    original = test_file.read_text()
    
    # First prepare (should apply patch)
    prepare_bluepad32(bp_dir, patch_file)
    after_first = test_file.read_text()
    assert after_first != original
    assert "line 2" in after_first
    
    # Second prepare (should be idempotent)
    prepare_bluepad32(bp_dir, patch_file)
    after_second = test_file.read_text()
    assert after_first == after_second


def test_prepare_bluepad32_with_defaults(temp_repo_structure):
    """Test prepare_bluepad32 uses correct defaults when paths not provided."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "bluepad32-sdl3-imu.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Change to root directory and call with defaults
    import os
    original_cwd = os.getcwd()
    try:
        os.chdir(root)
        prepare_bluepad32()  # Use defaults
    finally:
        os.chdir(original_cwd)
    
    # Verify patch was applied
    test_file = bp_dir / "test.txt"
    assert "line 2" in test_file.read_text()


def test_patch_application_with_conflicting_content(temp_repo_structure):
    """Test that patch with conflicting content is rejected."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    # Create a patch that adds a specific change
    patch_content = """--- a/test.txt
+++ b/test.txt
@@ -1 +1,3 @@
 line 1
+line 2
+line 3
"""
    
    patch_file = patches_dir / "conflict.patch"
    patch_file.write_text(patch_content)
    
    # Modify the file to have different content that won't match the patch context
    test_file = bp_dir / "test.txt"
    test_file.write_text("modified line 1\n")
    subprocess.run(["git", "add", "test.txt"], cwd=bp_dir, check=True, capture_output=True)
    subprocess.run(["git", "commit", "-m", "modify"], cwd=bp_dir, check=True, capture_output=True)
    
    # Try to apply patch - should fail due to context mismatch
    with pytest.raises(PatchError):
        apply_patch(bp_dir, patch_file)


def test_cli_with_explicit_paths(temp_repo_structure):
    """Test CLI argument parsing with explicit paths."""
    root, bp_dir, patches_dir = temp_repo_structure
    
    patch_file = patches_dir / "test.patch"
    create_simple_patch(bp_dir, patch_file, "test.txt")
    
    # Simulate CLI call
    sys.argv = [
        "prepare_bluepad32.py",
        "--bluepad32", str(bp_dir),
        "--patch", str(patch_file),
    ]
    
    from prepare_bluepad32 import main
    
    # Should not raise
    try:
        main()
    except SystemExit as e:
        # main() calls sys.exit on success, which we need to catch
        if e.code != 0:
            raise
