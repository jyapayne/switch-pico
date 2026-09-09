import os
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("batching", "dedicated"),
    [(False, False), (True, False), (True, True)],
    ids=["default", "batched", "dedicated"],
)
def test_btstack_credit_batch_native(
    tmp_path: Path, batching: bool, dedicated: bool
) -> None:
    root = Path(__file__).resolve().parents[1]
    sdk = Path(
        os.environ.get("PICO_SDK_PATH", root / "build" / "_deps" / "pico_sdk-src")
    )
    btstack = sdk / "lib" / "btstack"
    source = btstack / "src"
    if not (source / "hci.c").is_file():
        pytest.skip("requires a populated Pico SDK; set PICO_SDK_PATH")
    compiler = shutil.which("cc") or shutil.which("gcc")
    assert compiler is not None, "a host C compiler is required"
    patch = shutil.which("patch")
    assert patch is not None, "the patch utility is required"

    # Patch only a temporary copy: neither SDK originals nor build/_deps are modified.
    patched_source = tmp_path / "lib" / "btstack" / "src"
    patched_source.mkdir(parents=True)
    shutil.copyfile(source / "hci.c", patched_source / "hci.c")
    subprocess.run(
        [
            patch,
            "--batch",
            "--forward",
            "-p1",
            "-i",
            str(root / "patches" / "btstack-credit-batch.patch"),
        ],
        check=True,
        cwd=tmp_path,
    )
    executable = tmp_path / "btstack_credit_batch_test"
    units = [
        "ad_parser.c",
        "btstack_linked_list.c",
        "btstack_memory.c",
        "btstack_memory_pool.c",
        "btstack_run_loop.c",
        "btstack_util.c",
        "hci_cmd.c",
        "hci_dump.c",
    ]
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-ffunction-sections",
            "-fdata-sections",
            *(["-DSWITCH_PICO_HCI_CREDIT_BATCH=1"] if batching else []),
            *(["-DSWITCH_PICO_HCI_CREDIT_BUFFER=1"] if dedicated else []),
            f"-I{root / 'tests' / 'btstack_credit_batch_native_stubs'}",
            f"-I{patched_source}",
            f"-I{source}",
            f"-I{btstack / 'platform' / 'embedded'}",
            str(root / "tests" / "btstack_credit_batch_test.c"),
            *(str(source / unit) for unit in units),
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
