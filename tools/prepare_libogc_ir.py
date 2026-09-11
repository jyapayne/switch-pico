#!/usr/bin/env python3
"""Extract the pinned libogc Wiiuse IR math into a build-local C source file."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

UPSTREAM_COMMIT = "a4064a86487c46d8ab76d4fdf99e8059a62c4fa2"
IR_GIT_BLOB = "d9659f5da575861f574d3d0480887955ef8f8faf"
COMPONENT = Path(__file__).resolve().parents[1] / "external" / "libogc_ir"


class PreparationError(RuntimeError):
    pass


def _check_sha256(data: bytes, expected: str, label: str) -> None:
    actual = hashlib.sha256(data).hexdigest()
    if actual != expected:
        raise PreparationError(
            f"{label}: SHA-256 mismatch: expected {expected}, got {actual}"
        )


def _section(data: bytes, section: dict) -> bytes:
    first, last = section["first_line"], section["last_line"]
    lines = data.splitlines(keepends=True)
    label = f"{section['source']}:{first}-{last}"
    if not 1 <= first <= last <= len(lines):
        raise PreparationError(f"{label}: extraction range is outside the source")
    selected = lines[first - 1 : last]
    if (
        selected[0].decode("utf-8").rstrip("\r\n") != section["first_line_text"]
        or selected[-1].decode("utf-8").rstrip("\r\n") != section["last_line_text"]
    ):
        raise PreparationError(f"{label}: extraction boundary marker mismatch")
    content = b"".join(selected)
    _check_sha256(content, section["sha256"], label)
    return content


def prepare(output: Path) -> Path:
    """Write unchanged upstream math sections; preserve mtime when bytes match.

    Input hashes, extraction boundaries and header fidelity are strict build
    prerequisites. This does not fetch sources, invoke Git, rewrite C, or
    compile anything. The generated component retains the libogc GPL plus its
    section 18 linking exception; see external/libogc_ir/NOTICE.txt.
    """
    output = Path(output).resolve()
    if output.is_relative_to(COMPONENT):
        raise PreparationError(
            "output must be outside the vendored libogc IR component"
        )

    manifest = json.loads((COMPONENT / "UPSTREAM.json").read_bytes())
    if manifest["format_version"] != 1 or manifest["commit"] != UPSTREAM_COMMIT:
        raise PreparationError("unsupported libogc IR manifest version or source pin")
    if manifest["ir_git_blob_sha1"] != IR_GIT_BLOB:
        raise PreparationError("unexpected pinned ir.c Git blob")

    sources = {}
    for name, record in manifest["sources"].items():
        data = (COMPONENT / record["local_path"]).read_bytes()
        _check_sha256(data, record["sha256"], name)
        blob = hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data)
        if blob.hexdigest() != record["git_blob_sha1"]:
            raise PreparationError(f"{name}: Git blob checksum mismatch")
        if len(data) != record["bytes"] or len(data.splitlines()) != record["lines"]:
            raise PreparationError(f"{name}: source size or line count mismatch")
        sources[name] = data
    ir_source = sources["wiiuse/ir.c"]
    ir_blob = hashlib.sha1(
        b"blob " + str(len(ir_source)).encode("ascii") + b"\0" + ir_source
    ).hexdigest()
    if ir_blob != IR_GIT_BLOB:
        raise PreparationError("ir.c differs from the pinned upstream Git blob")

    header = manifest["portable_header"]
    _check_sha256((COMPONENT / header["path"]).read_bytes(), header["sha256"], "ir.h")
    for section in manifest["extraction"]["header_sections"]:
        _section(sources[section["source"]], section)

    chunks = [
        b'#include "libogc_ir/ir.h"\n#include <math.h>\n\n',
        (
            "/*\n"
            " * Generated libogc Wiiuse IR math extraction; modified 2026-09-11.\n"
            f" * Upstream commit: {UPSTREAM_COMMIT}\n"
            " * Function bodies and algorithm constants below are unchanged.\n"
            " * Only platform isolation, declarations and provenance are adapted.\n"
            " * GNU GPL with the libogc section 18 linking exception, explicitly\n"
            " * retained and extended to this modified component. See\n"
            " * external/libogc_ir/license_libogc.txt, libogc_license.txt,\n"
            " * NOTICE.txt and UPSTREAM.json. The original Wiiuse notice follows.\n"
            " */\n"
        ).encode(),
    ]
    for section in manifest["extraction"]["c_sections"]:
        source = section["source"]
        local_path = manifest["sources"][source]["local_path"]
        chunks.append(
            f'\n#line {section["first_line"]} "external/libogc_ir/{local_path}"\n'.encode()
        )
        chunks.append(_section(sources[source], section))
        if source == "wiiuse/definitions.h" and section["first_line"] == 31:
            # Upstream uses the GNU empty-tail form DEBUG(fmt, ...). A purely
            # variadic disabled macro has identical expansion and is valid C11.
            chunks.append(
                b"\n/* C11 portability only: logging stays disabled. */\n"
                b"#undef WIIUSE_DEBUG\n#define WIIUSE_DEBUG(...)\n"
            )
    generated = b"".join(chunks)

    if not output.exists() or output.read_bytes() != generated:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(generated)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        prepare(args.output)
    except (PreparationError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
