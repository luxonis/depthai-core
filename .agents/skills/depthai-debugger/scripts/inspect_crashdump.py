#!/usr/bin/env python3
"""Inspect crash artifacts offline; extract only recognized SDK payloads."""

import argparse
import bz2
from contextlib import ExitStack
import gzip
import hashlib
import json
import lzma
from pathlib import Path, PurePosixPath
import shutil
import struct
import sys
import tarfile

JSON_LIMIT = 8 * 1024 * 1024
PAYLOADS = {"metadata.json", "extra.json", "crash_reports.json", "crash_dump.tar.gz"}


class LimitedReader:
    """Bound decompressed reads before tarfile parses extension headers."""

    def __init__(self, stream, budget):
        self.stream = stream
        self.budget = budget

    def read(self, size=-1):
        limit = self.budget["expanded_bytes"] + 1
        data = self.stream.read(limit if size < 0 else min(size, limit))
        self.budget["expanded_bytes"] -= len(data)
        if self.budget["expanded_bytes"] < 0:
            raise ValueError("archive exceeds --max-bytes (including headers and nested streams)")
        return data


def fingerprint(path):
    result = {"path": str(path.resolve()), "size": path.stat().st_size}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    result["sha256"] = digest.hexdigest()
    return result


def inspect_tar(path, output, budget, nested=False):
    result = {"kind": "unknown_archive", "members": [], "warnings": []}
    seen = set()
    documents = {}
    with ExitStack() as stack:
        stream = stack.enter_context(path.open("rb"))
        signature = stream.read(6)
        stream.seek(0)
        for magic, opener in ((b"\x1f\x8b", gzip.open), (b"BZh", bz2.open), (b"\xfd7zXZ\x00", lzma.open)):
            if signature.startswith(magic):
                stream = stack.enter_context(opener(stream, "rb"))
                break
        archive = stack.enter_context(tarfile.open(fileobj=LimitedReader(stream, budget), mode="r|"))
        for member in archive:
            name = PurePosixPath(member.name)
            if name.is_absolute() or ".." in name.parts or "\\" in member.name or ":" in member.name:
                raise ValueError("unsafe archive path: " + member.name)
            normalized = str(name)
            if normalized in seen:
                raise ValueError("duplicate archive member: " + normalized)
            seen.add(normalized)
            budget["members"] -= 1
            if budget["members"] < 0:
                raise ValueError("archive member limit exceeded")
            if not member.isfile() and not member.isdir():
                raise ValueError("archive links/special files are not supported: " + normalized)
            budget["bytes"] -= member.size
            if member.size < 0 or budget["bytes"] < 0:
                raise ValueError("archive exceeds --max-bytes (including nested members)")
            result["members"].append({"name": normalized, "size": member.size})
            if member.isdir():
                continue
            if name.name.startswith("core."):
                result["kind"] = "core_candidate_archive"
                with archive.extractfile(member) as source:
                    header = source.read(64)
                candidate = {"name": normalized, "elf_core_header": False}
                if len(header) >= 20 and header.startswith(b"\x7fELF") and header[5] in (1, 2):
                    elf_type, machine = struct.unpack(("<" if header[5] == 1 else ">") + "HH", header[16:20])
                    candidate.update(elf_core_header=elf_type == 4, elf_machine=machine)
                result.setdefault("core_candidates", []).append(candidate)
                if not candidate["elf_core_header"]:
                    result["warnings"].append(normalized + ": no uncompressed ELF core header; check format/integrity.")
            if nested or normalized not in PAYLOADS:
                continue
            if normalized.endswith(".json") and member.size > JSON_LIMIT:
                raise ValueError("JSON payload exceeds 8 MiB: " + normalized)
            destination = output / normalized
            source = archive.extractfile(member)
            if source is None:
                raise ValueError("unreadable payload: " + normalized)
            with source, destination.open("xb") as target:
                shutil.copyfileobj(source, target, 1024 * 1024)
            if normalized in ("metadata.json", "crash_reports.json"):
                document = json.loads(destination.read_text(encoding="utf-8"))
                if not isinstance(document, dict):
                    raise ValueError(normalized + " must contain a JSON object")
                documents[normalized] = document

    metadata = documents.get("metadata.json", {})
    if metadata:
        result["metadata"] = metadata
    reports = documents.get("crash_reports.json")
    if reports is not None:
        if not isinstance(reports.get("crashReports"), list):
            raise ValueError("crash_reports.json lacks a crashReports list")
        result.update(
            kind="rvc2_archive",
            firmware_commit=reports.get("depthaiCommitHash"),
            report_count=len(reports["crashReports"]),
        )
        if metadata.get("platform") not in (None, "RVC2"):
            raise ValueError("metadata platform conflicts with RVC2 payload")

    payload = output / "crash_dump.tar.gz"
    if not nested and payload.exists():
        if reports is not None or metadata.get("platform") not in (None, "RVC4"):
            raise ValueError("metadata/payload conflict in RVC4 archive")
        result.update(
            kind="rvc4_archive",
            nested=inspect_tar(payload, output, budget, nested=True),
        )
    if result["kind"] == "unknown_archive":
        result["warnings"].append("No recognized crash payload; inspect member inventory.")
    if reports is not None and not reports["crashReports"]:
        result["warnings"].append("RVC2 crashReports is empty.")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; parent must exist")
    parser.add_argument("--symbol-file", type=Path, action="append", default=[], help="Hash only; does not establish a symbol match")
    parser.add_argument("--max-bytes", type=int, default=512 * 1024 * 1024, help="Limit payload and decompressed tar bytes across all layers, including headers/padding (default: 512 MiB)")
    args = parser.parse_args()
    if args.max_bytes <= 0:
        parser.error("--max-bytes must be positive")
    if not args.artifact.is_file():
        parser.error("artifact must be an existing regular file")
    try:
        args.output_dir.mkdir(mode=0o700)
    except OSError as error:
        parser.error(str(error))

    report = {"schema_version": 1, "status": "inspecting", "symbol_match": "not_verified"}
    result = 0
    try:
        report["artifact"] = fingerprint(args.artifact)
        report["symbol_files"] = [fingerprint(path) for path in args.symbol_file]
        with args.artifact.open("rb") as stream:
            prefix = stream.read(64)
        if prefix.startswith(b"\x7fELF"):
            if len(prefix) < 20 or prefix[5] not in (1, 2):
                raise ValueError("invalid ELF header")
            elf_type, machine = struct.unpack(("<" if prefix[5] == 1 else ">") + "HH", prefix[16:20])
            report["content"] = {
                "kind": "elf_core" if elf_type == 4 else "elf_non_core",
                "elf_machine": machine,
                "note": "Architecture alone does not identify the process, platform or matching symbols.",
            }
        elif args.artifact.suffix.lower() == ".json" or prefix.lstrip().startswith(b"{"):
            if args.artifact.stat().st_size > min(JSON_LIMIT, args.max_bytes):
                raise ValueError("JSON exceeds size limit")
            document = json.loads(args.artifact.read_text(encoding="utf-8"))
            if not isinstance(document, dict) or not isinstance(document.get("crashReports"), list):
                raise ValueError("not a recognized legacy RVC2 crash report")
            report["content"] = {
                "kind": "rvc2_json", "firmware_commit": document.get("depthaiCommitHash"),
                "report_count": len(document["crashReports"]),
                "warnings": [] if document["crashReports"] else ["RVC2 crashReports is empty."],
            }
            with (args.output_dir / "crash_reports.json").open("xb") as target:
                with args.artifact.open("rb") as source:
                    shutil.copyfileobj(source, target)
        else:
            report["content"] = inspect_tar(
                args.artifact, args.output_dir,
                {"bytes": args.max_bytes, "expanded_bytes": args.max_bytes, "members": 4096},
            )
        report["status"] = "inspected"
    except (OSError, ValueError, tarfile.TarError, EOFError, lzma.LZMAError) as error:
        report.update(status="failed", error=str(error))
        result = 2
    (args.output_dir / "inspection.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8",
    )
    print(args.output_dir / "inspection.json")
    return result


if __name__ == "__main__":
    sys.exit(main())
