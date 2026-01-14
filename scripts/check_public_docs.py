#!/usr/bin/env python3
import argparse
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Optional, Tuple


DOC_LINE_RE = re.compile(r"\s*///")
DOC_BLOCK_START_RE = re.compile(r"\s*/\*\*")
DOC_BLOCK_END_RE = re.compile(r".*\*/\s*$")
ACCESS_RE = re.compile(r"^\s*(public|protected|private)\s*:\s*$")
IGNORE_LINE_RE = re.compile(r"\bDEPTHAI_SERIALIZE(?:_EXT)?\s*\(")
IGNORE_OVERRIDE_RE = re.compile(r"\boverride\b")
IGNORE_VIRTUAL_RE = re.compile(r"\bvirtual\b")
IGNORE_METHOD_NAME_RE = re.compile(r"^(getLINKiD|getLinkId)$")
IGNORE_CLASS_NAME_RE = re.compile(r".*(Impl|Internal|Private)$")
IGNORE_DIR_PARTS = {
    "/pipeline/node/internal/",
}
IGNORE_FILES = {
    "CapatibulityRange.hpp",
    "CapabilityRange.hpp",
    "Zoo.hpp",
    "OpenVINO.hpp",
    "Points3fRGBA.hpp",
    "Point3fRGBA.hpp",
    "Size2f.hpp",
    "Variant.hpp",
    "optional.hpp",
    "ADatatypeSharedPtrSerialization.hpp",
    "TesorInfo.hpp",
    "TensorInfo.hpp",
    "variant.hpp",
}
IGNORE_FILE_SUBSTRINGS = {
    "Variant",
}
IGNORE_CONSTRUCTORS_FOR = {
    "ImgTransformation",
}

# Heuristic for function declarations/definitions in headers.
FUNC_RE = re.compile(
    r"""
    ^\s*
    (?:template\s*<[^>]*>\s*)?            # template
    (?:inline\s+|virtual\s+|static\s+|constexpr\s+|friend\s+|explicit\s+|final\s+|override\s+)*  # specifiers
    (?:[\w:<>~*&]+(?:\s+[\w:<>~*&]+)*)    # return type (requires at least one token)
    \s+                                   # space between return type and name
    \b([~\w:]+)\s*                         # name
    \([^;{)]*\)                            # params
    (?:\s*const|\s*noexcept|\s*override|\s*final|\s*=\s*0|\s*&|\s*&&|\s*->\s*[\w:<>]+)*  # qualifiers
    \s*(?:;|\{)                            # end or inline body
    """,
    re.VERBOSE,
)


@dataclass
class Context:
    brace_depth: int
    access: str
    class_name: str


def iter_headers(root: Path) -> Iterator[Path]:
    for dirpath, _, filenames in os.walk(root):
        normalized = dirpath.replace("\\", "/")
        if "/utility/" in normalized or normalized.endswith("/utility"):
            continue
        if any(part in normalized for part in IGNORE_DIR_PARTS) or normalized.endswith("/pipeline/node/internal"):
            continue
        for name in filenames:
            if name in IGNORE_FILES or any(sub in name for sub in IGNORE_FILE_SUBSTRINGS):
                continue
            if name.endswith((".hpp", ".h")):
                yield Path(dirpath) / name


def is_class_or_struct(line: str) -> Optional[Tuple[str, str]]:
    match = re.search(r"\b(class|struct)\s+([A-Za-z_]\w*)\b", line)
    if not match:
        return None
    return match.group(1), match.group(2)


def update_doc_state(
    line: str,
    in_doc_block: bool,
    last_doc_end: Optional[int],
    line_no: int,
) -> Tuple[bool, Optional[int]]:
    if in_doc_block:
        if DOC_BLOCK_END_RE.search(line):
            return False, line_no
        return True, last_doc_end

    if DOC_BLOCK_START_RE.search(line):
        if DOC_BLOCK_END_RE.search(line):
            return False, line_no
        return True, last_doc_end

    if DOC_LINE_RE.search(line):
        return False, line_no

    return False, last_doc_end


def is_method_candidate(line: str, class_name: str) -> Optional[str]:
    stripped = line.lstrip()
    if re.match(r"^[A-Za-z_]\w*\s+\w+\s*\([^)]*\)\s*;", stripped):
        return None
    if stripped.startswith("{") or stripped.startswith("}"):
        return None
    if stripped.startswith(("if ", "for ", "while ", "switch ", "catch ", "return ", "throw ", "else ", "do ", "case ", "default ")):
        return None
    if stripped.startswith(("//", "/*", "*", ":", ",")):
        return None
    if class_name:
        ctor_re = rf"^\s*(?:explicit\s+)?{re.escape(class_name)}\s*\("
        if re.match(ctor_re, line):
            if " = default" in line or "= default" in line:
                return None
            if class_name in IGNORE_CONSTRUCTORS_FOR:
                return None
            return class_name
    match = FUNC_RE.match(line)
    if not match:
        return None
    return match.group(1)


def has_intervening_code(lines: List[str], start: int, end: int) -> bool:
    for idx in range(start, end):
        text = lines[idx].strip()
        if not text:
            continue
        if text.startswith("template"):
            continue
        if text in {"{", "}", "};"}:
            continue
        if text.startswith("//") or text.startswith("/*") or text.startswith("*"):
            continue
        return True
    return False


def check_file(path: Path) -> List[Tuple[str, int, str]]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    results: List[Tuple[str, int, str]] = []
    ctx_stack: List[Context] = []
    brace_depth = 0
    access = "global"
    last_doc_end: Optional[int] = None
    in_doc_block = False

    for idx, line in enumerate(lines):
        line_no = idx + 1

        in_doc_block, last_doc_end = update_doc_state(line, in_doc_block, last_doc_end, line_no)

        access_match = ACCESS_RE.match(line)
        if access_match and ctx_stack:
            access = access_match.group(1)
            ctx_stack[-1].access = access

        decl = is_class_or_struct(line)
        if decl and "{" in line:
            decl_kind, class_name = decl
            default_access = "public" if decl_kind == "struct" else "private"
            ctx_stack.append(
                Context(
                    brace_depth=brace_depth + line.count("{") - line.count("}"),
                    access=default_access,
                    class_name=class_name,
                )
            )
            access = default_access

        method_name = is_method_candidate(line, ctx_stack[-1].class_name if ctx_stack else "")
        if method_name and ctx_stack and ctx_stack[-1].access == "public":
            if IGNORE_LINE_RE.search(line):
                continue
            if IGNORE_OVERRIDE_RE.search(line):
                continue
            if IGNORE_VIRTUAL_RE.search(line):
                continue
            if IGNORE_METHOD_NAME_RE.match(method_name):
                continue
            if IGNORE_CLASS_NAME_RE.match(ctx_stack[-1].class_name):
                continue
            documented = False
            if last_doc_end is not None:
                doc_end_idx = last_doc_end - 1
                if not has_intervening_code(lines, doc_end_idx + 1, idx):
                    documented = True
            if not documented:
                results.append((str(path), line_no, line.strip()))

        brace_depth += line.count("{") - line.count("}")
        while ctx_stack and brace_depth < ctx_stack[-1].brace_depth:
            ctx_stack.pop()
            access = ctx_stack[-1].access if ctx_stack else "global"

    return results


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Report public methods without adjacent doc comments."
    )
    parser.add_argument(
        "root",
        nargs="?",
        default="include/depthai",
        help="Root directory to scan (default: include/depthai)",
    )
    args = parser.parse_args()

    root = Path(args.root)
    if not root.exists():
        print(f"error: root not found: {root}", file=sys.stderr)
        return 2

    missing = []
    for header in iter_headers(root):
        missing.extend(check_file(header))

    for path, line_no, sig in missing:
        print(f"{path}:{line_no}: {sig}")

    if missing:
        print(f"error: {len(missing)} public methods are missing docs", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
