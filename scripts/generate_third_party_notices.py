#!/usr/bin/env python3

from __future__ import annotations

import argparse
import filecmp
from pathlib import Path


NOTICE_FILE_NAMES = (
    "LICENSE",
    "LICENSE.txt",
    "LICENSE.md",
    "COPYING",
    "COPYING.txt",
    "COPYING.md",
    "NOTICE",
    "NOTICE.txt",
    "COPYRIGHT",
    "copyright",
)

THIRD_PARTY_NOTICES_COVER = """Third-Party Notices

The third-party notice files in this repository and in generated DepthAI
distributions are provided to reproduce license terms, copyright notices,
attribution notices, and related notices for third-party software included
with, linked into, or distributed alongside DepthAI artifacts.

Each third-party component remains licensed by its own copyright holders under
the terms reproduced for that component. This notice does not replace, modify,
narrow, or expand those third-party license terms.

To the extent Luxonis owns copyright in the selection, arrangement, headings,
or explanatory text added around those third-party notices, Luxonis permits
copying and distribution of that added material solely as part of reproducing
the corresponding third-party notices with DepthAI artifacts.
"""

VENDORED_COMPONENTS = (
    ("xtl", "3rdparty/xtl/LICENSE"),
    ("xtensor", "3rdparty/xtensor/LICENSE"),
    ("xtensor-python", "bindings/python/external/xtensor-python/LICENSE"),
    ("pybind11-opencv-numpy", "bindings/python/external/pybind11_opencv_numpy/LICENSE"),
    ("foxglove-websocket", "3rdparty/foxglove/ws-protocol/cpp/foxglove-websocket/LICENSE"),
    ("foxglove-ws-protocol", "3rdparty/foxglove/ws-protocol/LICENSE.md"),
    ("depthai-bootloader-shared", "shared/depthai-bootloader-shared/LICENSE"),
)

VENDORED_COMPONENTS_WITH_INLINE_NOTICES = (
    (
        "hedley",
        "bindings/python/external/hedley/include/hedley/hedley.h",
        "Hedley declares CC0-1.0 in its public header.",
    ),
    (
        "pybind11-json",
        "bindings/python/external/pybind11_json/include/pybind11_json/pybind11_json.hpp",
        "pybind11_json declares BSD-3-Clause in its public header.",
    ),
)

OPTION_CACHE_KEYS = (
    "DEPTHAI_OPENCV_SUPPORT",
    "DEPTHAI_PCL_SUPPORT",
    "DEPTHAI_RTABMAP_SUPPORT",
    "DEPTHAI_BASALT_SUPPORT",
    "DEPTHAI_ENABLE_KOMPUTE",
    "DEPTHAI_ENABLE_CURL",
    "DEPTHAI_ENABLE_REMOTE_CONNECTION",
    "DEPTHAI_ENABLE_PROTOBUF",
    "DEPTHAI_ENABLE_MP4V2",
    "DEPTHAI_ENABLE_LIBUSB",
    "DEPTHAI_ENABLE_APRIL_TAG",
    "DEPTHAI_ENABLE_BACKWARD",
    "DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate third-party notices for DepthAI builds.")
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--install-root", type=Path)
    parser.add_argument("--triplet")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--check-file", type=Path)
    return parser.parse_args()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace").strip()


def read_leading_block_comment(path: Path) -> str:
    text = path.read_text(encoding="utf-8", errors="replace").lstrip()
    if not text.startswith("/*"):
        return read_text(path)
    end = text.find("*/")
    if end == -1:
        return read_text(path)
    return text[: end + 2].strip()


def read_cmake_cache(build_dir: Path) -> dict[str, str]:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return {}

    values: dict[str, str] = {}
    for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_type, value = line.split("=", 1)
        key = key_type.split(":", 1)[0]
        values[key] = value
    return values


def find_notice_file(directory: Path) -> Path | None:
    for name in NOTICE_FILE_NAMES:
        candidate = directory / name
        if candidate.is_file():
            return candidate
    return None


def collect_vcpkg_notices(install_root: Path, triplet: str) -> list[tuple[str, Path]]:
    share_root = install_root / triplet / "share"
    if not share_root.is_dir():
        return []

    notices: list[tuple[str, Path]] = []
    for package_dir in sorted(path for path in share_root.iterdir() if path.is_dir()):
        notice_path = find_notice_file(package_dir)
        if notice_path:
            notices.append((package_dir.name, notice_path))
    return notices


def collect_vendored_notices(repo_root: Path) -> list[tuple[str, Path, str | None]]:
    notices: list[tuple[str, Path, str | None]] = []
    for name, relative_path in VENDORED_COMPONENTS:
        path = repo_root / relative_path
        if path.is_file():
            notices.append((name, path, None))

    for name, relative_path, note in VENDORED_COMPONENTS_WITH_INLINE_NOTICES:
        path = repo_root / relative_path
        if path.is_file():
            notices.append((name, path, note))
    return notices


def section(lines: list[str], title: str, underline: str = "=") -> None:
    lines.extend([title, underline * len(title), ""])


def append_notice(lines: list[str], name: str, source: Path, text: str, note: str | None = None) -> None:
    lines.extend([name, "-" * len(name), ""])
    lines.append(f"Source notice: {source.as_posix()}")
    if note:
        lines.append(f"Note: {note}")
    lines.extend(["", text.rstrip(), ""])


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    build_dir = args.build_dir.resolve()
    output_dir = args.output_dir.resolve()
    cache = read_cmake_cache(build_dir)

    install_root = args.install_root
    if install_root is None:
        install_root = Path(cache.get("VCPKG_INSTALLED_DIR", build_dir / "vcpkg_installed"))
    install_root = install_root.resolve()

    triplet = args.triplet or cache.get("VCPKG_TARGET_TRIPLET")
    if not triplet:
        candidates = sorted(install_root.glob("*/share"))
        if candidates:
            triplet = candidates[0].parent.name
    if not triplet:
        raise RuntimeError("Cannot determine VCPKG_TARGET_TRIPLET for third-party notice generation.")

    vcpkg_notices = collect_vcpkg_notices(install_root, triplet)
    if not vcpkg_notices:
        raise FileNotFoundError(f"No vcpkg license notices found under {install_root / triplet / 'share'}")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "THIRD_PARTY_NOTICES"

    lines: list[str] = []
    section(lines, "DepthAI Third-Party Notices")
    lines.extend(
        [
            THIRD_PARTY_NOTICES_COVER.rstrip(),
            "",
            "Build Options",
            "-------------",
            "",
        ]
    )
    for key in OPTION_CACHE_KEYS:
        if key in cache:
            lines.append(f"{key}: {cache[key]}")
    lines.extend(["", f"VCPKG_TARGET_TRIPLET: {triplet}", ""])

    section(lines, "Vendored Components")
    for name, path, note in collect_vendored_notices(repo_root):
        if path.suffix in (".h", ".hpp"):
            notice_text = read_leading_block_comment(path)
        else:
            notice_text = read_text(path)
        append_notice(lines, name, path.relative_to(repo_root), notice_text, note)

    section(lines, "vcpkg Components")
    for name, path in vcpkg_notices:
        append_notice(lines, name, path.relative_to(install_root), read_text(path))

    output_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    if args.check_file:
        check_file = args.check_file.resolve()
        if not filecmp.cmp(output_path, check_file, shallow=False):
            raise RuntimeError(
                "Generated third-party notices do not match the committed file.\n"
                f"Generated: {output_path}\n"
                f"Committed: {check_file}\n"
                "Regenerate notices/THIRD_PARTY_NOTICES if dependency changes are intentional."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
