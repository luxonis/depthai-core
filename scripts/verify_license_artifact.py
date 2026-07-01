#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify DepthAI license files in packaged artifacts.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    wheel = subparsers.add_parser("wheel", help="Verify Python wheel license files.")
    wheel.add_argument("wheels", type=Path, nargs="+")

    install_tree = subparsers.add_parser("install-tree", help="Verify CMake install-tree license files.")
    install_tree.add_argument("prefix", type=Path)

    return parser.parse_args()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def verify_wheel(wheel_path: Path) -> None:
    _require(wheel_path.is_file(), f"Wheel does not exist: {wheel_path}")
    with zipfile.ZipFile(wheel_path) as wheel:
        names = wheel.namelist()

        has_root_license = any(
            name.endswith(".dist-info/licenses/LICENSE")
            for name in names
        )
        has_third_party_notices = any(
            name.endswith(".dist-info/licenses/notices/THIRD_PARTY_NOTICES")
            for name in names
        )

        _require(has_root_license, f"Wheel is missing root LICENSE: {wheel_path}")
        _require(has_third_party_notices, f"Wheel is missing .dist-info/licenses/notices/THIRD_PARTY_NOTICES: {wheel_path}")


def verify_install_tree(prefix: Path) -> None:
    _require(prefix.is_dir(), f"Install prefix does not exist: {prefix}")
    candidates = (
        prefix / "share" / "depthai",
        prefix / "share" / "depthai-core",
    )
    license_dirs = [candidate for candidate in candidates if candidate.is_dir()]
    _require(bool(license_dirs), f"Install tree is missing share/depthai license directory: {prefix}")

    root = license_dirs[0]
    _require((root / "LICENSE").is_file(), f"Install tree is missing root LICENSE under: {root}")
    _require(
        (root / "notices" / "THIRD_PARTY_NOTICES").is_file(),
        f"Install tree is missing notices/THIRD_PARTY_NOTICES under: {root}",
    )


def main() -> int:
    args = parse_args()
    try:
        if args.command == "wheel":
            for wheel in args.wheels:
                verify_wheel(wheel)
        elif args.command == "install-tree":
            verify_install_tree(args.prefix)
        else:
            raise AssertionError(args.command)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
