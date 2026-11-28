#!/usr/bin/env python3

from __future__ import annotations

import argparse
import configparser
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


NOASSERTION = "NOASSERTION"
NOTICE_FILE_NAMES = (
    "LICENSE",
    "LICENSE.txt",
    "LICENSE.md",
    "LICENSE.MIT",
    "COPYING",
    "COPYING.txt",
    "COPYING.README",
    "NOTICE",
    "NOTICE.txt",
    "COPYRIGHT",
)
NOTICE_LICENSE_PATTERNS = [
    ("Apache-2.0", ("apache license", "version 2.0")),
    ("MIT", ("mit license",)),
    ("BSD-3-Clause", ("bsd 3-clause license",)),
    ("BSD-2-Clause", ("bsd 2-clause license",)),
    ("BSL-1.0", ("boost software license",)),
    ("ISC", ("isc license",)),
    ("Zlib", ("zlib license",)),
    ("libpng-2.0", ("png reference library license",)),
    ("MPL-2.0", ("mozilla public license", "version 2.0")),
    ("LGPL-2.1-or-later", ("gnu lesser general public license", "version 2.1")),
    ("LGPL-3.0-or-later", ("gnu lesser general public license", "version 3")),
    ("GPL-2.0-only", ("gnu general public license", "version 2")),
    ("GPL-3.0-only", ("gnu general public license", "version 3")),
]
VENDORED_DEPENDENCIES = {  # Paths relative to device/rvc4; Eigen comes from the vcpkg install tree
    "external/cpp-subprocess": {
        "package": "cpp-subprocess",
        "license_expression": "MIT",
        "description": "C++ subprocess helper library.",
    },
}

PRIVATE_MODULE_NAME = "depthai-device-rvc4-vendor"
PRIVATE_MODULE_ENV = "DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR"
PRIVATE_MODULE_VENDORED_DEPENDENCIES = {
    "depthmapwrapper-vd55h1/glog": {
        "package": "glog",  # A copy of glog (no submodule revision): the version comes from its CMake file
        "version_file": ("CMakeLists.txt", r"project\s*\(\s*glog\s+VERSION\s+([0-9][0-9.]*)"),
        "license_expression": "BSD-3-Clause",
        "supplier": "Google Inc.",
        "description": "Google logging library, used by the VD55H1 IPP library.",
    },
    "depthmapwrapper-vd55h1/vd55h1-ipp-adreno": {
        "package": "VD55H1 IPP Library",
        "version_file": ("CMakeLists.txt", r"project\([^)]*?VERSION\s+([0-9][0-9.]*)"),
        "note": "Modified by Luxonis",
        "license_expression": "LicenseRef-ST-SLA0106",
        "supplier": "STMicroelectronics",
        "description": "STMicroelectronics VD55H1 time-of-flight image post-processing (IPP) library for Adreno GPUs.",
        "notice_files": ["ipp-opencl-lib/src/*.cpp", "LICENSE.txt"],
    },
}
VCPKG_LICENSE_OVERRIDES = {
    "libnop": "Apache-2.0",
    "libarchive": "BSD-2-Clause AND BSD-3-Clause AND (CC0-1.0 OR OpenSSL OR Apache-2.0)",
    "liblzma": "0BSD",
    "mp4v2": "MPL-1.1",
    "pybind11": "BSD-3-Clause",
}

EXCLUDED_SUBMODULE_PATHS = {
    "vcpkg": "Package manager; the packages that it builds are collected from the vcpkg install tree.",
}

NOT_LISTED_PATHS: dict[str, str] = {}

PRIVATE_MODULE_NOT_LISTED_PATHS = {  # fwp_sbom.UNLISTED_SOURCES has the same classification
    "depthmapwrapper-vd55h1": "depthmapwrapper-lib was received from ST with the VD55H1 IPP library; its "
    "sources are marked 'Copyright Qualcomm Technologies, Inc. Confidential and Proprietary' and have no "
    "open-source license (the IPP library itself is under ST SLA0106). Its third-party parts glog and "
    "vd55h1-ipp-adreno are listed separately.",
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Generate third_party_licenses.json for depthai-device from the "
            "RVC4 vcpkg install tree and repo-local vendored dependencies."
        )
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=repo_root,
        help="Path to the depthai-device repository root.",
    )
    parser.add_argument(
        "--manifest-root",
        type=Path,
        default=None,
        help="Directory of the vcpkg.json manifest (default: the repository root; the depthai-core root when "
        "the firmware is built as device/rvc4 of the depthai-core repository).",
    )
    parser.add_argument(
        "--triplet",
        default="rvc4",
        help="vcpkg target triplet to collect.",
    )
    parser.add_argument(
        "--vcpkg-root",
        type=Path,
        default=repo_root / "vcpkg",
        help="Path to the vcpkg checkout to use for vcpkg list/license-report.",
    )
    parser.add_argument(
        "--install-root",
        type=Path,
        default=repo_root / "build_docker_arm64_rvc4" / "vcpkg_installed",
        help="Path to the vcpkg installed tree root.",
    )
    parser.add_argument(
        "--private-module-root",
        type=Path,
        default=None,
        help=f"Path to the private {PRIVATE_MODULE_NAME} checkout (default: ${PRIVATE_MODULE_ENV}). Its entries are "
        "skipped when the checkout is absent (a build with the prebuilt vendor archive).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root / "third_party_licenses.json",
        help="Path to the generated JSON output.",
    )
    parser.add_argument(
        "--install",
        action="store_true",
        help="Run 'vcpkg install' before collecting licenses.",
    )
    parser.add_argument(
        "--skip-license-report",
        action="store_true",
        help="Skip invoking 'vcpkg license-report'.",
    )
    return parser.parse_args()


def run(
    cmd: list[str],
    *,
    cwd: Path | None = None,
    capture_output: bool = False,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        check=True,
        text=True,
        capture_output=capture_output,
    )


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def normalize_optional(value: Any) -> Any:
    if value in (None, "", NOASSERTION):
        return None
    return value


def normalize_license_field(value: Any) -> str | None:
    normalized = normalize_optional(value)
    if normalized == "LicenseRef-vcpkg-null":
        return None
    return normalized


def slugify_package_name(value: str) -> str:
    return value.strip().lower().replace("_", "-")


def repo_relative_path(repo_root: Path, path: Path) -> str:
    return str(path.resolve().relative_to(repo_root.resolve()))


def install_relative_path(install_root: Path, path: Path) -> str:
    return str(path.resolve().relative_to(install_root.resolve()))


def path_is_relative_to(path: Path, base: Path) -> bool:
    try:
        path.resolve().relative_to(base.resolve())
    except ValueError:
        return False
    return True


def find_private_module_root(repo_root: Path, override: Path | None = None) -> Path | None:
    """The checkout of the private module: --private-module-root, then $DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR (in the build
    container /depthai-device-rvc4-vendor). No other place: a build with the prebuilt vendor archive has no private-module
    sources, even when a checkout is near. None when no candidate is a directory; the private-module entries are then
    skipped."""
    candidates = (override, os.environ.get(PRIVATE_MODULE_ENV))
    for candidate in candidates:
        if candidate and Path(candidate).is_dir():
            return Path(candidate).resolve()
    return None


def load_builtin_baseline(manifest_root: Path) -> str | None:
    manifest = load_json(manifest_root / "vcpkg.json")
    return normalize_optional(manifest.get("builtin-baseline"))


def run_vcpkg_install(
    vcpkg_executable: Path,
    repo_root: Path,
    install_root: Path,
    triplet: str,
) -> None:
    install_root.mkdir(parents=True, exist_ok=True)
    run(
        [
            str(vcpkg_executable),
            "install",
            f"--x-manifest-root={repo_root}",
            f"--x-install-root={install_root}",
            f"--triplet={triplet}",
        ],
        cwd=repo_root,
    )


def run_vcpkg_list(
    vcpkg_executable: Path,
    install_root: Path,
    triplet: str,
) -> list[dict[str, str]]:
    result = run(
        [
            str(vcpkg_executable),
            "list",
            f"--x-install-root={install_root}",
            f"--triplet={triplet}",
        ],
        capture_output=True,
    )
    packages: list[dict[str, str]] = []
    pattern = re.compile(r"^([^\s:]+):([^\s]+)\s+([^\s]+)")
    for line in result.stdout.splitlines():
        match = pattern.match(line.strip())
        if not match or match.group(2) != triplet:
            continue
        if "[" in match.group(1):
            continue
        packages.append(
            {
                "package": match.group(1),
                "triplet": match.group(2),
                "version": match.group(3),
            }
        )
    return packages


def run_vcpkg_license_report(vcpkg_executable: Path, install_root: Path) -> dict[str, Any]:
    cmd = [
        str(vcpkg_executable),
        "license-report",
        f"--x-install-root={install_root}",
    ]
    try:
        result = run(cmd, capture_output=True)
    except subprocess.CalledProcessError as error:
        stderr = error.stderr.strip() if error.stderr else ""
        if "invalid command: license-report" in stderr:
            reason = "vcpkg command 'license-report' is not available in this checkout"
            stderr = "error: invalid command: license-report"
        else:
            reason = f"command failed with exit code {error.returncode}"
        return {
            "available": False,
            "reason": reason,
            "stdout": error.stdout.strip() if error.stdout else None,
            "stderr": stderr or None,
        }
    return {
        "available": True,
        "stdout": result.stdout.strip(),
    }


def find_package(packages: list[dict[str, Any]], spdx_id: str) -> dict[str, Any] | None:
    for package in packages:
        if package.get("SPDXID") == spdx_id:
            return package
    return None


def collect_download_locations(packages: list[dict[str, Any]]) -> list[str]:
    locations: list[str] = []
    for package in packages:
        spdx_id = package.get("SPDXID", "")
        download_location = normalize_optional(package.get("downloadLocation"))
        if not spdx_id.startswith("SPDXRef-resource-") or not download_location:
            continue
        if download_location not in locations:
            locations.append(download_location)
    return locations


def read_notice_text(path: Path | None) -> str | None:
    if not path or not path.is_file():
        return None
    return path.read_text(encoding="utf-8", errors="replace")


def read_notice_file(path: Path) -> str:
    """The text of a notice file; for a C/C++ source or header, only its leading comment block."""
    text = path.read_text(encoding="utf-8", errors="replace")
    if path.suffix in (".h", ".hpp", ".c", ".cpp"):
        match = re.match(r"\s*(/\*.*?\*/)", text, re.S)
        if match:
            return match.group(1)
    return text


def find_third_party_gaps(repo_root: Path) -> list[str]:  # Skips ../ submodules: depthai-core's compliance covers them
    """Git submodules and tracked directories in external/ that no table classifies."""
    candidates = {path for path in read_git_submodule_status(repo_root) if not path.startswith("../")}
    tracked = run(["git", "-C", str(repo_root), "ls-files", "external"], capture_output=True).stdout.splitlines()
    candidates.update("/".join(path.split("/")[:2]) for path in tracked if path.count("/") >= 2)
    gaps = []
    for path in sorted(candidates):
        if path in VENDORED_DEPENDENCIES or path in NOT_LISTED_PATHS:
            continue
        if any(path == excluded or path.startswith(excluded + "/") for excluded in EXCLUDED_SUBMODULE_PATHS):
            continue
        gaps.append(path)
    return gaps


def detect_license_from_notice(notice_text: str | None) -> str | None:
    if not notice_text:
        return None

    lowered = notice_text[:4000].lower()
    if "spdx-license-identifier: mpl-2.0" in lowered:
        return "MPL-2.0"
    if "mozilla public license" in lowered and "version 1.1" in lowered:
        return "MPL-1.1"
    if "spdx-license-identifier: lgpl-2.1-or-later" in lowered:
        return "LGPL-2.1-or-later"
    if "bsd-3-clause" in lowered:
        return "BSD-3-Clause"
    if (
        "redistribution and use in source and binary forms" in lowered
        and (
            "contributors may be used to endorse or promote products derived from" in lowered
            or "contributors to endorse or promote products derived from" in lowered
        )
    ):
        return "BSD-3-Clause"

    for license_id, markers in NOTICE_LICENSE_PATTERNS:
        if all(marker in lowered for marker in markers):
            return license_id
    return None


def read_git_submodule_status(repo_root: Path, *, strict: bool = True) -> dict[str, str]:
    """Submodule path -> checked-out revision, for the submodules below repo_root (the firmware tree is a
    directory of the depthai-core repository; the other submodules of that repository are not read).
    Nested submodules are included when git can enter the submodules. It cannot when the checkout is a git
    worktree that is mounted in a container at another path (the .git file of a submodule of a worktree has a
    relative path to the main checkout): then the first level comes from the index, with the recorded revisions.
    With strict=False a directory that is not a git checkout (for example the private module mounted as a
    plain directory) gives an empty result."""
    result = None
    error: Exception | None = None
    for recursion in (["--recursive"], []):
        try:
            result = run(
                ["git", "-C", str(repo_root), "submodule", "status", *recursion, "--", "."],
                capture_output=True,
            )
            break
        except (OSError, subprocess.CalledProcessError) as exc:
            error = exc
    if result is None:
        try:
            index = run(["git", "-C", str(repo_root), "ls-files", "--stage", "--", "."], capture_output=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            if strict:
                raise error or exc
            return {}
        recorded: dict[str, str] = {}
        for line in index.stdout.splitlines():
            fields, _, path = line.partition("\t")
            mode, revision, *_ = fields.split()
            if mode == "160000":
                recorded[path] = revision
        return recorded
    revisions: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
        revision = line[1:41].strip()
        remainder = line[41:].strip()
        path = remainder.split(" ", 1)[0]
        revisions[path] = revision
    return revisions


def parse_gitmodules(repo_root: Path, *, strict: bool = True) -> dict[str, dict[str, str]]:
    """Submodules of a checkout, by path, from its .gitmodules and from the .gitmodules of its submodules and
    first-level directories (for example glog in depthmapwrapper-vd55h1 of the private module)."""
    revisions = read_git_submodule_status(repo_root, strict=strict)
    submodules: dict[str, dict[str, str]] = {}
    nested_roots = set(revisions)
    nested_roots.update(path.parent.relative_to(repo_root).as_posix() for path in repo_root.glob("*/.gitmodules"))
    for submodule_path in sorted(nested_roots):
        nested = repo_root / submodule_path / ".gitmodules"
        if not nested.is_file():
            continue
        nested_parser = configparser.ConfigParser()
        nested_parser.read(nested, encoding="utf-8")
        for section in nested_parser.sections():
            path = nested_parser.get(section, "path", fallback=None)
            if path:
                full_path = f"{submodule_path}/{path}"
                submodules[full_path] = {
                    "logical_name": section[len('submodule "') : -1],
                    "path": full_path,
                    "url": nested_parser.get(section, "url", fallback="") or "",
                    "revision": revisions.get(full_path, ""),
                }
    gitmodules_path = repo_root / ".gitmodules"
    prefix = ""
    if not gitmodules_path.is_file():
        try:
            toplevel = Path(run(["git", "-C", str(repo_root), "rev-parse", "--show-toplevel"], capture_output=True).stdout.strip())
        except (OSError, subprocess.CalledProcessError):
            if strict:
                raise
            return submodules
        if toplevel.resolve() == repo_root.resolve() or not (toplevel / ".gitmodules").is_file():
            return submodules
        gitmodules_path = toplevel / ".gitmodules"
        prefix = repo_root.resolve().relative_to(toplevel.resolve()).as_posix() + "/"
    parser = configparser.ConfigParser()
    parser.read(gitmodules_path, encoding="utf-8")
    for section in parser.sections():
        if not section.startswith('submodule "'):
            continue
        logical_name = section[len('submodule "') : -1]
        path = parser.get(section, "path", fallback=None)
        url = parser.get(section, "url", fallback=None)
        if not path:
            continue
        if prefix:
            if not path.startswith(prefix):
                continue
            path = path[len(prefix) :]
        submodules[path] = {
            "logical_name": logical_name,
            "path": path,
            "url": url or "",
            "revision": revisions.get(path, ""),
        }
    return submodules


def find_notice_file(base_dir: Path) -> Path | None:
    for relative_name in NOTICE_FILE_NAMES:
        candidate = base_dir / relative_name
        if candidate.is_file():
            return candidate

    for depth in (1, 2):
        pattern = "*/" * depth
        for relative_name in NOTICE_FILE_NAMES:
            matches = sorted(base_dir.glob(f"{pattern}{relative_name}"))
            if matches:
                return matches[0]
    return None


def find_vcpkg_buildtree_notice(vcpkg_root: Path, package: str) -> Path | None:
    buildtree = vcpkg_root / "buildtrees" / package / "src"
    if not buildtree.is_dir():
        return None

    for source_dir in sorted(buildtree.glob("*.clean")):
        notice_path = find_notice_file(source_dir)
        if notice_path:
            return notice_path
    return None


def notice_path_for_output(repo_root: Path, install_root: Path, notice_path: Path | None) -> str | None:
    if not notice_path:
        return None
    if path_is_relative_to(notice_path, install_root):
        return install_relative_path(install_root, notice_path)
    if path_is_relative_to(notice_path, repo_root):
        return repo_relative_path(repo_root, notice_path)
    return str(notice_path)


def build_package_entry(
    repo_root: Path,
    install_root: Path,
    vcpkg_root: Path,
    triplet: str,
    share_dir: Path,
) -> dict[str, Any]:
    spdx_path = share_dir / "vcpkg.spdx.json"
    notice_path = share_dir / "copyright"
    if not notice_path.is_file():
        notice_path = find_vcpkg_buildtree_notice(vcpkg_root, share_dir.name) or notice_path

    spdx_document = load_json(spdx_path) if spdx_path.is_file() else {}
    packages = spdx_document.get("packages", [])
    port_package = find_package(packages, "SPDXRef-port") or {}
    binary_package = find_package(packages, "SPDXRef-binary") or {}

    raw_license_concluded = normalize_optional(
        port_package.get("licenseConcluded") or binary_package.get("licenseConcluded")
    )
    raw_license_declared = normalize_optional(
        port_package.get("licenseDeclared") or binary_package.get("licenseDeclared")
    )
    license_concluded = normalize_license_field(raw_license_concluded)
    license_declared = normalize_license_field(raw_license_declared)
    notice_text = read_notice_text(notice_path)
    notice_license_hint = detect_license_from_notice(notice_text)
    license_expression = (
        VCPKG_LICENSE_OVERRIDES.get(share_dir.name)
        or license_concluded
        or license_declared
        or notice_license_hint
    )
    inferred_license_expression = license_expression
    spdx_license_unresolved = any(
        value == "LicenseRef-vcpkg-null"
        for value in (raw_license_concluded, raw_license_declared)
    )

    return {
        "package": share_dir.name,
        "source_type": "vcpkg",
        "source_root": "vcpkg",
        "source_path": f"{triplet}/share/{share_dir.name}",
        "source_url": normalize_optional(port_package.get("downloadLocation")),
        "triplet": triplet,
        "version": normalize_optional(port_package.get("versionInfo")),
        "binary_version": normalize_optional(binary_package.get("versionInfo")),
        "homepage": normalize_optional(port_package.get("homepage")),
        "description": normalize_optional(port_package.get("description")),
        "license_expression": license_expression,
        "license_concluded": license_concluded,
        "license_declared": license_declared,
        "spdx_license_unresolved": spdx_license_unresolved,
        "raw_license_concluded": raw_license_concluded,
        "raw_license_declared": raw_license_declared,
        "inferred_license_expression": inferred_license_expression,
        "resource_download_locations": collect_download_locations(packages),
        "spdx_document_namespace": normalize_optional(
            spdx_document.get("documentNamespace")
        ),
        "spdx_path": install_relative_path(install_root, spdx_path) if spdx_path.is_file() else None,
        "notice_path": notice_path_for_output(repo_root, install_root, notice_path if notice_path.is_file() else None),
        "notice_license_hint": notice_license_hint,
        "notice_text": notice_text,
        "submodule_name": None,
    }


def build_vendored_entry(
    repo_root: Path,
    dependency_path: str,
    metadata: dict[str, Any],
    submodule: dict[str, str] | None,
    *,
    base_root: Path | None = None,
    base_name: str | None = None,
) -> dict[str, Any]:
    """A vendored dependency at base_root/dependency_path. base_root is the repository (base_name None) or the
    private module checkout (base_name PRIVATE_MODULE_NAME); the paths in the entry are relative to it."""
    base_root = base_root or repo_root
    source_path = base_root / dependency_path
    if metadata.get("notice_files"):
        notice_paths = []
        for name in metadata["notice_files"]:
            matches = sorted(source_path.glob(name)) if "*" in name else [source_path / name]
            if not matches or not matches[0].is_file():
                raise FileNotFoundError(f"Notice file {name} of {metadata['package']} does not exist in {source_path}")
            notice_paths.append(matches[0])
        notice_path = notice_paths[0]
        notice_text = "\n\n".join(read_notice_file(path).rstrip() for path in notice_paths) + "\n"
    else:
        notice_path = find_notice_file(source_path)
        notice_text = read_notice_text(notice_path)
    notice_license_hint = detect_license_from_notice(notice_text)
    license_expression = metadata.get("license_expression")
    version = None
    if metadata.get("version_file"):
        version_file, pattern = metadata["version_file"]
        match = re.search(pattern, (source_path / version_file).read_text(encoding="utf-8", errors="replace"), re.S)
        if not match:
            raise ValueError(f"No version of {metadata['package']} found in {source_path / version_file}")
        version = match.group(1)
    url = submodule.get("url") if submodule else None  # A relative submodule URL is an internal path: do not publish it
    if url and not url.startswith(("https://", "http://")):
        url = None
    if url and base_name and not url.startswith("https://github.com/"):  # Publish only public upstream URLs
        url = None

    def output_path(path: Path | None) -> str | None:
        if not path:
            return None
        relative = path.resolve().relative_to(base_root.resolve())
        return str(relative) if base_name is None else f"{base_name}/{relative}"

    return {
        "package": metadata["package"],
        "supplier": metadata.get("supplier"),
        "note": metadata.get("note"),
        "source_type": "vendored",
        "source_root": base_name or "repo",
        "source_path": dependency_path,
        "source_url": normalize_optional(url),
        "triplet": None,
        "version": version,
        "binary_version": normalize_optional(submodule.get("revision") if submodule else None),
        "homepage": normalize_optional(url),
        "description": metadata.get("description"),
        "license_expression": license_expression,
        "license_concluded": license_expression,
        "license_declared": None,
        "spdx_license_unresolved": license_expression is None
        or license_expression.startswith("LicenseRef-"),
        "raw_license_concluded": None,
        "raw_license_declared": None,
        "inferred_license_expression": notice_license_hint or license_expression,
        "resource_download_locations": [],
        "spdx_document_namespace": None,
        "spdx_path": None,
        "notice_path": output_path(notice_path),
        "notice_license_hint": notice_license_hint,
        "notice_text": notice_text,
        "submodule_name": submodule.get("logical_name") if submodule else None,
    }


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    manifest_root = (args.manifest_root or args.repo_root).resolve()
    triplet = args.triplet
    install_root = args.install_root.resolve()
    vcpkg_root = args.vcpkg_root.resolve()
    vcpkg_executable = vcpkg_root / "vcpkg"

    if not vcpkg_executable.is_file():
        raise FileNotFoundError(f"vcpkg executable does not exist: {vcpkg_executable}")

    if args.install:
        run_vcpkg_install(vcpkg_executable, manifest_root, install_root, triplet)

    gaps = find_third_party_gaps(repo_root)
    if gaps:
        raise SystemExit(
            "error: these paths are not in the third-party license tables of "
            "utility_scripts/generate_third_party_licenses.py:\n  "
            + "\n  ".join(gaps)
            + "\nAdd each one to VENDORED_DEPENDENCIES (license, supplier, notice) or, with a reason, "
            "to EXCLUDED_SUBMODULE_PATHS or NOT_LISTED_PATHS."
        )

    share_root = install_root / triplet / "share"
    if not share_root.is_dir():
        raise FileNotFoundError(f"Share directory does not exist: {share_root}")

    vcpkg_list_packages = run_vcpkg_list(vcpkg_executable, install_root, triplet)
    listed_vcpkg_names = {entry["package"] for entry in vcpkg_list_packages}

    package_entries: list[dict[str, Any]] = []
    for share_dir in sorted(path for path in share_root.iterdir() if path.is_dir()):
        if share_dir.name not in listed_vcpkg_names:
            continue
        spdx_path = share_dir / "vcpkg.spdx.json"
        notice_path = share_dir / "copyright"
        if not spdx_path.is_file() and not notice_path.is_file():
            continue
        package_entries.append(
            build_package_entry(repo_root, install_root, vcpkg_root, triplet, share_dir)
        )

    private_root = find_private_module_root(repo_root, args.private_module_root)
    if private_root:
        print(f"Private module {PRIVATE_MODULE_NAME}: {private_root}", file=sys.stderr)
    else:
        print(
            f"warning: private module {PRIVATE_MODULE_NAME} not found (set {PRIVATE_MODULE_ENV} or "
            "--private-module-root); the SBOMs do not list its third-party components (the license text of the "
            "module lists them)",
            file=sys.stderr,
        )

    vendored_tables: list[tuple[Path, str | None, dict[str, dict[str, Any]]]] = [
        (repo_root, None, VENDORED_DEPENDENCIES),
    ]
    if private_root:
        vendored_tables.append((private_root, PRIVATE_MODULE_NAME, PRIVATE_MODULE_VENDORED_DEPENDENCIES))
    for base_root, base_name, table in vendored_tables:
        submodules = parse_gitmodules(base_root, strict=base_name is None)
        for dependency_path, metadata in sorted(table.items()):
            if base_name is None and dependency_path in EXCLUDED_SUBMODULE_PATHS:
                continue
            if not (base_root / dependency_path).exists():
                print(
                    f"warning: {metadata['package']} not found at {base_root / dependency_path}; not listed",
                    file=sys.stderr,
                )
                continue
            package_entries.append(
                build_vendored_entry(
                    repo_root,
                    dependency_path,
                    metadata,
                    submodules.get(dependency_path),
                    base_root=base_root,
                    base_name=base_name,
                )
            )

    package_entries.sort(key=lambda entry: (entry["source_type"], entry["package"]))

    if args.skip_license_report:
        license_report = {
            "available": False,
            "reason": "skipped by flag",
            "stdout": None,
        }
    else:
        license_report = run_vcpkg_license_report(vcpkg_executable, install_root)

    vcpkg_entry_names = {
        entry["package"] for entry in package_entries if entry["source_type"] == "vcpkg"
    }
    missing_from_share = sorted(listed_vcpkg_names - vcpkg_entry_names)

    payload = {
        "schema_version": 3,
        "generator": {
            "name": "generate_third_party_licenses.py",
            "ecosystem": "vcpkg+vendored",
        },
        "profile": {
            "name": "depthai-device-rvc4",
            "triplet": triplet,
            "manifest_features": [],
            "notes": [
                "Uses the depthai-device RVC4 manifest and existing build_docker_arm64_rvc4 vcpkg install tree by default.",
                "depthai-core (the repository that contains this firmware tree) and its dependencies are intentionally excluded because depthai-core is handled by its own compliance flow.",
                "Every git submodule and every directory in external/ is either listed as a vendored dependency "
                "or excluded with a reason (find_third_party_gaps).",
                f"Third-party parts of the private {PRIVATE_MODULE_NAME} module (outside this repository) are "
                "listed from PRIVATE_MODULE_VENDORED_DEPENDENCIES when that checkout is found.",
            ],
            "private_module": {
                "name": PRIVATE_MODULE_NAME,
                "path": str(private_root) if private_root else None,
                "found": private_root is not None,
                "not_listed": PRIVATE_MODULE_NOT_LISTED_PATHS,
            },
        },
        "vcpkg": {
            "tool_root": repo_relative_path(repo_root, vcpkg_root)
            if path_is_relative_to(vcpkg_root, repo_root)
            else str(vcpkg_root),
            "install_root": repo_relative_path(repo_root, install_root)
            if path_is_relative_to(install_root, repo_root)
            else str(install_root),
            "registry_builtin_baseline": load_builtin_baseline(manifest_root),
            "license_report": license_report,
        },
        "summary": {
            "package_count": len(package_entries),
            "packages_with_spdx": sum(1 for entry in package_entries if entry["spdx_path"]),
            "packages_with_notice": sum(
                1 for entry in package_entries if entry["notice_path"]
            ),
            "packages_with_license_expression": sum(
                1 for entry in package_entries if entry["license_expression"]
            ),
            "packages_with_inferred_license_expression": sum(
                1 for entry in package_entries if entry["inferred_license_expression"]
            ),
            "packages_with_notice_license_hint": sum(
                1 for entry in package_entries if entry["notice_license_hint"]
            ),
            "packages_with_unresolved_spdx_license": sum(
                1 for entry in package_entries if entry["spdx_license_unresolved"]
            ),
            "vcpkg_package_count": sum(
                1 for entry in package_entries if entry["source_type"] == "vcpkg"
            ),
            "vendored_package_count": sum(
                1 for entry in package_entries if entry["source_type"] == "vendored"
            ),
            "vcpkg_list_count": len(vcpkg_list_packages),
            "vcpkg_list_missing_from_share_count": len(missing_from_share),
            "vcpkg_list_missing_from_share": missing_from_share,
        },
        "packages": package_entries,
    }

    args.output.resolve().write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
