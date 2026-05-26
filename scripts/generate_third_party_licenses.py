#!/usr/bin/env python3

from __future__ import annotations

import argparse
import configparser
import json
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any


NOASSERTION = "NOASSERTION"
DEFAULT_VCPKG_TOOL_REF = "2026.04.27"
DEFAULT_VCPKG_REMOTE = "https://github.com/microsoft/vcpkg.git"
NOTICE_FILE_NAMES = (
    "LICENSE",
    "LICENSE.txt",
    "LICENSE.md",
    "COPYING",
    "COPYING.txt",
    "NOTICE",
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
FETCHCONTENT_INCLUDE = {"xlink"}
_PYTHON_WHEEL_LINUX_NO_GUI_FEATURES = [
    "apriltag",
    "public-deps",
    "rtabmap",
    "basalt",
    "opencv-support",
    "pcl-support",
    "xtensor-support",
    "recording",
    "backward",
    "protobuf-support",
    "curl-support",
    "python-bindings",
    "remote-connection-support",
    "usb",
]
PROFILE_CONFIGS = {
    "python-wheel-linux": {
        "triplet": "x64-linux",
        "features": list(_PYTHON_WHEEL_LINUX_NO_GUI_FEATURES),
        "notes": [
            "Matches the Linux Python wheel build profile in .github/workflows/python-main.yml.",
            "Kompute is intentionally excluded because bindings/python/setup.py currently passes DEPTHAI_KOMPUTE_SUPPORT, but CMake only recognizes DEPTHAI_ENABLE_KOMPUTE.",
            "OpenCV GUI support is excluded because wheel builds do not set DEPTHAI_BUILD_EXAMPLES.",
        ],
    },
    "python-wheel-linux-no-gui": {
        "triplet": "x64-linux",
        "features": list(_PYTHON_WHEEL_LINUX_NO_GUI_FEATURES),
        "notes": [
            "Fresh Linux Python wheel profile without OpenCV GUI extras.",
            "Uses the same manifest features as the intended wheel build, but writes into an isolated install root to avoid contamination from old GUI-enabled trees.",
            "Kompute is intentionally excluded because bindings/python/setup.py currently passes DEPTHAI_KOMPUTE_SUPPORT, but CMake only recognizes DEPTHAI_ENABLE_KOMPUTE.",
        ],
    },
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    cache_root = repo_root / ".cache" / "third_party_licenses"
    parser = argparse.ArgumentParser(
        description=(
            "Generate third_party_licenses.json for a pinned vcpkg profile by "
            "installing or reconciling the manifest dependencies and extracting "
            "license data from the installed tree."
        )
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=repo_root,
        help="Path to the depthai-core repository root.",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_CONFIGS),
        default="python-wheel-linux-no-gui",
        help="Dependency profile to resolve before collecting licenses.",
    )
    parser.add_argument(
        "--triplet",
        default=None,
        help="Override the vcpkg target triplet for the selected profile.",
    )
    parser.add_argument(
        "--vcpkg-tool-ref",
        default=DEFAULT_VCPKG_TOOL_REF,
        help=(
            "Pinned vcpkg git ref to bootstrap. Defaults to a release that "
            "supports 'vcpkg license-report'."
        ),
    )
    parser.add_argument(
        "--vcpkg-root",
        type=Path,
        default=None,
        help="Path to the vcpkg checkout to bootstrap and use.",
    )
    parser.add_argument(
        "--vcpkg-remote",
        default=DEFAULT_VCPKG_REMOTE,
        help="Git remote used when bootstrapping vcpkg.",
    )
    parser.add_argument(
        "--install-root",
        type=Path,
        default=None,
        help="Path to the vcpkg installed tree root.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root / "third_party_licenses.json",
        help="Path to the generated JSON output.",
    )
    parser.add_argument(
        "--skip-bootstrap",
        action="store_true",
        help="Assume the vcpkg checkout already exists and skip bootstrap.",
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="Skip 'vcpkg install' and only read the existing install root.",
    )
    parser.add_argument(
        "--skip-license-report",
        action="store_true",
        help="Skip invoking 'vcpkg license-report'.",
    )
    parser.add_argument(
        "--no-submodules",
        action="store_true",
        help="Exclude repo submodules from the generated JSON.",
    )
    parser.add_argument(
        "--no-fetchcontent",
        action="store_true",
        help="Exclude selected FetchContent dependencies from the generated JSON.",
    )
    parser.add_argument(
        "--cache-root",
        type=Path,
        default=cache_root,
        help="Base directory used for cached vcpkg tool checkouts.",
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


def load_builtin_baseline(repo_root: Path) -> str:
    manifest = load_json(repo_root / "vcpkg.json")
    baseline = manifest.get("builtin-baseline")
    if not baseline:
        raise RuntimeError("vcpkg.json does not define 'builtin-baseline'")
    return baseline


def default_vcpkg_root(cache_root: Path, tool_ref: str) -> Path:
    return cache_root / "vcpkg-tool" / tool_ref


def default_install_root(repo_root: Path, profile: str) -> Path:
    return repo_root / ".cache" / "third_party_licenses" / profile / "vcpkg_installed"


def ensure_git_available() -> None:
    if shutil.which("git") is None:
        raise RuntimeError("git is required to bootstrap vcpkg")


def ensure_vcpkg_checkout(vcpkg_root: Path, remote: str, tool_ref: str) -> None:
    ensure_git_available()
    if not vcpkg_root.exists():
        vcpkg_root.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", remote, str(vcpkg_root)])
    if not (vcpkg_root / ".git").is_dir():
        raise RuntimeError(f"{vcpkg_root} exists but is not a git checkout")

    run(["git", "fetch", "--tags", "origin"], cwd=vcpkg_root)
    run(["git", "checkout", "--force", tool_ref], cwd=vcpkg_root)


def bootstrap_vcpkg(vcpkg_root: Path) -> Path:
    executable = vcpkg_root / "vcpkg"
    if executable.is_file():
        return executable
    run(["./bootstrap-vcpkg.sh", "-disableMetrics"], cwd=vcpkg_root)
    if not executable.is_file():
        raise RuntimeError(f"Bootstrapped vcpkg executable was not created at {executable}")
    return executable


def ensure_vcpkg_command(vcpkg_root: Path, skip_bootstrap: bool) -> Path:
    executable = vcpkg_root / "vcpkg"
    if skip_bootstrap:
        if not executable.is_file():
            raise RuntimeError(f"--skip-bootstrap was set but no vcpkg executable exists at {executable}")
        return executable
    return bootstrap_vcpkg(vcpkg_root)


def run_vcpkg_install(
    vcpkg_executable: Path,
    repo_root: Path,
    install_root: Path,
    triplet: str,
    features: list[str],
) -> None:
    install_root.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(vcpkg_executable),
        "install",
        f"--x-manifest-root={repo_root}",
        f"--x-install-root={install_root}",
        f"--triplet={triplet}",
    ]
    for feature in features:
        cmd.append(f"--x-feature={feature}")
    run(cmd, cwd=repo_root)


def run_vcpkg_list(
    vcpkg_executable: Path,
    install_root: Path,
    triplet: str,
) -> list[dict[str, str]]:
    cmd = [
        str(vcpkg_executable),
        "list",
        f"--x-install-root={install_root}",
        f"--triplet={triplet}",
    ]
    result = run(cmd, capture_output=True)
    packages: list[dict[str, str]] = []
    pattern = re.compile(r"^([^\s:]+):([^\s]+)\s+([^\s]+)")
    for line in result.stdout.splitlines():
        match = pattern.match(line.strip())
        if not match:
            continue
        packages.append(
            {
                "package": match.group(1),
                "triplet": match.group(2),
                "version": match.group(3),
            }
        )
    return packages


def run_vcpkg_license_report(
    vcpkg_executable: Path,
    install_root: Path,
) -> dict[str, Any]:
    cmd = [
        str(vcpkg_executable),
        "license-report",
        f"--x-install-root={install_root}",
    ]
    result = run(cmd, capture_output=True)
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


def read_notice_text(path: Path) -> str | None:
    if not path.is_file():
        return None
    return path.read_text(encoding="utf-8", errors="replace")


def detect_license_from_notice(notice_text: str | None) -> str | None:
    if not notice_text:
        return None

    lowered = notice_text[:4000].lower()
    if (
        "redistribution and use in source and binary forms" in lowered
        and "contributors may be used to endorse or promote products derived from" in lowered
    ):
        return "BSD-3-Clause"

    for license_id, markers in NOTICE_LICENSE_PATTERNS:
        if all(marker in lowered for marker in markers):
            return license_id
    return None


def read_git_submodule_status(repo_root: Path) -> dict[str, str]:
    result = run(
        ["git", "-C", str(repo_root), "submodule", "status", "--recursive"],
        capture_output=True,
    )
    revisions: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
        revision = line[1:41].strip()
        remainder = line[41:].strip()
        path = remainder.split(" ", 1)[0]
        revisions[path] = revision
    return revisions


def parse_gitmodules(repo_root: Path) -> list[dict[str, str]]:
    gitmodules_path = repo_root / ".gitmodules"
    if not gitmodules_path.is_file():
        return []

    parser = configparser.ConfigParser()
    parser.read(gitmodules_path, encoding="utf-8")

    revisions = read_git_submodule_status(repo_root)
    submodules: list[dict[str, str]] = []
    for section in parser.sections():
        if not section.startswith('submodule "'):
            continue
        logical_name = section[len('submodule "') : -1]
        path = parser.get(section, "path", fallback=None)
        url = parser.get(section, "url", fallback=None)
        if not path:
            continue
        submodule_path = repo_root / path
        if not submodule_path.exists():
            continue
        submodules.append(
            {
                "logical_name": logical_name,
                "path": path,
                "url": url or "",
                "revision": revisions.get(path, ""),
            }
        )
    return submodules


def find_notice_file(base_dir: Path) -> Path | None:
    for relative_name in NOTICE_FILE_NAMES:
        candidate = base_dir / relative_name
        if candidate.is_file():
            return candidate

    for depth in (1, 2, 3):
        pattern = "*/" * depth
        for relative_name in NOTICE_FILE_NAMES:
            matches = sorted(base_dir.glob(f"{pattern}{relative_name}"))
            if matches:
                return matches[0]
    return None


def parse_fetchcontent_declarations(cmake_path: Path) -> list[dict[str, str]]:
    text = cmake_path.read_text(encoding="utf-8")
    pattern = re.compile(
        r"FetchContent_Declare\(\s*([A-Za-z0-9_\-]+)\s*(.*?)\n\s*\)",
        re.DOTALL,
    )

    declarations: list[dict[str, str]] = []
    for match in pattern.finditer(text):
        name = match.group(1)
        body = match.group(2)
        repo_match = re.search(r"GIT_REPOSITORY\s+([^\s)]+)", body)
        tag_match = re.search(r"GIT_TAG\s+([^\s)]+)", body)
        url_match = re.search(r'URL\s+"([^"]+)"', body)
        declarations.append(
            {
                "name": name,
                "git_repository": repo_match.group(1) if repo_match else "",
                "git_tag": tag_match.group(1) if tag_match else "",
                "url": url_match.group(1) if url_match else "",
            }
        )
    return declarations


def find_fetchcontent_source_dir(repo_root: Path, declaration_name: str) -> Path | None:
    normalized_name = slugify_package_name(declaration_name)
    candidates = [
        f"{declaration_name}-src",
        f"{declaration_name.lower()}-src",
        f"{normalized_name}-src",
        declaration_name,
        declaration_name.lower(),
        normalized_name,
    ]

    build_dirs = sorted(
        path
        for path in repo_root.iterdir()
        if path.is_dir() and (path.name == "build" or path.name.startswith("build-"))
    )
    for build_dir in build_dirs:
        deps_dir = build_dir / "_deps"
        if not deps_dir.is_dir():
            continue
        for candidate in candidates:
            source_dir = deps_dir / candidate
            if source_dir.is_dir():
                return source_dir
    return None


def build_package_entry(install_root: Path, triplet: str, share_dir: Path) -> dict[str, Any]:
    spdx_path = share_dir / "vcpkg.spdx.json"
    notice_path = share_dir / "copyright"

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
    inferred_license_expression = notice_license_hint or license_concluded or license_declared
    spdx_license_unresolved = any(
        value == "LicenseRef-vcpkg-null"
        for value in (raw_license_concluded, raw_license_declared)
    )

    return {
        "package": share_dir.name,
        "source_type": "vcpkg",
        "source_path": f"{triplet}/share/{share_dir.name}",
        "source_url": normalize_optional(port_package.get("downloadLocation")),
        "triplet": triplet,
        "version": normalize_optional(port_package.get("versionInfo")),
        "binary_version": normalize_optional(binary_package.get("versionInfo")),
        "homepage": normalize_optional(port_package.get("homepage")),
        "description": normalize_optional(port_package.get("description")),
        "license_expression": license_concluded or license_declared,
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
        "notice_path": install_relative_path(install_root, notice_path) if notice_path.is_file() else None,
        "notice_license_hint": notice_license_hint,
        "notice_text": notice_text,
        "submodule_name": None,
    }


def build_submodule_entry(repo_root: Path, submodule: dict[str, str]) -> dict[str, Any]:
    submodule_path = repo_root / submodule["path"]
    notice_path = find_notice_file(submodule_path)
    notice_text = read_notice_text(notice_path) if notice_path else None
    notice_license_hint = detect_license_from_notice(notice_text)

    return {
        "package": slugify_package_name(Path(submodule["path"]).name),
        "source_type": "submodule",
        "source_path": submodule["path"],
        "source_url": normalize_optional(submodule["url"]),
        "triplet": None,
        "version": None,
        "binary_version": normalize_optional(submodule["revision"]),
        "homepage": normalize_optional(submodule["url"]),
        "description": None,
        "license_expression": None,
        "license_concluded": None,
        "license_declared": None,
        "spdx_license_unresolved": False,
        "raw_license_concluded": None,
        "raw_license_declared": None,
        "inferred_license_expression": notice_license_hint,
        "resource_download_locations": [],
        "spdx_document_namespace": None,
        "spdx_path": None,
        "notice_path": repo_relative_path(repo_root, notice_path) if notice_path else None,
        "notice_license_hint": notice_license_hint,
        "notice_text": notice_text,
        "submodule_name": submodule["logical_name"],
    }


def build_fetchcontent_entry(
    declaration: dict[str, str], repo_root: Path, cmake_path: Path
) -> dict[str, Any]:
    name = slugify_package_name(declaration["name"])
    source_url = normalize_optional(declaration["git_repository"]) or normalize_optional(
        declaration["url"]
    )
    version = normalize_optional(declaration["git_tag"])
    source_dir = find_fetchcontent_source_dir(repo_root, declaration["name"])
    notice_path = find_notice_file(source_dir) if source_dir else None
    notice_text = read_notice_text(notice_path) if notice_path else None
    notice_license_hint = detect_license_from_notice(notice_text)

    return {
        "package": name,
        "source_type": "fetchcontent",
        "source_path": (
            repo_relative_path(repo_root, source_dir)
            if source_dir and source_dir.is_relative_to(repo_root)
            else repo_relative_path(repo_root, cmake_path)
        ),
        "source_url": source_url,
        "triplet": None,
        "version": version,
        "binary_version": None,
        "homepage": source_url,
        "description": None,
        "license_expression": None,
        "license_concluded": None,
        "license_declared": None,
        "spdx_license_unresolved": False,
        "raw_license_concluded": None,
        "raw_license_declared": None,
        "inferred_license_expression": notice_license_hint,
        "resource_download_locations": [],
        "spdx_document_namespace": None,
        "spdx_path": None,
        "notice_path": repo_relative_path(repo_root, notice_path) if notice_path else None,
        "notice_license_hint": notice_license_hint,
        "notice_text": notice_text,
        "submodule_name": None,
    }


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    profile = PROFILE_CONFIGS[args.profile]
    triplet = args.triplet or profile["triplet"]
    builtin_baseline = load_builtin_baseline(repo_root)
    vcpkg_root = (
        args.vcpkg_root.resolve()
        if args.vcpkg_root
        else default_vcpkg_root(args.cache_root.resolve(), args.vcpkg_tool_ref).resolve()
    )
    install_root = (
        args.install_root.resolve()
        if args.install_root
        else default_install_root(repo_root, args.profile).resolve()
    )

    if not args.skip_bootstrap:
        ensure_vcpkg_checkout(vcpkg_root, args.vcpkg_remote, args.vcpkg_tool_ref)
    vcpkg_executable = ensure_vcpkg_command(vcpkg_root, args.skip_bootstrap)

    if not args.skip_install:
        run_vcpkg_install(
            vcpkg_executable=vcpkg_executable,
            repo_root=repo_root,
            install_root=install_root,
            triplet=triplet,
            features=profile["features"],
        )

    share_root = install_root / triplet / "share"
    if not share_root.is_dir():
        raise FileNotFoundError(
            f"Share directory does not exist after install: {share_root}"
        )

    vcpkg_list_packages = run_vcpkg_list(vcpkg_executable, install_root, triplet)
    if not vcpkg_list_packages:
        raise RuntimeError(f"No packages were listed for triplet {triplet} in {install_root}")

    package_entries: list[dict[str, Any]] = []
    for share_dir in sorted(path for path in share_root.iterdir() if path.is_dir()):
        spdx_path = share_dir / "vcpkg.spdx.json"
        notice_path = share_dir / "copyright"
        if not spdx_path.is_file() and not notice_path.is_file():
            continue
        package_entries.append(build_package_entry(install_root, triplet, share_dir))

    if not args.no_submodules:
        for submodule in parse_gitmodules(repo_root):
            package_entries.append(build_submodule_entry(repo_root, submodule))

    if not args.no_fetchcontent:
        cmake_path = repo_root / "cmake" / "depthaiDependencies.cmake"
        for declaration in parse_fetchcontent_declarations(cmake_path):
            if slugify_package_name(declaration["name"]) in FETCHCONTENT_INCLUDE:
                package_entries.append(
                    build_fetchcontent_entry(declaration, repo_root, cmake_path)
                )

    package_entries.sort(key=lambda entry: (entry["source_type"], entry["package"]))

    license_report: dict[str, Any]
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
    listed_vcpkg_names = {entry["package"] for entry in vcpkg_list_packages}
    missing_from_share = sorted(listed_vcpkg_names - vcpkg_entry_names)

    payload = {
        "schema_version": 3,
        "generator": {
            "name": "generate_third_party_licenses.py",
            "ecosystem": "vcpkg",
        },
        "profile": {
            "name": args.profile,
            "triplet": triplet,
            "manifest_features": profile["features"],
            "notes": profile["notes"],
        },
        "vcpkg": {
            "tool_ref": args.vcpkg_tool_ref,
            "registry_builtin_baseline": builtin_baseline,
            "tool_root": repo_relative_path(repo_root, vcpkg_root)
            if vcpkg_root.is_relative_to(repo_root)
            else str(vcpkg_root),
            "install_root": repo_relative_path(repo_root, install_root)
            if install_root.is_relative_to(repo_root)
            else str(install_root),
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
            "submodule_package_count": sum(
                1 for entry in package_entries if entry["source_type"] == "submodule"
            ),
            "fetchcontent_package_count": sum(
                1 for entry in package_entries if entry["source_type"] == "fetchcontent"
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
