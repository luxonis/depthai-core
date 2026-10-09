#!/usr/bin/env python3
"""Generate SPDX 2.3 and CycloneDX 1.6 SBOMs for depthai-core.

Modes:
  build   SBOM of one build tree: reads the CMake cache, the vcpkg install tree and the downloaded
          artifacts. Use it for release binaries (install tree, wheels).
  source  SBOM of the source tree: resolves every vcpkg manifest feature with `vcpkg install --dry-run`.
          Use it for the source archives.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import io
import json
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import uuid
from pathlib import Path
from urllib.parse import quote
from typing import Any

NOASSERTION = "NOASSERTION"
REPO_URL = "https://github.com/luxonis/depthai-core"
FIRMWARE_SBOM_PRODUCT = "depthai-device-rvc4-fwp"

PLAN_RE = re.compile(r"^\s*(\*)?\s*([a-z0-9-]+)(?:\[([^\]]*)\])?:([\w-]+)@(\S+) -- (.+)$")
GITHUB_RE = re.compile(r"^(?:git\+)?https://github\.com/([^/@]+/[^/@]+?)(?:\.git)?(?:@(.+))?$")

Cache = dict  # CMake cache: name -> value


def on(cache: Cache, name: str) -> bool:
    return cache.get(name, "").upper() in ("ON", "TRUE", "1", "YES", "Y")


def always(cache: Cache) -> bool:
    return True


# vcpkg manifest feature -> CMake configuration that enables it (cmake/depthaiVcpkgFeatures.cmake)
FEATURE_CMAKE = {
    "opencv-support": "DEPTHAI_OPENCV_SUPPORT=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "opencv-gui": "DEPTHAI_OPENCV_SUPPORT=ON, DEPTHAI_BUILD_EXAMPLES=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "pcl-support": "DEPTHAI_PCL_SUPPORT=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "curl-support": "DEPTHAI_ENABLE_CURL=ON",
    "kompute-support": "DEPTHAI_ENABLE_KOMPUTE=ON",
    "protobuf-support": "DEPTHAI_ENABLE_PROTOBUF=ON",
    "python-bindings": "DEPTHAI_BUILD_PYTHON=ON",
    "remote-connection-support": "DEPTHAI_ENABLE_REMOTE_CONNECTION=ON",
    "rtabmap": "DEPTHAI_RTABMAP_SUPPORT=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "apriltag": "DEPTHAI_ENABLE_APRIL_TAG=ON",
    "backward": "DEPTHAI_ENABLE_BACKWARD=ON",
    "basalt": "DEPTHAI_BASALT_SUPPORT=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "tests": "DEPTHAI_BUILD_TESTS=ON",
    "recording": "DEPTHAI_ENABLE_MP4V2=ON",
    "xtensor-support": "DEPTHAI_XTENSOR_SUPPORT=ON and DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "public-deps": "DEPTHAI_VCPKG_INTERNAL_ONLY=OFF",
    "usb": "DEPTHAI_ENABLE_LIBUSB=ON",
    "rerun-sdk": "DEPTHAI_BUILD_EXAMPLES=ON with PCL, Basalt or RTABMap support",
    "zoo-helper": "DEPTHAI_BUILD_ZOO_HELPER=ON",
}

# Ports whose vcpkg.json has no "license" field: read from share/<port>/copyright of a vcpkg install
LICENSE_FROM_COPYRIGHT = {
    "backward-cpp": ("MIT", None),
    "fp16": ("MIT", None),
    "psimd": ("MIT", None),
    "mp4v2": ("MPL-1.1", None),
    "pybind11": ("BSD-3-Clause", None),
    "websocketpp": ("BSD-3-Clause", "Main library; bundled parts carry their own notices"),
    "liblzma": ("0BSD", "liblzma only; the xz command line tools and scripts have other terms"),
    "libarchive": ("BSD-2-Clause", "Main license; some files are BSD-3-Clause, public domain or CC0-1.0 OR OpenSSL OR Apache-2.0"),
}

# FetchContent dependency -> (license, supplier, source-mode scope, enabled by, build-mode condition)
FETCHCONTENT = {
    "XLink": ("Apache-2.0", "Luxonis", "required", "Always, unless DEPTHAI_XLINK_LOCAL is set",
              lambda cache: not cache.get("DEPTHAI_XLINK_LOCAL")),
    "libnop": ("Apache-2.0", None, "required", "DEPTHAI_LIBNOP_EXTERNAL=OFF",
               lambda cache: not on(cache, "DEPTHAI_LIBNOP_EXTERNAL")),
    "nlohmann_json": ("MIT", None, "required", "DEPTHAI_JSON_EXTERNAL=OFF",
                      lambda cache: not on(cache, "DEPTHAI_JSON_EXTERNAL")),
    "xtl": ("BSD-3-Clause", None, "optional", "DEPTHAI_XTENSOR_SUPPORT=ON and DEPTHAI_XTENSOR_EXTERNAL=OFF",
            lambda cache: on(cache, "DEPTHAI_XTENSOR_SUPPORT") and not on(cache, "DEPTHAI_XTENSOR_EXTERNAL")),
    "xtensor": ("BSD-3-Clause", None, "optional", "DEPTHAI_XTENSOR_SUPPORT=ON and DEPTHAI_XTENSOR_EXTERNAL=OFF",
                lambda cache: on(cache, "DEPTHAI_XTENSOR_SUPPORT") and not on(cache, "DEPTHAI_XTENSOR_EXTERNAL")),
    "trompeloeil": ("BSL-1.0", None, "test", "DEPTHAI_BUILD_TESTS=ON (test only)",
                    lambda cache: on(cache, "DEPTHAI_BUILD_TESTS")),
}
FETCHCONTENT_FILES = ("cmake/depthaiDependencies.cmake", "tests/CMakeLists.txt")

# Submodule path -> (name, license, supplier, source-mode scope, enabled by, build-mode condition)
SUBMODULES = {
    "shared/depthai-bootloader-shared": ("depthai-bootloader-shared", "MIT", "Luxonis", "required", "Always", always),
    "shared/depthai_boards": ("depthai-boards", None, "Luxonis", "required", "DEPTHAI_GENERATE_HOUSING_COORDS=ON",
                              lambda cache: on(cache, "DEPTHAI_GENERATE_HOUSING_COORDS")),
    "3rdparty/foxglove/ws-protocol": ("foxglove-ws-protocol", "MIT", "Foxglove Technologies Inc", "optional",
                                      "DEPTHAI_ENABLE_REMOTE_CONNECTION=ON (builds cpp/foxglove-websocket)",
                                      lambda cache: on(cache, "DEPTHAI_ENABLE_REMOTE_CONNECTION")),
    "3rdparty/xtensor": ("xtensor", "BSD-3-Clause", None, "unused",
                         "Not referenced by CMake; the build uses the FetchContent fork", lambda cache: False),
    "3rdparty/xtl": ("xtl", "BSD-3-Clause", None, "unused",
                     "Not referenced by CMake; the build uses the FetchContent fork", lambda cache: False),
    "bindings/python/external/xtensor-python": ("xtensor-python", "BSD-3-Clause", None, "optional", "DEPTHAI_BUILD_PYTHON=ON",
                                                lambda cache: on(cache, "DEPTHAI_BUILD_PYTHON")),
    "bindings/python/external/pybind11_opencv_numpy": (
        "pybind11-opencv-numpy", "Apache-2.0", None, "optional", "DEPTHAI_BUILD_PYTHON=ON and DEPTHAI_MERGED_TARGET=ON",
        lambda cache: on(cache, "DEPTHAI_BUILD_PYTHON") and on(cache, "DEPTHAI_MERGED_TARGET"),
    ),
}

# Code copied into the tree: (path, name, (version file, regex), license, upstream, source-mode scope, enabled by, condition)
VENDORED = (
    ("include/3rdparty/mcap", "mcap", ("include/3rdparty/mcap/types.hpp", r'#define MCAP_LIBRARY_VERSION "([^"]+)"'),
     "MIT", "https://github.com/foxglove/mcap", "required", "Always", always),
    ("include/3rdparty/nanorpc", "nanorpc", None, "MIT", "https://github.com/tdv/nanorpc", "required", "Always", always),
    ("bindings/python/external/hedley", "hedley",
     ("bindings/python/external/hedley/include/hedley/hedley.h", r"#define HEDLEY_VERSION (\d+)"),
     "CC0-1.0", "https://github.com/nemequ/hedley", "optional", "DEPTHAI_BUILD_PYTHON=ON",
     lambda cache: on(cache, "DEPTHAI_BUILD_PYTHON")),
    ("bindings/python/external/pybind11_json", "pybind11_json", None, "BSD-3-Clause", "https://github.com/pybind/pybind11_json",
     "optional", "DEPTHAI_BUILD_PYTHON=ON", lambda cache: on(cache, "DEPTHAI_BUILD_PYTHON")),
)

# Luxonis prebuilt artifacts: (name, type, config file, version var, maturity var, license, license file,
#                              source-mode scope, enabled by, CMake option)
ARTIFACTS = (
    ("depthai-device-rvc2", "firmware", "DepthaiDeviceSideConfig.cmake", "DEPTHAI_DEVICE_SIDE_COMMIT",
     "DEPTHAI_DEVICE_SIDE_MATURITY", "LicenseRef-Luxonis-Firmware-Package", "notices/depthai-device-RVC2-LICENSE",
     "required", "DEPTHAI_ENABLE_DEVICE_FW=ON (default)", "DEPTHAI_ENABLE_DEVICE_FW"),
    ("depthai-bootloader", "firmware", "DepthaiBootloaderConfig.cmake", "DEPTHAI_BOOTLOADER_VERSION",
     "DEPTHAI_BOOTLOADER_MATURITY", None, None,
     "required", "DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW=ON (default)", "DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW"),
    ("depthai-device-rvc4", "firmware", "DepthaiDeviceRVC4Config.cmake", "DEPTHAI_DEVICE_RVC4_VERSION",
     "DEPTHAI_DEVICE_RVC4_MATURITY", "LicenseRef-Luxonis-Firmware-Package", "notices/depthai-device-RVC4-LICENSE",
     "required", "DEPTHAI_ENABLE_DEVICE_RVC4_FW=ON (default)", "DEPTHAI_ENABLE_DEVICE_RVC4_FW"),
    ("depthai-device-kb-rvc3", "firmware", "DepthaiDeviceKbConfig.cmake", "DEPTHAI_DEVICE_RVC3_VERSION",
     "DEPTHAI_DEVICE_KB_MATURITY", None, None,
     "optional", "DEPTHAI_ENABLE_DEVICE_RVC3_FW=ON (default OFF)", "DEPTHAI_ENABLE_DEVICE_RVC3_FW"),
    ("depthai-visualizer", "application", "DepthaiVisualizerConfig.cmake", "DEPTHAI_VISUALIZER_COMMIT",
     None, "LicenseRef-Luxonis-Visualizer", "notices/depthai-visualizer-LICENSE",
     "optional", "DEPTHAI_EMBED_FRONTEND=ON", "DEPTHAI_EMBED_FRONTEND"),
    ("dynamic_calibration", "library", "DepthaiDynamicCalibrationConfig.cmake", "DEPTHAI_DYNAMIC_CALIBRATION_VERSION",
     None, "LicenseRef-Luxonis-Dynamic-Calibration", "notices/dynamic_calibration-LICENSE",
     "optional", "DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=ON (default)", "DEPTHAI_DYNAMIC_CALIBRATION_SUPPORT"),
)

# Custom licenses: readable names. The SBOM does not repeat their full text; it is in the notices/ files that
# ship with depthai-core (linked with seeAlso at the exact commit).
CUSTOM_LICENSE_NAMES = {
    "LicenseRef-Luxonis-Firmware-Package": "Luxonis Firmware Package License",
    "LicenseRef-Luxonis-Visualizer": "Luxonis Proprietary Software License (depthai-visualizer)",
    "LicenseRef-Luxonis-Dynamic-Calibration": "Luxonis Dynamic Calibration License",
}

# How a component relates to its parent. "test" and "build-tool" are not shipped.
SCOPE_OF_RELATION = {"depends": "required", "contains": "required", "optional": "optional",
                     "test": "excluded", "build-tool": "excluded", "unused": "excluded"}


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def run(cmd: list[str], cwd: Path | None = None) -> str:
    return subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, capture_output=True, text=True).stdout


# The checkout can belong to another user (for example in a manylinux container job), and git then
# refuses to read it. The SBOM only reads the repositories of this build.
GIT = ["git", "-c", "safe.directory=*"]


def git(path: Path, *args: str) -> str | None:
    try:
        return run([*GIT, "-C", str(path), *args]).strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def read_cmake_cache(build_dir: Path) -> Cache:
    values: Cache = {}
    path = build_dir / "CMakeCache.txt"
    if not path.is_file():
        raise FileNotFoundError(f"No CMakeCache.txt in {build_dir}")
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_type, value = line.split("=", 1)
        values[key_type.split(":", 1)[0]] = value
    return values


def read_cmake_set(path: Path, var: str) -> str | None:
    match = re.search(rf'^\s*set\({var}\s+"([^"]*)"\)', path.read_text(encoding="utf-8"), re.M)
    return match.group(1) if match else None


def project_version(repo: Path) -> str:
    text = (repo / "CMakeLists.txt").read_text(encoding="utf-8")
    version = re.search(r'project\(depthai VERSION "([^"]+)"', text).group(1)
    pre_type = re.search(r'set\(DEPTHAI_PRE_RELEASE_TYPE "([^"]*)"\)', text)
    pre_number = re.search(r'set\(DEPTHAI_PRE_RELEASE_VERSION "([^"]*)"\)', text)
    if pre_type and pre_type.group(1):
        version += f"-{pre_type.group(1)}.{pre_number.group(1) if pre_number else '0'}"
    return version


def github_purl(url: str | None, ref: str | None) -> str | None:
    match = GITHUB_RE.match(url or "")
    if not match or not (ref or match.group(2)):
        return None
    return f"pkg:github/{match.group(1).lower()}@{quote(ref or match.group(2), safe='')}"


def checkout_revision(path: Path) -> tuple[str | None, str | None]:
    """(commit, tag description) of a git checkout root. An uninitialised submodule is an empty
    directory inside the parent repository, and git would report the parent's commit for it."""
    if not (path / ".git").exists():
        return None, None
    return git(path, "rev-parse", "HEAD"), git(path, "describe", "--tags")


def git_download(url: str | None, ref: str | None) -> str | None:
    """SPDX download location for a public https git URL; None for a relative or unknown URL."""
    if not url or not url.startswith("https://"):
        return None
    url = re.sub(r"^https://[^/@]+@", "https://", url)  # never write credentials
    return f"git+{url}@{ref}" if ref else f"git+{url}"


def component(ref: str, name: str, relation: str, **fields: Any) -> dict[str, Any]:
    entry: dict[str, Any] = {"ref": ref, "name": name, "type": "library", "relation": relation, "properties": {}}
    entry.update({key: value for key, value in fields.items() if value is not None})
    return entry


# ---------------------------------------------------------------------
# vcpkg
# ---------------------------------------------------------------------
def read_vcpkg_status(install_root: Path, triplet: str) -> dict[str, dict[str, Any]]:
    """Installed packages of one triplet: version, port version, features and dependencies."""
    database = install_root / "vcpkg"
    paths = [database / "status"]
    if (database / "updates").is_dir():
        paths += sorted(path for path in (database / "updates").iterdir() if path.is_file())
    records: dict[tuple[str, str, str], dict[str, str]] = {}
    for path in paths:  # later paragraphs (the update files) replace earlier ones
        if not path.is_file():
            continue
        for paragraph in re.split(r"\n\s*\n", path.read_text(encoding="utf-8", errors="replace")):
            fields: dict[str, str] = {}
            for line in paragraph.splitlines():
                key, separator, value = line.partition(":")
                if separator and not line.startswith((" ", "\t")):
                    fields[key.strip()] = value.strip()
            if "Package" in fields and "Architecture" in fields:
                records[(fields["Package"], fields.get("Feature", ""), fields["Architecture"])] = fields

    packages: dict[str, dict[str, Any]] = {}
    for (name, feature, architecture), fields in sorted(records.items()):
        if architecture != triplet or not fields.get("Status", "").endswith(" installed"):
            continue
        entry = packages.setdefault(name, {"version": None, "port_version": "0", "features": ["core"], "depends": set()})
        if feature:
            entry["features"].append(feature)
        else:
            entry["version"] = fields.get("Version")
            entry["port_version"] = fields.get("Port-Version", "0")
        for dependency in fields.get("Depends", "").split(","):
            dependency_name, _, dependency_triplet = dependency.strip().partition(":")
            dependency_name = dependency_name.split("[", 1)[0].strip()
            if dependency_name and dependency_name != name and dependency_triplet in ("", triplet):
                entry["depends"].add(dependency_name)
    return packages


def read_port_spdx(share_dir: Path) -> dict[str, Any]:
    """License, homepage and upstream download location from share/<port>/vcpkg.spdx.json."""
    path = share_dir / "vcpkg.spdx.json"
    if not path.is_file():
        return {}
    packages = json.loads(path.read_text(encoding="utf-8")).get("packages", [])
    port = next((package for package in packages if package.get("SPDXID") == "SPDXRef-port"), {})
    license_expression = None
    for key in ("licenseConcluded", "licenseDeclared"):
        value = port.get(key)
        if value and value not in (NOASSERTION, "LicenseRef-vcpkg-null"):
            license_expression = value
            break
    downloads = [
        package["downloadLocation"]
        for package in packages
        if package.get("SPDXID", "").startswith("SPDXRef-resource-")
        and package.get("downloadLocation") not in (None, "", NOASSERTION, "NONE")
        # Some ports write an unexpanded variable (for example ${VERSION}) into their SPDX file
        and "${" not in package["downloadLocation"]
    ]
    return {
        "license": license_expression,
        "homepage": port.get("homepage"),
        "description": port.get("description"),
        "port_location": port.get("downloadLocation"),
        "download": downloads[0] if downloads else None,
    }


def export_baseline_ports(vcpkg_root: Path, baseline: str, destination: Path) -> Path:
    """The ports folder at the manifest baseline. The vcpkg checkout itself can be at a newer commit
    (cmake/vcpkg.cmake checks out the newest tag), and its ports then have other dependency lists."""
    try:
        archive = subprocess.run([*GIT, "-C", str(vcpkg_root), "archive", "--format=tar", baseline, "ports"],
                                 check=True, capture_output=True).stdout
    except (OSError, subprocess.CalledProcessError):
        print(f"warning: {vcpkg_root} has no commit {baseline}; port manifests come from its current ports folder")
        return vcpkg_root / "ports"
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        if hasattr(tarfile, "data_filter"):
            tar.extractall(destination, filter="data")
        else:
            tar.extractall(destination)
    return destination / "ports"


def port_manifest(repo: Path, ports_root: Path, name: str) -> dict[str, Any]:
    for directory in (repo / "cmake" / "ports" / name, ports_root / name):  # overlay ports come first, as in vcpkg
        if (directory / "vcpkg.json").is_file():
            manifest = json.loads((directory / "vcpkg.json").read_text(encoding="utf-8"))
            manifest["_dir"] = str(directory)
            return manifest
    return {}


def host_only_dependencies(manifest: dict[str, Any], features: list[str]) -> set[str]:
    """Names listed only as host (build tool) deps; a name also listed as a normal dep is linked."""
    dependencies = list(manifest.get("dependencies", []))
    for feature in features:
        dependencies += manifest.get("features", {}).get(feature, {}).get("dependencies", [])
    host = {d["name"] for d in dependencies if isinstance(d, dict) and d.get("host")}
    linked = {d if isinstance(d, str) else d["name"] for d in dependencies if isinstance(d, str) or not d.get("host")}
    return host - linked


def manifest_dependency_names(root_manifest: dict[str, Any]) -> set[str]:
    dependencies = list(root_manifest.get("dependencies", []))
    for feature in root_manifest.get("features", {}).values():
        dependencies += feature.get("dependencies", [])
    return {d if isinstance(d, str) else d["name"] for d in dependencies if isinstance(d, str) or not d.get("host")}


def linked_packages(direct: set[str], edges: dict[str, set[str]], hosts: dict[str, set[str]]) -> set[str]:
    """Packages reachable from the direct deps without a host edge: these are linked or shipped."""
    reached: set[str] = set()
    stack = list(direct)
    while stack:
        name = stack.pop()
        if name in reached:
            continue
        reached.add(name)
        stack.extend(child for child in edges.get(name, ()) if child not in hosts.get(name, ()))
    return reached


def portfile_upstream(portfile: Path, version: str) -> dict[str, str]:
    if not portfile.is_file():
        return {}
    text = portfile.read_text(encoding="utf-8", errors="replace")

    def expand(value: str) -> str | None:
        value = value.strip('"').replace("${VERSION}", version)
        return None if "${" in value else value

    match = re.search(r"vcpkg_from_github\s*\((.*?)\)", text, re.S)
    if match:
        repo = re.search(r"\bREPO\s+(\S+)", match.group(1))
        ref = re.search(r"\bREF\s+(\S+)", match.group(1))
        if repo and expand(repo.group(1)):
            return {"url": f"https://github.com/{expand(repo.group(1))}", "ref": expand(ref.group(1)) if ref else None}
    match = re.search(r"vcpkg_from_gitlab\s*\((.*?)\)", text, re.S)
    if match:
        base = re.search(r"\bGITLAB_URL\s+(\S+)", match.group(1))
        repo = re.search(r"\bREPO\s+(\S+)", match.group(1))
        ref = re.search(r"\bREF\s+(\S+)", match.group(1))
        if base and repo and expand(base.group(1)) and expand(repo.group(1)):
            return {"url": f"{expand(base.group(1))}/{expand(repo.group(1))}", "ref": expand(ref.group(1)) if ref else None}
    match = re.search(r"vcpkg_from_git\s*\((.*?)\)", text, re.S)
    if match:
        url = re.search(r"\bURL\s+(\S+)", match.group(1))
        ref = re.search(r"\bREF\s+(\S+)", match.group(1))
        if url and expand(url.group(1)):
            return {"url": expand(url.group(1)), "ref": expand(ref.group(1)) if ref else None}
    match = re.search(r"vcpkg_download_distfile\s*\((.*?)\)", text, re.S)
    if match:
        url = re.search(r"\bURLS\s+(\S+)", match.group(1))
        if url and expand(url.group(1)):
            return {"distribution": expand(url.group(1))}
    return {}


def vcpkg_component(name: str, version: str, port_version: str, features: list[str], triplet: str,
                    relation: str, info: dict[str, Any]) -> dict[str, Any]:
    license_expression = info.get("license")
    license_note = None
    if not license_expression and name in LICENSE_FROM_COPYRIGHT:
        license_expression, note = LICENSE_FROM_COPYRIGHT[name]
        license_note = "From the port copyright file" + (f"; {note}" if note else "")
    entry = component(
        f"vcpkg-{name}", name, relation,
        version=version,
        description=info.get("description"),
        license=license_expression,
        homepage=info.get("homepage"),
        download=info.get("download"),
        purl=info.get("purl"),
    )
    entry["properties"].update({
        "depthai:source": "vcpkg",
        "vcpkg:port-version": port_version,
        "vcpkg:features": ",".join(features),
        "vcpkg:triplet": triplet,
    })
    if info.get("port_location"):
        entry["properties"]["vcpkg:port-location"] = info["port_location"]
    if license_note:
        entry["properties"]["depthai:license-note"] = license_note
    return entry


def vcpkg_from_build(repo: Path, build_dir: Path, cache: Cache, inventory: dict[str, Any]) -> None:
    install_root = Path(cache.get("VCPKG_INSTALLED_DIR") or cache.get("_VCPKG_INSTALLED_DIR") or build_dir / "vcpkg_installed")
    triplet = cache.get("VCPKG_TARGET_TRIPLET")
    if not triplet:
        raise RuntimeError("VCPKG_TARGET_TRIPLET is not in the CMake cache")
    vcpkg_root = Path(cache.get("Z_VCPKG_ROOT_DIR") or repo / "vcpkg")
    status = read_vcpkg_status(install_root, triplet)
    status = {name: entry for name, entry in status.items() if not name.startswith("vcpkg-")}
    inventory["properties"]["vcpkg:triplet"] = triplet

    with tempfile.TemporaryDirectory() as scratch:
        ports_root = export_baseline_ports(vcpkg_root, inventory["properties"]["vcpkg:baseline"], Path(scratch))
        manifests = {name: port_manifest(repo, ports_root, name) for name in status}
    edges = {name: {dep for dep in entry["depends"] if dep in status} for name, entry in status.items()}
    hosts = {name: host_only_dependencies(manifests[name], status[name]["features"]) for name in status}
    root_manifest = json.loads((repo / "vcpkg.json").read_text(encoding="utf-8"))
    direct = {name for name in manifest_dependency_names(root_manifest) if name in status}
    linked = linked_packages(direct, edges, hosts)

    for name, entry in sorted(status.items()):
        info = read_port_spdx(install_root / triplet / "share" / name)
        match = GITHUB_RE.match(info.get("download") or "")
        if match and match.group(2):
            info["purl"] = github_purl(info["download"], None)
        relation = "build-tool" if name not in linked else "depends"
        inventory["components"].append(
            vcpkg_component(name, entry["version"], entry["port_version"], entry["features"], triplet, relation, info)
        )
        child_refs = {f"vcpkg-{child}" for child in edges[name] if child in linked and child not in hosts[name]}
        if child_refs:
            inventory["edges"].setdefault(f"vcpkg-{name}", set()).update(child_refs)
        if name in direct or name not in linked:
            inventory["edges"]["root"].add(f"vcpkg-{name}")
    # A linked package that no edge reaches from a direct dep still belongs to the root
    reached = set()
    for ref in list(inventory["edges"]["root"]):
        stack = [ref]
        while stack:
            current = stack.pop()
            if current not in reached:
                reached.add(current)
                stack.extend(inventory["edges"].get(current, ()))
    for entry in inventory["components"]:
        if entry["ref"].startswith("vcpkg-") and entry["ref"] not in reached:
            inventory["edges"]["root"].add(entry["ref"])


def parse_plan(text: str) -> dict[str, dict[str, Any]]:
    plan: dict[str, dict[str, Any]] = {}
    for line in text.splitlines():
        match = PLAN_RE.match(line)
        if not match:
            continue
        _, name, features, triplet, version, source = match.groups()
        version, _, port_version = version.partition("#")
        plan[name] = {"features": [f for f in (features or "core").split(",") if f], "triplet": triplet,
                      "version": version, "port_version": port_version or "0", "source": source.strip()}
    return plan


def port_location(repo: Path, source: str) -> str:
    if source.startswith("git+"):
        return source
    try:
        return str(Path(source).resolve().relative_to(repo.resolve()).as_posix())
    except ValueError:  # an overlay outside the repository
        return source


def vcpkg_from_source(repo: Path, vcpkg_root: Path, triplet: str, inventory: dict[str, Any]) -> None:
    vcpkg = vcpkg_root / ("vcpkg.exe" if (vcpkg_root / "vcpkg.exe").is_file() else "vcpkg")
    root_manifest = json.loads((repo / "vcpkg.json").read_text(encoding="utf-8"))
    features = list(root_manifest.get("features", {}))
    inventory["properties"]["vcpkg:triplet"] = triplet

    with tempfile.TemporaryDirectory() as scratch:
        def dry_run(extra: list[str]) -> dict[str, dict[str, Any]]:
            output = run([str(vcpkg), "install", "--dry-run", f"--x-manifest-root={repo}",
                          f"--x-install-root={Path(scratch) / 'installed'}", f"--triplet={triplet}",
                          "--x-no-default-features", *extra], cwd=repo)
            return parse_plan(output)

        base = dry_run([])
        per_feature = {feature: dry_run([f"--x-feature={feature}"]) for feature in features}
        full = dry_run([f"--x-feature={feature}" for feature in features])
        full = {name: entry for name, entry in full.items() if not name.startswith("vcpkg-")}

        # Classic mode (no manifest in the working directory) on the baseline ports gives the dependency edges
        ports_root = export_baseline_ports(vcpkg_root, inventory["properties"]["vcpkg:baseline"], Path(scratch))
        specs = [f"{name}[{','.join(entry['features'])}]:{entry['triplet']}" for name, entry in sorted(full.items())]
        dot = run([str(vcpkg), "depend-info", *specs, f"--x-builtin-ports-root={ports_root}",
                   f"--overlay-ports={repo / 'cmake' / 'ports'}", f"--overlay-triplets={repo / 'cmake' / 'triplets'}",
                   "--format=dot"], cwd=Path(scratch))
        manifests = {name: port_manifest(repo, ports_root, name) for name in full}
        upstreams = {name: portfile_upstream(Path(manifests[name].get("_dir", "")) / "portfile.cmake", entry["version"])
                     for name, entry in full.items()}
    edges: dict[str, set[str]] = {name: set() for name in full}
    for parent, child in re.findall(r'"([^"]+)"\s*->\s*"([^"]+)"', dot):
        if parent in full and child in full:
            edges[parent].add(child)

    hosts = {name: host_only_dependencies(manifests[name], full[name]["features"]) for name in full}
    direct = {name for name in manifest_dependency_names(root_manifest) if name in full}
    linked = linked_packages(direct, edges, hosts)

    for name, entry in sorted(full.items()):
        manifest = manifests[name]
        pulled_by = [feature for feature in features if name in per_feature[feature] and name not in base]
        if name not in linked:
            relation = "build-tool"
        elif name in base:
            relation = "depends"
        elif pulled_by == ["tests"]:
            relation = "test"
        else:
            relation = "optional"
        description = manifest.get("description")
        upstream = upstreams[name]
        info = {
            "license": manifest.get("license"),
            "homepage": manifest.get("homepage"),
            "description": " ".join(description) if isinstance(description, list) else description,
            "download": git_download(upstream.get("url"), upstream.get("ref")) or upstream.get("distribution"),
            "purl": github_purl(upstream.get("url"), upstream.get("ref")),
            "port_location": port_location(repo, entry["source"]),
        }
        item = vcpkg_component(name, entry["version"], entry["port_version"], entry["features"], entry["triplet"], relation, info)
        if pulled_by:
            item["properties"]["depthai:vcpkg-manifest-features"] = ",".join(pulled_by)
            item["properties"]["depthai:enabled-by"] = "; ".join(sorted({FEATURE_CMAKE.get(f, f) for f in pulled_by}))
        inventory["components"].append(item)
        children = {f"vcpkg-{child}" for child in edges[name] if child in linked and child not in hosts[name]}
        if children:
            inventory["edges"].setdefault(f"vcpkg-{name}", set()).update(children)
        if name in direct or name not in linked:
            inventory["edges"]["root"].add(f"vcpkg-{name}")


# ---------------------------------------------------------------------
# Non-vcpkg components
# ---------------------------------------------------------------------
def add_fetchcontent(repo: Path, build_dir: Path | None, cache: Cache | None, inventory: dict[str, Any]) -> None:
    pattern = re.compile(r"FetchContent_Declare\(\s*(\w+)\s+GIT_REPOSITORY\s+(\S+)\s+GIT_TAG\s+(\S+)", re.S)
    for cmake_file in FETCHCONTENT_FILES:
        for name, url, tag in pattern.findall((repo / cmake_file).read_text(encoding="utf-8")):
            if name not in FETCHCONTENT:
                print(f"warning: FetchContent dependency {name} has no metadata in generate_sbom.py")
                license_expression, supplier, scope, enabled_by, condition = None, None, "required", None, always
            else:
                license_expression, supplier, scope, enabled_by, condition = FETCHCONTENT[name]
            commit = None
            if cache is not None:
                source_dir = build_dir / "_deps" / f"{name.lower()}-src"
                if not condition(cache) or not source_dir.is_dir():
                    continue
                commit, _ = checkout_revision(source_dir)
            relation = {"required": "depends", "optional": "optional", "test": "test"}[scope] if cache is None else (
                "test" if scope == "test" else "depends")
            entry = component(
                f"fetchcontent-{name}", name, relation,
                version=tag, supplier=supplier, license=license_expression,
                download=git_download(url, commit or tag), purl=github_purl(url, commit or tag),
            )
            entry["properties"].update({"depthai:source": "cmake-fetchcontent", "depthai:declared-in": cmake_file})
            if commit:
                entry["properties"]["depthai:commit"] = commit
            if enabled_by:
                entry["properties"]["depthai:enabled-by"] = enabled_by
            inventory["components"].append(entry)
            inventory["edges"]["root"].add(entry["ref"])


def add_submodules(repo: Path, cache: Cache | None, inventory: dict[str, Any]) -> None:
    config = git(repo, "config", "-f", ".gitmodules", "--get-regexp", r"submodule\..*\.(path|url)") or ""
    paths: dict[str, str] = {}
    urls: dict[str, str] = {}
    for line in config.splitlines():
        key, _, value = line.partition(" ")
        (paths if key.endswith(".path") else urls)[key.rsplit(".", 1)[0]] = value
    for key, path in sorted(paths.items(), key=lambda item: item[1]):
        if path not in SUBMODULES:
            print(f"warning: submodule {path} has no metadata in generate_sbom.py")
            name, license_expression, supplier, scope, enabled_by, condition = Path(path).name, None, None, "required", None, always
        else:
            name, license_expression, supplier, scope, enabled_by, condition = SUBMODULES[path]
        if cache is not None and not condition(cache):
            continue
        commit, described = checkout_revision(repo / path)
        relation = "unused" if scope == "unused" else ("contains" if cache is None else "depends")
        entry = component(
            f"submodule-{path}", name, relation,
            type="data" if name == "depthai-boards" else "library",
            version=described if described and re.match(r"v?\d", described) else commit,
            supplier=supplier, license=license_expression,
            download=git_download(urls.get(key), commit), purl=github_purl(urls.get(key), commit),
        )
        entry["properties"].update({"depthai:source": "git-submodule", "depthai:path": path})
        if commit:
            entry["properties"]["depthai:commit"] = commit
        if enabled_by:
            entry["properties"]["depthai:enabled-by"] = enabled_by
        inventory["components"].append(entry)
        inventory["edges"]["root"].add(entry["ref"])


def add_vendored(repo: Path, cache: Cache | None, inventory: dict[str, Any]) -> None:
    for path, name, version_source, license_expression, url, scope, enabled_by, condition in VENDORED:
        if not (repo / path).exists() or (cache is not None and not condition(cache)):
            continue
        version = None
        if version_source:
            match = re.search(version_source[1], (repo / version_source[0]).read_text(encoding="utf-8", errors="replace"))
            version = match.group(1) if match else None
        entry = component(f"vendored-{name}", name, "contains", version=version, license=license_expression, download=git_download(url, None))
        entry["properties"].update({"depthai:source": "vendored", "depthai:path": path, "depthai:enabled-by": enabled_by})
        if not version:
            entry["properties"]["depthai:note"] = "Version is not recorded in the tree"
        inventory["components"].append(entry)
        inventory["edges"]["root"].add(entry["ref"])


def add_artifacts(repo: Path, build_dir: Path | None, cache: Cache | None, inventory: dict[str, Any], output_dir: Path) -> None:
    config_dir = repo / "cmake" / "Depthai"
    for name, kind, config, version_var, maturity_var, license_expression, license_file, scope, enabled_by, option in ARTIFACTS:
        if cache is not None and not on(cache, option):
            continue
        version = read_cmake_set(config_dir / config, version_var)
        relation = "contains" if cache is not None or scope == "required" else "optional"
        entry = component(f"luxonis-{name}", name, relation, type=kind, version=version, supplier="Luxonis",
                          license=license_expression, download="https://artifacts.luxonis.com/artifactory")
        entry["properties"].update({"depthai:source": "luxonis-artifactory", "depthai:enabled-by": enabled_by})
        if maturity_var:
            entry["properties"]["depthai:maturity"] = read_cmake_set(config_dir / config, maturity_var) or ""
        if license_file:
            entry["properties"]["depthai:license-file"] = license_file
            files = inventory["license_files"].setdefault(license_expression, [])
            if license_file not in files:
                files.append(license_file)
        if name == "depthai-device-rvc4" and build_dir is not None:
            # CMakeLists.txt appends the sanitizer variant to the firmware version
            if version and on(cache, "DEPTHAI_SANITIZE"):
                version += "-tsan" if on(cache, "SANITIZE_THREAD") else "-asan-ubsan"
                entry["version"] = version
            link_firmware_sbom(build_dir, version, entry, inventory, output_dir)
        inventory["components"].append(entry)
        inventory["edges"]["root"].add(entry["ref"])


def link_firmware_sbom(build_dir: Path, version: str | None, entry: dict[str, Any], inventory: dict[str, Any], output_dir: Path) -> None:
    """Copy the firmware SBOMs (downloaded next to the fwp) to the output and reference them."""
    resources = build_dir / "resources"
    for extension, key in ((".spdx.json", "spdx"), (".cdx.json", "cyclonedx")):
        candidates = sorted(resources.glob(f"{FIRMWARE_SBOM_PRODUCT}-*{extension}")) if resources.is_dir() else []
        exact = [path for path in candidates if version and path.name == f"{FIRMWARE_SBOM_PRODUCT}-{version}{extension}"]
        if not exact:
            continue
        source = exact[0]
        data = source.read_bytes()
        try:
            document = json.loads(data.decode("utf-8"))
        except ValueError:
            print(f"warning: {source} is not valid JSON; the SBOM does not link it")
            continue
        shutil.copyfile(source, output_dir / source.name)
        inventory["external_sboms"].append({
            "format": key,
            "ref": entry["ref"],
            "file": source.name,
            "sha1": hashlib.sha1(data).hexdigest(),
            "sha256": hashlib.sha256(data).hexdigest(),
            "namespace": document.get("documentNamespace"),
            "serial": document.get("serialNumber"),
            "bom_version": document.get("version", 1),
        })
        entry["properties"][f"depthai:{key}-sbom"] = source.name
    if not any(sbom["ref"] == entry["ref"] for sbom in inventory["external_sboms"]):
        entry["properties"]["depthai:note"] = "This firmware version has no published SBOM"


def add_system_opencv(cache: Cache, inventory: dict[str, Any]) -> None:
    """OpenCV found on the build system (DEPTHAI_VCPKG_INTERNAL_ONLY=ON). It is not shipped."""
    if not on(cache, "DEPTHAI_OPENCV_SUPPORT") or any(entry["name"] == "opencv4" for entry in inventory["components"]):
        return
    opencv_dir = cache.get("OpenCV_DIR")
    if not opencv_dir or opencv_dir.endswith("-NOTFOUND"):
        return
    version = None
    for candidate in sorted(Path(opencv_dir).glob("OpenCVConfig-version.cmake")):
        match = re.search(r"set\(OpenCV_VERSION ([0-9][^)\s]*)\)", candidate.read_text(encoding="utf-8", errors="replace"))
        version = match.group(1) if match else None
    license_expression = None
    if version:  # OpenCV changed from BSD-3-Clause to Apache-2.0 in 4.5.0
        major_minor = tuple(int(part) for part in re.findall(r"\d+", version)[:2])
        license_expression = "Apache-2.0" if major_minor >= (4, 5) else "BSD-3-Clause"
    entry = component("system-opencv", "opencv", "depends", version=version, license=license_expression,
                      homepage="https://opencv.org")
    entry["properties"].update({"depthai:source": "system", "depthai:note": "Found on the build system; not shipped with depthai-core"})
    inventory["components"].append(entry)
    inventory["edges"]["root"].add(entry["ref"])


def add_python_requirements(repo: Path, inventory: dict[str, Any]) -> None:
    setup = (repo / "bindings" / "python" / "setup.py").read_text(encoding="utf-8")
    match = re.search(r"install_requires=\[(.*?)\]", setup, re.S)
    for requirement in (match.group(1).split(",") if match else []):
        requirement = requirement.strip().strip("\"'")
        parsed = re.match(r"([A-Za-z0-9_.-]+)(.*)", requirement)
        if not parsed:
            continue
        name = parsed.group(1)
        entry = component(f"pypi-{name}", name, "depends", purl=f"pkg:pypi/{name.lower()}")
        entry["properties"].update({"depthai:source": "pypi", "depthai:version-constraint": parsed.group(2).strip(),
                                    "depthai:enabled-by": "Runtime requirement of the depthai Python wheel"})
        inventory["components"].append(entry)
        inventory["edges"]["root"].add(entry["ref"])


# ---------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------
def custom_license(inventory: dict[str, Any], license_id: str) -> tuple[str, list[str]]:
    """Readable name and URLs (the notices file at the commit) of a custom LicenseRef."""
    name = CUSTOM_LICENSE_NAMES.get(license_id, license_id[len("LicenseRef-"):])
    commit = inventory["root"]["commit"]
    files = inventory["license_files"].get(license_id, [])
    urls = [f"{REPO_URL}/blob/{commit}/{path}" for path in files] if commit != NOASSERTION else []
    return name, urls


def cyclonedx_licenses(inventory: dict[str, Any], expression: str) -> list[dict[str, Any]]:
    if re.fullmatch(r"LicenseRef-[A-Za-z0-9.-]+", expression):
        name, urls = custom_license(inventory, expression)
        license_entry: dict[str, str] = {"name": name}
        if urls:
            license_entry["url"] = urls[0]
        return [{"license": license_entry}]
    return [{"expression": expression}]


def spdx_id(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9.-]+", "-", value.strip()).strip("-") or "item"
    return f"SPDXRef-{clean}"


def unique_spdx_ids(refs: list[tuple[str, str]]) -> dict[str, str]:
    """ref -> SPDX id. spdx_id() maps several characters to "-", so two refs can give the same id."""
    ids: dict[str, str] = {}
    used: set[str] = set()
    for ref, value in refs:
        candidate, counter = spdx_id(value), 1
        while candidate in used:
            counter += 1
            candidate = f"{spdx_id(value)}-{counter}"
        used.add(candidate)
        ids[ref] = candidate
    return ids


def to_spdx(inventory: dict[str, Any], generated_at: str) -> dict[str, Any]:
    root = inventory["root"]
    ids = unique_spdx_ids([("root", root["name"])] + [(entry["ref"], f"package-{entry['ref']}") for entry in inventory["components"]])
    by_ref = {entry["ref"]: entry for entry in inventory["components"]}

    packages = [{
        "name": root["name"],
        "SPDXID": ids["root"],
        "versionInfo": root["version"],
        "supplier": "Organization: Luxonis",
        "downloadLocation": root["download"],
        "filesAnalyzed": False,
        "licenseConcluded": "MIT",
        "licenseDeclared": "MIT",
        "copyrightText": "Copyright (c) 2020 Luxonis LLC",
        "externalRefs": [{"referenceCategory": "PACKAGE-MANAGER", "referenceType": "purl", "referenceLocator": root["purl"]}],
        "comment": "; ".join(f"{key}={value}" for key, value in sorted(inventory["properties"].items())),
    }]
    used_refs: set[str] = set()
    for entry in inventory["components"]:
        license_expression = entry.get("license") or NOASSERTION
        used_refs.update(re.findall(r"LicenseRef-[A-Za-z0-9.-]+", license_expression))
        package: dict[str, Any] = {
            "name": entry["name"],
            "SPDXID": ids[entry["ref"]],
            "downloadLocation": entry.get("download") or NOASSERTION,
            "filesAnalyzed": False,
            "licenseConcluded": NOASSERTION,
            "licenseDeclared": license_expression,
            "copyrightText": NOASSERTION,
            "primaryPackagePurpose": {"firmware": "FIRMWARE", "application": "APPLICATION", "data": "OTHER"}.get(entry["type"], "LIBRARY"),
        }
        for key, spdx_key in (("version", "versionInfo"), ("homepage", "homepage"), ("description", "summary")):
            if entry.get(key):
                package[spdx_key] = entry[key]
        if entry.get("supplier"):
            package["supplier"] = f"Organization: {entry['supplier']}"
        if entry.get("purl"):
            package["externalRefs"] = [{"referenceCategory": "PACKAGE-MANAGER", "referenceType": "purl", "referenceLocator": entry["purl"]}]
        properties = dict(entry["properties"], **{"depthai:relation": entry["relation"]})
        package["comment"] = "; ".join(f"{key}={value}" for key, value in sorted(properties.items()))
        packages.append(package)

    relationships = [{"spdxElementId": "SPDXRef-DOCUMENT", "relationshipType": "DESCRIBES", "relatedSpdxElement": ids["root"]}]
    for parent, children in sorted(inventory["edges"].items()):
        for child in sorted(children):
            relation = by_ref[child]["relation"] if parent == "root" else "depends"
            if relation in ("depends", "contains"):
                relationships.append({"spdxElementId": ids[parent], "relationshipType": "DEPENDS_ON" if relation == "depends" else "CONTAINS",
                                      "relatedSpdxElement": ids[child]})
            elif relation == "unused":
                relationships.append({"spdxElementId": ids[parent], "relationshipType": "CONTAINS", "relatedSpdxElement": ids[child],
                                      "comment": "In the source tree; not used by the build"})
            else:
                kind = {"optional": "OPTIONAL_DEPENDENCY_OF", "test": "TEST_DEPENDENCY_OF", "build-tool": "BUILD_TOOL_OF"}[relation]
                relationships.append({"spdxElementId": ids[child], "relationshipType": kind, "relatedSpdxElement": ids[parent]})

    external_refs = []
    for sbom in (item for item in inventory["external_sboms"] if item["format"] == "spdx"):
        document_ref = f"DocumentRef-{spdx_id(sbom['file'][: -len('.spdx.json')])[len('SPDXRef-'):]}"
        external_refs.append({"externalDocumentId": document_ref, "spdxDocument": sbom["namespace"],
                              "checksum": {"algorithm": "SHA1", "checksumValue": sbom["sha1"]}})
        relationships.append({"spdxElementId": ids[sbom["ref"]], "relationshipType": "DESCRIBED_BY",
                              "relatedSpdxElement": f"{document_ref}:SPDXRef-DOCUMENT", "comment": sbom["file"]})

    document: dict[str, Any] = {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{root['name']}-{root['version']}",
        "documentNamespace": f"https://luxonis.com/spdx/{root['name']}/{root['version']}/{uuid.uuid4()}",
        "creationInfo": {"created": generated_at, "creators": ["Organization: Luxonis", "Tool: generate_sbom.py"]},
        "documentDescribes": [ids["root"]],
        "packages": packages,
        "relationships": relationships,
    }
    if external_refs:
        document["externalDocumentRefs"] = external_refs
    if used_refs:
        infos = []
        for license_id in sorted(used_refs):
            name, urls = custom_license(inventory, license_id)
            files = inventory["license_files"].get(license_id, [])
            where = " and ".join(files) if files else "the notices of depthai-core"
            info: dict[str, Any] = {"licenseId": license_id, "name": name,
                                    "extractedText": f"{name}. The full text is in {where}, which "
                                                     f"{'ship' if len(files) > 1 else 'ships'} with depthai-core."}
            if urls:
                info["seeAlso"] = urls
            infos.append(info)
        document["hasExtractedLicensingInfos"] = infos
    return document


def to_cyclonedx(inventory: dict[str, Any], generated_at: str) -> dict[str, Any]:
    root = inventory["root"]
    external = {sbom["ref"]: sbom for sbom in inventory["external_sboms"] if sbom["format"] == "cyclonedx" and sbom.get("serial")}

    def properties(values: dict[str, str]) -> list[dict[str, str]]:
        return [{"name": key, "value": value} for key, value in sorted(values.items())]

    components = []
    for entry in inventory["components"]:
        item: dict[str, Any] = {"type": entry["type"], "bom-ref": entry["ref"], "name": entry["name"]}
        if entry.get("supplier"):
            item["supplier"] = {"name": entry["supplier"]}
        for key in ("version", "description", "purl"):
            if entry.get(key):
                item[key] = entry[key]
        if entry.get("license"):
            item["licenses"] = cyclonedx_licenses(inventory, entry["license"])
        references = []
        if entry.get("homepage"):
            references.append({"type": "website", "url": entry["homepage"]})
        download = entry.get("download")
        if download and download.startswith("git+"):
            url, _, ref = download[len("git+"):].rpartition("@")
            if not url or "/" in ref:
                url, ref = download[len("git+"):], ""
            references.append({"type": "vcs", "url": url, **({"comment": f"ref {ref}"} if ref else {})})
        elif download:
            references.append({"type": "distribution", "url": download})
        if entry["ref"] in external:
            sbom = external[entry["ref"]]
            references.append({"type": "bom", "url": f"urn:cdx:{sbom['serial'][len('urn:uuid:'):]}/{sbom['bom_version']}",
                               "comment": sbom["file"], "hashes": [{"alg": "SHA-256", "content": sbom["sha256"]}]})
        if references:
            item["externalReferences"] = references
        item["scope"] = SCOPE_OF_RELATION[entry["relation"]]
        item_properties = dict(entry["properties"], **{"depthai:relation": entry["relation"]})
        item["properties"] = properties(item_properties)
        components.append(item)

    refs = ["root"] + [entry["ref"] for entry in inventory["components"]]
    return {
        "$schema": "http://cyclonedx.org/schema/bom-1.6.schema.json",
        "bomFormat": "CycloneDX",
        "specVersion": "1.6",
        "serialNumber": f"urn:uuid:{uuid.uuid4()}",
        "version": 1,
        "metadata": {
            "timestamp": generated_at,
            "tools": {"components": [{"type": "application", "name": "generate_sbom.py", "supplier": {"name": "Luxonis"}}]},
            "component": {
                "type": "library", "bom-ref": "root", "supplier": {"name": "Luxonis"}, "name": root["name"],
                "version": root["version"], "licenses": [{"expression": "MIT"}], "purl": root["purl"],
                "externalReferences": [{"type": "vcs", "url": REPO_URL, "comment": f"commit {root['commit']}"}],
            },
            "properties": properties(inventory["properties"]),
        },
        "components": components,
        "dependencies": [{"ref": ref, "dependsOn": sorted(inventory["edges"].get(ref, ()))} for ref in refs],
    }


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("build", "source"))
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--build-dir", type=Path,
                        help="build mode: the configured and built CMake build directory; source mode: where to find vcpkg")
    parser.add_argument("--vcpkg-root", type=Path,
                        help="source mode: vcpkg checkout (default: Z_VCPKG_ROOT_DIR of --build-dir, else <repo>/vcpkg)")
    parser.add_argument("--triplet", default="x64-linux", help="source mode: vcpkg triplet to resolve")
    parser.add_argument("--wheel", action="store_true", help="describe the depthai Python wheel instead of the C++ library")
    parser.add_argument("--version", help="version of the described artifact (default: the CMake project version)")
    parser.add_argument("--name", help="base name of the output files (default: depthai-core, or depthai with --wheel)")
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    commit = git(repo, "rev-parse", "HEAD")
    if commit is None:
        print("warning: the git commit of the repository is unknown")
    product = "depthai" if args.wheel else "depthai-core"
    version = args.version or project_version(repo)
    if args.wheel:
        root_purl = f"pkg:pypi/depthai@{quote(version, safe='')}"
    elif commit:
        root_purl = f"pkg:github/luxonis/depthai-core@{commit}"
    else:
        root_purl = "pkg:github/luxonis/depthai-core"
    vcpkg_config = json.loads((repo / "vcpkg-configuration.json").read_text(encoding="utf-8"))
    inventory: dict[str, Any] = {
        "root": {
            "name": product,
            "version": version,
            "commit": commit or NOASSERTION,
            "download": f"git+{REPO_URL}@{commit}" if commit else NOASSERTION,
            "purl": root_purl,
        },
        "properties": {"depthai:git-commit": commit or NOASSERTION, "depthai:sbom-mode": args.mode,
                       "vcpkg:baseline": vcpkg_config["default-registry"]["baseline"]},
        "components": [],
        "edges": {"root": set()},
        "external_sboms": [],
        "license_files": {},
    }

    if args.mode == "build":
        if not args.build_dir:
            raise SystemExit("build mode needs --build-dir")
        build_dir = args.build_dir.resolve()
        cache = read_cmake_cache(build_dir)
        vcpkg_from_build(repo, build_dir, cache, inventory)
        add_fetchcontent(repo, build_dir, cache, inventory)
        add_submodules(repo, cache, inventory)
        add_vendored(repo, cache, inventory)
        add_artifacts(repo, build_dir, cache, inventory, output_dir)
        add_system_opencv(cache, inventory)
        if args.wheel:
            add_python_requirements(repo, inventory)
    else:
        vcpkg_root = args.vcpkg_root
        if vcpkg_root is None and args.build_dir and read_cmake_cache(args.build_dir.resolve()).get("Z_VCPKG_ROOT_DIR"):
            vcpkg_root = Path(read_cmake_cache(args.build_dir.resolve())["Z_VCPKG_ROOT_DIR"])
        vcpkg_root = (vcpkg_root or repo / "vcpkg").resolve()
        vcpkg_from_source(repo, vcpkg_root, args.triplet, inventory)
        add_fetchcontent(repo, None, None, inventory)
        add_submodules(repo, None, inventory)
        add_vendored(repo, None, inventory)
        add_artifacts(repo, None, None, inventory, output_dir)
        add_python_requirements(repo, inventory)
        inventory["properties"]["depthai:scope-note"] = (
            "Every vcpkg manifest feature is resolved for this triplet. 'required' is in every build, 'optional' needs "
            "a CMake option, 'excluded' is a build tool, a test dependency or unused. Other triplets can resolve a "
            "different set of indirect dependencies.")

    generated_at = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    name = args.name or product
    (output_dir / f"{name}.spdx.json").write_text(json.dumps(to_spdx(inventory, generated_at), indent=2) + "\n", encoding="utf-8")
    (output_dir / f"{name}.cdx.json").write_text(json.dumps(to_cyclonedx(inventory, generated_at), indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {name}.spdx.json and {name}.cdx.json to {output_dir}: {len(inventory['components'])} components")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as error:
        # run() captures the output, and vcpkg writes its errors to stdout
        print(f"{error.stdout or ''}{error.stderr or ''}", file=sys.stderr)
        raise
