#!/usr/bin/env python3
"""SPDX 2.3 and CycloneDX 1.6 SBOMs for the depthai-device firmware package (fwp).

The inventory comes from:
  - the license collector payload (vcpkg packages and vendored submodules)
  - the vcpkg status database (versions, features and dependency edges)
  - the fwp archive (packaged files, and the shared libraries that its ELF files need from the OS)
  - the tables below, for code that the collector does not list
"""

from __future__ import annotations

import hashlib
import json
import re
import struct
import subprocess
import tarfile
import uuid
from pathlib import Path
from typing import Any, Callable
from urllib.parse import quote

NOASSERTION = "NOASSERTION"
FWP_LICENSE_REF = "LicenseRef-Luxonis-Firmware-Package"
# depthai-core and the code that it compiles in are not in this SBOM: the firmware is distributed with
# depthai-core, which has its own SBOM
CORE_NOTE = "depthai-core is linked into the firmware and is described by its own SBOM"
# Custom licenses: (readable name, public URL). The SBOM does not repeat their full text; it is in the
# firmware package LICENSE bundle that is published next to the SBOM (its preamble is LICENSE.fwp in the
# repository; the source code itself is under the MIT license of depthai-core).
CUSTOM_LICENSES = {
    FWP_LICENSE_REF: ("Luxonis Firmware Package License", None),
    "LicenseRef-ST-SLA0106": (
        "STMicroelectronics Production Limited License Agreement SLA0106",
        "https://www.st.com/resource/en/license_agreement/dm01158080.pdf",
    ),
}

# The licensee-only Qualcomm/ST code lives in the private module depthai-device-rvc4-vendor (the vendor module)
# (CMake: DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR, or its prebuilt vendor archive), outside this repository. The collector reports where it found that checkout
# in payload["profile"]["private_module"]; when it is absent, the entries that need it are skipped.
PRIVATE_MODULE_NAME = "depthai-device-rvc4-vendor"
REPO_ROOT_NAME = "repo"

# Code compiled into the firmware that the license collector does not list (no open-source notice):
# (root, path, name, license, supplier, note); root is REPO_ROOT_NAME or PRIVATE_MODULE_NAME
UNLISTED_SOURCES = (
    (
        PRIVATE_MODULE_NAME,
        "depthmapwrapper-vd55h1",
        "depthmapwrapper-vd55h1",
        NOASSERTION,
        "STMicroelectronics",
        "Received from ST with the VD55H1 IPP library (ST SLA0106); contains code marked 'Copyright Qualcomm "
        "Technologies, Inc. Confidential and Proprietary'. Built from the private depthai-device-rvc4-vendor module",
    ),
)

GITHUB_DOWNLOAD_RE = re.compile(r"^git\+https://github\.com/([^/@]+/[^/@]+?)(?:\.git)?@(.+)$")


def strip_prefix(value: str, prefix: str) -> str:
    return value[len(prefix) :] if value.startswith(prefix) else value


def spdx_id(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9.-]+", "-", value.strip()).strip("-") or "item"
    if not clean[0].isalpha():
        clean = f"item-{clean}"
    return f"SPDXRef-{clean}"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_vcpkg_status(install_root: Path, triplet: str) -> dict[str, dict[str, Any]]:
    """Installed packages of one triplet: version, port version, features and same-triplet dependencies."""
    database = install_root / "vcpkg"
    paths = [database / "status"]
    if (database / "updates").is_dir():
        paths += sorted(path for path in (database / "updates").iterdir() if path.is_file())

    # Later paragraphs (the update files) replace earlier ones
    records: dict[tuple[str, str, str], dict[str, str]] = {}
    for path in paths:
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
            # A dependency with another triplet is a host build tool, not linked into the firmware
            if dependency_name and dependency_name != name and dependency_triplet in ("", triplet):
                entry["depends"].add(dependency_name)
    return packages


def elf_needed(data: bytes) -> list[str]:
    """DT_NEEDED entries of an ELF file (the shared libraries that the loader must find)."""
    try:
        return _elf_needed(data)
    except (struct.error, ValueError, IndexError):
        # A file that is not a well-formed ELF file does not stop the firmware build
        return []


def _elf_needed(data: bytes) -> list[str]:
    if data[:4] != b"\x7fELF":
        return []
    is_64 = data[4] == 2
    endian = "<" if data[5] == 1 else ">"
    if is_64:
        (phoff,) = struct.unpack_from(endian + "Q", data, 0x20)
        phentsize, phnum = struct.unpack_from(endian + "HH", data, 0x36)
    else:
        (phoff,) = struct.unpack_from(endian + "I", data, 0x1C)
        phentsize, phnum = struct.unpack_from(endian + "HH", data, 0x2A)

    loads: list[tuple[int, int, int]] = []
    dynamic: tuple[int, int] | None = None
    for index in range(phnum):
        offset = phoff + index * phentsize
        if is_64:
            p_type, _, p_offset, p_vaddr, _, p_filesz = struct.unpack_from(endian + "IIQQQQ", data, offset)
        else:
            p_type, p_offset, p_vaddr, _, p_filesz = struct.unpack_from(endian + "IIIII", data, offset)
        if p_type == 1:  # PT_LOAD
            loads.append((p_vaddr, p_offset, p_filesz))
        elif p_type == 2:  # PT_DYNAMIC
            dynamic = (p_offset, p_filesz)
    if dynamic is None:
        return []

    entry_format = endian + ("qQ" if is_64 else "iI")
    entry_size = struct.calcsize(entry_format)
    needed_offsets: list[int] = []
    strtab_address: int | None = None
    for offset in range(dynamic[0], dynamic[0] + dynamic[1] - entry_size + 1, entry_size):
        tag, value = struct.unpack_from(entry_format, data, offset)
        if tag == 0:  # DT_NULL
            break
        if tag == 1:  # DT_NEEDED
            needed_offsets.append(value)
        elif tag == 5:  # DT_STRTAB
            strtab_address = value
    if strtab_address is None:
        return []
    strtab = next((offset + strtab_address - vaddr for vaddr, offset, size in loads if vaddr <= strtab_address < vaddr + size), None)
    if strtab is None:
        return []
    names = []
    for name_offset in needed_offsets:
        start = strtab + name_offset
        names.append(data[start : data.index(b"\0", start)].decode("utf-8", errors="replace"))
    return names


def read_fwp_files(archive: Path) -> list[dict[str, Any]]:
    files = []
    with tarfile.open(archive, "r:*") as tar:
        for member in tar:
            if not member.isfile():
                continue
            data = tar.extractfile(member).read()
            files.append(
                {
                    "path": strip_prefix(member.name, "./"),
                    "size": len(data),
                    "sha1": hashlib.sha1(data).hexdigest(),
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "elf": data[:4] == b"\x7fELF",
                    "needed": elf_needed(data),
                }
            )
    return sorted(files, key=lambda entry: entry["path"])


def git_output(path: Path, *args: str) -> str | None:
    # The checkout can belong to another user than the build container user; the SBOM only reads it
    try:
        return subprocess.run(
            ["git", "-c", "safe.directory=*", "-C", str(path), *args], check=True, text=True, capture_output=True
        ).stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def git_checkout_info(path: Path) -> tuple[str | None, str | None]:
    """(commit, version) of a git checkout root; (None, None) for a plain directory."""
    if not (path / ".git").exists():
        return None, None
    commit = git_output(path, "rev-parse", "HEAD")
    describe = git_output(path, "describe", "--tags")
    return commit, describe or commit


def gitmodules_urls(checkout: Path) -> dict[str, str]:
    """Submodule path -> URL from .gitmodules. The remote config is not used: in CI it can hold a token."""
    output = git_output(checkout, "config", "-f", ".gitmodules", "--get-regexp", r"submodule\..*\.(path|url)")
    paths: dict[str, str] = {}
    urls: dict[str, str] = {}
    for line in (output or "").splitlines():
        key, _, value = line.partition(" ")
        target = paths if key.endswith(".path") else urls
        target[key.rsplit(".", 1)[0]] = value
    return {paths[key]: urls[key] for key in paths if key in urls}


def public_download(url: str | None, commit: str | None) -> str | None:
    """SPDX download location for a public https git URL; None for relative or unknown URLs."""
    if not url or not url.startswith("https://"):
        return None
    url = re.sub(r"^https://[^/@]+@", "https://", url)
    return f"git+{url}@{commit}" if commit else f"git+{url}"


def purl_for_download(download: str | None) -> str | None:
    match = GITHUB_DOWNLOAD_RE.match(download or "")
    if not match:
        return None
    return f"pkg:github/{match.group(1).lower()}@{quote(match.group(2), safe='')}"


def component(ref: str, name: str, **fields: Any) -> dict[str, Any]:
    entry = {"ref": ref, "name": name, "type": "library", "properties": {}}
    entry.update({key: value for key, value in fields.items() if value is not None})
    return entry


def build_inventory(
    payload: dict[str, Any],
    *,
    license_of: Callable[[dict[str, Any]], str],
    repo_root: Path,
    install_root: Path,
    triplet: str,
    product: str,
    version: str,
    commit: str,
    fwp_archive: Path | None,
) -> dict[str, Any]:
    status = read_vcpkg_status(install_root, triplet)
    components: list[dict[str, Any]] = []
    depends: dict[str, set[str]] = {"root": set()}
    runtime: dict[str, set[str]] = {}
    contains: list[str] = []

    # vcpkg packages (all statically linked; the rvc4 triplet builds static libraries)
    vcpkg_refs: dict[str, str] = {}
    for package in payload.get("packages", []):
        if package.get("source_type") != "vcpkg":
            continue
        name = package["package"]
        installed = status.get(name, {})
        # Some vcpkg ports write an unexpanded variable (for example ${VERSION}) into their SPDX file
        downloads = [url for url in package.get("resource_download_locations") or [] if "${" not in url and " " not in url]
        download = downloads[0] if downloads else None
        ref = f"vcpkg-{name}"
        vcpkg_refs[name] = ref
        entry = component(
            ref,
            name,
            version=installed.get("version") or package.get("version"),
            description=package.get("description"),
            license=license_of(package),
            download=download,
            homepage=package.get("homepage"),
            purl=purl_for_download(download),
        )
        entry["properties"].update(
            {
                "depthai:source": "vcpkg",
                "vcpkg:triplet": triplet,
                "vcpkg:port-version": installed.get("port_version", "0"),
                "vcpkg:features": ",".join(installed.get("features", ["core"])),
            }
        )
        if package.get("source_url"):
            entry["properties"]["vcpkg:port-location"] = package["source_url"]
        components.append(entry)
    for name, ref in vcpkg_refs.items():
        depends[ref] = {vcpkg_refs[dependency] for dependency in status.get(name, {}).get("depends", ()) if dependency in vcpkg_refs}
    manifest_names = set()
    manifest_path = repo_root / "vcpkg.json"
    if manifest_path.is_file():
        for dependency in json.loads(manifest_path.read_text(encoding="utf-8")).get("dependencies", []):
            manifest_names.add(dependency if isinstance(dependency, str) else dependency.get("name"))
    reachable: set[str] = set()
    stack = [vcpkg_refs[name] for name in manifest_names if name in vcpkg_refs]
    depends["root"].update(stack)
    while stack:
        ref = stack.pop()
        if ref not in reachable:
            reachable.add(ref)
            stack.extend(depends.get(ref, ()))
    # A package that no edge reaches is still linked in; attach it to the root
    depends["root"].update(ref for ref in vcpkg_refs.values() if ref not in reachable)

    # Submodules and in-tree code, from this repository and from the private module (when the collector found it)
    private_module = (payload.get("profile") or {}).get("private_module") or {}
    roots: dict[str, Path] = {REPO_ROOT_NAME: repo_root}
    if private_module.get("path"):
        roots[private_module.get("name") or PRIVATE_MODULE_NAME] = Path(private_module["path"])
    submodule_urls = {root_name: gitmodules_urls(base) for root_name, base in roots.items()}

    def add_source(root_name: str, path: str, name: str, license_expression: str, supplier: str | None, source: str, urls: dict[str, str]) -> str:
        checkout = roots[root_name] / path
        commit_hash, described = git_checkout_info(checkout)
        relative = strip_prefix(path, "./")
        if root_name != REPO_ROOT_NAME:
            relative = f"{root_name}/{relative}"
        ref = f"src-{relative}"
        entry = component(
            ref,
            name,
            version=described,
            supplier=supplier,
            license=license_expression,
            download=public_download(urls.get(path), commit_hash),
        )
        entry["purl"] = purl_for_download(entry.get("download"))
        entry["properties"].update({"depthai:source": source, "depthai:path": relative})
        if commit_hash:
            entry["properties"]["depthai:commit"] = commit_hash
        components.append(entry)
        return ref

    for package in payload.get("packages", []):
        path = package.get("source_path")
        root_name = package.get("source_root") or REPO_ROOT_NAME
        if package.get("source_type") != "vendored" or root_name not in roots or not (roots[root_name] / path).exists():
            continue
        source = "git-submodule" if (roots[root_name] / path / ".git").exists() else "vendored"
        ref = add_source(root_name, path, package["package"], license_of(package), package.get("supplier"), source,
                         {path: package.get("source_url")})
        if package.get("version"):  # the vendor version; the git commit stays a property
            components[-1]["version"] = package["version"]
        if package.get("note"):
            components[-1]["properties"]["depthai:note"] = package["note"]
        depends["root"].add(ref)

    for root_name, path, name, license_expression, supplier, note in UNLISTED_SOURCES:
        if root_name in roots and (roots[root_name] / path).exists():
            source = "git-submodule" if (roots[root_name] / path / ".git").exists() else "vendored"
            depends["root"].add(add_source(root_name, path, name, license_expression, supplier, source, submodule_urls[root_name]))
            components[-1]["properties"]["depthai:note"] = note

    # Files in the fwp archive, and the shared libraries that they load from the OS at runtime
    archive_sha256 = None
    files: list[dict[str, Any]] = []
    if fwp_archive and fwp_archive.is_file():
        archive_sha256 = sha256_file(fwp_archive)
        files = read_fwp_files(fwp_archive)
    os_refs: dict[str, str] = {}
    for file in files:
        ref = f"file-{file['path']}"
        contains.append(ref)
        for soname in file["needed"]:
            if soname not in os_refs:
                os_refs[soname] = f"os-{soname}"
                os_entry = component(os_refs[soname], soname)
                os_entry["properties"].update(
                    {"depthai:source": "luxonis-os", "depthai:note": "Shared library from the Luxonis OS image; not in the fwp"}
                )
                components.append(os_entry)
            runtime.setdefault(ref, set()).add(os_refs[soname])

    return {
        "product": product,
        "version": version,
        "commit": commit,
        "fwp_archive": fwp_archive.name if fwp_archive and fwp_archive.is_file() else None,
        "fwp_sha256": archive_sha256,
        "triplet": triplet,
        "components": components,
        "files": files,
        "depends": depends,
        "runtime": runtime,
        "contains": contains,
    }


def custom_license(license_id: str) -> tuple[str, str | None]:
    return CUSTOM_LICENSES.get(license_id, (strip_prefix(license_id, "LicenseRef-"), None))


def cyclonedx_licenses(expression: str) -> list[dict[str, Any]]:
    if re.fullmatch(r"LicenseRef-[A-Za-z0-9.-]+", expression):
        name, url = custom_license(expression)
        license_entry: dict[str, str] = {"name": name}
        if url:
            license_entry["url"] = url
        return [{"license": license_entry}]
    return [{"expression": expression}]


def license_refs(expression: str | None) -> list[str]:
    return re.findall(r"LicenseRef-[A-Za-z0-9.-]+", expression or "")


def to_spdx(
    inventory: dict[str, Any],
    *,
    generated_at: str,
    license_output_name: str,
) -> dict[str, Any]:
    product, version = inventory["product"], inventory["version"]
    # spdx_id() maps several characters to "-", so two refs can give the same id: make each one unique
    ids: dict[str, str] = {}
    refs = [("root", product)] + [(entry["ref"], f"package-{entry['ref']}") for entry in inventory["components"]]
    refs += [(f"file-{file['path']}", f"file-{file['path']}") for file in inventory["files"]]
    for ref, value in refs:
        candidate, counter = spdx_id(value), 1
        while candidate in ids.values():
            counter += 1
            candidate = f"{spdx_id(value)}-{counter}"
        ids[ref] = candidate
    root_id = ids["root"]

    root: dict[str, Any] = {
        "name": product,
        "SPDXID": root_id,
        "versionInfo": version,
        "supplier": "Organization: Luxonis",
        "downloadLocation": NOASSERTION,
        "licenseConcluded": FWP_LICENSE_REF,
        "licenseDeclared": FWP_LICENSE_REF,
        "copyrightText": NOASSERTION,
        "comment": f"Built from depthai-device commit {inventory['commit']}, vcpkg triplet {inventory['triplet']}. "
        f"{CORE_NOTE}.",
        "externalRefs": [
            {
                "referenceCategory": "OTHER",
                "referenceType": "http://spdx.org/rdf/references/other",
                "referenceLocator": license_output_name,
                "comment": "Sibling human-readable license artifact.",
            }
        ],
    }
    if inventory["files"]:
        verification = hashlib.sha1("".join(sorted(file["sha1"] for file in inventory["files"])).encode()).hexdigest()
        root.update(
            {
                "filesAnalyzed": True,
                "packageVerificationCode": {"packageVerificationCodeValue": verification},
                "licenseInfoFromFiles": [NOASSERTION],
            }
        )
    else:
        root["filesAnalyzed"] = False
    if inventory["fwp_archive"]:
        root["packageFileName"] = inventory["fwp_archive"]
        root["checksums"] = [{"algorithm": "SHA256", "checksumValue": inventory["fwp_sha256"]}]

    packages = [root]
    used_refs = set(license_refs(FWP_LICENSE_REF))
    for entry in inventory["components"]:
        license_expression = entry.get("license") or NOASSERTION
        used_refs.update(license_refs(license_expression))
        package: dict[str, Any] = {
            "name": entry["name"],
            "SPDXID": ids[entry["ref"]],
            "downloadLocation": entry.get("download") or NOASSERTION,
            "filesAnalyzed": False,
            "licenseConcluded": license_expression,
            "licenseDeclared": license_expression,
            "copyrightText": NOASSERTION,
        }
        if entry.get("version"):
            package["versionInfo"] = entry["version"]
        if entry.get("supplier"):
            package["supplier"] = f"Organization: {entry['supplier']}"
        if entry.get("homepage"):
            package["homepage"] = entry["homepage"]
        if entry.get("description"):
            package["summary"] = entry["description"]
        if entry.get("purl"):
            package["externalRefs"] = [{"referenceCategory": "PACKAGE-MANAGER", "referenceType": "purl", "referenceLocator": entry["purl"]}]
        if entry["properties"]:
            package["comment"] = "; ".join(f"{key}={value}" for key, value in sorted(entry["properties"].items()))
        packages.append(package)

    spdx_files = []
    for file in inventory["files"]:
        spdx_files.append(
            {
                "fileName": f"./{file['path']}",
                "SPDXID": ids[f"file-{file['path']}"],
                "checksums": [
                    {"algorithm": "SHA1", "checksumValue": file["sha1"]},
                    {"algorithm": "SHA256", "checksumValue": file["sha256"]},
                ],
                "fileTypes": ["BINARY"] if file["elf"] or file["path"].endswith(".dlc") else ["TEXT"],
                "licenseConcluded": NOASSERTION,
                "copyrightText": NOASSERTION,
            }
        )

    relationships = [{"spdxElementId": "SPDXRef-DOCUMENT", "relationshipType": "DESCRIBES", "relatedSpdxElement": root_id}]
    for ref in inventory["contains"]:
        relationships.append({"spdxElementId": root_id, "relationshipType": "CONTAINS", "relatedSpdxElement": ids[ref]})
    for ref, targets in sorted(inventory["depends"].items()):
        for target in sorted(targets):
            relationships.append({"spdxElementId": ids[ref], "relationshipType": "DEPENDS_ON", "relatedSpdxElement": ids[target]})
    for ref, targets in sorted(inventory["runtime"].items()):
        for target in sorted(targets):
            relationships.append({"spdxElementId": ids[target], "relationshipType": "RUNTIME_DEPENDENCY_OF", "relatedSpdxElement": ids[ref]})

    extracted = []
    for license_id in sorted(used_refs):
        name, url = custom_license(license_id)
        text = f"{name}. The full text is in the firmware package LICENSE file, which is published next to this SBOM."
        info: dict[str, Any] = {"licenseId": license_id, "name": name, "extractedText": text}
        if url:
            info["seeAlso"] = [url]
        extracted.append(info)

    document: dict[str, Any] = {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{product}-{version}",
        "documentNamespace": f"https://luxonis.com/spdx/{product}/{version}/{inventory['commit']}-{uuid.uuid4()}",
        "creationInfo": {"created": generated_at, "creators": ["Organization: Luxonis", "Tool: generate_license.py"]},
        "documentDescribes": [root_id],
        "packages": packages,
        "relationships": relationships,
    }
    if spdx_files:
        document["files"] = spdx_files
    if extracted:
        document["hasExtractedLicensingInfos"] = extracted
    return document


def cyclonedx_external_refs(entry: dict[str, Any]) -> list[dict[str, str]]:
    refs = []
    if entry.get("homepage"):
        refs.append({"type": "website", "url": entry["homepage"]})
    download = entry.get("download")
    if download and download.startswith("git+"):
        url, _, ref = strip_prefix(download, "git+").rpartition("@")
        if not url:
            url, ref = ref, ""
        vcs = {"type": "vcs", "url": url}
        if ref:
            vcs["comment"] = f"ref {ref}"
        refs.append(vcs)
    elif download:
        refs.append({"type": "distribution", "url": download})
    return refs


def to_cyclonedx(inventory: dict[str, Any], *, generated_at: str) -> dict[str, Any]:
    def properties(values: dict[str, str]) -> list[dict[str, str]]:
        return [{"name": key, "value": value} for key, value in sorted(values.items())]

    file_components = []
    for file in inventory["files"]:
        file_entry: dict[str, Any] = {
            "type": "file",
            "bom-ref": f"file-{file['path']}",
            "name": file["path"],
            "hashes": [{"alg": "SHA-1", "content": file["sha1"]}, {"alg": "SHA-256", "content": file["sha256"]}],
        }
        if file["path"].endswith(".dlc"):
            file_entry["supplier"] = {"name": "Luxonis"}
            file_entry["properties"] = properties({"depthai:kind": "neural-network-model"})
        file_components.append(file_entry)

    root: dict[str, Any] = {
        "type": "firmware",
        "bom-ref": "root",
        "supplier": {"name": "Luxonis"},
        "name": inventory["product"],
        "version": inventory["version"],
        "licenses": cyclonedx_licenses(FWP_LICENSE_REF),
        "properties": properties(
            {"depthai:git-commit": inventory["commit"], "vcpkg:triplet": inventory["triplet"], "depthai:not-included": CORE_NOTE}
        ),
    }
    if inventory["fwp_sha256"]:
        root["hashes"] = [{"alg": "SHA-256", "content": inventory["fwp_sha256"]}]
    if file_components:
        root["components"] = file_components

    components = []
    for entry in inventory["components"]:
        cyclonedx_entry: dict[str, Any] = {"type": entry["type"], "bom-ref": entry["ref"], "name": entry["name"]}
        if entry.get("version"):
            cyclonedx_entry["version"] = entry["version"]
        if entry.get("supplier"):
            cyclonedx_entry["supplier"] = {"name": entry["supplier"]}
        if entry.get("description"):
            cyclonedx_entry["description"] = entry["description"]
        if entry.get("license") and entry["license"] != NOASSERTION:
            cyclonedx_entry["licenses"] = cyclonedx_licenses(entry["license"])
        if entry.get("purl"):
            cyclonedx_entry["purl"] = entry["purl"]
        external = cyclonedx_external_refs(entry)
        if external:
            cyclonedx_entry["externalReferences"] = external
        if entry["properties"]:
            cyclonedx_entry["properties"] = properties(entry["properties"])
        components.append(cyclonedx_entry)

    edges: dict[str, set[str]] = {}
    for source in (inventory["depends"], inventory["runtime"]):
        for ref, targets in source.items():
            edges.setdefault(ref, set()).update(targets)
    refs = ["root"] + [file["bom-ref"] for file in file_components] + [entry["ref"] for entry in inventory["components"]]
    dependencies = [{"ref": ref, "dependsOn": sorted(edges.get(ref, ()))} for ref in refs]

    return {
        "$schema": "http://cyclonedx.org/schema/bom-1.6.schema.json",
        "bomFormat": "CycloneDX",
        "specVersion": "1.6",
        "serialNumber": f"urn:uuid:{uuid.uuid4()}",
        "version": 1,
        "metadata": {
            "timestamp": generated_at,
            "tools": {"components": [{"type": "application", "name": "generate_license.py", "supplier": {"name": "Luxonis"}}]},
            "component": root,
        },
        "components": components,
        "dependencies": dependencies,
    }
