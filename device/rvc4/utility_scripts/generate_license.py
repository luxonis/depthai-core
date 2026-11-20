#!/usr/bin/env python3
"""License bundle and SBOMs for the depthai-device firmware package (fwp).

The bundle opens with the firmware package license (LICENSE.fwp): the binary package bundles proprietary
ST/Qualcomm-derived components, so it is not under the MIT license that covers the source code
(LICENSE of the depthai-core repository). After the preamble it lists every third-party component that the collector
(generate_third_party_licenses.py) found in the vcpkg install tree and in this repository. The bundle ends with the
legal files of the private depthai-device-rvc4-vendor module (--vendor-legal-dir), in the layout of the Luxonis OS
BSP packages: NOTICE.txt, legal/DEPTHAI-EULA.txt (the Luxonis DepthAI Vendor Module EULA, which covers the vendor
module in the package; LICENSE.fwp refers to it), legal/NOTICE.md and the
texts in legal/licenses/, each under its path. The module writes these files (from its sources, or at the top of its
prebuilt archive), so the components that the collector found in the module sources are not listed a second time.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

from fwp_sbom import PRIVATE_MODULE_NAME, build_inventory, to_cyclonedx, to_spdx


NOASSERTION = "NOASSERTION"
FWP_LICENSE_FILE_NAME = "LICENSE.fwp"
SOURCE_LICENSE_NOTE = (
    "The public part of the source code of this firmware (directory device/rvc4 of the depthai-core repository)\n"
    "is available under the MIT License (see LICENSE in that repository). The license above and the\n"
    "third-party terms below apply to this binary firmware package."
)


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Generate FWP package-level license bundle and optional SPDX and CycloneDX SBOMs.")
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--product", required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--fwp-archive", type=Path, default=None)
    parser.add_argument("--manifest-root", type=Path, default=None, help="Directory of vcpkg.json (default: the repository root)")
    parser.add_argument("--triplet", default="rvc4")
    parser.add_argument("--vcpkg-root", type=Path, default=repo_root / "vcpkg")
    parser.add_argument("--install-root", type=Path, default=repo_root / "build_docker_arm64_rvc4" / "vcpkg_installed")
    parser.add_argument(
        "--private-module-root",
        type=Path,
        default=None,
        help="Path to the private depthai-device-rvc4-vendor checkout (default: $DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR). Forwarded to "
        "the collector.",
    )
    parser.add_argument(
        "--license-input",
        type=Path,
        default=repo_root / FWP_LICENSE_FILE_NAME,
        help=f"Firmware package license text that opens the bundle. A path named LICENSE is redirected to a sibling "
        f"{FWP_LICENSE_FILE_NAME} when that file exists, because LICENSE is a source license.",
    )
    parser.add_argument(
        "--vendor-legal-dir",
        type=Path,
        default=None,
        help=f"Folder with the legal files of the {PRIVATE_MODULE_NAME} module (NOTICE.txt and legal/): legal-package "
        "of its build, or its prebuilt archive. They are appended to the bundle.",
    )
    parser.add_argument("--license-output", type=Path, required=True)
    parser.add_argument("--spdx-output", type=Path, default=None)
    parser.add_argument("--cyclonedx-output", type=Path, default=None)
    parser.add_argument(
        "--generated-at",
        default=None,
        help="Optional ISO-8601 timestamp. Defaults to the current UTC time.",
    )
    return parser.parse_args()


def run(cmd: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        check=True,
        text=True,
        capture_output=True,
    )


def resolve_license_input(path: Path) -> Path:
    """The file that opens the bundle.

    A caller can pass <dir>/LICENSE, which is a source license; the firmware package
    license lives next to it in LICENSE.fwp and is used instead when it exists.
    """
    if path.name == "LICENSE":
        fwp_license = path.with_name(FWP_LICENSE_FILE_NAME)
        if fwp_license.is_file():
            return fwp_license
        print(f"warning: {fwp_license} does not exist; the bundle opens with {path}", file=sys.stderr)
    return path


def collect_packages(args: argparse.Namespace) -> dict[str, Any]:
    collector = args.repo_root / "utility_scripts" / "generate_third_party_licenses.py"
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as file:
        output_path = Path(file.name)

    cmd = [
        sys.executable,
        str(collector),
        "--repo-root",
        str(args.repo_root),
        *(["--manifest-root", str(args.manifest_root)] if args.manifest_root else []),
        "--triplet",
        args.triplet,
        "--vcpkg-root",
        str(args.vcpkg_root),
        "--install-root",
        str(args.install_root),
        "--output",
        str(output_path),
        "--skip-license-report",
    ]
    if args.private_module_root:
        cmd += ["--private-module-root", str(args.private_module_root)]
    try:
        result = run(cmd, cwd=args.repo_root)
        # The collector reports where it found (or did not find) the private module
        if result.stderr:
            sys.stderr.write(result.stderr)
        return json.loads(output_path.read_text(encoding="utf-8"))
    except subprocess.CalledProcessError as error:
        sys.stderr.write(error.stdout or "")
        sys.stderr.write(error.stderr or "")
        raise SystemExit(f"error: {collector.name} failed with exit code {error.returncode}") from error
    finally:
        output_path.unlink(missing_ok=True)


def package_license(package: dict[str, Any]) -> str:
    return (
        package.get("license_expression")
        or package.get("inferred_license_expression")
        or package.get("license_concluded")
        or package.get("license_declared")
        or NOASSERTION
    )


def package_version(package: dict[str, Any]) -> str | None:
    return package.get("version") or package.get("binary_version")


def package_homepage(package: dict[str, Any]) -> str | None:
    return package.get("homepage") or package.get("source_url") or upstream_source(package)


def upstream_source(package: dict[str, Any]) -> str | None:
    """The upstream source that vcpkg downloaded, for a port without a homepage (for example a Luxonis fork).
    MPL code (mp4v2) must say where its source is available."""
    for location in package.get("resource_download_locations") or []:
        if "${" in location:
            continue
        match = re.match(r"^git\+(https://github\.com/[^@]+?)(?:\.git)?@(.+)$", location)
        if match:
            return f"{match.group(1)}/tree/{match.group(2)}"
        if location.startswith(("https://", "git+https://")):
            return location[len("git+") :] if location.startswith("git+") else location
    return None


def sorted_packages(payload: dict[str, Any]) -> list[dict[str, Any]]:
    packages = payload.get("packages")
    if not isinstance(packages, list):
        raise SystemExit("Collector output does not contain a package list.")
    return sorted(packages, key=lambda package: (package.get("source_type") or "", package.get("package") or ""))


def vendor_legal_text(legal_dir: Path) -> str:
    """The legal files of the vendor module, each under its path: NOTICE.txt, legal/DEPTHAI-EULA.txt, legal/NOTICE.md,
    then the texts in legal/licenses/ in name order."""
    files = [legal_dir / "NOTICE.txt", legal_dir / "legal" / "DEPTHAI-EULA.txt", legal_dir / "legal" / "NOTICE.md"]
    for path in files:
        if not path.is_file():
            raise SystemExit(f"error: {path} not found: the folder {legal_dir} has no legal files of the {PRIVATE_MODULE_NAME} module")
    files += sorted((legal_dir / "legal" / "licenses").glob("*"))
    title = f"Proprietary components: {PRIVATE_MODULE_NAME}"
    lines = [title, "=" * len(title), ""]
    for path in files:
        name = path.relative_to(legal_dir).as_posix()
        lines.extend([name, "-" * len(name), "", path.read_text(encoding="utf-8", errors="replace").rstrip(), ""])
    return "\n".join(lines)


def generate_license_bundle(
    *,
    payload: dict[str, Any],
    license_text: str,
    vendor_license_text: str | None = None,
) -> str:
    packages = sorted_packages(payload)
    if vendor_license_text:
        # The license text of the vendor module lists its third-party components itself
        packages = [package for package in packages if package.get("source_root") != PRIVATE_MODULE_NAME]
    lines = [
        license_text.rstrip(),
        "",
        "---",
        "",
        SOURCE_LICENSE_NOTE,
        "",
        "---",
        "",
        "Third-Party Licenses",
        "====================",
        "",
        "This firmware package includes the following third-party components.",
        "",
    ]

    for source_type, title in (("vcpkg", "Third-Party Dependencies"), ("vendored", "Vendored Components")):
        selected = [package for package in packages if package.get("source_type") == source_type]
        if not selected:
            continue
        lines.extend([title, "-" * len(title), ""])
        for package in selected:
            name = package["package"]
            lines.extend([name, "~" * len(name), ""])
            lines.append(f"License: {package_license(package)}")
            version_info = package_version(package)
            if version_info:
                lines.append(f"Version: {version_info}")
            homepage = package_homepage(package)
            if homepage:
                lines.append(f"Source: {homepage}")
            lines.append("")
            notice_text = (package.get("notice_text") or "").strip()
            if notice_text:
                lines.extend([notice_text, ""])
            else:
                lines.append("No license text was found by the collector.")
                lines.append("")

    if vendor_license_text:
        lines.extend(["---", "", vendor_license_text.rstrip()])
    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    args = parse_args()
    args.repo_root = args.repo_root.resolve()
    args.vcpkg_root = args.vcpkg_root.resolve()
    args.install_root = args.install_root.resolve()
    generated_at = args.generated_at or dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

    payload = collect_packages(args)  # the collector reports the private module that it found (or did not find)

    license_input = resolve_license_input(args.license_input)
    print(f"Firmware package license preamble: {license_input}")
    license_text = license_input.read_text(encoding="utf-8")
    vendor_license_text = None
    if args.vendor_legal_dir:
        print(f"Legal files of the {PRIVATE_MODULE_NAME} module: {args.vendor_legal_dir}")
        vendor_license_text = vendor_legal_text(args.vendor_legal_dir)
    else:
        print(f"warning: no --vendor-legal-dir: the bundle has no legal files of the {PRIVATE_MODULE_NAME} module", file=sys.stderr)
    license_bundle = generate_license_bundle(
        payload=payload,
        license_text=license_text,
        vendor_license_text=vendor_license_text,
    )

    args.license_output.parent.mkdir(parents=True, exist_ok=True)
    args.license_output.write_text(license_bundle, encoding="utf-8")
    if args.spdx_output or args.cyclonedx_output:
        inventory = build_inventory(
            payload,
            license_of=package_license,
            repo_root=args.repo_root,
            install_root=args.install_root,
            triplet=args.triplet,
            product=args.product,
            version=args.version,
            commit=args.commit,
            fwp_archive=args.fwp_archive,
        )
    if args.spdx_output:
        spdx = to_spdx(
            inventory,
            generated_at=generated_at,
            license_output_name=args.license_output.name,
        )
        args.spdx_output.parent.mkdir(parents=True, exist_ok=True)
        args.spdx_output.write_text(json.dumps(spdx, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.cyclonedx_output:
        cyclonedx = to_cyclonedx(inventory, generated_at=generated_at)
        args.cyclonedx_output.parent.mkdir(parents=True, exist_ok=True)
        args.cyclonedx_output.write_text(json.dumps(cyclonedx, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
