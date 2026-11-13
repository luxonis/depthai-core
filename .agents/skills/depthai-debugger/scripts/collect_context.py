#!/usr/bin/env python3
"""Collect local debugging context without importing DepthAI or probing devices."""

import argparse
import hashlib
import importlib.machinery
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone

ENVIRONMENT_KEYS = """
DEPTHAI_LEVEL XLINK_LEVEL DEPTHAI_DEBUG DEPTHAI_DEBUGGER
DEPTHAI_PIPELINE_DEBUGGING DEPTHAI_PROFILING DEPTHAI_PLATFORM DEPTHAI_PROTOCOL
DEPTHAI_DEVICE_ID_LIST DEPTHAI_DEVICE_MXID_LIST DEPTHAI_DEVICE_NAME_LIST
DEPTHAI_DEVICE_BINARY DEPTHAI_DEVICE_RVC4_FWP DEPTHAI_RECORD DEPTHAI_REPLAY
DEPTHAI_SEARCH_TIMEOUT DEPTHAI_BOOTUP_TIMEOUT DEPTHAI_CONNECT_TIMEOUT
DEPTHAI_RECONNECT_TIMEOUT DEPTHAI_RPC_READ_TIMEOUT DEPTHAI_RPC_WRITE_TIMEOUT
DEPTHAI_WATCHDOG DEPTHAI_WATCHDOG_INITIAL_DELAY DEPTHAI_CRASHDUMP
DEPTHAI_CRASHDUMP_TIMEOUT DEPTHAI_DISABLE_FEEDBACK
DEPTHAI_DISABLE_CRASHDUMP_COLLECTION DEPTHAI_TELEMETRY
DEPTHAI_INSTALL_SIGNAL_HANDLER DEPTHAI_AUTOCALIBRATION
""".split()


def repository_context(path):
    result = {"path": str(path)}
    commands = {
        "commit": ["rev-parse", "HEAD"],
        "branch": ["branch", "--show-current"],
        "status": ["status", "--short", "--untracked-files=normal"],
        "submodules": ["submodule", "status"],
    }
    for key, command in commands.items():
        try:
            process = subprocess.run(
                ["git", "-C", str(path), *command],
                capture_output=True, text=True, errors="replace", timeout=15,
            )
            result[key] = {
                "returncode": process.returncode,
                "stdout": process.stdout.strip(),
                "stderr": process.stderr.strip(),
            }
        except (OSError, subprocess.TimeoutExpired) as error:
            result[key] = {"error": str(error)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument("--related-repo", type=Path, action="append", default=[])
    parser.add_argument("--build-dir", type=Path, help="Existing build directory; never configured")
    parser.add_argument("--artifact", type=Path, action="append", default=[], help="File to SHA-256")
    parser.add_argument("--output", type=Path, required=True, help="New JSON file; parent must exist")
    args = parser.parse_args()

    # Exclusive creation prevents replacing another investigation's evidence.
    try:
        output = args.output.open("x", encoding="utf-8")
    except OSError as error:
        parser.error(str(error))

    with output:
        report = {
            "schema_version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "host": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
            "python": {"executable": sys.executable, "version": sys.version},
            "cwd": str(Path.cwd()),
            "environment": {key: os.environ[key] for key in ENVIRONMENT_KEYS if key in os.environ},
            "repositories": [repository_context(p.resolve()) for p in [args.repo, *args.related_repo]],
            "tools": {name: shutil.which(name) for name in ("git", "gdb", "gdb-multiarch", "lldb", "readelf", "addr2line", "ctest")},
            "artifacts": [],
            "notes": ["No SDK import or device probe performed; candidate module and checkout pins are not runtime proof."],
        }
        try:
            report["python"]["depthai_distribution_version"] = importlib.metadata.version("depthai")
        except importlib.metadata.PackageNotFoundError:
            report["python"]["depthai_distribution_version"] = None
        spec = importlib.machinery.PathFinder.find_spec("depthai", [str(Path.cwd()), *sys.path])
        report["python"]["depthai_module_candidate"] = spec.origin if spec else None

        report["firmware_pins"] = {}
        for filename in ("DepthaiDeviceSideConfig.cmake", "DepthaiDeviceRVC4Config.cmake"):
            path = args.repo / "cmake" / "Depthai" / filename
            try:
                report["firmware_pins"][filename] = dict(re.findall(
                    r'^set\((DEPTHAI_DEVICE_\w+)\s+"([^"]*)"\)', path.read_text(), re.MULTILINE
                ))
            except OSError as error:
                report["firmware_pins"][filename] = {"error": str(error)}

        if args.build_dir:
            cache = args.build_dir / "CMakeCache.txt"
            report["cmake_cache"] = {"path": str(cache.resolve())}
            try:
                selected = {}
                for line in cache.read_text(errors="replace").splitlines():
                    match = re.match(r"([^#/:][^:]*):[^=]+=(.*)", line)
                    if match and (match[1].startswith(("DEPTHAI_", "SANITIZE_", "XLINK_"))
                                  or match[1] in ("CMAKE_BUILD_TYPE", "CMAKE_GENERATOR", "CMAKE_CXX_COMPILER", "CMAKE_HOME_DIRECTORY")):
                        # Do not collect credential-like cache entries.
                        if not re.search(r"KEY|TOKEN|PASSWORD|SECRET|URL", match[1], re.IGNORECASE):
                            selected[match[1]] = match[2]
                report["cmake_cache"]["values"] = selected
            except OSError as error:
                report["cmake_cache"]["error"] = str(error)

        for path in args.artifact:
            item = {"path": str(path.resolve())}
            try:
                digest = hashlib.sha256()
                with path.open("rb") as artifact:
                    for chunk in iter(lambda: artifact.read(1024 * 1024), b""):
                        digest.update(chunk)
                item.update(size=path.stat().st_size, sha256=digest.hexdigest())
            except OSError as error:
                item["error"] = str(error)
            report["artifacts"].append(item)
        json.dump(report, output, indent=2)
        output.write("\n")
    print(args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
