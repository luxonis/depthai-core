"""Run an example with GIL checks and optional dependency skips for CTest."""

import argparse
import ast
import importlib.util
import os
from pathlib import Path
import runpy
import sys


# Keep SDK/import regressions as failures. Only these optional third-party
# packages may cause a skip, and only when explicitly requested by the build.
OPTIONAL_MODULES = {"av", "cv2", "matplotlib", "open3d", "rerun", "scipy", "websockets"}


def check_gil():
    if os.environ.get("DEPTHAI_REQUIRE_GIL_DISABLED") == "1":
        assert hasattr(sys, "_is_gil_enabled") and not sys._is_gil_enabled(), "GIL must remain disabled"


def missing_optional_import(script):
    # Some examples catch ImportError themselves and exit with an installation
    # hint. Check their declared optional imports before executing that code.
    tree = ast.parse(script.read_text(encoding="utf-8"), filename=str(script))
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            modules.add(node.module.split(".")[0])
    for module in sorted(modules & OPTIONAL_MODULES):
        if importlib.util.find_spec(module) is None:
            return module
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-missing-dependencies", action="store_true")
    parser.add_argument("script", type=Path)
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    script = args.script.resolve()
    check_gil()
    # Match `python path/to/example.py`, including imports of sibling helpers.
    sys.path[0] = str(script.parent)
    sys.argv = [str(script), *args.arguments]
    if args.skip_missing_dependencies:
        missing = missing_optional_import(script)
        if missing:
            check_gil()
            print(f"SKIP: optional Python dependency {missing!r} unavailable for {script.name}", flush=True)
            return 0
    try:
        runpy.run_path(str(script), run_name="__main__")
    except ModuleNotFoundError as exc:
        if args.skip_missing_dependencies and (exc.name or "").split(".")[0] in OPTIONAL_MODULES:
            print(f"SKIP: optional Python dependency {exc.name!r} unavailable for {script.name}", flush=True)
            return 0  # CTest recognizes the skip message, including through test_wrapper.
        raise
    finally:
        check_gil()
    return 0


if __name__ == "__main__":
    sys.exit(main())
