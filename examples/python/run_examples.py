#!/usr/bin/env python3
import argparse
import pathlib
from typing import List
import glob
import sys
import subprocess
import time
import os
import signal

pythonExamplesDir = pathlib.Path(__file__).parent

# Platform specific dirs are added per platform
dirsToSkip = [
    "models",
    "__pycache__",
    "v2_examples",
    "utilities",
    "downloader",
    "RVC2",
    "RVC4",
]

examplesToSkip = [
    ""
]

RVC2Dirs = ["RVC2"]
RVC4Dirs = ["RVC4"]


def filter_invalid_dirs(dirs: List[pathlib.Path]) -> List[pathlib.Path]:
    # Skip all the directories that start with _ or .
    return [
        dir
        for dir in dirs
        if not dir.name.startswith("_") and not dir.name.startswith(".")
    ]


def dirs_to_run(platform: str):
    # Get all subdirectories of the examples folder
    dirs = [dir for dir in pythonExamplesDir.iterdir() if dir.is_dir()]
    dirs = [dir for dir in dirs if dir.name not in dirsToSkip]
    # Add back in the platform specific dirs
    if platform == "rvc2":
        dirs.extend([pathlib.Path(pythonExamplesDir, dir) for dir in RVC2Dirs])
    elif platform == "rvc4":
        dirs.extend([pathlib.Path(pythonExamplesDir, dir) for dir in RVC4Dirs])
    else:
        raise ValueError(f"Platform {platform} not recognized")
    return filter_invalid_dirs(dirs)


def get_examples_to_run(dirs: List[pathlib.Path]):
    examples = []
    for dir in dirs:
        ex = glob.glob(f"{dir}/**/*.py", recursive=True)
        examples.extend(ex)
    return examples



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--folder",
        type=str,
        default="",
        help="Run all tests in a specific folder. All folders if not specified.",
    )
    parser.add_argument(
        "--timeout", type=int, default=20, help="Timeout for each example in seconds"
    )
    parser.add_argument(
        "--time-between-examples", type=int, default=10, help="Time between examples in seconds"
    )
    parser.add_argument(
        "--platform", type=str, default="rvc2", help="Platform to run examples on"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Print the examples that would be run"
    )
    args = parser.parse_args()
    if args.folder:
        folders = [pathlib.Path(pythonExamplesDir, args.folder)]
    else:
        folders = dirs_to_run(args.platform)
    examples = get_examples_to_run(folders)

    if args.dry_run:
        print("Examples to run:")
        for example in examples:
            print(example)
        exit(0)

    executable = sys.executable
    print("=" * 100)
    for example in examples:
        time.sleep(args.time_between_examples)
        p = None
        try:
            start_time = time.time()
            command = executable + " " + f"{os.path.abspath(example)}"

            p = subprocess.Popen(
                command,
                shell=True,
                cwd=os.path.dirname(example),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            p.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            pass

        success = (p.returncode == 0 or p.returncode is None)
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)  # Send the signal to all the process groups
        except ProcessLookupError:
            pass

        RED = "\033[91m"
        GREEN = "\033[92m"
        RESET = "\033[0m"

        if success:
            print(f"{GREEN}Success running {example}{RESET}")
        else:
            print(f"{RED}Error while in {example}{RESET}")
            print(p.stdout.read() if p.stdout else "")
            print(p.stderr.read() if p.stderr else "")
        print("=" * 100)
