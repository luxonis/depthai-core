#!/usr/bin/env python3

import os
import sys
import signal
import glob
import argparse
import time
import subprocess

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
    "--time-between-examples", type=int, default=3, help="Time between examples in seconds"
)
args = parser.parse_args()

dirs = [dir for dir in os.listdir() if os.path.isdir(dir)]
examples = []
for dir in dirs:
    if "v2_examples" in dir:
        continue
    print(args.folder, dir.lower())

    ex = glob.glob(f"{dir}/**/*.py", recursive=True)

    for example in ex:
        if args.folder and not example.lower().startswith(args.folder.lower()):
            continue
        examples.append(example)

print("Running examples:")
for example in examples:
    print(f"   + {example}")


executable = sys.executable
print("=" * 100)
for example in examples:
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

    time.sleep(args.time_between_examples)
    
    print("=" * 100)