import os
import subprocess
import threading
import argparse
from functools import reduce
import pathlib


class ResultThread(threading.Thread):

    def __init__(self, cmd, env, name):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.env = env
        self.name = name
        self.result = None
        self.stdout_lines = []
        self.stderr_lines = []

    def run(self):
        process = subprocess.Popen(
            self.cmd,
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        # Capture stdout in real-time
        while True:
            output = process.stdout.readline()
            if output == "" and process.poll() is not None:
                break
            if output:
                print(f"[{self.name}] {output.strip()}")
                self.stdout_lines.append(output.strip())

        # Capture stderr in real-time
        stderr_output, _ = process.communicate()
        if stderr_output:
            print(f"[{self.name} ERROR] {stderr_output.strip()}")
            self.stderr_lines.append(stderr_output.strip())

        self.result = process

# Function to run ctest with specific environment variables and labels
def run_ctest(env_vars, labels, blocking=True, name=""):
    env = os.environ.copy()
    env_vars["DEPTHAI_PIPELINE_DEBUGGING"] = "1"
    env.update(env_vars)

    cmd = ["ctest", "--no-tests=error", "-VV", "-L", "^ci$", "--timeout", "1000", "-C", "Release"]

    if os.name == "nt":
        cmd.extend(["-LE", "^nowindows$"])

    for label in labels:
        # Encapsulate label in ^label$ to match exactly
        label = f"^{label}$"
        cmd.extend(["-L", label])
    thread = ResultThread(cmd, env, name)
    thread.start()
    if blocking:
        thread.join()
    return thread


def compute_or_result(results):
    if results:
        return reduce(lambda x, y: x | y, results)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # Positional argument for the path to the test directory
    default_path = pathlib.Path(__file__) / ".." / ".." / "build"
    parser.add_argument(
        "--test_dir",
        type=str,
        required=False,
        help="Path to the test directory",
        default=default_path,
    )

    parser.add_argument(
        "--rvc4",
        action="store_true",
        required=False,
    )
    
    parser.add_argument(
        "--rvc4rgb",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--rvc2",
        action="store_true",
        required=False,
    )

    args = parser.parse_args()
    test_dir = args.test_dir
    print("Going to run tests in directory:", test_dir)
    # cd to the test directory
    os.chdir(pathlib.Path(test_dir).resolve())
    # Example test configurations
    all_configs = [
        {
            "name": "Host",
            "env": {},
            "labels": ["onhost"],
        },
        {
            "name": "RVC4",
            "env": {"DEPTHAI_PLATFORM": "rvc4"},
            "labels": ["rvc4"],
        },
        {
            "name": "RVC4 - RGB",
            "env": {"DEPTHAI_PLATFORM": "rvc4"},
            "labels": ["rvc4rgb"],
        },
        {
            "name": "RVC2 - USB",
            "env": {"DEPTHAI_PLATFORM": "rvc2", "DEPTHAI_PROTOCOL": "usb"},
            "labels": ["rvc2", "usb"],
        },
        {
            "name": "RVC2 - POE",
            "env": {"DEPTHAI_PLATFORM": "rvc2", "DEPTHAI_PROTOCOL": "tcpip"},
            "labels": ["rvc2", "poe"],
        },
    ]

    # List to keep track of results
    resultThreads = []

    # Filter configurations based on command-line arguments
    if args.rvc4==args.rvc2==args.rvc4rgb:
        test_configs = [config for config in all_configs if "rvc2" in config.get("labels", []) or "rvc4" in config.get("labels", []) or "onhost" in config.get("labels", [])]
    elif args.rvc4:
        test_configs = [config for config in all_configs if "rvc4" in config.get("labels", [])]
    elif args.rvc2:
        test_configs = [config for config in all_configs if "rvc2" in config.get("labels", []) or "onhost" in config.get("labels", [])]
    elif args.rvc4rgb:
        test_configs = [config for config in all_configs if "rvc4rgb" in config.get("labels", [])]


    for config in test_configs:
        name = config["name"]
        env_vars = config["env"]
        labels = config.get("labels")

        print(f"Running tests for configuration: {name}")
        resultThread = run_ctest(env_vars, labels, blocking=False, name=name)
        resultThreads.append((name, resultThread))

    # Process the results
    any_failures = False
    for name, resultThread in resultThreads:
        resultThread.join()
        result = resultThread.result
        if result.returncode != 0:
            print(f"Tests failed for configuration: {name}")
            any_failures = True
        else:
            print(f"Tests passed for configuration: {name}")

    if any_failures:
        print("Some tests failed")
        exit(1)
    else:
        print("All tests passed")
        exit(0)
