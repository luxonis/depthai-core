import os
import subprocess
import threading
import argparse
from functools import reduce
import pathlib


# Custom Thread class to capture the result of subprocess.run
class ResultThread(threading.Thread):
    def __init__(self, cmd, env):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.env = env
        self.result = None

    def run(self):
        self.result = subprocess.run(
            self.cmd, env=self.env, capture_output=True, text=True
        )


# Function to run ctest with specific environment variables and labels
def run_ctest(env_vars, labels, blocking=True):
    env = os.environ.copy()
    env.update(env_vars)

    cmd = ["ctest", "--no-tests=error", "-VV", "-L", "^ci$"]
    for label in labels:
        # Encapsulate label in ^label$ to match exactly
        label = f"^{label}$"
        cmd.extend(["-L", label])
        thread = ResultThread(cmd, env)
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
    default_path = pathlib.Path(__file__) / ".." / ".." / "build" / "tests"
    parser.add_argument(
        "--test_dir",
        type=str,
        required=False,
        help="Path to the test directory",
        default=default_path,
    )

    args = parser.parse_args()
    test_dir = args.test_dir
    print("Going to run tests in directory:", test_dir)
    # cd to the test directory
    os.chdir(pathlib.Path(test_dir).resolve())
    # Example test configurations
    test_configs = [
        {
            "name": "Host",
            "env": {},
            "label": ["onhost"],
        },
        {
            "name": "RVC4",
            "env": {"DEPTHAI_PLATFORM": "rvc4"},
            "labels": ["rvc4"],
        },
        {
            "name": "RVC2 - USB",
            "env": {"DEPTHAI_PLATFORM": "rvc2", "DEPTHAI_PROTOCOL": "usb"},
            "label": ["rvc2", "usb"],
        },
        {
            "name": "RVC2 - POE",
            "env": {"DEPTHAI_PLATFORM": "rvc2", "DEPTHAI_PROTOCOL": "tcpip"},
            "label": ["rvc2", "poe"],
        },
    ]

    # List to keep track of results
    resultThreads = []

    for config in test_configs:
        name = config["name"]
        env_vars = config["env"]
        labels = config.get("labels") or config.get("label")

        print(f"Running tests for configuration: {name}")
        resultThread = run_ctest(env_vars, labels, blocking=False)
        resultThreads.append((name, resultThread))

    # Process the results
    any_failures = False
    for name, resultThread in resultThreads:
        resultThread.join()
        result = resultThread.result
        if result.returncode != 0:
            print(f"Tests failed for configuration: {name}")
            print(result.stdout)
            print(result.stderr)
            any_failures = True
        else:
            print(f"Tests passed for configuration: {name}")
            print(result.stdout)

    if any_failures:
        print("Some tests failed")
        exit(1)
    else:
        print("All tests passed")
        exit(0)
