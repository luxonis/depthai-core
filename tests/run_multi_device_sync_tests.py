#!/usr/bin/env python3

import adbutils
import argparse
import datetime
import time
import pathlib
import os
import subprocess
import signal
from threading import Event
from threading import Thread

interrupted = Event()

def signal_handler(signal, frame):
    interrupted.set()
    print("Received SIGINT, exiting...")

class ResultThread(Thread):

    def __init__(self, cmd, env, name):
        Thread.__init__(self)
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
            stderr=subprocess.STDOUT,
            text=True,
        )

        for output in process.stdout:
            line = output.rstrip()
            print(f"[{self.name}] {line}", flush=True)
            self.stdout_lines.append(line)

        process.wait()
        self.result = process

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fsync",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--ptp",
        action="store_true",
        required=False,
    )

    args = parser.parse_args()
    num_devices = 4
    if args.fsync:
        test_executable = "multi_device_fsync_test"
    elif args.ptp:
        test_executable = "multi_device_ptp_test"
    else:
        raise RuntimeError("Must specify either --fsync or --ptp")

    signal.signal(signal.SIGINT, signal_handler)

    timeout_sec = 5*60 # 5 minutes

    print(f"Waiting for all {num_devices} devices to come online...")
    start_time = datetime.datetime.now()
    while True:
        if interrupted.is_set():
            raise RuntimeError("Interrupted by SIGINT")
        devices = adbutils.adb.device_list()
        if len(devices) == num_devices:
            break
        if datetime.datetime.now() - start_time > datetime.timedelta(seconds=timeout_sec):
            raise RuntimeError("Timeout waiting for all devices to come online")
        time.sleep(1)

    print("All devices are online")
    try:
        print("Sleeping 2 minutes for PTP to settle")

        if interrupted.wait(120):
            raise RuntimeError("Interrupted by SIGINT")

        try:
            print("Running tests...")
            envvars = os.environ.copy()
            envvars["DEPTHAI_PROTOCOL"] = "tcpip"

            # Eight PTP cases budget 2360 seconds across phases; allow setup/teardown headroom.
            test_timeout_sec = 3600

            default_path = pathlib.Path(__file__) / ".." / ".." / "build"
            print("Going to run tests in directory:", default_path)
            print("abs path:", pathlib.Path(default_path).resolve())
            os.chdir(pathlib.Path(default_path).resolve())

            cmd = [
                "ctest",
                "--no-tests=error",
                "-VV",
                "-R",
                f"^({test_executable})$",
                "--timeout",
                str(test_timeout_sec),
                "-C",
                "Release",
                "--test-output-size-failed",
                "500000",
                "--test-output-truncation",
                "tail",
            ]

            thread = ResultThread(cmd, envvars, test_executable)
            thread.start()
            thread.join()
            if thread.result.returncode != 0:
                raise RuntimeError(f"Failed to run tests: {thread.stderr_lines}")
        except Exception as e:
            print(f"Failed to run tests: {e}")
            raise e
    except Exception as e:
        print(f"Failed to run tests: {e}")
        raise e

if __name__ == "__main__":
    main()
