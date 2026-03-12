#!/usr/bin/env python3

import json
import os
import threading
import time

import depthai as dai


def main():
    os.environ["DEPTHAI_CRASH_DEVICE"] = "1"
    os.environ["DEPTHAI_DISABLE_CRASHDUMP_COLLECTION"] = "1"

    received = {"dump": None}
    callback_invoked = threading.Event()

    with dai.Device() as device:
        # Register crash dump callback
        def on_crashdump(dump: dai.CrashDump):
            # Basic crash dump usage
            print("Crash dump callback invoked.")
            print(f"  Platform: {dump.getPlatform()}")
            print(f"  Device ID: {dump.deviceId}")
            print(f"  Crash timestamp: {dump.crashdumpTimestamp}")
            print(f"  DepthAI commit: {dump.depthaiCommitHash}")

            # Advanced extra custom user data usage
            dump.extra["example"] = "python_crash_dump"
            dump.extra["host_time_ms"] = int(time.time() * 1000)

            received["dump"] = dump
            callback_invoked.set()

        device.registerCrashdumpCallback(on_crashdump)

        # Trigger the crash manually. In a real scenario, this would be triggered by an actual device crash.
        print("Triggering crash manually ...")
        device.crashDevice()

        # Wait for the callback to be invoked (with timeout)
        if not callback_invoked.wait(timeout=60):
            raise RuntimeError("Timed out waiting for crash dump callback.")

        # Check if the device crash was detected by hasCrashed() within a reasonable time frame
        crash_detected = False
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if device.hasCrashed():
                crash_detected = True
                break
            time.sleep(0.2)

        if not crash_detected:
            raise RuntimeError("Device crash was not detected by hasCrashed() within timeout.")

        # Print user defined extra data
        print("Crash dump extra data:")
        print(json.dumps(received["dump"].extra, indent=2))


if __name__ == "__main__":
    main()
