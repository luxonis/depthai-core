#!/usr/bin/env python3
"""Reproduce slow RVC2 PoE recovery after an ungraceful host exit.

Run with an idle camera: python tcp_host_disconnect.py --device <MXID-or-IP>
Optionally add --firmware /path/to/depthai.cmd to compare firmware images.
The child is killed intentionally; no persistent firmware/calibration is written.
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="Idle RVC2 PoE camera MXID or IP")
    parser.add_argument("--firmware", type=Path, help="Optional RAM-loaded firmware image")
    parser.add_argument("--timeout", type=float, default=30, help="Recovery limit in seconds (default: 30)")
    parser.add_argument("--child-ready", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.firmware:
        if not args.firmware.is_file():
            parser.error("--firmware must point to an existing firmware image")
        os.environ["DEPTHAI_DEVICE_BINARY"] = str(args.firmware.resolve())

    import depthai as dai

    if args.child_ready:
        with dai.Device(dai.DeviceInfo(args.device)) as device:
            if device.getPlatform() != dai.Platform.RVC2 or device.getDeviceInfo().protocol != dai.XLinkProtocol.X_LINK_TCP_IP:
                raise RuntimeError("This reproducer requires an RVC2 PoE camera")
            device.setLogLevel(dai.LogLevel.INFO)
            device.setSystemInformationLoggingRate(1)
            marker = args.child_ready.with_suffix(".tmp")
            marker.write_text(device.getDeviceInfo().getDeviceId())
            marker.replace(args.child_ready)
            time.sleep(120)
        return

    with tempfile.TemporaryDirectory(prefix="depthai-tcp-disconnect-") as temp:
        ready = Path(temp) / "ready"
        log = Path(temp) / "owner.log"
        command = [sys.executable, str(Path(__file__).resolve()), "--device", args.device, "--child-ready", str(ready)]
        # The firmware override is inherited before the child imports DepthAI.
        with log.open("w") as output:
            child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 60
                while not ready.exists() and child.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not ready.exists() or child.poll() is not None:
                    raise RuntimeError("Device owner did not become ready:\n" + log.read_text(errors="replace"))
                device_id = ready.read_text()
                print(f"Opened {device_id}; killing host with system logging enabled", flush=True)
                time.sleep(2)
                started = time.monotonic()
                child.kill()
                child.wait(timeout=5)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5)

        deadline = started + args.timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                devices = dai.Device.getAllConnectedDevices()
            except RuntimeError as error:
                # Explicit-IP discovery can fail while the camera is resetting.
                last_error = error
                devices = []
            for info in devices:
                if info.getDeviceId() == device_id and info.state == dai.XLinkDeviceState.X_LINK_BOOTLOADER:
                    elapsed = time.monotonic() - started
                    if elapsed >= args.timeout:
                        break
                    print(f"Recovered to bootloader in {elapsed:.2f}s", flush=True)
                    with dai.Device(info) as reopened:
                        assert reopened.getDeviceInfo().getDeviceId() == device_id
                    print("Reopen succeeded")
                    return
            time.sleep(0.2)
        raise RuntimeError(f"{device_id} did not recover within {args.timeout:g}s; last error: {last_error}")


if __name__ == "__main__":
    main()
