#!/usr/bin/env python3
"""Check whether the connected device supports GPUStereo (OpenCL GPU available)."""

import argparse
import depthai as dai


parser = argparse.ArgumentParser(description="Check GPUStereo support on a device")
parser.add_argument("--device", "-d", type=str, default=None, help="Device IP address (default: auto-discover)")
args = parser.parse_args()

device = dai.Device(args.device) if args.device else dai.Device()

print("Platform:", device.getPlatformAsString())
print("GPUStereo supported:", device.isGpuStereoSupported())

