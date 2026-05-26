#!/usr/bin/env python3
"""Print basic device GPU availability info."""

import argparse

import depthai as dai


parser = argparse.ArgumentParser(description="Check device GPU availability")
parser.add_argument("--device", "-d", type=str, default=None, help="Device IP address (default: auto-discover)")
args = parser.parse_args()

device = dai.Device(args.device) if args.device else dai.Device()

print("Product:", device.getProductName())
print("Platform:", device.getPlatformAsString())
print("GPU available:", device.hasGPU())

