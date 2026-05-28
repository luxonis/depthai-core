#!/usr/bin/env python3

import sys
import time
from datetime import timedelta

import depthai as dai


def usb_generation_to_string(generation: dai.UsbGeneration) -> str:
    if generation == dai.UsbGeneration.USB_1_0:
        return "USB 1.0"
    if generation == dai.UsbGeneration.USB_1_1:
        return "USB 1.1"
    if generation == dai.UsbGeneration.USB_2_0:
        return "USB 2.0"
    if generation == dai.UsbGeneration.USB_3_0:
        return "USB 3.0"
    if generation == dai.UsbGeneration.USB_3_1:
        return "USB 3.1"
    return "UNKNOWN"


def print_messages(title: str, messages) -> None:
    if not messages:
        return

    print(f"{title}:")
    for key in sorted(messages):
        print(f"  {key}: {messages[key]}")


def main() -> int:
    config = dai.HealthCheckConfig()
    config.timeout = timedelta(milliseconds=10000)
    config.checkUsbSpeed = True
    config.measureBandwidth = True
    config.verifyCameras = True
    config.verifyIMU = True
    config.verifyPowerSupply = True

    found, device_info = dai.Device.getFirstAvailableDevice(False)
    if not found:
        print("No DepthAI device found.", file=sys.stderr)
        return 1

    print(f"Device: {device_info}")
    print(f"Timeout: {int(config.timeout.total_seconds() * 1000)} ms")
    print("Running health check...")

    start = time.monotonic()
    metrics = dai.Device.performHealthCheck(device_info, config)
    elapsed_ms = int((time.monotonic() - start) * 1000)

    print()
    print(f"Health check completed in {elapsed_ms} ms")
    print(f"App running on device: {'true' if metrics.appRunningOnDevice else 'false'}")
    print(f"Device in setup mode: {'true' if metrics.inSetupMode else 'false'}")
    print(f"Udev rules set: {'true' if metrics.udevRulesSet else 'false'}")

    if metrics.usbSpeed is not None:
        print(f"USB speed: {metrics.usbSpeed}")
    else:
        print("USB speed: not detected")

    if metrics.usbGeneration is not None:
        print(f"USB generation: {usb_generation_to_string(metrics.usbGeneration)}")
    else:
        print("USB generation: not detected")

    if metrics.bandwidthMbps is not None:
        print(f"Bandwidth: {metrics.bandwidthMbps} Mbps")
    else:
        print("Bandwidth: not measured")

    print(f"Camera calibration: {'pass' if metrics.cameraCalibration else 'fail' if metrics.cameraCalibration is not None else 'not run'}")
    print(f"IMU calibration: {'pass' if metrics.imuCalibration else 'fail' if metrics.imuCalibration is not None else 'not run'}")
    print(f"Camera functionality: {'pass' if metrics.cameraFunctionality else 'fail' if metrics.cameraFunctionality is not None else 'not run'}")
    print(f"IMU functionality: {'pass' if metrics.imuFunctionality else 'fail' if metrics.imuFunctionality is not None else 'not run'}")
    print(f"Power supply: {'pass' if metrics.powerSupplyFunctionality else 'fail' if metrics.powerSupplyFunctionality is not None else 'not run'}")

    print_messages("Warnings", metrics.warnings)
    print_messages("Errors", metrics.errors)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
