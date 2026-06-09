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


def health_check_result_to_string(result: dai.HealthCheckResult) -> str:
    if result == dai.HealthCheckResult.PASS:
        return "pass"
    if result == dai.HealthCheckResult.FAIL:
        return "fail"
    return "not run"


def print_messages(title: str, messages) -> None:
    if not messages:
        return

    print(f"{title}:")
    for key in sorted(messages):
        print(f"  {key}: {messages[key]}")


def main() -> int:
    config = dai.HealthCheckConfig()
    config.checkUsbSpeed = True
    config.measureBandwidth = True
    config.verifyCameraFunctionality = True
    config.verifyCameraCalibration = True
    config.verifyImuFunctionality = True
    config.verifyImuCalibration = True
    config.verifyPowerSupply = True
    config.powerSupplyCheckDuration = timedelta(milliseconds=10000)

    found, device_info = dai.Device.getFirstAvailableDevice(False)
    if not found:
        print("No DepthAI device found.", file=sys.stderr)
        return 1

    print(f"Device: {device_info}")
    print(f"Power supply check duration: {int(config.powerSupplyCheckDuration.total_seconds() * 1000)} ms")
    print("Running health check...")

    start = time.monotonic()
    metrics = dai.Device.performHealthCheck(device_info, config)
    elapsed_ms = int((time.monotonic() - start) * 1000)

    print()
    print(f"Health check completed in {elapsed_ms} ms")
    print(f"App running on device: {'true' if metrics.appRunningOnDevice else 'false'}")
    print(f"Device in setup mode: {'true' if metrics.inSetupMode else 'false'}")
    print(f"Udev rules set: {'true' if metrics.udevRulesSet else 'false'}")
    print(f"USB generation: {usb_generation_to_string(metrics.usbGeneration)}")
    print(f"Bandwidth: {metrics.bandwidthMbps} Mbps")
    print(f"Camera calibration: {health_check_result_to_string(metrics.cameraCalibration)}")
    print(f"IMU calibration: {health_check_result_to_string(metrics.imuCalibration)}")
    print(f"Camera functionality: {health_check_result_to_string(metrics.cameraFunctionality)}")
    print(f"IMU functionality: {health_check_result_to_string(metrics.imuFunctionality)}")
    print(f"Power supply: {health_check_result_to_string(metrics.powerSupplyFunctionality)}")

    print_messages("Warnings", metrics.warnings)
    print_messages("Errors", metrics.errors)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
