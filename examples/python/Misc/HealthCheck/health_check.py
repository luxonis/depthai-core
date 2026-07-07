#!/usr/bin/env python3

import sys
import time
from datetime import timedelta

import depthai as dai


def main() -> int:
    config = dai.HealthCheckConfig(powerSupplyCheckDuration=timedelta(milliseconds=20000))

    deviceInfos = dai.Device.getAllConnectedDevices()
    if not deviceInfos:
        print("No DepthAI device found.", file=sys.stderr)
        return 1

    deviceInfo = deviceInfos[0]

    print(f"Device: {deviceInfo}")
    print(f"Power supply check duration: {int(config.powerSupplyCheckDuration.total_seconds() * 1000)} ms")
    print("Running health check...")

    start = time.monotonic()
    metrics = dai.Device.performHealthCheck(deviceInfo, config)
    elapsedMs = int((time.monotonic() - start) * 1000)

    print()
    print(f"Health check completed in {elapsedMs} ms")
    print(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
