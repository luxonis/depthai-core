#!/usr/bin/env python3

import sys
import time
from datetime import timedelta

import depthai as dai


def main() -> int:
    config = dai.HealthCheckConfig(powerSupplyCheckDuration=timedelta(milliseconds=20000))

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
    print(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
