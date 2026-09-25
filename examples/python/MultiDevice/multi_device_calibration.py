#!/usr/bin/env python3

import argparse
from datetime import timedelta
from pathlib import Path

import depthai as dai

parser = argparse.ArgumentParser(description="Estimate cross-device calibration between two or more devices.")
parser.add_argument(
    "-d",
    "--devices",
    nargs="+",
    default=[],
    help="Device IDs or IPs to calibrate (at least two). Defaults to the first two available devices.",
)
parser.add_argument("-n", "--sample-count", type=int, default=10, help="Synchronized image groups to collect before solving")
parser.add_argument(
    "-o",
    "--output",
    type=Path,
    default=Path(__file__).with_name("multi_device_calibration.json"),
    help="Where to save the calibration JSON",
)
args = parser.parse_args()

if args.devices:
    if len(args.devices) < 2:
        parser.error("at least two devices are required")
    device_infos = [dai.DeviceInfo(d) for d in args.devices]
else:
    device_infos = dai.Device.getAllAvailableDevices()[:2]
    if len(device_infos) < 2:
        print("At least two devices are required for this example.")
        raise SystemExit(0)

sockets = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    calibration = pipeline.create(dai.beta.node.MultiDeviceCalibration)
    calibration.setSampleCount(args.sample_count)
    calibration.sync.setSyncThreshold(timedelta(seconds=5))

    for info in device_infos:
        device = pipeline.addDevice(info)
        device_id = device.getDeviceId()
        print(f"Using device {device_id}")

        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=5)
            calibration.addCamera(device_id, socket, camera.requestFullResolutionOutput(fps=5))

        calibration.setStereoPair(device_id, *sockets)

    control_queue = calibration.inputControl.createInputQueue()
    result_queue = calibration.calibrationOutput.createOutputQueue()

    print("Point all devices at the same textured scene and keep them still.")
    pipeline.start()
    control_queue.send(dai.beta.MultiDeviceCalibrationControl.start())
    result = result_queue.get(timedelta(minutes=3))

    if result is None or not result.passed or result.graph is None:
        raise RuntimeError(result.info if result is not None else "Calibration timed out")
    if not result.getHandler().toJsonFile(args.output):
        raise RuntimeError(f"Failed to save calibration to {args.output}")

    print(f"Calibration saved to {args.output}")
    print(f"Confidence: {result.dataConfidence:.3f}")
    print(f"Sampson error: {result.sampsonError:.6g}")
