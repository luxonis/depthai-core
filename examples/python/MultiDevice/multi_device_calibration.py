#!/usr/bin/env python3

from datetime import timedelta
from pathlib import Path

import depthai as dai


device_infos = dai.Device.getAllAvailableDevices()
if len(device_infos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

output_path = Path(__file__).with_name("multi_device_calibration.json")
sockets = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    calibration = pipeline.create(dai.beta.node.MultiDeviceCalibration)
    calibration.setSampleCount(10)
    calibration.sync.setSyncThreshold(timedelta(seconds=5))

    for info in device_infos[:2]:
        device = pipeline.addDevice(info)
        device_id = device.getDeviceId()

        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=5)
            calibration.addCamera(device_id, socket, camera.requestFullResolutionOutput(fps=5))

        calibration.setStereoPair(device_id, *sockets)

    control_queue = calibration.inputControl.createInputQueue()
    result_queue = calibration.calibrationOutput.createOutputQueue()

    print("Point both devices at the same textured scene and keep them still.")
    pipeline.start()
    control_queue.send(dai.MultiDeviceCalibrationControl.start())
    result = result_queue.get(timedelta(minutes=3))

    if result is None or not result.passed or result.handler is None:
        raise RuntimeError(result.info if result is not None else "Calibration timed out")
    if not result.handler.toJsonFile(output_path):
        raise RuntimeError(f"Failed to save calibration to {output_path}")

    print(f"Calibration saved to {output_path}")
    print(f"Confidence: {result.dataConfidence:.3f}")
    print(f"Sampson error: {result.sampsonError:.6g}")
