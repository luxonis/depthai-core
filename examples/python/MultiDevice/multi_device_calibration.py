#!/usr/bin/env python3
"""Estimate the rig calibration of a multi-device setup.

The per-device calibration (intrinsics, distortion, the pose between CAM_B and CAM_C) is factory calibrated and read
from the device. What is missing in a multi-device setup is the pose *between* the devices, and that is what the
MultiDeviceCalibration node estimates from images of a shared scene.

Two things cannot be observed from images alone and have to be given:
  * a rough initial guess of where each device sits w.r.t. the first one (--guess),
  * the metric scale. The stereo pair of every device provides it here, since the baselines are known; on devices
    without one, call setKnownDistance() for a measured inter-device distance instead.

The result is written as a rig json, which the stitching examples load with Pipeline.setMultiDeviceCalibration().
"""
import argparse
import math
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or device id. The first one is the reference")
parser.add_argument("-n", "--num-devices", type=int, default=2, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-g", "--guess", action="append", nargs=4, type=float, metavar=("YAW", "X", "Y", "Z"),
                    help="Rough pose of a device w.r.t. the first one: yaw around the vertical axis in degrees and translation in cm. "
                         "Repeat once per additional device, in the order the devices are given")
parser.add_argument("-o", "--output", type=Path, default=Path("rig_calibration.json"), help="Where to write the estimated rig")
parser.add_argument("-s", "--samples", type=int, default=10, help="Number of synchronized image sets to estimate from")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(1280, 800), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
args = parser.parse_args()

# The stereo pair of every device: seeing the scene from two cameras of known baseline is what fixes the metric scale
SOCKETS = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)

devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device] if args.device else [dai.Device() for _ in range(args.num_devices)]
if len(devices) < 2:
    raise SystemExit("At least two devices are needed to estimate a rig")

guesses = args.guess or []
if len(guesses) != len(devices) - 1:
    raise SystemExit(f"Expected {len(devices) - 1} --guess arguments, one per device after the first one, got {len(guesses)}")


def pose(yawDegrees: float, x: float, y: float, z: float) -> dai.Extrinsics:
    """Extrinsics of a device rotated around the vertical (Y) axis of the reference camera and moved by (x, y, z) cm."""
    yaw = math.radians(yawDegrees)
    extrinsics = dai.Extrinsics()
    extrinsics.setTransformationMatrix(
        [
            [math.cos(yaw), 0.0, math.sin(yaw), x],
            [0.0, 1.0, 0.0, y],
            [-math.sin(yaw), 0.0, math.cos(yaw), z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dai.LengthUnit.CENTIMETER,
    )
    return extrinsics


with dai.Pipeline(createImplicitDevice=False) as pipeline:
    sources = []
    for device in devices:
        for socket in SOCKETS:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            sources.append((dai.CoordinateFrame(device.getDeviceId(), socket), camera.requestOutput(tuple(args.resolution), fps=args.fps)))

    calibration = pipeline.create(dai.node.MultiDeviceCalibration).build(sources)
    calibration.setSampleCount(args.samples)
    # Free running cameras are not hardware synced, so allow a group to span a couple of frame intervals
    calibration.sync.setSyncThreshold(timedelta(seconds=2.0 / args.fps))

    reference = dai.CoordinateFrame(devices[0].getDeviceId(), SOCKETS[0])
    for device, guess in zip(devices[1:], guesses):
        calibration.setInitialGuess(dai.CoordinateFrame(device.getDeviceId(), SOCKETS[0]), reference, pose(*guess))

    rigQueue = calibration.rigCalibration.createOutputQueue()
    previewQueues = [source.createOutputQueue(maxSize=1, blocking=False) for _, source in sources]

    pipeline.start()
    print(f"Estimating the rig of {len(devices)} devices from {args.samples} image sets - keep a textured, static scene in view of all of them")

    while pipeline.isRunning():
        preview = [queue.get().getCvFrame() for queue in previewQueues]
        preview = [cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image for image in preview]
        cv2.imshow("multi device calibration", cv2.hconcat([cv2.resize(image, (320, 200)) for image in preview]))

        result = rigQueue.tryGet()
        if result is not None:
            print(f"{result.info} (data confidence {result.dataConfidence:.3f})")
            if result.passed:
                handler = dai.MultiDeviceCalibrationHandler(result.calibration)
                for edge in result.calibration.edges:
                    translation = [round(value[3], 1) for value in handler.getTransform(edge.from_, edge.to)[:3]]
                    print(f"  {edge.from_} -> {edge.to}: {translation} cm")
                handler.toJsonFile(args.output)
                print(f"Rig written to {args.output}")
            break

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
