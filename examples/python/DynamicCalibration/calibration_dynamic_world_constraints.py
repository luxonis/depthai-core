#!/usr/bin/env python3
"""Dynamic calibration with a metric reference from markers in the scene.

Image features fix the relative pose of the stereo pair except for its vergence, the turn about the axis
perpendicular to the baseline and the optical axis, which sets the depth scale while barely changing the
epipolar error. Known distances between marker centers (or from a marker center to a camera socket) and known
marker sizes fix that turn; camera centers and the baseline length are never changed.
Place the markers from the JSON file (ArUco by default; world_constraints_luxonis.json uses Luxonis Markers, the
Luxonis lens logo printed as a concentric-ring fiducial with ids 0-5) in view of both cameras and run:

    python3 calibration_dynamic_world_constraints.py [--constraints world_constraints.json] [--host]
"""
import argparse
import pathlib
import time

import depthai as dai

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--constraints", default=str(pathlib.Path(__file__).with_name("world_constraints.json")))
parser.add_argument("--host", action="store_true", help="run the DynamicCalibration node on the host")
parser.add_argument("--apply", action="store_true", help="apply the new calibration to the device (RAM only)")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    dynCalib = pipeline.create(dai.node.DynamicCalibration)
    dynCalib.setWorldConstraintsFromFile(args.constraints)
    if args.host:
        dynCalib.setRunOnHost(True)
    left.requestFullResolutionOutput().link(dynCalib.left)
    right.requestFullResolutionOutput().link(dynCalib.right)

    calibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
    coverageQueue = dynCalib.coverageOutput.createOutputQueue()
    control = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    current = device.getCalibration()

    pipeline.start()
    time.sleep(1)
    control.send(dai.DynamicCalibrationControl.setPerformanceMode(dai.DynamicCalibrationControl.PerformanceMode.OPTIMIZE_PERFORMANCE))
    control.send(dai.DynamicCalibrationControl.startCalibration())

    while pipeline.isRunning():
        coverage = coverageQueue.tryGet()
        if coverage is not None:
            print(f"coverage {coverage.meanCoverage:.1f} %, data {coverage.dataAcquired:.1f} %")
        result = calibrationQueue.tryGet()
        if result is None:
            time.sleep(0.1)
            continue
        print(f"Dynamic calibration status: {result.info}")
        if result.calibrationData:
            new = result.calibrationData.newCalibration
            diff = result.calibrationData.calibrationDifference
            for (socketA, socketB), rotation in diff.pairwiseRotationDifference.items():
                print(f"Rotation change {socketA}-{socketB}: {tuple(round(float(r), 4) for r in rotation)} deg")
            if args.apply:
                control.send(dai.DynamicCalibrationControl.applyCalibration(new))
            break
    pipeline.stop()
