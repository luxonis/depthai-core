#!/usr/bin/env python3
"""Dynamic calibration with a metric reference from ArUco markers.

Image features fix the relative rotation and the direction of the stereo baseline, but not its length.
Known distances between marker centers (or from a marker center to a camera socket) supply that length.
Place the markers from the JSON file in view of both cameras and run:

    python3 calibration_dynamic_aruco_scale.py [--constraints aruco_scene_constraints.json] [--host]
"""
import argparse
import pathlib
import time

import depthai as dai

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--constraints", default=str(pathlib.Path(__file__).with_name("aruco_scene_constraints.json")))
parser.add_argument("--host", action="store_true", help="run the DynamicCalibration node on the host")
parser.add_argument("--apply", action="store_true", help="apply the new calibration to the device (RAM only)")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    dynCalib = pipeline.create(dai.node.DynamicCalibration)
    dynCalib.setArucoConstraintsFromFile(args.constraints)
    if args.host:
        dynCalib.setRunOnHost(True)
    left.requestFullResolutionOutput().link(dynCalib.left)
    right.requestFullResolutionOutput().link(dynCalib.right)

    calibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
    coverageQueue = dynCalib.coverageOutput.createOutputQueue()
    control = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    current = device.getCalibration()
    baselineBefore = current.getBaselineDistance(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, True)
    print(f"Baseline CAM_B-CAM_C before: {baselineBefore:.4f} cm")

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
            baselineAfter = new.getBaselineDistance(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, True)
            print(f"Baseline CAM_B-CAM_C after:  {baselineAfter:.4f} cm ({(baselineAfter / baselineBefore - 1) * 100:+.2f} %)")
            if args.apply:
                control.send(dai.DynamicCalibrationControl.applyCalibration(new))
            break
    pipeline.stop()
