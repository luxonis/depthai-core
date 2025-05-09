#!/usr/bin/env python3

import depthai as dai
import time

pipeline = dai.Pipeline()
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
dynamicCalibration = pipeline.create(dai.node.DynamicCalibration)

leftOut = left.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
rightOut = right.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
leftOut.link(dynamicCalibration.left)
rightOut.link(dynamicCalibration.right)

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        quality = dynamicCalibration.getCalibrationQuality()
        print("Calibration quality: ", quality)
        time.sleep(1)