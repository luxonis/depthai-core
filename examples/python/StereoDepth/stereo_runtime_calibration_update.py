#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

device = dai.Device()
calibration = device.getCalibration()
pipeline = dai.Pipeline(device)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
monoLeftOut = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

rectifiedLeftQueue = stereo.rectifiedLeft.createOutputQueue()
rectifiedRightQueue = stereo.rectifiedRight.createOutputQueue()
depthQueue = stereo.depth.createOutputQueue()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        leftRectified = rectifiedLeftQueue.get()
        rightRectified = rectifiedRightQueue.get()
        depth = depthQueue.get()
        assert isinstance(leftRectified, dai.ImgFrame)
        assert isinstance(rightRectified, dai.ImgFrame)
        assert isinstance(depth, dai.ImgFrame)
        cv2.imshow("left", leftRectified.getCvFrame())
        cv2.imshow("right", rightRectified.getCvFrame())
        colorizedDepth = dai.colorizeDepthFrame(depth, 500, 12000, cv2.COLORMAP_JET, useLog=True).getCvFrame()
        cv2.imshow("depth", colorizedDepth)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
        elif key == ord('u'):
            randomDistortionCoeffs = np.random.rand(14)
            calibration.setDistortionCoefficients(dai.CameraBoardSocket.CAM_B, randomDistortionCoeffs)
            calibration.setDistortionCoefficients(dai.CameraBoardSocket.CAM_C, randomDistortionCoeffs)
            try:
                device.setCalibration(calibration)
            except:
                print("Failed to update calibration!")
            try:
                updatedCalib = device.getCalibration()
                distortionCoeffs = updatedCalib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C)
                print("Updated distortion coefficients: ", distortionCoeffs)
            except:
                pass
