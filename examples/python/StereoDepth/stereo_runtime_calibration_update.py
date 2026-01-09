#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

device = dai.Device()
calibration = device.readCalibration()
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
disparityQueue = stereo.disparity.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

with pipeline:
    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        leftRectified = rectifiedLeftQueue.get()
        rightRectified = rectifiedRightQueue.get()
        disparity = disparityQueue.get()
        assert isinstance(leftRectified, dai.ImgFrame)
        assert isinstance(rightRectified, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)
        cv2.imshow("left", leftRectified.getCvFrame())
        cv2.imshow("right", rightRectified.getCvFrame())
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)
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

