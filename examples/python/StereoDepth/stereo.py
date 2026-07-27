#!/usr/bin/env python3

import time

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
monoLeftOut = monoLeft.requestOutput(((640, 400)), fps=30)
monoRightOut = monoRight.requestOutput(((640, 400)), fps=30)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)
stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_GPU)
benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
stereo.disparity.link(benchmarkIn.input)

disparityQueue = stereo.disparity.createOutputQueue()
stereoLeftQueue = stereo.rectifiedLeft.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

with pipeline:
    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        rectifiedLeft = stereoLeftQueue.get()
        assert isinstance(disparity, dai.ImgFrame)
        assert isinstance(rectifiedLeft, dai.ImgFrame)
        cv2.imshow("rectified left", rectifiedLeft.getCvFrame())
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
