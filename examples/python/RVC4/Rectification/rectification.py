#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
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

syncedLeftQueue = stereo.rectifiedLeft.createOutputQueue()
syncedRightQueue = stereo.rectifiedRight.createOutputQueue()
disparityQueue = stereo.disparity.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

rectification = pipeline.create(dai.node.Rectification)
monoLeftOut.link(rectification.input1)
monoRightOut.link(rectification.input2)

rectifiedLeftQueue = rectification.output1.createOutputQueue()
rectifiedRightQueue = rectification.output2.createOutputQueue()
rectification.setRunOnHost(True)

with pipeline:
    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        leftSynced = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        disparity = disparityQueue.get()
        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)
        cv2.imshow("left", leftSynced.getCvFrame())
        cv2.imshow("right", rightSynced.getCvFrame())
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)

        rectifiedLeft = rectifiedLeftQueue.get()
        rectifiedRight = rectifiedRightQueue.get()
        assert isinstance(rectifiedLeft, dai.ImgFrame)
        assert isinstance(rectifiedRight, dai.ImgFrame)
        cv2.imshow("rectified left", rectifiedLeft.getCvFrame())
        cv2.imshow("rectified right", rectifiedRight.getCvFrame())

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
