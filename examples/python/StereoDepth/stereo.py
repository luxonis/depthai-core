#!/usr/bin/env python3

import cv2
import depthai as dai

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

depthQueue = stereo.depth.createOutputQueue()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depth = depthQueue.get()
        assert isinstance(depth, dai.ImgFrame)
        colorizedDepth = dai.utility.colorizeDepthFrame(depth).getCvFrame()
        cv2.imshow("depth", colorizedDepth)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
