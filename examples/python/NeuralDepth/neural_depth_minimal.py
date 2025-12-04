#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

FPS = 10

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    cameraRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)
    leftOutput = cameraLeft.requestFullResolutionOutput()
    rightOutput = cameraRight.requestFullResolutionOutput()

    neuralDepth = pipeline.create(dai.node.NeuralDepth).build(leftOutput, rightOutput, dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)

    disparityQueue = neuralDepth.disparity.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    maxDisparity = 1
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

    while pipeline.isRunning():
        disparityData = disparityQueue.get()
        assert isinstance(disparityData, dai.ImgFrame)
        npDisparity = disparityData.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break

        if cv2.waitKey(1) == ord('q'):
            break
