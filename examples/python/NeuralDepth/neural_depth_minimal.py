#!/usr/bin/env python3

import cv2
import depthai as dai

FPS = 25

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    cameraRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)
    leftOutput = cameraLeft.requestFullResolutionOutput()
    rightOutput = cameraRight.requestFullResolutionOutput()

    neuralDepth = pipeline.create(dai.node.NeuralDepth).build(leftOutput, rightOutput, dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)

    depthQueue = neuralDepth.depth.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        depthData = depthQueue.get()
        assert isinstance(depthData, dai.ImgFrame)
        colorizedDepth = dai.colorizeDepthFrame(depthData, 500, 12000, cv2.COLORMAP_JET, useLog=True).getCvFrame()
        cv2.imshow("depth", colorizedDepth)

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break

        if cv2.waitKey(1) == ord('q'):
            break
