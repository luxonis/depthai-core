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

    confidenceQueue = neuralDepth.confidence.createOutputQueue()
    edgeQueue = neuralDepth.edge.createOutputQueue()
    disparityQueue = neuralDepth.disparity.createOutputQueue()

    inputConfigQueue = neuralDepth.inputConfig.createInputQueue()
    # Connect to device and start pipeline
    pipeline.start()
    maxDisparity = 1
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
    currentConfig = neuralDepth.initialConfig
    print("For adjusting thresholds, use keys:")
    print(" - 'w': Increase confidence threshold")
    print(" - 's': Decrease confidence threshold")
    print(" - 'd': Increase edge threshold")
    print(" - 'a': Decrease edge threshold")
    while pipeline.isRunning():
        confidenceData = confidenceQueue.get()
        assert isinstance(confidenceData, dai.ImgFrame)
        npConfidence = confidenceData.getFrame()
        colorizedConfidence = cv2.applyColorMap(((npConfidence)).astype(np.uint8), colorMap)
        cv2.imshow("confidence", colorizedConfidence)

        edgeData = edgeQueue.get()
        assert isinstance(edgeData, dai.ImgFrame)
        npEdge = edgeData.getFrame()
        colorizedEdge = cv2.applyColorMap(((npEdge)).astype(np.uint8), colorMap)
        cv2.imshow("edge", colorizedEdge)


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
        if key == ord('w'):
            currentThreshold = currentConfig.getConfidenceThreshold()
            currentConfig.setConfidenceThreshold((currentThreshold + 5) % 255)
            print("Setting confidence threshold to:", currentConfig.getConfidenceThreshold())
            inputConfigQueue.send(currentConfig)
        if key == ord('s'):
            currentThreshold = currentConfig.getConfidenceThreshold()
            currentConfig.setConfidenceThreshold((currentThreshold - 5) % 255)
            print("Setting confidence threshold to:", currentConfig.getConfidenceThreshold())
            inputConfigQueue.send(currentConfig)
        if key == ord('d'):
            currentThreshold = currentConfig.getEdgeThreshold()
            currentConfig.setEdgeThreshold((currentThreshold + 1) % 255)
            print("Setting edge threshold to:", currentConfig.getEdgeThreshold())
            inputConfigQueue.send(currentConfig)
        if key == ord('a'):
            currentThreshold = currentConfig.getEdgeThreshold()
            currentConfig.setEdgeThreshold((currentThreshold - 1) % 255)
            print("Setting edge threshold to:", currentConfig.getEdgeThreshold())
            inputConfigQueue.send(currentConfig)

        if cv2.waitKey(1) == ord('q'):
            break
