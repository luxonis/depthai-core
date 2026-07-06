#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

FPS = 25
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    cameraRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)
    leftOutput = cameraLeft.requestFullResolutionOutput()
    rightOutput = cameraRight.requestFullResolutionOutput()

    neuralDepth = pipeline.create(dai.node.NeuralDepth).build(leftOutput, rightOutput, dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)
    neuralDepth.inputConfig.setBlocking(False)

    confidenceQueue = neuralDepth.confidence.createOutputQueue()
    edgeQueue = neuralDepth.edge.createOutputQueue()
    disparityQueue = neuralDepth.disparity.createOutputQueue()
    depthQueue = neuralDepth.depth.createOutputQueue()

    inputConfigQueue = neuralDepth.inputConfig.createInputQueue()
    currentConfig = neuralDepth.initialConfig
    currentConfig.postProcessing.thresholdFilter.maxRange = 10000

    # Connect to device and start pipeline
    pipeline.start()
    maxDisparity = 1
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
    print("For adjusting thresholds, use keys:")
    print(" - 'w': Increase confidence threshold")
    print(" - 's': Decrease confidence threshold")
    print(" - 'd': Increase edge threshold")
    print(" - 'a': Decrease edge threshold")
    print(" - 'i': Increase threshold filter max range")
    print(" - 'k': Decrease threshold filter max range")
    print(" - 'l': Increase threshold filter min range")
    print(" - 'j': Decrease threshold filter min range")
    print(" - 't': Toggle temporal filtering")
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

        depthData = depthQueue.get()
        assert isinstance(depthData, dai.ImgFrame)
        npDepth = depthData.getFrame()
        maxRange = max(currentConfig.postProcessing.thresholdFilter.maxRange, 1)
        depthFrame = np.clip((npDepth / maxRange) * 255, 0, 255).astype(np.uint8)
        colorizedDepth = cv2.applyColorMap(depthFrame, colorMap)
        cv2.imshow("depth", colorizedDepth)

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
        if key == ord('t'):
            currentConfig.postProcessing.temporalFilter.enable = not currentConfig.postProcessing.temporalFilter.enable
            print("Temporal filtering:", "on" if currentConfig.postProcessing.temporalFilter.enable else "off")
            inputConfigQueue.send(currentConfig)
        if key == ord('i'):
            currentMaxRange = currentConfig.postProcessing.thresholdFilter.maxRange
            currentConfig.postProcessing.thresholdFilter.maxRange = min(currentMaxRange + 100, 65535)
            print("Setting threshold filter max range to:", currentConfig.postProcessing.thresholdFilter.maxRange)
            inputConfigQueue.send(currentConfig)
        if key == ord('k'):
            currentMaxRange = currentConfig.postProcessing.thresholdFilter.maxRange
            minRange = currentConfig.postProcessing.thresholdFilter.minRange
            currentConfig.postProcessing.thresholdFilter.maxRange = max(currentMaxRange - 100, minRange)
            print("Setting threshold filter max range to:", currentConfig.postProcessing.thresholdFilter.maxRange)
            inputConfigQueue.send(currentConfig)
        if key == ord('l'):
            currentMinRange = currentConfig.postProcessing.thresholdFilter.minRange
            maxRange = currentConfig.postProcessing.thresholdFilter.maxRange
            currentConfig.postProcessing.thresholdFilter.minRange = min(currentMinRange + 100, maxRange)
            print("Setting threshold filter min range to:", currentConfig.postProcessing.thresholdFilter.minRange)
            inputConfigQueue.send(currentConfig)
        if key == ord('j'):
            currentMinRange = currentConfig.postProcessing.thresholdFilter.minRange
            currentConfig.postProcessing.thresholdFilter.minRange = max(currentMinRange - 100, 0)
            print("Setting threshold filter min range to:", currentConfig.postProcessing.thresholdFilter.minRange)
            inputConfigQueue.send(currentConfig)
