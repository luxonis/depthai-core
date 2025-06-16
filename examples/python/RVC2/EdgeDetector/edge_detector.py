#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define cameras
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# Request outputs
frameWidth, frameHeight = 1920, 1080
rgbOut = camRgb.requestOutput((frameWidth, frameHeight), type=dai.ImgFrame.Type.GRAY8)
leftOut = monoLeft.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8)
rightOut = monoRight.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8)

# Define edge detectors
edgeDetectorLeft = pipeline.create(dai.node.EdgeDetector)
edgeDetectorRight = pipeline.create(dai.node.EdgeDetector)
edgeDetectorRgb = pipeline.create(dai.node.EdgeDetector)
edgeDetectorRgb.setMaxOutputFrameSize(frameWidth * frameHeight)

# Create input queues
edgeCfgLeftQueue = edgeDetectorLeft.inputConfig.createInputQueue()
edgeCfgRightQueue = edgeDetectorRight.inputConfig.createInputQueue()
edgeCfgRgbQueue = edgeDetectorRgb.inputConfig.createInputQueue()

# Link camera outputs to edge detectors
leftOut.link(edgeDetectorLeft.inputImage)
rightOut.link(edgeDetectorRight.inputImage)
rgbOut.link(edgeDetectorRgb.inputImage)

# Create output queues
edgeLeftQueue = edgeDetectorLeft.outputImage.createOutputQueue()
edgeRightQueue = edgeDetectorRight.outputImage.createOutputQueue()
edgeRgbQueue = edgeDetectorRgb.outputImage.createOutputQueue()

# Start pipeline
pipeline.start()
print("Switch between sobel filter kernels using keys '1' and '2'")
while pipeline.isRunning():
    edgeLeft = edgeLeftQueue.get()
    edgeRight = edgeRightQueue.get()
    edgeRgb = edgeRgbQueue.get()

    # Convert to OpenCV format
    cv2.imshow("edge left", edgeLeft.getCvFrame())
    cv2.imshow("edge right", edgeRight.getCvFrame())
    cv2.imshow("edge rgb", edgeRgb.getCvFrame())

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    if key == ord('1'):
        print("Switching sobel filter kernel.")
        cfg = dai.EdgeDetectorConfig()
        sobelHorizontalKernel = [[1, 0, -1], [2, 0, -2], [1, 0, -1]]
        sobelVerticalKernel = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]]
        cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel)
        edgeCfgLeftQueue.send(cfg)
        edgeCfgRightQueue.send(cfg)
        edgeCfgRgbQueue.send(cfg)

    if key == ord('2'):
        print("Switching sobel filter kernel.")
        cfg = dai.EdgeDetectorConfig()
        sobelHorizontalKernel = [[3, 0, -3], [10, 0, -10], [3, 0, -3]]
        sobelVerticalKernel = [[3, 10, 3], [0, 0, 0], [-3, -10, -3]]
        cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel)
        edgeCfgLeftQueue.send(cfg)
        edgeCfgRightQueue.send(cfg)
        edgeCfgRgbQueue.send(cfg)
