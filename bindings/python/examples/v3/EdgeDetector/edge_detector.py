#!/usr/bin/env python3

import cv2
import depthai as dai


def init_configs():
    cfg1 = dai.EdgeDetectorConfig()
    sobelHorizontalKernel1 = [[1, 0, -1], [2, 0, -2], [1, 0, -1]]
    sobelVerticalKernel1 = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]]
    cfg1.setSobelFilterKernels(sobelHorizontalKernel1, sobelVerticalKernel1)

    cfg2 = dai.EdgeDetectorConfig()
    sobelHorizontalKernel2 = [[3, 0, -3], [10, 0, -10], [3, 0, -3]]
    sobelVerticalKernel2 = [[3, 10, 3], [0, 0, 0], [-3, -10, -3]]
    cfg2.setSobelFilterKernels(sobelHorizontalKernel2, sobelVerticalKernel2)

    return {'1': cfg1, '2': cfg2}

CONFIGS = init_configs()


with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera('left')

    monoRight = pipeline.create(dai.node.MonoCamera)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera('right')

    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)

    edgeDetectorLeft = pipeline.create(dai.node.EdgeDetector)
    monoLeft.out.link(edgeDetectorLeft.inputImage)

    edgeDetectorRight = pipeline.create(dai.node.EdgeDetector)
    monoRight.out.link(edgeDetectorRight.inputImage)

    edgeDetectorRgb = pipeline.create(dai.node.EdgeDetector)
    edgeDetectorRgb.setMaxOutputFrameSize(camRgb.getVideoWidth() * camRgb.getVideoHeight())
    camRgb.video.link(edgeDetectorRgb.inputImage)

    edgeDetectors = [edgeDetectorLeft, edgeDetectorRight, edgeDetectorRgb]

    edgeLeftQueue = edgeDetectorLeft.outputImage.createQueue(blocking=False, maxSize=8)
    edgeRightQueue = edgeDetectorRight.outputImage.createQueue(blocking=False, maxSize=8)
    edgeRgbQueue = edgeDetectorRgb.outputImage.createQueue(blocking=False, maxSize=8)
    videoQueue = camRgb.video.createQueue(blocking=False, maxSize=8)

    pipeline.start()
    while pipeline.isRunning():
        edgeLeftFrame : dai.ImgFrame = edgeLeftQueue.get()
        edgeRightFrame : dai.ImgFrame = edgeRightQueue.get()
        edgeRgbFrame : dai.ImgFrame = edgeRgbQueue.get()
        videoFrame : dai.ImgFrame = videoQueue.get()

        cv2.imshow('left edges', edgeLeftFrame.getCvFrame())
        cv2.imshow('right edges', edgeRightFrame.getCvFrame())
        cv2.imshow('rgb edges', edgeRgbFrame.getCvFrame())
        cv2.imshow('rgb orig', videoFrame.getCvFrame())

        key = cv2.waitKey(1)
        try:
            key = chr(key)
        except ValueError:
            pass

        if key == 'q':
            break
        elif key in CONFIGS:
            print(f'Switching Sobel filter to {key}...')
            for edgeDetector in edgeDetectors:
                edgeDetector.inputConfig.send(CONFIGS[key])

    pipeline.stop()
    pipeline.wait()
