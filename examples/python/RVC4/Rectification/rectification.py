#!/usr/bin/env python3

import cv2
import depthai as dai

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# Linking
monoLeftOut = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()

rectification = pipeline.create(dai.node.Rectification)
rectification.setRunOnHost(True)
monoLeftOut.link(rectification.input1)
monoRightOut.link(rectification.input2)

rectifiedLeftQueue = rectification.output1.createOutputQueue()
rectifiedRightQueue = rectification.output2.createOutputQueue()
validLeftQueue = rectification.validMask1.createOutputQueue()
validRightQueue = rectification.validMask2.createOutputQueue()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        rectifiedLeft = rectifiedLeftQueue.get()
        rectifiedRight = rectifiedRightQueue.get()
        assert isinstance(rectifiedLeft, dai.ImgFrame)
        assert isinstance(rectifiedRight, dai.ImgFrame)
        cv2.imshow("rectified left", rectifiedLeft.getCvFrame())
        cv2.imshow("rectified right", rectifiedRight.getCvFrame())
        # Masks arrive only when rectification changes; keep displaying the latest one.
        # 255 = usable source data; 0 = border padding, including partially padded pixels.
        validLeft = validLeftQueue.tryGet()
        validRight = validRightQueue.tryGet()
        if validLeft is not None:
            cv2.imshow("usable left", validLeft.getCvFrame())
        if validRight is not None:
            cv2.imshow("usable right", validRight.getCvFrame())

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
