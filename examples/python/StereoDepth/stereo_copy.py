#!/usr/bin/env python3

import cv2
import depthai as dai
import time

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
rgbCamera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)


# Linking
monoLeftOut = monoLeft.requestOutput(size=(320, 240), type=dai.ImgFrame.Type.NV12, fps=13.3)
monoRightOut = monoRight.requestOutput(size=(320, 240), type=dai.ImgFrame.Type.NV12, fps=13.3)
videoOut = rgbCamera.requestOutput(size=(320, 240), type=dai.ImgFrame.Type.NV12, fps=3.0)

qmono_left = monoLeftOut.createOutputQueue(blocking=False)
qmono_right = monoRightOut.createOutputQueue(blocking=False)
qvideo = videoOut.createOutputQueue(blocking=False)

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        time.sleep(0.05)

        if qmono_left.has():
            mono_left = qmono_left.get()
            cv2.imshow("mono_left", mono_left.getCvFrame())
            print("mono_left")
        if qmono_right.has():
            mono_right = qmono_right.get()
            cv2.imshow("mono_right", mono_right.getCvFrame())
            print("mono_right")
        if qvideo.has():
            video = qvideo.get()
            print("video")
            cv2.imshow("video", video.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
