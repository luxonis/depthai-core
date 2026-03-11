#!/usr/bin/env python3

import cv2
import depthai as dai
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--source", default="recordings/recording.tar", help="Recording path")
args = parser.parse_args()

# Create pipeline
with dai.Pipeline(True) as pipeline:
    # Define source and output
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camRgbOut = camRgb.requestOutput((1280, 800), fps=30, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP)
    camb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    cambOutput = camb.requestOutput((1280, 800), fps=30, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP)

    pipeline.enableHolisticReplay(args.source)

    videoQueue = camRgbOut.createOutputQueue()
    cambQueue = cambOutput.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn : dai.ImgFrame = videoQueue.get()
        cambIn : dai.ImgFrame = cambQueue.get()

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoIn.getCvFrame())
        cv2.imshow("camb", cambIn.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
