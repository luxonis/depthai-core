#!/usr/bin/env python3

import cv2
import depthai as dai



def callback(x: dai.Device.ReconnectionStatus):
    print(f"Reconnecting state {x}")

# Create pipeline
device = dai.Device()
device.setMaxReconnectionAttempts(3, callback)

# You can try unplugging the camera to see the reconnection in action
with dai.Pipeline(device) as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((640,400)).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
