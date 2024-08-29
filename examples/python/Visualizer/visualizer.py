#!/usr/bin/env python3
import depthai as dai
import time
import cv2



remoteConnector = dai.RemoteConnector()
# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    output = cam.requestOutput((640,400))
    videoQueue = output.createOutputQueue()

    remoteConnector.addTopic("video", output, "main")

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
