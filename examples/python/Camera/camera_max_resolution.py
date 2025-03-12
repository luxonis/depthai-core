#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    # In some cases (IMX586), this requires an 8k screen to be able to see the full resolution at once
    videoQueue = cam.requestFullResolutionOutput(useHighestResolution=True).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
