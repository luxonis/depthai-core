#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()
# Define source and output
tof = pipeline.create(dai.node.ToF).build()
depthQueue = tof.depth.createOutputQueue()

with pipeline:

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        depth = depthQueue.get()
        assert isinstance(depth, dai.ImgFrame)
        cv2.imshow("depth", depth.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
