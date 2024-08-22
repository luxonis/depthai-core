#!/usr/bin/env python3

import cv2
import depthai as dai

testing = 0
#testing = True

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((640,400)).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        # log dai.imgFrame
        testing += 1
        if testing == 100:
            videoIn.print_for_test()
            #testing = 0

        #if testing:
        #    testing = False
        #    videoIn.print_for_test()


        if cv2.waitKey(1) == ord("q"):
            break
