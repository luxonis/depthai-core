#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera)

    # Properties
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    cap = dai.ImgFrameCapability()
    cap.size.fixed((640, 480))
    # cap.type = dai.ImgFrame.Type.BGR888p
    videoQueue = cam.requestOutput(cap, True).createOutputQueue()
    # videoQueue2 = cam.requestOutput((300,300), type=dai.ImgFrame.Type.BGR888i, resizeMode=dai.ImgResizeMode.CROP).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        # videoIn2: dai.ImgFrame = videoQueue2.get()
        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoIn.getCvFrame())
        # cv2.imshow("video2", videoIn2.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
