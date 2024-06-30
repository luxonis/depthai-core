#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline(True) as pipeline:
    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)

    # Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setVideoSize(1920, 1080)

    videoQueue = camRgb.video.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn : dai.ImgFrame = videoQueue.get()

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
    pipeline.stop()
