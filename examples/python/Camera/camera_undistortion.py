#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    croppedQueue = cam.requestOutput((300,300), resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True).createOutputQueue()
    stretchedQueue = cam.requestOutput((300,300), resizeMode=dai.ImgResizeMode.STRETCH, enableUndistortion=True).createOutputQueue()
    letterBoxedQueue = cam.requestOutput((300,300), resizeMode=dai.ImgResizeMode.LETTERBOX, enableUndistortion=True).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        croppedIn = croppedQueue.get()
        assert isinstance(croppedIn, dai.ImgFrame)
        cv2.imshow("cropped undistorted", croppedIn.getCvFrame())

        stretchedIn = stretchedQueue.get()
        assert isinstance(stretchedIn, dai.ImgFrame)
        cv2.imshow("stretched undistorted", stretchedIn.getCvFrame())

        letterBoxedIn = letterBoxedQueue.get()
        assert isinstance(letterBoxedIn, dai.ImgFrame)
        cv2.imshow("letterboxed undistorted", letterBoxedIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
