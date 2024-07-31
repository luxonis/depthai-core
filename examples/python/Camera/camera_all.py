#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    outputQueues = {}
    connectedCameras = device.getConnectedCameraFeatures()
    for cameraFeature in connectedCameras:
        cam = pipeline.create(dai.node.Camera)
        cam.setBoardSocket(cameraFeature.socket)
        cap = dai.ImgFrameCapability()
        cap.size.fixed((cameraFeature.width, cameraFeature.height))
        videoQueue = cam.requestOutput(cap, True).createOutputQueue()
        outputQueues[str(cameraFeature.socket) + cameraFeature.name] = videoQueue
    pipeline.start()
    while pipeline.isRunning():
        for name in outputQueues.keys():
            queue = outputQueues[name]
            videoIn = queue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            # Visualizing the frame on slower hosts might have overhead
            cv2.imshow(name, videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
