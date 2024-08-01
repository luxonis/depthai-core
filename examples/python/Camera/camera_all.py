#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    outputQueues = {}
    sockets = device.getConnectedCameras()
    for socket in sockets:
        cam = pipeline.create(dai.node.Camera).build(socket)
        outputQueues[str(socket)] = cam.requestFullResolutionOutput().createOutputQueue()

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
