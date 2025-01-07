#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time


# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.Camera).build()
    # Properties

    qRgb = camRgb.requestOutput((256,256)).createOutputQueue()

    eventMan = dai.EventsManager()
    eventMan.setLogResponse(True)

    eventMan.sendEvent("test1", None, [], ["tag1", "tag2"], {"key1": "value1"})
    time.sleep(2)
    fileData = dai.EventData(b'Hello, world!', "hello.txt", "text/plain")
    eventMan.sendEvent("test2", None,  [fileData], ["tag1", "tag2"], {"key1": "value1"})
    pipeline.start()

    frame = None
    counter = 0


    eventSent = False
    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            if not eventSent:
                eventMan.sendSnap("rgb", inRgb, [], ["tag1", "tag2"], {"key1": "value1"})
                eventSent = True

        if frame is not None:
            cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
