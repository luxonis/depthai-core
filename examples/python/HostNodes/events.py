
#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time


# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    # Properties
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(15)


    qRgb = camRgb.preview.createOutputQueue()

    eventMan = dai.EventsManager()
    eventMan.setLogResponse(True)
    eventMan.setUrl("https://events-ingest.apps.stg.hubcloud")

    eventMan.sendEvent("test1", None, [], ["tag1", "tag2"], {"key1": "value1"})
    time.sleep(2)
    fileData = dai.EventData(b'Hello, world!', "hello.txt", "text/plain")
    eventMan.sendEvent("test2", None,  [fileData], ["tag1", "tag2"], {"key1": "value1"})
    fileData2 = dai.EventData("/test.txt")
    # will fail, sendSnap needs an image
    eventMan.sendSnap("test3", None, [fileData2], ["tag1", "tag2"], {"key1": "value1"})
    eventMan.sendEvent("test4", None, [fileData, fileData2], ["tag1", "tag2"], {"key1": "value1"})
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
                # will fail, sendSnap requires only image and no extra data
                eventMan.sendSnap("rgb2", inRgb, [fileData2], ["tag1", "tag2"], {"key1": "value1"})
                eventSent = True

        if frame is not None:
            cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
