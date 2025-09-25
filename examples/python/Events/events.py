#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time


# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.Camera).build()
    qRgb = camRgb.requestOutput((256,256)).createOutputQueue()

    # Enter your hubs api-key
    eventMan = dai.EventsManager()
    eventMan.setUrl("https://events.cloud-stg.luxonis.com")
    eventMan.setToken("")

    pipeline.start()

    data = []

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()

        name = f"image_{len(data)}"
        if inRgb is not None:
            rgbData = dai.FileData(inRgb, name)
            data.append(rgbData)

        if len(data) == 5:
            eventMan.sendSnap("ImgFrame", ["EventsExample", "Python"], {"key_0" : "value_0", "key_1" : "value_1"}, "", data)
            data.clear()
            time.sleep(3)
    
        time.sleep(0.4)