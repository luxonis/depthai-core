#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time

remoteConnector = dai.RemoteConnection()
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        cameraNode, dai.NNModelDescription("yolov6-nano")
    )

    cameraOutputVisualize = cameraNode.requestOutput((512, 288), dai.ImgFrame.Type.NV12)
    remoteConnector.addTopic("detections", detectionNetwork.out, "img")
    remoteConnector.addTopic("images", cameraOutputVisualize, "img")

    pipeline.start()
    remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
