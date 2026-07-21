#!/usr/bin/env python3
"""Fused ToF and neural depth."""

import cv2
import depthai as dai


with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
    fusion = pipeline.create(dai.beta.node.ToFStereoFusion).build(left, right)

    depth_queue = fusion.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        depth = depth_queue.get()
        cv2.imshow("Fused depth", dai.utility.colorizeDepthFrame(depth).getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
