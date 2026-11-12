#!/usr/bin/env python3
"""ToF + stereo fusion with the STEREO_NETWORK model.

One network takes the rectified stereo pair and the ToF reprojected into the rectified-left view, and the device composes
its disparity with the ToF. The network ships with the RVC4 firmware.
"""

import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
    fusion = pipeline.create(dai.beta.node.ToFStereoFusion)
    fusion.setModel(dai.beta.node.ToFStereoFusion.Model.STEREO_NETWORK)
    fusion.build(left, right)

    depth_queue = fusion.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        depth = depth_queue.get()
        cv2.imshow("Fused depth (rectified left)", dai.utility.colorizeDepthFrame(depth).getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
