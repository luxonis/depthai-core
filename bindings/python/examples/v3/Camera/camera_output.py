#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline

info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3
with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        # Define source and output
        cam = pipeline.create(dai.node.Camera)

        # Properties
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        cap = dai.ImgFrameCapability()
        fixedSize = dai.CapabilityRangeUintTuple()
        fixedSize.value = [640, 480]
        cap.size = fixedSize
        videoQueue = cam.requestNewOutput(cap, True).createQueue()

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            videoIn: dai.ImgFrame = videoQueue.get()

            # Get BGR frame from NV12 encoded video frame to show with opencv
            # Visualizing the frame on slower hosts might have overhead
            cv2.imshow("video", videoIn.getCvFrame())

            if cv2.waitKey(1) == ord("q"):
                break
