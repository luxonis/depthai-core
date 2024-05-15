#!/usr/bin/env python3

import sys

import cv2
import depthai as dai

# Create pipeline


def exit_usage():
    print(
        "WRONG USAGE! correct usage example:\n"
        "python camera_multiple_outputs.py 640 480 0 300 300 0 300 300 1\n"
        "where 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX"
    )
    exit(1)


args = sys.argv[1:]
if len(args) < 3 or len(args) % 3 != 0:
    exit_usage()
info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3
with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        # Define source and output
        cam = pipeline.create(dai.node.Camera)

        # Properties
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        queues = []
        for i in range(0, len(args), 3):
            cap = dai.ImgFrameCapability()
            cap.size.fixed([int(args[i]), int(args[i + 1])])
            cropArg = int(args[i + 2])
            if cropArg == 0:
                cap.resizeMode = dai.ImgResizeMode.CROP
            elif cropArg == 1:
                cap.resizeMode = dai.ImgResizeMode.STRETCH
            elif cropArg == 2:
                cap.resizeMode = dai.ImgResizeMode.LETTERBOX
            else:
                exit_usage()
            queues.append(cam.requestOutput(cap, True).createQueue())

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            for index, queue in enumerate(queues):
                videoIn: dai.ImgFrame = queue.tryGet()
                if videoIn is not None:
                    # Get BGR frame from NV12 encoded video frame to show with opencv
                    # Visualizing the frame on slower hosts might have overhead
                    cv2.imshow("video " + str(index), videoIn.getCvFrame())

            if cv2.waitKey(1) == ord("q"):
                break
