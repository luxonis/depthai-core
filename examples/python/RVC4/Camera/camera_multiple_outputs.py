#!/usr/bin/env python3

import sys

import cv2
import depthai as dai

# Create pipeline


def exit_usage() -> None:
    print(
        "WRONG USAGE! correct usage example:\n"
        "python camera_multiple_outputs.py 640 480 0 30 CAM_A 300 300 0 30 CAM_A 300 300 1 30 CAM_A \n"
        "where 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX\n"
        "and 30 is FPS"
    )
    exit(1)


args = sys.argv[1:]
if len(args) < 5 or len(args) % 5 != 0:
    exit_usage()

with dai.Pipeline() as pipeline:
    cams: dict = {}
    queues = []
    for i in range(0, len(args), 5):
        cap = dai.ImgFrameCapability()
        cap.size.fixed((int(args[i]), int(args[i + 1])))
        cropArg = int(args[i + 2])
        if cropArg == 0:
            cap.resizeMode = dai.ImgResizeMode.CROP
        elif cropArg == 1:
            cap.resizeMode = dai.ImgResizeMode.STRETCH
        elif cropArg == 2:
            cap.resizeMode = dai.ImgResizeMode.LETTERBOX
        else:
            exit_usage()
        cap.fps.fixed(int(args[i + 3]))
        camArg = args[i + 4]
        socket: dai.CameraBoardSocket
        if camArg == "CAM_A":
            socket = dai.CameraBoardSocket.CAM_A
        elif camArg == "CAM_B":
            socket = dai.CameraBoardSocket.CAM_B
        elif camArg == "CAM_C":
            socket = dai.CameraBoardSocket.CAM_C
        elif camArg == "CAM_D":
            socket = dai.CameraBoardSocket.CAM_D
        else:
            exit_usage()
        if socket not in cams:
            cams[socket] = pipeline.create(dai.node.Camera)
            cams[socket].setBoardSocket(socket)
        queues.append(cams[socket].requestOutput(cap, True).createOutputQueue())

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        for index, queue in enumerate(queues):
            videoIn = queue.tryGet()
            if videoIn is not None:
                assert isinstance(videoIn, dai.ImgFrame)
                print(
                    f"frame {videoIn.getWidth()}x{videoIn.getHeight()} | {videoIn.getSequenceNum()}: exposure={videoIn.getExposureTime()}us, timestamp: {videoIn.getTimestampDevice()}"
                )
                # Get BGR frame from NV12 encoded video frame to show with opencv
                # Visualizing the frame on slower hosts might have overhead
                cv2.imshow("video " + str(index), videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
