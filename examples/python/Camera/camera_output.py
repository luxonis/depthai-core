#!/usr/bin/env python3

import argparse
import cv2
import depthai as dai


parser = argparse.ArgumentParser()
parser.add_argument(
    "--fps_limit", type=float, default=None, help="Limit output FPS (float, optional)"
)
args = parser.parse_args()

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    videoOutput = cam.requestOutput((640, 400))
    videoQueue = videoOutput.createOutputQueue()

    pipeline.build()

    # Optionally update internal settings
    # Note: xlink bridges are only generated after pipeline.build() is called
    if args.fps_limit is not None:
        xlinkBridge = videoOutput.getXLinkBridge()
        assert xlinkBridge is not None
        assert isinstance(xlinkBridge, dai.node.internal.XLinkOutBridge)
        xlinkBridge.xLinkOut.setFpsLimit(args.fps_limit)

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
