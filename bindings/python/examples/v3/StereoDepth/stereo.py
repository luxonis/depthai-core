#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    depth.setLeftRightCheck(lr_check)
    depth.setExtendedDisparity(extended_disparity)
    depth.setSubpixel(subpixel)

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)
    depthQueue = depth.disparity.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        inDisparity : dai.ImgFrame = depthQueue.get() # blocking call, will wait until a new data has arrived
        frame = inDisparity.getFrame()

        # Normalization for better visualization
        frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
        cv2.imshow("disparity", frame)

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", frame)

        if cv2.waitKey(1) == ord('q'):
            break