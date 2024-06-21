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
    # monoLeft.setCamera("left")
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    # monoRight.setCamera("right")
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    depth.setLeftRightCheck(lr_check)
    depth.setExtendedDisparity(extended_disparity)
    depth.setSubpixel(subpixel)
    # configQueue = monoLeft.inputControl.createInputQueue()
    # depth2 = pipeline.create(dai.node.StereoDepth)
    # depth2.inputConfig.createInputQueue() # TEST
    # depth2.left.createInputQueue() # TEST
    # depth2.right.createInputQueue() # TEST
    # depth2.disparity.createOutputQueue() # TEST
    depth.inputConfig.setBlocking(False)
    # configQueue = depth.inputConfig.createInputQueue()
    configQueue = depth.inputConfig.createInputQueue()

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)
    depthQueue = depth.disparity.createOutputQueue()

    threshold = 1
    # # Serialize the pipeline to a file
    # with open('stereo_depth_threshold_test', 'w') as file:
    #     json = pipeline.serializeToJson()
    #     file.write(json)
    # exit(0)
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

        def update():
            print(f"Updating to {threshold}")
            message = dai.StereoDepthConfig()
            message.setConfidenceThreshold(threshold)
            configQueue.send(message)

        key = cv2.waitKey(1)
        if key == ord('q'): break
        if key == ord('j'):
            threshold += 1
            update()
        if key == ord('k'):
            threshold -= 1
            if threshold < 1: threshold = 1
            update()