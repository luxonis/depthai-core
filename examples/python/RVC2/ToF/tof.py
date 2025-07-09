#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    # Log the depth, minDepth and maxDepth
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        # Clip the values to be in the 0-255 range
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Interpolate only valid logDepth values, setting the rest based on the mask
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros(
            (frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8
        )
    except Exception as e:
        raise e
    return depthFrameColor


# Create pipeline
pipeline = dai.Pipeline()
# Define source and output
tof = pipeline.create(dai.node.ToF).build(
    dai.CameraBoardSocket.AUTO, dai.ImageFiltersPresetMode.DEFAULT
)
depthQueue = tof.depth.createOutputQueue()
depthRawQueue = tof.rawDepth.createOutputQueue()

with pipeline:
    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        depth = depthQueue.get()
        assert isinstance(depth, dai.ImgFrame)
        visualizedDepth = colorizeDepth(depth.getFrame())
        cv2.imshow("depth", visualizedDepth)

        depthRaw = depthRawQueue.get()
        assert isinstance(depthRaw, dai.ImgFrame)
        visualizedDepthRaw = colorizeDepth(depthRaw.getFrame())
        cv2.imshow("depthRaw", visualizedDepthRaw)

        if cv2.waitKey(1) == ord("q"):
            break
