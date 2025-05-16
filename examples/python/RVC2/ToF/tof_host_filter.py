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
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor

# Create pipeline
pipeline = dai.Pipeline()
# Define source and output
tof = pipeline.create(dai.node.ToF).build()

confidenceFilter = pipeline.create(dai.node.DepthConfidenceFilter)
confidenceFilter.setConfidenceThreshold(0.1)
confidenceFilter.setRunOnHost(True)

# link
tof.depth.link(confidenceFilter.depth)
tof.amplitude.link(confidenceFilter.amplitude)

# queues
confidenceQueue = confidenceFilter.confidence.createOutputQueue()
filteredDepthQueue = confidenceFilter.filteredDepth.createOutputQueue()

with pipeline:

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        confidence = confidenceQueue.get()
        assert isinstance(confidence, dai.ImgFrame)
        visualizedConfidence = colorizeDepth(confidence.getFrame())
        cv2.imshow("confidence", visualizedConfidence)

        filteredDepth = filteredDepthQueue.get()
        assert isinstance(filteredDepth, dai.ImgFrame)
        visualizedFilteredDepth = colorizeDepth(filteredDepth.getFrame())
        cv2.imshow("filteredDepth", visualizedFilteredDepth)

        if cv2.waitKey(1) == ord("q"):
            break
