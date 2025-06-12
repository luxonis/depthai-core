#!/usr/bin/env python3

import time
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
tof = pipeline.create(dai.node.ToF).build()

confidenceFilter = pipeline.create(dai.node.ToFDepthConfidenceFilter)
confidenceFilter.setConfidenceThreshold(0.1)
confidenceFilter.setRunOnHost(True)

# link
tof.depth.link(confidenceFilter.depth)
tof.amplitude.link(confidenceFilter.amplitude)

# queues
confidenceQueue = confidenceFilter.confidence.createOutputQueue()
filteredDepthQueue = confidenceFilter.filteredDepth.createOutputQueue()
configQueue = confidenceFilter.config.createInputQueue()


with pipeline:

    # Last config change time
    t_last = time.time()

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

        # Change the config every second
        if time.time() - t_last > 1:
            print("Changing config")
            config = dai.ToFDepthConfidenceFilterConfig()
            config.confidenceThreshold = np.random.choice([0.1, 0.2, 0.3, 0.4, 0.5])
            configQueue.send(config)
            t_last = time.time()

        if cv2.waitKey(1) == ord("q"):
            break
