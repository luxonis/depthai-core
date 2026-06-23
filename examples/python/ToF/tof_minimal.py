#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    invalidMask = frameDepth == 0  # zero depth is invalid

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


def main():
    pipeline = dai.Pipeline()

    # ToF node
    socket, preset_mode = dai.CameraBoardSocket.AUTO, dai.ImageFiltersPresetMode.TOF_MID_RANGE
    tof = pipeline.create(dai.node.ToF).build(socket, preset_mode)

    # Output queues
    depthQueue = tof.depth.createOutputQueue()
    depthRawQueue = tof.rawDepth.createOutputQueue()

    with pipeline as p:
        p.start()
        while p.isRunning():
            ## Visualize raw depth (unfiltered depth directly from the ToF sensor)
            depthRaw: dai.ImgFrame = depthRawQueue.get()
            depthRawImage = colorizeDepth(depthRaw.getFrame())
            cv2.imshow("depthRaw", depthRawImage)

            ## Visualize depth (which is filtered depthRaw)
            depth: dai.ImgFrame = depthQueue.get()
            depthImage = colorizeDepth(depth.getFrame())
            cv2.imshow("depth", depthImage)

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
