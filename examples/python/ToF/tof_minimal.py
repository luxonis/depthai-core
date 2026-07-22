#!/usr/bin/env python3
"""Minimal ToF script showing the main output stream.

Displays depth.
For more streams, see tof_all_queues.py.

Press 'q' to quit.
"""

import cv2
import numpy as np
import depthai as dai


def colorizeDepth(frame: np.ndarray, minDepth: float, maxDepth: float) -> np.ndarray:
    invalidMask = frame == 0
    try:
        logDepth = np.log(frame.astype(np.float32) + 1e-6)
        logDepth[invalidMask] = 0.0
        logMin, logMax = np.log(minDepth + 1e-6), np.log(maxDepth + 1e-6)
        logDepth = np.clip(logDepth, logMin, logMax)
        colored = np.interp(logDepth, (logMin, logMax), (0, 255))
        colored = colored.astype(np.uint8)
        colored = cv2.applyColorMap(colored, cv2.COLORMAP_JET)
        colored[invalidMask] = 0
    except (IndexError, ValueError):
        colored = np.zeros((*frame.shape, 3), dtype=np.uint8)
    return colored


def main():
    pipeline = dai.Pipeline()

    # show depth in range 0.1m - 7m
    minDepth = 100
    maxDepth = 7000

    # choose one of profiles LOW_RANGE / MID_RANGE / HIGH_RANGE
    profile = dai.ToFConfig.Profile.MID_RANGE

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=profile
    )

    depthOutputQueue = tof.depth.createOutputQueue()

    with pipeline as p:
        p.start()
        while p.isRunning():
            depth = depthOutputQueue.get()
            cv2.imshow("depth", colorizeDepth(depth.getCvFrame(), minDepth, maxDepth))

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
