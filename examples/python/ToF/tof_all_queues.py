#!/usr/bin/env python3
"""Simple ToF script showing all main ToF output queues.

Displays depth, rawDepth, amplitude, intensity, confidence, phase and raw.

Press 'q' to quit.
"""

import cv2
import numpy as np
import sys
sys.path.insert(0, '/home/jakub/Code/depthai-device-kb/external/depthai-core/build/bindings/python/')
import depthai as dai


def colorizeDepth(frame: np.ndarray, minDepth: float, maxDepth: float) -> np.ndarray:
    invalidMask = frame == 0
    try:
        logDepth = np.log(frame.astype(np.float32) + 1e-6)
        logDepth[invalidMask] = 0.0
        logDepth = np.clip(logDepth, np.log(minDepth + 1e-6), np.log(maxDepth + 1e-6))
        colored = np.interp(logDepth, (logDepth[~invalidMask].min(), logDepth[~invalidMask].max()), (0, 255))
        colored = colored.astype(np.uint8)
        colored = cv2.applyColorMap(colored, cv2.COLORMAP_JET)
        colored[invalidMask] = 0
    except (IndexError, ValueError):
        colored = np.zeros((*frame.shape, 3), dtype=np.uint8)
    return colored


def normalizeFrame(frame: np.ndarray) -> np.ndarray:
    return cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

def main():
    pipeline = dai.Pipeline()

    # show depth in range 0.5m - 10m
    minDepth = 500
    maxDepth = 10000

    # choose one of profiles LOW_RANGE / MID_RANGE / HIGH_RANGE
    profile = dai.ToFConfig.Profile.MID_RANGE

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=profile
    )

    outputQueues = {
        "depth": tof.depth.createOutputQueue(maxSize=1, blocking=False),
        "amplitude": tof.amplitude.createOutputQueue(maxSize=1, blocking=False),
        "intensity": tof.intensity.createOutputQueue(maxSize=1, blocking=False),
        # "rawDepth": tof.rawDepth.createOutputQueue(maxSize=1, blocking=False), # not supported on RVC4
        # "confidence": tof.confidence.createOutputQueue(maxSize=1, blocking=False), # not supported on RVC2
    }

    with pipeline as p:
        p.start()
        while p.isRunning():
            for name, queue in outputQueues.items():
                frame = queue.tryGet()
                if frame is None:
                    continue

                if name in {"depth", "rawDepth"}:
                    display = colorizeDepth(frame.getCvFrame(), minDepth, maxDepth)
                else:
                    display = normalizeFrame(frame.getCvFrame())
                cv2.imshow(name, display)

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
