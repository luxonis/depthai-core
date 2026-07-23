#!/usr/bin/env python3
"""Simple ToF script showing all main ToF output queues.

RVC2 displays: depth, amplitude, intensity, rawDepth.
RVC4 displays: depth, amplitude, intensity, confidence.

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


def normalizeFrame(frame: np.ndarray) -> np.ndarray:
    return cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)


def main():
    pipeline = dai.Pipeline()

    minDepth = 100
    maxDepth = 7000

    profile = dai.ToFConfig.Profile.MID_RANGE

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=profile
    )

    with pipeline as p:
        device = p.getDefaultDevice()
        isRVC2 = device.getPlatform() == dai.Platform.RVC2

        outputQueues = {
            "depth": tof.depth.createOutputQueue(maxSize=1, blocking=False),
            "amplitude": tof.amplitude.createOutputQueue(maxSize=1, blocking=False),
            "intensity": tof.intensity.createOutputQueue(maxSize=1, blocking=False),
        }
        if isRVC2:
            # rawDepth are only supported on RVC2
            outputQueues["rawDepth"] = tof.rawDepth.createOutputQueue(maxSize=1, blocking=False)
        else:
            # confidence is only supported on RVC4
            outputQueues["confidence"] = tof.confidence.createOutputQueue(maxSize=1, blocking=False)

        platformName = "RVC2" if isRVC2 else "RVC4"
        print(f"Detected {platformName} - showing queues: {', '.join(outputQueues)}")

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
