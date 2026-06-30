#!/usr/bin/env python3
"""Minimal ToF script showing the main output streams.

Displays: depth, amplitude, intensity, confidence.
For phase, raw and rawDepth streams see tof_raw_streams.py.

Press 'q' to quit.
"""

import cv2
import depthai as dai
import numpy as np


def get_frame(msg: dai.ImgFrame) -> np.ndarray:
    """Decode ImgFrame to a 2-D numpy array, falling back to getData() reshape."""
    try:
        return msg.getFrame()
    except Exception:
        data = np.frombuffer(msg.getData(), dtype=np.uint16)
        h, w = msg.getHeight(), msg.getWidth()
        if h > 0 and w > 0 and data.size == h * w:
            return data.reshape(h, w)
        return data.reshape(-1, max(w, 1))


def colorizeDepth(frame: np.ndarray) -> np.ndarray:
    invalidMask = frame == 0
    try:
        minDepth = np.percentile(frame[frame != 0], 3)
        maxDepth = np.percentile(frame[frame != 0], 95)
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


def normalize(frame: np.ndarray) -> np.ndarray:
    f = frame.astype(np.float32)
    lo, hi = f.min(), f.max()
    if hi > lo:
        f = (f - lo) / (hi - lo) * 255.0
    return f.astype(np.uint8)


def main():
    pipeline = dai.Pipeline()

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=dai.ToFConfig.Profile.MID_RANGE,
    )

    queues = {
        "depth":      (tof.depth.createOutputQueue(),      colorizeDepth),
        "amplitude":  (tof.amplitude.createOutputQueue(),  normalize),
        "intensity":  (tof.intensity.createOutputQueue(),  normalize),
        "confidence": (tof.confidence.createOutputQueue(), normalize),
    }

    with pipeline as p:
        p.start()
        while p.isRunning():
            for name, (q, fn) in queues.items():
                msg = q.tryGet()
                if msg is not None:
                    cv2.imshow(name, fn(get_frame(msg)))

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
