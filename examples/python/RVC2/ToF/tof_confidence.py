#!/usr/bin/env python3
"""RVC2 ToF: tof.confidence aliases amplitude (per-pixel quality map)."""

import cv2
import depthai as dai
import numpy as np


def normalize_float(frame: np.ndarray) -> np.ndarray:
    f = frame.astype(np.float32)
    lo, hi = np.min(f), np.max(f)
    if hi <= lo:
        return np.zeros(f.shape, dtype=np.uint8)
    return ((f - lo) / (hi - lo) * 255).astype(np.uint8)


def main() -> None:
    pipeline = dai.Pipeline()

    tof = pipeline.create(dai.node.ToF).build(
        dai.CameraBoardSocket.AUTO,
        dai.ImageFiltersPresetMode.TOF_MID_RANGE,
    )

    amp_q = tof.amplitude.createOutputQueue()
    conf_q = tof.confidence.createOutputQueue()

    with pipeline as p:
        p.start()
        print("On RVC2, tof.confidence streams the same amplitude frames as tof.amplitude.")
        while p.isRunning():
            amp = amp_q.tryGet()
            conf = conf_q.tryGet()
            if amp is not None:
                cv2.imshow("amplitude", normalize_float(amp.getFrame()))
            if conf is not None:
                cv2.imshow("confidence (amplitude alias)", normalize_float(conf.getFrame()))
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
