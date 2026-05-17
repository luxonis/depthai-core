#!/usr/bin/env python3
"""
Stereo-style demo using the unified dai.node.Depth group (AUTO: NeuralDepth on RVC4, ToF on RVC2 with a ToF sensor,
otherwise StereoDepth on RVC2/RVC3).

- On RVC4 the NeuralDepth zoo model is fixed inside the ``Depth`` wiring path (not user-configurable).
- Create host queues on ``depthNode.depth`` / ``confidence`` **before** ``pipeline.build()`` (the pipeline
  rejects new queues after build). The first ``depth``/``confidence`` access wires the chosen backend (and stereo cameras when that backend needs them).
- For a non-``AUTO`` backend, call ``depthNode.build(dai.node.Depth.Algorithm.TOF)`` before first
  ``depth`` / ``confidence`` access. ``Algorithm.GPU_STEREO`` has no confidence output; use ``hasConfidence()`` before creating a confidence queue.
- Use ``--ip ADDR`` to connect over Ethernet (e.g. PoE) instead of auto-picking the first USB device.
"""

import argparse

import cv2
import depthai as dai
import numpy as np

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument(
    "--ip",
    metavar="ADDR",
    default=None,
    help="Device IP for TCP/IP (e.g. PoE). If omitted, the default device search is used (often first USB).",
)
args = parser.parse_args()

if args.ip:
    device = dai.Device(dai.DeviceInfo(args.ip))
    pipeline = dai.Pipeline(device)
else:
    pipeline = dai.Pipeline()

depthNode = pipeline.create(dai.node.Depth)

depthQueue = depthNode.depth.createOutputQueue()
confidenceQueue = None
if getattr(depthNode, "hasConfidence", lambda: True)():
    confidenceQueue = depthNode.confidence.createOutputQueue()

pipeline.build()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # invalid / zero depth pixels stay black


def colorizeDepthMm(frame: np.ndarray) -> np.ndarray:
    """RAW16 depth in millimeters; zero means invalid."""
    valid = frame > 0
    if not np.any(valid):
        return np.zeros((*frame.shape, 3), dtype=np.uint8)
    vmax = float(np.max(frame[valid]))
    norm = np.zeros_like(frame, dtype=np.uint8)
    norm[valid] = ((frame[valid].astype(np.float32) / vmax) * 255).astype(np.uint8)
    return cv2.applyColorMap(norm, colorMap)


def colorizeConfidence(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint16:
        vmax = int(np.max(frame))
        if vmax <= 0:
            return np.zeros((*frame.shape, 3), dtype=np.uint8)
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8)
    else:
        vis = frame
    return cv2.applyColorMap(vis, colorMap)


with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth", colorizeDepthMm(depthFrame.getFrame()))

        if confidenceQueue is not None:
            confidenceFrame = confidenceQueue.get()
            assert isinstance(confidenceFrame, dai.ImgFrame)
            cv2.imshow("confidence", colorizeConfidence(confidenceFrame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
