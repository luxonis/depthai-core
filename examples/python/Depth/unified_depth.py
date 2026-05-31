#!/usr/bin/env python3
"""
Stereo-style demo using the unified dai.node.Depth group (AUTO: NeuralDepth on RVC4, ToF on RVC2 with a ToF sensor,
otherwise StereoDepth on RVC2/RVC3).

- On RVC4 the NeuralDepth zoo model is fixed inside the ``Depth`` wiring path (not user-configurable).
- Create host queues on ``depthNode.depth`` / ``confidence`` **before** ``pipeline.build()`` (the pipeline
  rejects new queues after build). The first ``depth``/``confidence`` access wires the chosen backend (and stereo cameras when that backend needs them).
- For a non-``AUTO`` backend, call ``depthNode.build(dai.node.Depth.Algorithm.TOF)`` (or another algorithm)
  before first ``depth`` / ``confidence`` access. Every backend exposes a confidence-like stream:
  ``confidence()`` returns StereoDepth/NAS confidence, NeuralDepth confidence, GPUStereo confidence map, or ToF amplitude.
- Depth is colorized with adaptive log-scale percentile clipping (same helper as ``tof_align.py``).
- Use ``--ip ADDR`` to connect over Ethernet (e.g. PoE) instead of auto-picking the first USB device.
"""

import argparse

import cv2
import depthai as dai
import numpy as np


_COLOR_MAP = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
_COLOR_MAP[0] = [0, 0, 0]


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    """Log-scaled depth colorization with adaptive 3rd..95th percentile clipping (zero = invalid).

    Same helper used across the repo (e.g. ``RVC2/ToF/tof_align.py``, ``ImageAlign/depth_align.py``).
    """
    invalidMask = frameDepth == 0
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        depthFrameColor[invalidMask] = 0
    except IndexError:
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    return depthFrameColor


def colorizeConfidence(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint16:
        vmax = int(np.max(frame))
        if vmax <= 0:
            return np.zeros((*frame.shape, 3), dtype=np.uint8)
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8)
    else:
        vis = frame
    return cv2.applyColorMap(vis, _COLOR_MAP)


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

benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
benchmarkIn.setRunOnHost(True)

depthNode = pipeline.create(dai.node.Depth)

depthQueue = depthNode.depth.createOutputQueue()
confidenceQueue = depthNode.confidence.createOutputQueue()

depthNode.depth.link(benchmarkIn.input)

pipeline.build()

with pipeline:
    pipeline.start()
    print("Algorithm:", depthNode.getResolvedAlgorithm())
    print("Preset:", depthNode.getResolvedPreset())
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth", colorizeDepth(depthFrame.getFrame()))

        confidenceFrame = confidenceQueue.get()
        assert isinstance(confidenceFrame, dai.ImgFrame)
        cv2.imshow("confidence", colorizeConfidence(confidenceFrame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
