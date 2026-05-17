#!/usr/bin/env python3
"""
Same flow as unified_depth.py, but left/right ``Camera`` nodes are created explicitly on the device's
first stereo pair.

For stereo backends (``AUTO`` -> ``StereoDepth`` / ``NeuralDepth`` / …, or explicit non-ToF algorithms),
``Depth`` reuses those cameras and requests stereo-sized outputs (it does not adopt the camera nodes).

On RVC2 with a ToF sensor, ``AUTO`` resolves to the ToF backend: the pre-built stereo cameras are not used
for depth. On RVC4 the NeuralDepth zoo model inside ``Depth`` is fixed (not user-configurable).

Create ``depth_node.depth`` / ``confidence`` queues **before** ``pipeline.build()``; wiring happens on first access.

This script still requires ``getStereoPairs()`` to be non-empty so the demo can build left/right cameras (they are unused when ``AUTO`` selects ToF on RVC2).
"""

import sys

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
device = pipeline.getDefaultDevice()
if device is None:
    print("Connect a device (host-only pipeline cannot use Depth).", file=sys.stderr)
    sys.exit(1)

stereoPairs = device.getStereoPairs()
if not stereoPairs:
    print("This device has no stereo pair; Depth cannot run.", file=sys.stderr)
    sys.exit(1)

stereoPair = stereoPairs[0]

pipeline.create(dai.node.Camera).build(stereoPair.left, sensorResolution=(1280, 800), sensorFps=30)
pipeline.create(dai.node.Camera).build(stereoPair.right, sensorResolution=(1280, 800), sensorFps=30)

depthNode = pipeline.create(dai.node.Depth)

depthQueue = depthNode.depth.createOutputQueue()
confidenceQueue = depthNode.confidence.createOutputQueue()

pipeline.build()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]


def colorizeDepthMm(frame: np.ndarray) -> np.ndarray:
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
        cv2.imshow("depth (user cameras)", colorizeDepthMm(depthFrame.getFrame()))

        confidenceFrame = confidenceQueue.get()
        assert isinstance(confidenceFrame, dai.ImgFrame)
        cv2.imshow("confidence", colorizeConfidence(confidenceFrame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
