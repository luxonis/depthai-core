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
from pathlib import Path

import cv2
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_example_common as dec

pipeline = dai.Pipeline()
try:
    device = dec.require_default_device(pipeline)
    stereoPair = dec.require_first_stereo_pair(device)
except RuntimeError as ex:
    print(ex, file=sys.stderr)
    sys.exit(1)

pipeline.create(dai.node.Camera).build(stereoPair.left, sensorResolution=(1280, 800), sensorFps=30)
pipeline.create(dai.node.Camera).build(stereoPair.right, sensorResolution=(1280, 800), sensorFps=30)

depthNode = pipeline.create(dai.node.Depth)
depthQueue, confidenceQueue = dec.create_depth_output_queues(depthNode)

pipeline.build()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth (user cameras)", dec.colorizeDepthMm(depthFrame.getFrame()))

        if confidenceQueue is not None:
            confidenceFrame = confidenceQueue.get()
            assert isinstance(confidenceFrame, dai.ImgFrame)
            cv2.imshow("confidence", dec.colorizeConfidence(confidenceFrame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
