#!/usr/bin/env python3
"""
``dai.node.Depth`` with ``Algorithm.GPU_STEREO`` (RVC4, stereo pair, board revision R9+).

GPUStereo exposes depth only: ``hasConfidence()`` is false and ``confidence()`` throws.
Exits successfully with a message when the device cannot wire GPU_STEREO (e.g. older board rev).
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
    if device.getPlatform() != dai.Platform.RVC4:
        print("GPU_STEREO requires an RVC4 device.", file=sys.stderr)
        sys.exit(0)
    dec.require_first_stereo_pair(device)
except RuntimeError as ex:
    print(ex, file=sys.stderr)
    sys.exit(1)

depthNode = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.GPU_STEREO)

try:
    if depthNode.hasConfidence():
        print("Unexpected: GPU_STEREO reported confidence support.", file=sys.stderr)
        sys.exit(1)
    depthQueue = depthNode.depth.createOutputQueue()
except Exception as ex:
    print(f"GPU_STEREO not available on this device: {ex}", file=sys.stderr)
    sys.exit(0)

pipeline.build()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth (GPU_STEREO)", dec.colorizeDepthMm(depthFrame.getFrame()))

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
