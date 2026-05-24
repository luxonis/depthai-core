#!/usr/bin/env python3
"""
``dai.node.Depth`` with ``Algorithm.GPU_STEREO`` (RVC4 with GPU, stereo pair required).

GPUStereo exposes both ``depth`` and a per-pixel ``confidence`` map.
Exits successfully with a message when the device cannot wire GPU_STEREO
(e.g. non-RVC4 device or ``device.isGpuStereoSupported()`` returns false).
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
    if not device.isGpuStereoSupported():
        print("GPU_STEREO not supported on this device (no GPU).", file=sys.stderr)
        sys.exit(0)
    dec.require_first_stereo_pair(device)
except RuntimeError as ex:
    print(ex, file=sys.stderr)
    sys.exit(1)

depthNode = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.GPU_STEREO)

try:
    depthQueue, confidenceQueue = dec.create_depth_output_queues(depthNode)
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

        confidenceFrame = confidenceQueue.get()
        assert isinstance(confidenceFrame, dai.ImgFrame)
        cv2.imshow("confidence (GPU_STEREO)", dec.colorizeConfidence(confidenceFrame.getFrame()))

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
