#!/usr/bin/env python3
"""
Stereo-style demo using the unified dai.node.Depth group (AUTO: NeuralDepth on RVC4, ToF on RVC2 with a ToF sensor,
otherwise StereoDepth on RVC2/RVC3).

- On RVC4 the NeuralDepth zoo model is fixed inside the ``Depth`` wiring path (not user-configurable).
- Create host queues on ``depthNode.depth`` / ``confidence`` **before** ``pipeline.build()`` (the pipeline
  rejects new queues after build). The first ``depth``/``confidence`` access wires the chosen backend (and stereo cameras when that backend needs them).
- For a non-``AUTO`` backend, see ``unified_depth_algorithm.py`` or call ``depthNode.build(dai.node.Depth.Algorithm.TOF)`` before first
  ``depth`` / ``confidence`` access. ``Algorithm.GPU_STEREO`` has no confidence output; use ``hasConfidence()`` before creating a confidence queue.
- Depth is colorized with a fixed 0..15 m range (see ``depth_example_common``).
- Use ``--ip ADDR`` to connect over Ethernet (e.g. PoE) instead of auto-picking the first USB device.
"""

import argparse
import sys
from pathlib import Path

import cv2
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_example_common as dec

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
depthQueue, confidenceQueue = dec.create_depth_output_queues(depthNode)

pipeline.build()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow("depth", dec.colorizeDepthMm(depthFrame.getFrame()))

        if confidenceQueue is not None:
            confidenceFrame = confidenceQueue.get()
            assert isinstance(confidenceFrame, dai.ImgFrame)
            cv2.imshow("confidence", dec.colorizeConfidence(confidenceFrame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
