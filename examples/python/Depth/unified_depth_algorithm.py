#!/usr/bin/env python3
"""
``dai.node.Depth`` with an explicit ``Algorithm`` (not ``AUTO``).

Call ``depthNode.build(algorithm)`` (optional ``fps=``) before the first ``depth`` / ``confidence`` access.
Unsupported combinations throw at wiring time (e.g. ``TOF`` without a ToF sensor, ``GPU_STEREO`` off RVC4).

Use ``--algorithm AUTO`` for the same behavior as ``unified_depth.py``.
"""

import argparse
import sys
from pathlib import Path

import cv2
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_example_common as dec

_ALGORITHM_CHOICES = {
    "AUTO": dai.node.Depth.Algorithm.AUTO,
    "STEREO": dai.node.Depth.Algorithm.STEREO,
    "NEURAL": dai.node.Depth.Algorithm.NEURAL,
    "NEURAL_ASSISTED_STEREO": dai.node.Depth.Algorithm.NEURAL_ASSISTED_STEREO,
    "TOF": dai.node.Depth.Algorithm.TOF,
    "GPU_STEREO": dai.node.Depth.Algorithm.GPU_STEREO,
}

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument(
    "--algorithm",
    choices=sorted(_ALGORITHM_CHOICES.keys()),
    default="AUTO",
    help="Depth backend to request (default: AUTO).",
)
parser.add_argument(
    "--fps",
    type=float,
    default=30.0,
    metavar="FPS",
    help="Optional stereo camera FPS for stereo-based backends (before wiring).",
)
parser.add_argument(
    "--ip",
    metavar="ADDR",
    default=None,
    help="Device IP for TCP/IP (e.g. PoE).",
)
args = parser.parse_args()

algorithm = _ALGORITHM_CHOICES[args.algorithm]

if args.ip:
    device = dai.Device(dai.DeviceInfo(args.ip))
    pipeline = dai.Pipeline(device)
else:
    pipeline = dai.Pipeline()

depthNode = pipeline.create(dai.node.Depth)
if args.fps is not None and algorithm == dai.node.Depth.Algorithm.AUTO:
    depthNode.build(args.fps)
elif args.fps is not None:
    depthNode.build(algorithm, args.fps)
elif algorithm != dai.node.Depth.Algorithm.AUTO:
    depthNode.build(algorithm)

depthQueue, confidenceQueue = dec.create_depth_output_queues(depthNode)
pipeline.build()

window = f"depth ({args.algorithm})"

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depthFrame = depthQueue.get()
        assert isinstance(depthFrame, dai.ImgFrame)
        cv2.imshow(window, dec.colorizeDepthMm(depthFrame.getFrame()))

        if confidenceQueue is not None:
            confidenceFrame = confidenceQueue.get()
            assert isinstance(confidenceFrame, dai.ImgFrame)
            cv2.imshow("confidence", dec.colorizeConfidence(confidenceFrame.getFrame()))

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
