#!/usr/bin/env python3
"""
Unified Depth node demo.

The Depth node picks an algorithm and backend config automatically from the connected device,
target FPS, and/or stereo resolution (Algorithm.AUTO). You can also pin the algorithm yourself
via CLI or Depth.build().

Supported algorithms: AUTO, STEREO, NEURAL, NEURAL_ASSISTED_STEREO, TOF, GPU_STEREO.

Configure the node via CLI to exercise Depth.build() overloads (fps-only, algorithm + fps/res,
algorithm + config) or pre-built user stereo cameras.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import depthai as dai
import numpy as np

_COLOR_MAP = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
_COLOR_MAP[0] = [0, 0, 0]

_ALGORITHM_CHOICES = {
    "auto": dai.node.Depth.Algorithm.AUTO,
    "stereo": dai.node.Depth.Algorithm.STEREO,
    "neural": dai.node.Depth.Algorithm.NEURAL,
    "neural_assisted_stereo": dai.node.Depth.Algorithm.NEURAL_ASSISTED_STEREO,
    "tof": dai.node.Depth.Algorithm.TOF,
    "gpu_stereo": dai.node.Depth.Algorithm.GPU_STEREO,
}


def colorizeConfidence(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint16:
        vmax = int(np.max(frame))
        if vmax <= 0:
            return np.zeros((*frame.shape, 3), dtype=np.uint8)
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8)
    else:
        vis = frame
    return cv2.applyColorMap(vis, _COLOR_MAP)


def parseConfig(name: str):
    if hasattr(dai.DeviceModelZoo, name):
        return getattr(dai.DeviceModelZoo, name)
    if hasattr(dai.node.StereoDepth.PresetMode, name):
        return getattr(dai.node.StereoDepth.PresetMode, name)
    raise argparse.ArgumentTypeError(
        f"Unknown config '{name}'. Use a DeviceModelZoo name (e.g. NEURAL_DEPTH_LARGE) "
        "or StereoDepth.PresetMode name (e.g. DEFAULT)."
    )


def sizeFromArgs(args: argparse.Namespace) -> tuple[int, int] | None:
    if args.width is None and args.height is None:
        return None
    if args.width is None or args.height is None:
        raise argparse.ArgumentTypeError("--width and --height must be specified together")
    return args.width, args.height


def buildUserStereoCameras(pipeline: dai.Pipeline, args: argparse.Namespace) -> None:
    device = pipeline.getDefaultDevice()
    if device is None:
        raise RuntimeError("Connect a device (host-only pipeline cannot use Depth).")

    stereoPairs = device.getStereoPairs()
    if not stereoPairs:
        raise RuntimeError("This device has no stereo pair; Depth cannot run.")

    stereoPair = stereoPairs[0]
    size = sizeFromArgs(args)
    pipeline.create(dai.node.Camera).build(stereoPair.left, size, args.fps)
    pipeline.create(dai.node.Camera).build(stereoPair.right, size, args.fps)


def configureDepth(depthNode: dai.node.Depth, args: argparse.Namespace) -> None:
    fps = None if args.userCameras else args.fps
    size = None if args.userCameras else sizeFromArgs(args)
    algorithm = _ALGORITHM_CHOICES[args.algorithm] if args.algorithm is not None else None
    config = parseConfig(args.config) if args.config is not None else None

    if algorithm is not None and config is not None:
        depthNode.build(algorithm, config, fps=fps, size=size)
    elif algorithm is not None:
        depthNode.build(algorithm, fps=fps, size=size)
    elif fps is not None and size is not None:
        depthNode.build(dai.node.Depth.Algorithm.AUTO, fps=fps, size=size)
    elif fps is not None:
        depthNode.build(fps=fps)
    elif size is not None:
        depthNode.build(dai.node.Depth.Algorithm.AUTO, size=size)


def buildParser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  %(prog)s
  %(prog)s --fps 30
  %(prog)s --width 640 --height 480
  %(prog)s --algorithm neural --config NEURAL_DEPTH_LARGE --fps 8
  %(prog)s --algorithm stereo --config DEFAULT --fps 20
  %(prog)s --user-cameras --width 1280 --height 800 --fps 30
  %(prog)s --ip 192.168.1.10
""",
    )
    parser.add_argument(
        "--ip",
        metavar="ADDR",
        default=None,
        help="Device IP for TCP/IP (e.g. PoE). If omitted, the default device search is used (often first USB).",
    )
    parser.add_argument("--fps", type=float, default=None, help="Stereo camera FPS for Depth.build() or user cameras")
    parser.add_argument("--width", type=int, default=None, help="Stereo frame width (requires --height)")
    parser.add_argument("--height", type=int, default=None, help="Stereo frame height (requires --width)")
    parser.add_argument(
        "--algorithm",
        choices=sorted(_ALGORITHM_CHOICES.keys()),
        default=None,
        help="Depth backend selection (default: AUTO when omitted)",
    )
    parser.add_argument(
        "--config",
        type=parseConfig,
        default=None,
        metavar="NAME",
        help="Algorithm config: DeviceModelZoo (NEURAL*) or StereoDepth.PresetMode (DEFAULT, ...)",
    )
    parser.add_argument(
        "--user-cameras",
        dest="userCameras",
        action="store_true",
        help="Pre-build stereo Camera nodes; fps/res come from args instead of Depth.build()",
    )
    return parser


def main() -> int:
    parser = buildParser()
    args = parser.parse_args()

    try:
        sizeFromArgs(args)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    if args.config is not None and args.algorithm is None:
        parser.error("--config requires --algorithm")

    if args.ip:
        device = dai.Device(dai.DeviceInfo(args.ip))
        pipeline = dai.Pipeline(device)
    else:
        pipeline = dai.Pipeline()

    if args.userCameras:
        buildUserStereoCameras(pipeline, args)

    depthNode = pipeline.create(dai.node.Depth)
    configureDepth(depthNode, args)

    depthQueue = depthNode.depth.createOutputQueue()
    confidenceQueue = depthNode.confidence.createOutputQueue()

    pipeline.build()

    print("Resolved algorithm:", depthNode.getResolvedAlgorithm())
    print("Resolved config:", depthNode.getResolvedConfig())

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            depthFrame = depthQueue.get()
            assert isinstance(depthFrame, dai.ImgFrame)
            cv2.imshow("depth", dai.utility.colorizeDepthFrame(depthFrame).getCvFrame())

            confidenceFrame = confidenceQueue.get()
            assert isinstance(confidenceFrame, dai.ImgFrame)
            cv2.imshow("confidence", colorizeConfidence(confidenceFrame.getFrame()))

            if cv2.waitKey(1) == ord("q"):
                pipeline.stop()
                break

        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
