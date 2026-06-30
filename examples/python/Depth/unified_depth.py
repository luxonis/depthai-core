#!/usr/bin/env python3
"""
Unified Depth node demo (AUTO backend: NeuralDepth on RVC4, ToF or StereoDepth elsewhere).

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


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    """Log-scaled depth colorization with adaptive 3rd..95th percentile clipping (zero = invalid)."""
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


def parse_config(name: str):
    if hasattr(dai.DeviceModelZoo, name):
        return getattr(dai.DeviceModelZoo, name)
    if hasattr(dai.node.StereoDepth.PresetMode, name):
        return getattr(dai.node.StereoDepth.PresetMode, name)
    raise argparse.ArgumentTypeError(
        f"Unknown config '{name}'. Use a DeviceModelZoo name (e.g. NEURAL_DEPTH_LARGE) "
        "or StereoDepth.PresetMode name (e.g. DEFAULT)."
    )


def stereo_size_from_args(args: argparse.Namespace) -> tuple[int, int] | None:
    if args.width is None and args.height is None:
        return None
    if args.width is None or args.height is None:
        raise argparse.ArgumentTypeError("--width and --height must be specified together")
    return args.width, args.height


def build_user_stereo_cameras(pipeline: dai.Pipeline, args: argparse.Namespace) -> None:
    device = pipeline.getDefaultDevice()
    if device is None:
        raise RuntimeError("Connect a device (host-only pipeline cannot use Depth).")

    stereoPairs = device.getStereoPairs()
    if not stereoPairs:
        raise RuntimeError("This device has no stereo pair; Depth cannot run.")

    stereoPair = stereoPairs[0]
    stereoSize = stereo_size_from_args(args)
    pipeline.create(dai.node.Camera).build(stereoPair.left, stereoSize, args.fps)
    pipeline.create(dai.node.Camera).build(stereoPair.right, stereoSize, args.fps)


def configure_depth(depthNode: dai.node.Depth, args: argparse.Namespace) -> None:
    fps = None if args.user_cameras else args.fps
    stereoSize = None if args.user_cameras else stereo_size_from_args(args)
    algorithm = _ALGORITHM_CHOICES[args.algorithm] if args.algorithm is not None else None
    config = parse_config(args.config) if args.config is not None else None

    if algorithm is not None and config is not None:
        depthNode.build(algorithm, config, fps=fps, stereo_size=stereoSize)
    elif algorithm is not None:
        depthNode.build(algorithm, fps=fps, stereo_size=stereoSize)
    elif fps is not None and stereoSize is not None:
        depthNode.build(dai.node.Depth.Algorithm.AUTO, fps=fps, stereo_size=stereoSize)
    elif fps is not None:
        depthNode.build(fps=fps)
    elif stereoSize is not None:
        depthNode.build(dai.node.Depth.Algorithm.AUTO, stereo_size=stereoSize)


def build_parser() -> argparse.ArgumentParser:
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
        type=parse_config,
        default=None,
        metavar="NAME",
        help="Algorithm config: DeviceModelZoo (NEURAL*) or StereoDepth.PresetMode (DEFAULT, ...)",
    )
    parser.add_argument(
        "--user-cameras",
        action="store_true",
        help="Pre-build stereo Camera nodes; fps/res come from args instead of Depth.build()",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        stereo_size_from_args(args)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    if args.config is not None and args.algorithm is None:
        parser.error("--config requires --algorithm")

    if args.ip:
        device = dai.Device(dai.DeviceInfo(args.ip))
        pipeline = dai.Pipeline(device)
    else:
        pipeline = dai.Pipeline()

    if args.user_cameras:
        build_user_stereo_cameras(pipeline, args)

    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)

    depthNode = pipeline.create(dai.node.Depth)
    configure_depth(depthNode, args)

    depthQueue = depthNode.depth.createOutputQueue()
    confidenceQueue = depthNode.confidence.createOutputQueue()
    depthNode.depth.link(benchmarkIn.input)

    pipeline.build()

    print("Resolved algorithm:", depthNode.getResolvedAlgorithm())
    print("Resolved config:", depthNode.getResolvedConfig())

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            depthFrame = depthQueue.get()
            assert isinstance(depthFrame, dai.ImgFrame)
            cv2.imshow("depth", colorizeDepth(depthFrame.getFrame()))

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
