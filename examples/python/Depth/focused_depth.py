#!/usr/bin/env python3
"""
Focused Depth PoC demo.

Runs an object-detection network on the color camera and feeds the resulting
ImgDetections into the Depth node's inputDetections. The Depth node computes a
focused depth map (full frame, only the detected regions filled).

Works on RVC4 devices. The color camera's ImgDetections are automatically
remapped to the left stereo frame using the ImgDetections transformation.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import depthai as dai
import numpy as np

_COLOR_MAP = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
_COLOR_MAP[0] = [0, 0, 0]


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    """Log-scaled depth colorization with adaptive percentile clipping (zero = invalid)."""
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


def buildParser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--model",
        default="yolov6-nano",
        help="Model description for the DetectionNetwork (default: yolov6-nano)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Stereo camera FPS for the Depth node",
    )
    return parser


def main() -> int:
    args = buildParser().parse_args()

    pipeline = dai.Pipeline()

    # Detection network on the default color camera.
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
    detectionNetwork.setConfidenceThreshold(0.5)

    # Depth node with focused-depth input.
    depth = pipeline.create(dai.node.Depth)
    if args.fps is not None:
        depth.build(args.fps)
    else:
        depth.build()
    detectionNetwork.out.link(depth.inputDetections)

    # Output queues.
    rgbQueue = detectionNetwork.passthrough.createOutputQueue(maxSize=4, blocking=False)
    detQueue = detectionNetwork.out.createOutputQueue(maxSize=4, blocking=False)
    focusedDepthQueue = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)
    focusedConfQueue = depth.focusedConfidence.createOutputQueue(maxSize=4, blocking=False)

    pipeline.build()

    print("Resolved depth algorithm:", depth.getResolvedAlgorithm())
    print("Resolved depth config:", depth.getResolvedConfig())

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            inRgb = rgbQueue.get()
            inDet = detQueue.get()
            inFocusedDepth = focusedDepthQueue.get()
            inFocusedConf = focusedConfQueue.get()

            if inRgb is not None and inDet is not None:
                frame = inRgb.getCvFrame()
                for det in inDet.detections:
                    bbox = (
                        int(det.xmin * frame.shape[1]),
                        int(det.ymin * frame.shape[0]),
                        int(det.xmax * frame.shape[1]),
                        int(det.ymax * frame.shape[0]),
                    )
                    cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                cv2.imshow("rgb", frame)

            if inFocusedDepth is not None:
                cv2.imshow("focused depth", colorizeDepth(inFocusedDepth.getFrame()))

            if inFocusedConf is not None:
                cv2.imshow("focused confidence", colorizeConfidence(inFocusedConf.getFrame()))

            if cv2.waitKey(1) == ord("q"):
                pipeline.stop()
                break

        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
