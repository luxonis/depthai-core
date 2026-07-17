#!/usr/bin/env python3
"""
Focused Depth PoC demo.

The Depth node can compute a *focused* depth map: a full-frame map in which only
the regions described by an ImgDetections message are filled (the rest is zero).
The detections are fed into Depth.inputDetections and the result is read from
Depth.focusedDepth / Depth.focusedConfidence.

Two ways to provide the detections (spec use cases 1 and 2):

  --mode detector (default): run an object-detection network on the color camera
    and link detectionNetwork.out -> depth.inputDetections. The color-frame
    detections are automatically remapped to the left stereo frame using the
    ImgDetections transformation.

  --mode roi: skip the network and publish a fixed region-of-interest from the
    host into depth.inputDetections. Useful to demo focused depth without a
    detection model (e.g. when no Hub model is available).

Works on RVC4 devices.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np


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
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8) if vmax > 0 else np.zeros(frame.shape, dtype=np.uint8)
    else:
        vis = frame
    colored = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    colored[vis == 0] = 0
    return colored


def parseRoi(text: str) -> tuple[float, float, float, float]:
    parts = [float(p) for p in text.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("--roi expects 'xmin,ymin,xmax,ymax'")
    xmin, ymin, xmax, ymax = parts
    if not (0.0 <= xmin < xmax <= 1.0 and 0.0 <= ymin < ymax <= 1.0):
        raise argparse.ArgumentTypeError("--roi values must be normalized with xmin<xmax and ymin<ymax")
    return xmin, ymin, xmax, ymax


def makeDetections(rois: list[tuple[float, float, float, float]]) -> dai.ImgDetections:
    detections = []
    for xmin, ymin, xmax, ymax in rois:
        det = dai.ImgDetection()
        det.label = 0
        det.confidence = 1.0
        det.xmin, det.ymin, det.xmax, det.ymax = xmin, ymin, xmax, ymax
        detections.append(det)
    dets = dai.ImgDetections()
    dets.detections = detections
    dets.setTimestamp(dai.Clock.now())
    return dets


def buildParser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=["detector", "roi"],
        default="detector",
        help="How to source the detections fed to Depth.inputDetections (default: detector)",
    )
    parser.add_argument(
        "--model",
        default="yolov6-nano",
        help="Model description for the DetectionNetwork in detector mode (default: yolov6-nano)",
    )
    parser.add_argument(
        "--roi",
        type=parseRoi,
        default=(0.35, 0.30, 0.65, 0.70),
        help="Normalized 'xmin,ymin,xmax,ymax' ROI published in roi mode (default: centered box)",
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

    depth = pipeline.create(dai.node.Depth)
    if args.fps is not None:
        depth.build(args.fps)
    else:
        depth.build()

    rgbQueue = None
    detQueue = None
    roiInputQueue = None

    if args.mode == "detector":
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
        detectionNetwork.setConfidenceThreshold(0.5)
        detectionNetwork.out.link(depth.inputDetections)
        rgbQueue = detectionNetwork.passthrough.createOutputQueue(maxSize=4, blocking=False)
        detQueue = detectionNetwork.out.createOutputQueue(maxSize=4, blocking=False)
    else:
        roiInputQueue = depth.inputDetections.createInputQueue()

    focusedDepthQueue = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)
    focusedConfQueue = depth.focusedConfidence.createOutputQueue(maxSize=4, blocking=False)

    pipeline.build()

    print("Mode:", args.mode)
    print("Resolved depth algorithm:", depth.getResolvedAlgorithm())
    print("Resolved depth config:", depth.getResolvedConfig())

    stopEvent = threading.Event()

    def roiSender():
        # Publish the ROI continuously with fresh timestamps so the FocusController's
        # Sync can group it with the device's left/right frames.
        while not stopEvent.is_set():
            try:
                roiInputQueue.send(makeDetections([args.roi]))
            except Exception:
                return
            time.sleep(0.015)

    with pipeline:
        pipeline.start()

        senderThread = None
        if roiInputQueue is not None:
            senderThread = threading.Thread(target=roiSender, daemon=True)
            senderThread.start()

        while pipeline.isRunning():
            if rgbQueue is not None and detQueue is not None:
                inRgb = rgbQueue.get()
                inDet = detQueue.get()
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

            inFocusedDepth = focusedDepthQueue.get()
            inFocusedConf = focusedConfQueue.get()

            if inFocusedDepth is not None:
                cv2.imshow("focused depth", colorizeDepth(inFocusedDepth.getFrame()))

            if inFocusedConf is not None:
                cv2.imshow("focused confidence", colorizeConfidence(inFocusedConf.getFrame()))

            if cv2.waitKey(1) == ord("q"):
                pipeline.stop()
                break

        stopEvent.set()
        if senderThread is not None:
            senderThread.join(timeout=2)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
