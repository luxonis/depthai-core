#!/usr/bin/env python3
"""Align ToF depth over left or right camera and show a blended overlay.

Usage:
    python tof_align_overlay.py --camera left   # align over CAM_B (default)
    python tof_align_overlay.py --camera right  # align over CAM_C
"""

import argparse
from datetime import timedelta

import cv2
import depthai as dai
import numpy as np


FPS = 10.0
CAMERA_SIZE = (640, 400)

# show depth in range 0.1m - 7m
MIN_DEPTH = 100
MAX_DEPTH = 7000


def colorizeDepth(frameDepth: np.ndarray, minDepth: float, maxDepth: float) -> np.ndarray:
    invalidMask = frameDepth == 0
    try:
        logDepth = np.log(frameDepth.astype(np.float32) + 1e-6)
        logDepth[invalidMask] = 0.0
        logMin, logMax = np.log(minDepth + 1e-6), np.log(maxDepth + 1e-6)
        logDepth = np.clip(logDepth, logMin, logMax)
        # Map from the FIXED depth range (not the per-frame min/max) so a given depth
        # always maps to the same color -- otherwise the mapping shifts every frame and
        # the image flickers.
        depthFrameColor = np.interp(logDepth, (logMin, logMax), (0, 255))
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        depthFrameColor[invalidMask] = 0
    except (IndexError, ValueError):
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    return depthFrameColor


rgbWeight = 0.5
depthWeight = 0.5


def updateBlendWeights(percentRgb):
    global rgbWeight, depthWeight
    rgbWeight = float(percentRgb) / 100.0
    depthWeight = 1.0 - rgbWeight


def main():
    parser = argparse.ArgumentParser(description="ToF depth overlay on left or right camera")
    parser.add_argument(
        "--camera",
        choices=["left", "right"],
        default="left",
        help="Camera to align depth onto: left=CAM_B, right=CAM_C (default: left)",
    )
    args = parser.parse_args()

    pipeline = dai.Pipeline()

    camera_sockets = {
        "left": dai.CameraBoardSocket.CAM_B,
        "right": dai.CameraBoardSocket.CAM_C,
    }

    align_socket = camera_sockets[args.camera]
    print(f"Aligning ToF depth over {args.camera} camera ({align_socket})")

    tof = pipeline.create(dai.node.ToF)
    tof.build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=dai.ToFConfig.Profile.MID_RANGE,
        fps=FPS,
    )

    cam = pipeline.create(dai.node.Camera).build(align_socket)
    camOut = cam.requestOutput(CAMERA_SIZE, enableUndistortion=True, fps=FPS)

    align = pipeline.create(dai.node.ImageAlign)
    align.setRunOnHost(True)
    tof.depth.link(align.input)
    camOut.link(align.inputAlignTo)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(seconds=0.5 / FPS))
    sync.setRunOnHost(True)
    camOut.link(sync.inputs["rgb"])
    align.outputAligned.link(sync.inputs["depth_aligned"])
    sync.inputs["rgb"].setBlocking(False)

    syncQueue = sync.out.createOutputQueue()

    window_blend = f"tof-overlay-{args.camera}"
    window_depth = "depth-aligned"

    with pipeline as p:
        p.start()
        cv2.namedWindow(window_blend)
        cv2.namedWindow(window_depth)
        cv2.createTrackbar("RGB Weight %", window_blend, int(rgbWeight * 100), 100, updateBlendWeights)

        while p.isRunning():
            msgGroup = syncQueue.get()
            assert isinstance(msgGroup, dai.MessageGroup)

            frameRgb = msgGroup["rgb"]
            frameDepth = msgGroup["depth_aligned"]

            cvFrame = frameRgb.getCvFrame()
            if len(cvFrame.shape) == 2:
                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_GRAY2BGR)

            depthColorized = colorizeDepth(frameDepth.getFrame(), MIN_DEPTH, MAX_DEPTH)
            if depthColorized.shape[:2] != cvFrame.shape[:2]:
                depthColorized = cv2.resize(
                    depthColorized, (cvFrame.shape[1], cvFrame.shape[0])
                )

            cv2.imshow(window_depth, depthColorized)

            blended = cv2.addWeighted(cvFrame, rgbWeight, depthColorized, depthWeight, 0)
            cv2.imshow(window_blend, blended)

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
