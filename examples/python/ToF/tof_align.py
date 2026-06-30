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


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
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
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        depthFrameColor[invalidMask] = 0
    except IndexError:
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
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

            depthColorized = colorizeDepth(frameDepth.getFrame())
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
