#!/usr/bin/env python3
"""Minimal GPUStereo example — disparity from a stereo camera pair (RVC4 only)."""

import argparse
import cv2
import depthai as dai
import numpy as np

parser = argparse.ArgumentParser(description="Minimal GPUStereo disparity example (RVC4 only)")
parser.add_argument("--device", "-d", type=str, default=None, help="Device IP address/host (default: auto-discover)")
args = parser.parse_args()

device = dai.Device(args.device) if args.device else dai.Device()

if not device.isGpuStereoSupported():
    print("Exiting GPUStereo example: GPUStereo is not supported on this device.")
    raise SystemExit(0)

pipeline = dai.Pipeline(device)
cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

gpu = pipeline.create(dai.node.GPUStereo)
cam_left.requestOutput((1280, 800), type=dai.ImgFrame.Type.GRAY8, fps=30).link(gpu.left)
cam_right.requestOutput((1280, 800), type=dai.ImgFrame.Type.GRAY8, fps=30).link(gpu.right)
gpu.setRectification(True)
gpu.initialConfig.setConfidenceThreshold(25)

disp_q = gpu.disparity.createOutputQueue()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        frame = disp_q.get()
        d = frame.getFrame().astype(np.float32)
        d = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow("GPUStereo Disparity", cv2.applyColorMap(d, cv2.COLORMAP_JET))
        if cv2.waitKey(1) == ord("q"):
            break
