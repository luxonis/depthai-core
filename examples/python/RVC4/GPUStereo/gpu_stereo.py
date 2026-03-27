#!/usr/bin/env python3

import sys
import time
import argparse

import cv2
import depthai as dai
import numpy as np

if not hasattr(dai.node, "GPUStereo"):
    ver = getattr(dai, "__version__", "?")
    loc = getattr(dai, "__file__", "?")
    print(
        "dai.node.GPUStereo is missing: the installed depthai package was not built from this depthai-core tree.\n"
        "Rebuild/install Python bindings from the same checkout that adds GPUStereo, e.g.:\n"
        "  cd external/depthai-core/bindings/python   # or depthai-core/bindings/python\n"
        "  python3 -m pip install -e .\n"
        "Or build depthai-core with DEPTHAI_BUILD_PYTHON=ON and set PYTHONPATH to build/bindings/python.\n"
        f"Current depthai version={ver}, loaded from {loc}",
        file=sys.stderr,
    )
    sys.exit(1)


def colorize_disparity_u16(frame_u16: np.ndarray) -> np.ndarray:
    valid = frame_u16[frame_u16 > 0]
    if valid.size == 0:
        return np.zeros((*frame_u16.shape, 3), dtype=np.uint8)
    lo, hi = np.percentile(valid, [2, 98])
    if hi <= lo:
        hi = float(np.max(valid)) + 1.0
    norm = np.clip((frame_u16.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)


def colorize_depth_u16(frame_u16: np.ndarray) -> np.ndarray:
    small = frame_u16[::4, ::4]
    nz = small[small > 0]
    if nz.size == 0:
        return np.zeros((*frame_u16.shape, 3), dtype=np.uint8)
    lo = float(np.percentile(nz, 1))
    hi = float(np.percentile(nz, 99))
    if hi <= lo:
        hi = lo + 1.0
    norm = np.clip((frame_u16.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_HOT)


parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run without displaying images")
parser.add_argument("--device", type=str, default="10.11.0.51", help="Device IP address")
parser.add_argument("--resolution", type=str, default="1280x800", help="Resolution")
args = parser.parse_args()
width = int(args.resolution.split("x")[0])
height = int(args.resolution.split("x")[1])

device = dai.Device(args.device)
pipeline = dai.Pipeline(device)
mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
gpu = pipeline.create(dai.node.GPUStereo)

mono_left_out = mono_left.requestOutput((width, height), type=dai.ImgFrame.Type.GRAY8)
mono_right_out = mono_right.requestOutput((width, height), type=dai.ImgFrame.Type.GRAY8)
mono_left_out.link(gpu.left)
mono_right_out.link(gpu.right)

gpu.setRectification(True)

c = gpu.initialConfig
G = dai.GPUStereoConfig

c.maxDisparity = 128
c.numPyramidLevels = 3
c.subpixelBits = 4
c.useHostPtrBuffers = True
c.useQcomAcceleratedOps = False
c.useFp16 = True
c.downsampleMethod = G.DownsampleMethod.BOX_FILTER
c.prefilterMethod = G.PrefilterMethod.NONE
c.costMethod = G.CostMethod.ZNCC
c.blockMatchRadius = 3
c.secondPeakThreshold = 0.0
c.useCostVolume = False
c.lrCheck = True
c.lrCheckFast = True
c.medianSize = 3
c.confidenceThreshold = 25
c.temporalAlpha = 0.0

c.algorithmControl.depthUnit = dai.DepthUnit.MILLIMETER
c.algorithmControl.customDepthUnitMultiplier = 1000.0

disp_q = gpu.disparity.createOutputQueue()
# depth_q = gpu.depth.createOutputQueue()

with pipeline:
    pipeline.start()
    fps_window_start = time.monotonic()
    fps_window_frames = 0
    while pipeline.isRunning():
        disp = disp_q.get()
        # dep = depth_q.get()
        assert isinstance(disp, dai.ImgFrame)
        # assert isinstance(dep, dai.ImgFrame)
        d_u16 = disp.getFrame()
        fps_window_frames += 1
        now = time.monotonic()
        elapsed = now - fps_window_start
        if elapsed >= 5.0:
            print(f"FPS: {fps_window_frames / elapsed:.2f}")
            fps_window_start = now
            fps_window_frames = 0
        # z_u16 = dep.getFrame()
        if not args.headless:
            cv2.imshow("gpu stereo disparity", colorize_disparity_u16(d_u16))
            # cv2.imshow("gpu stereo depth (uint16)", colorize_depth_u16(z_u16))
            if cv2.waitKey(1) == ord("q"):
                pipeline.stop()
                break
