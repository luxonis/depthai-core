#!/usr/bin/env python3

import argparse
import os
import time
from collections import deque

import cv2
import depthai as dai
import numpy as np

BACKENDS = {
    "eva": dai.StereoDepthProperties.StereoBackend.EVA,
    "dsp-gpu": dai.StereoDepthProperties.StereoBackend.DSP_GPU,
    "dsp-rvc2": dai.StereoDepthProperties.StereoBackend.DSP_RVC2,
    "dsp-rvc2-default": dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT,
    "dsp-rvc2-default-64": dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT_64,
}
MEDIANS = {
    "off": dai.MedianFilter.MEDIAN_OFF,
    "3x3": dai.MedianFilter.KERNEL_3x3,
    "5x5": dai.MedianFilter.KERNEL_5x5,
    "7x7": dai.MedianFilter.KERNEL_7x7,
}

parser = argparse.ArgumentParser()
parser.add_argument("--device", default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"), help="device IP, name, or ID")
parser.add_argument("--backend", choices=BACKENDS, default="dsp-rvc2-default-64")
parser.add_argument("--fps", type=float, default=25)
parser.add_argument("--extended", action="store_true")
parser.add_argument("--raw", action="store_true")
parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4))
parser.add_argument("--median", choices=MEDIANS)
parser.add_argument("--no-speckle", action="store_true")
parser.add_argument("--speckle-range", type=int, help="override speckle component size (RVC2 clamps DEFAULT to 114 at 1280x800 without decimation)")
parser.add_argument("--no-spatial", action="store_true")
parser.add_argument("--no-temporal", action="store_true")
parser.add_argument("--confidence", type=int)
parser.add_argument("--frames", type=int, default=0, help="stop after this many disparity frames; zero runs until q")
parser.add_argument("--no-display", action="store_true", help="disable imshow for headless performance testing")
lr = parser.add_mutually_exclusive_group()
lr.add_argument("--lr", dest="lr", action="store_true")
lr.add_argument("--no-lr", dest="lr", action="store_false")
parser.set_defaults(lr=None)
args = parser.parse_args()

if args.extended and args.backend.startswith("dsp-rvc2"):
    parser.error("--extended is not supported by DSP_RVC2 backends")

pipeline = dai.Pipeline(dai.Device(dai.DeviceInfo(args.device))) if args.device else dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

if args.backend in ("dsp-rvc2-default", "dsp-rvc2-default-64"):
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

# Avoid the initial FPS dip
pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)

# Linking
monoLeftOut = monoLeft.requestOutput((1280, 800), fps=args.fps)
monoRightOut = monoRight.requestOutput((1280, 800), fps=args.fps)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)
# neuralDepth = pipeline.create(dai.node.NeuralDepth).build(monoLeftOut, monoRightOut, dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM)
# benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
# neuralDepth.depth.link(benchmarkIn.input)

stereo.setRectification(True)
stereo.setStereoBackend(BACKENDS[args.backend])
config = stereo.initialConfig

if args.extended:
    config.setExtendedDisparity(True)
if args.lr is not None:
    config.setLeftRightCheck(args.lr)
if args.confidence is not None:
    config.setConfidenceThreshold(args.confidence)
if args.raw:
    config.postProcessing.decimationFilter.decimationFactor = 1
    config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    config.postProcessing.speckleFilter.enable = False
    config.postProcessing.spatialFilter.enable = False
    config.postProcessing.temporalFilter.enable = False
if args.decimation is not None:
    config.postProcessing.decimationFilter.decimationFactor = args.decimation
if args.median is not None:
    config.setMedianFilter(MEDIANS[args.median])
if args.no_speckle:
    config.postProcessing.speckleFilter.enable = False
if args.speckle_range is not None:
    config.postProcessing.speckleFilter.speckleRange = args.speckle_range
if args.no_spatial:
    config.postProcessing.spatialFilter.enable = False
if args.no_temporal:
    config.postProcessing.temporalFilter.enable = False

disparityQueue = stereo.disparity.createOutputQueue(maxSize=4, blocking=False)
# stereoLeftQueue = stereo.rectifiedLeft.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

print(f"Backend: {args.backend}")
print(f"LR check: {config.algorithmControl.enableLeftRightCheck}")
print(f"Decimation: {config.postProcessing.decimationFilter.decimationFactor}")
print(f"Median: {config.postProcessing.median}")
print(f"Speckle: {config.postProcessing.speckleFilter.enable}")
print(f"Speckle range: {config.postProcessing.speckleFilter.speckleRange}")
print(f"Spatial: {config.postProcessing.spatialFilter.enable}")
print(f"Temporal: {config.postProcessing.temporalFilter.enable}")
if not args.no_display:
    print("Press q in the disparity window to stop")

with pipeline:
    pipeline.start()
    frameTimes = deque(maxlen=90)
    lastReport = time.monotonic()
    frameCount = 0
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        # rectifiedLeft = stereoLeftQueue.get()
        assert isinstance(disparity, dai.ImgFrame)
        # assert isinstance(rectifiedLeft, dai.ImgFrame)
        # cv2.imshow("rectified left", rectifiedLeft.getCvFrame())
        npDisparity = disparity.getFrame()
        now = time.monotonic()
        frameTimes.append(now)
        hostFps = (len(frameTimes) - 1) / (frameTimes[-1] - frameTimes[0]) if len(frameTimes) > 1 else 0.0
        maxDisparity = max(1, int(np.max(npDisparity)))
        if not args.no_display:
            colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
            colorizedDisparity[npDisparity == 0] = 0
            cv2.putText(colorizedDisparity, f"{hostFps:.1f} FPS", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("disparity", colorizedDisparity)
        if now - lastReport >= 1.0:
            print(f"Host disparity: {hostFps:.2f} FPS, shape={npDisparity.shape}, range=0..{maxDisparity}")
            lastReport = now
        frameCount += 1
        key = cv2.waitKey(1) if not args.no_display else -1
        if key == ord("q") or (args.frames and frameCount >= args.frames):
            pipeline.stop()
            break

cv2.destroyAllWindows()
