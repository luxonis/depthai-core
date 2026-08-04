#!/usr/bin/env python3

import argparse

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
parser.add_argument("--backend", choices=BACKENDS, default="dsp-gpu")
parser.add_argument("--fps", type=float, default=30)
parser.add_argument("--extended", action="store_true")
parser.add_argument("--raw", action="store_true")
parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4))
parser.add_argument("--median", choices=MEDIANS)
parser.add_argument("--no-speckle", action="store_true")
parser.add_argument("--no-spatial", action="store_true")
parser.add_argument("--no-temporal", action="store_true")
parser.add_argument("--confidence", type=int)
lr = parser.add_mutually_exclusive_group()
lr.add_argument("--lr", dest="lr", action="store_true")
lr.add_argument("--no-lr", dest="lr", action="store_false")
parser.set_defaults(lr=None)
args = parser.parse_args()

if args.extended and args.backend.startswith("dsp-rvc2"):
    parser.error("--extended is not supported by DSP_RVC2 backends")

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Avoid the initial FPS dip
pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)

# Linking
monoLeftOut = monoLeft.requestOutput((1280, 800), fps=args.fps)
monoRightOut = monoRight.requestOutput((1280, 800), fps=args.fps)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

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
if args.no_spatial:
    config.postProcessing.spatialFilter.enable = False
if args.no_temporal:
    config.postProcessing.temporalFilter.enable = False

benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
stereo.disparity.link(benchmarkIn.input)

disparityQueue = stereo.disparity.createOutputQueue()
# stereoLeftQueue = stereo.rectifiedLeft.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

print(f"Backend: {args.backend}")
print(f"LR check: {config.algorithmControl.enableLeftRightCheck}")
print(f"Decimation: {config.postProcessing.decimationFilter.decimationFactor}")
print(f"Median: {config.postProcessing.median}")

with pipeline:
    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        # rectifiedLeft = stereoLeftQueue.get()
        assert isinstance(disparity, dai.ImgFrame)
        # assert isinstance(rectifiedLeft, dai.ImgFrame)
        # cv2.imshow("rectified left", rectifiedLeft.getCvFrame())
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break
