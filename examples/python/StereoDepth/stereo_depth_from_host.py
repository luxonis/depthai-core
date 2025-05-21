#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai
from time import sleep
import datetime
import argparse
from pathlib import Path
import math
import os, re
import csv
import datetime

datasetDefault = str((Path(__file__).parent / Path("../models/dataset")).resolve().absolute())
parser = argparse.ArgumentParser()
parser.add_argument("-p", "--dataset", nargs="?", help="Path to recorded frames", default=None)
parser.add_argument("-d", "--debug", action="store_true", help="Enable debug outputs.")
parser.add_argument("-e", "--evaluate", help="Evaluate the disparity calculation.", default=None)
parser.add_argument("-dumpdispcost", "--dumpdisparitycostvalues", action="store_true", help="Dumps the disparity cost values for each disparity range. 96 byte for each pixel.")
parser.add_argument("--download", action="store_true", help="Downloads the 2014 Middlebury dataset.")
parser.add_argument("--calibration", help="Path to calibration file", default=None)
parser.add_argument("--rectify", action="store_true", help="Enable rectified streams")
parser.add_argument("--swapLR", action="store_true", help="Swap left and right cameras.")
args = parser.parse_args()

if args.evaluate is not None and args.dataset is not None:
    import sys
    raise ValueError("Cannot use both --dataset and --evaluate arguments at the same time.")

evaluation_mode = args.evaluate is not None
args.dataset = args.dataset or datasetDefault

if args.download and args.evaluate is None:
    import sys
    raise ValueError("Cannot use --download without --evaluate argument.")

if args.evaluate is None and not Path(args.dataset).exists():
    import sys
    raise FileNotFoundError(f"Required file/s not found, please run '{sys.executable} install_requirements.py'")

if args.evaluate is not None and not args.download and not Path(args.evaluate).exists():
    import sys
    raise FileNotFoundError(f"Evaluation dataset path does not exist, use the --evaluate argument to specify the path.")

if args.evaluate is not None and args.download and not Path(args.evaluate).exists():
    os.makedirs(args.evaluate)

def download_2014_middlebury(data_path):
    import requests, zipfile, io
    url = "https://vision.middlebury.edu/stereo/data/scenes2014/zip/"
    r = requests.get(url)
    c = r.content
    reg = re.compile(r"href=('|\")(.+\.zip)('|\")")
    matches = reg.findall(c.decode("utf-8"))
    files = [m[1] for m in matches]

    for f in files:
        if os.path.isdir(os.path.join(data_path, f[:-4])):
            print(f"Skipping {f}")
        else:
            print(f"Downloading {f} from {url + f}")
            r = requests.get(url + f)
            print(f"Extracting {f} to {data_path}")
            z = zipfile.ZipFile(io.BytesIO(r.content))
            z.extractall(data_path)

if args.download:
    download_2014_middlebury(args.evaluate)
    if not evaluation_mode:
        sys.exit(0)

class StereoConfigHandlerRVC2:

    class Trackbar:
        def __init__(self, trackbarName, windowName, minValue, maxValue, defaultValue, handler):
            self.min = minValue
            self.max = maxValue
            self.windowName = windowName
            self.trackbarName = trackbarName
            cv2.createTrackbar(trackbarName, windowName, minValue, maxValue, handler)
            cv2.setTrackbarPos(trackbarName, windowName, defaultValue)

        def set(self, value):
            if value < self.min:
                value = self.min
                print(f"{self.trackbarName} min value is {self.min}")
            if value > self.max:
                value = self.max
                print(f"{self.trackbarName} max value is {self.max}")
            cv2.setTrackbarPos(self.trackbarName, self.windowName, value)

    class CensusMaskHandler:

        stateColor = [(0, 0, 0), (255, 255, 255)]
        gridHeight = 50
        gridWidth = 50

        def fillRectangle(self, row, col):
            src = self.gridList[row][col]["topLeft"]
            dst = self.gridList[row][col]["bottomRight"]

            stateColor = self.stateColor[1] if self.gridList[row][col]["state"] else self.stateColor[0]
            self.changed = True

            cv2.rectangle(self.gridImage, src, dst, stateColor, -1)
            cv2.imshow(self.windowName, self.gridImage)


        def clickCallback(self, event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                col = x * (self.gridSize[1]) // self.width
                row = y * (self.gridSize[0]) // self.height
                self.gridList[row][col]["state"] = not self.gridList[row][col]["state"]
                self.fillRectangle(row, col)


        def __init__(self, windowName, gridSize):
            self.gridSize = gridSize
            self.windowName = windowName
            self.changed = False

            cv2.namedWindow(self.windowName)

            self.width = StereoConfigHandlerRVC2.CensusMaskHandler.gridWidth * self.gridSize[1]
            self.height = StereoConfigHandlerRVC2.CensusMaskHandler.gridHeight * self.gridSize[0]

            self.gridImage = np.zeros((self.height + 50, self.width, 3), np.uint8)

            cv2.putText(self.gridImage, "Click on grid to change mask!", (0, self.height+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))
            cv2.putText(self.gridImage, "White: ON   |   Black: OFF", (0, self.height+40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

            self.gridList = [[dict() for _ in range(self.gridSize[1])] for _ in range(self.gridSize[0])]

            for row in range(self.gridSize[0]):
                rowFactor = self.height // self.gridSize[0]
                srcY = row*rowFactor + 1
                dstY = (row+1)*rowFactor - 1
                for col in range(self.gridSize[1]):
                    colFactor = self.width // self.gridSize[1]
                    srcX = col*colFactor + 1
                    dstX = (col+1)*colFactor - 1
                    src = (srcX, srcY)
                    dst = (dstX, dstY)
                    self.gridList[row][col]["topLeft"] = src
                    self.gridList[row][col]["bottomRight"] = dst
                    self.gridList[row][col]["state"] = False
                    self.fillRectangle(row, col)

            cv2.setMouseCallback(self.windowName, self.clickCallback)
            cv2.imshow(self.windowName, self.gridImage)

        def getMask(self) -> np.uint64:
            mask = np.uint64(0)
            for row in range(self.gridSize[0]):
                for col in range(self.gridSize[1]):
                    if self.gridList[row][col]["state"]:
                        pos = row*self.gridSize[1] + col
                        mask = np.bitwise_or(mask, np.uint64(1) << np.uint64(pos))

            return mask

        def setMask(self, _mask: np.uint64):
            mask = np.uint64(_mask)
            for row in range(self.gridSize[0]):
                for col in range(self.gridSize[1]):
                    pos = row*self.gridSize[1] + col
                    if np.bitwise_and(mask, np.uint64(1) << np.uint64(pos)):
                        self.gridList[row][col]["state"] = True
                    else:
                        self.gridList[row][col]["state"] = False

                    self.fillRectangle(row, col)

        def isChanged(self):
            changed = self.changed
            self.changed = False
            return changed

        def destroyWindow(self):
            cv2.destroyWindow(self.windowName)


    censusMaskHandler = None
    newConfig = False
    config = None
    trSigma = list()
    trConfidence = list()
    trLrCheck = list()
    trFractionalBits = list()
    trLineqAlpha = list()
    trLineqBeta = list()
    trLineqThreshold = list()
    trCostAggregationP1 = list()
    trCostAggregationP2 = list()
    trTemporalAlpha = list()
    trTemporalDelta = list()
    trThresholdMinRange = list()
    trThresholdMaxRange = list()
    trSpeckleRange = list()
    trSpatialAlpha = list()
    trSpatialDelta = list()
    trSpatialHoleFilling = list()
    trSpatialNumIterations = list()
    trDecimationFactor = list()
    trDisparityShift = list()
    trCenterAlignmentShift = list()
    trInvalidateEdgePixels = list()

    def trackbarSigma(value):
        StereoConfigHandlerRVC2.config.postProcessing.bilateralSigmaValue = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSigma:
            tr.set(value)

    def trackbarConfidence(value):
        StereoConfigHandlerRVC2.config.costMatching.confidenceThreshold = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trConfidence:
            tr.set(value)

    def trackbarLrCheckThreshold(value):
        StereoConfigHandlerRVC2.config.algorithmControl.leftRightCheckThreshold = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trLrCheck:
            tr.set(value)

    def trackbarFractionalBits(value):
        StereoConfigHandlerRVC2.config.algorithmControl.subpixelFractionalBits = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trFractionalBits:
            tr.set(value)

    def trackbarLineqAlpha(value):
        StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.alpha = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trLineqAlpha:
            tr.set(value)

    def trackbarLineqBeta(value):
        StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.beta = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trLineqBeta:
            tr.set(value)

    def trackbarLineqThreshold(value):
        StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.threshold = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trLineqThreshold:
            tr.set(value)

    def trackbarCostAggregationP1(value):
        StereoConfigHandlerRVC2.config.costAggregation.horizontalPenaltyCostP1 = value
        StereoConfigHandlerRVC2.config.costAggregation.verticalPenaltyCostP1 = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trCostAggregationP1:
            tr.set(value)

    def trackbarCostAggregationP2(value):
        StereoConfigHandlerRVC2.config.costAggregation.horizontalPenaltyCostP2 = value
        StereoConfigHandlerRVC2.config.costAggregation.verticalPenaltyCostP2 = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trCostAggregationP2:
            tr.set(value)

    def trackbarTemporalFilterAlpha(value):
        StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.alpha = value / 100.
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trTemporalAlpha:
            tr.set(value)

    def trackbarTemporalFilterDelta(value):
        StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.delta = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trTemporalDelta:
            tr.set(value)

    def trackbarSpatialFilterAlpha(value):
        StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.alpha = value / 100.
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSpatialAlpha:
            tr.set(value)

    def trackbarSpatialFilterDelta(value):
        StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.delta = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSpatialDelta:
            tr.set(value)

    def trackbarSpatialFilterHoleFillingRadius(value):
        StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.holeFillingRadius = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSpatialHoleFilling:
            tr.set(value)

    def trackbarSpatialFilterNumIterations(value):
        StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.numIterations = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSpatialNumIterations:
            tr.set(value)

    def trackbarThresholdMinRange(value):
        StereoConfigHandlerRVC2.config.postProcessing.thresholdFilter.minRange = value * 1000
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trThresholdMinRange:
            tr.set(value)

    def trackbarThresholdMaxRange(value):
        StereoConfigHandlerRVC2.config.postProcessing.thresholdFilter.maxRange = value * 1000
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trThresholdMaxRange:
            tr.set(value)

    def trackbarSpeckleRange(value):
        StereoConfigHandlerRVC2.config.postProcessing.speckleFilter.speckleRange = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trSpeckleRange:
            tr.set(value)

    def trackbarDecimationFactor(value):
        StereoConfigHandlerRVC2.config.postProcessing.decimationFilter.decimationFactor = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trDecimationFactor:
            tr.set(value)

    def trackbarDisparityShift(value):
        StereoConfigHandlerRVC2.config.algorithmControl.disparityShift = value
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trDisparityShift:
            tr.set(value)

    def trackbarCenterAlignmentShift(value):
        if StereoConfigHandlerRVC2.config.algorithmControl.depthAlign != dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER:
            print("Center alignment shift factor requires CENTER alignment enabled!")
            return
        StereoConfigHandlerRVC2.config.algorithmControl.centerAlignmentShiftFactor = value / 100.
        print(f"centerAlignmentShiftFactor: {StereoConfigHandlerRVC2.config.algorithmControl.centerAlignmentShiftFactor:.2f}")
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trCenterAlignmentShift:
            tr.set(value)

    def trackbarInvalidateEdgePixels(value):
        StereoConfigHandlerRVC2.config.algorithmControl.numInvalidateEdgePixels = value
        print(f"numInvalidateEdgePixels: {StereoConfigHandlerRVC2.config.algorithmControl.numInvalidateEdgePixels:.2f}")
        StereoConfigHandlerRVC2.newConfig = True
        for tr in StereoConfigHandlerRVC2.trInvalidateEdgePixels:
            tr.set(value)

    def handleKeypress(key, stereoDepthConfigInQueue):
        if key == ord("m"):
            StereoConfigHandlerRVC2.newConfig = True
            medianSettings = [dai.MedianFilter.MEDIAN_OFF, dai.MedianFilter.KERNEL_3x3, dai.MedianFilter.KERNEL_5x5, dai.MedianFilter.KERNEL_7x7]
            currentMedian = StereoConfigHandlerRVC2.config.postProcessing.median
            nextMedian = medianSettings[(medianSettings.index(currentMedian)+1) % len(medianSettings)]
            print(f"Changing median to {nextMedian.name} from {currentMedian.name}")
            StereoConfigHandlerRVC2.config.postProcessing.median = nextMedian
        if key == ord("w"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.enable = not StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.enable
            state = "on" if StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.enable else "off"
            print(f"Spatial filter {state}")
        if key == ord("t"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.enable = not StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.enable
            state = "on" if StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.enable else "off"
            print(f"Temporal filter {state}")
        if key == ord("s"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.postProcessing.speckleFilter.enable = not StereoConfigHandlerRVC2.config.postProcessing.speckleFilter.enable
            state = "on" if StereoConfigHandlerRVC2.config.postProcessing.speckleFilter.enable else "off"
            print(f"Speckle filter {state}")
        if key == ord("r"):
            StereoConfigHandlerRVC2.newConfig = True
            temporalSettings = [dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_OFF,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_8_OUT_OF_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_OUT_OF_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_2,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_5,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_INDEFINITELY,
            ]
            currentTemporal = StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.persistencyMode
            nextTemporal = temporalSettings[(temporalSettings.index(currentTemporal)+1) % len(temporalSettings)]
            print(f"Changing temporal persistency to {nextTemporal.name} from {currentTemporal.name}")
            StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.persistencyMode = nextTemporal
        if key == ord("n"):
            StereoConfigHandlerRVC2.newConfig = True
            decimationSettings = [dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING,
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN,
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN,
            ]
            currentDecimation = StereoConfigHandlerRVC2.config.postProcessing.decimationFilter.decimationMode
            nextDecimation = decimationSettings[(decimationSettings.index(currentDecimation)+1) % len(decimationSettings)]
            print(f"Changing decimation mode to {nextDecimation.name} from {currentDecimation.name}")
            StereoConfigHandlerRVC2.config.postProcessing.decimationFilter.decimationMode = nextDecimation
        if key == ord("a"):
            StereoConfigHandlerRVC2.newConfig = True
            alignmentSettings = [dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT,
            dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT,
            dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER,
            ]
            currentAlignment = StereoConfigHandlerRVC2.config.algorithmControl.depthAlign
            nextAlignment = alignmentSettings[(alignmentSettings.index(currentAlignment)+1) % len(alignmentSettings)]
            print(f"Changing alignment mode to {nextAlignment.name} from {currentAlignment.name}")
            StereoConfigHandlerRVC2.config.algorithmControl.depthAlign = nextAlignment
        elif key == ord("c"):
            StereoConfigHandlerRVC2.newConfig = True
            censusSettings = [dai.StereoDepthConfig.CensusTransform.KernelSize.AUTO, dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5, dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x7, dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x9]
            currentCensus = StereoConfigHandlerRVC2.config.censusTransform.kernelSize
            nextCensus = censusSettings[(censusSettings.index(currentCensus)+1) % len(censusSettings)]
            if nextCensus != dai.StereoDepthConfig.CensusTransform.KernelSize.AUTO:
                censusGridSize = [(5,5), (7,7), (7,9)]
                censusDefaultMask = [np.uint64(0XA82415), np.uint64(0XAA02A8154055), np.uint64(0X2AA00AA805540155)]
                censusGrid = censusGridSize[nextCensus]
                censusMask = censusDefaultMask[nextCensus]
                StereoConfigHandlerRVC2.censusMaskHandler = StereoConfigHandlerRVC2.CensusMaskHandler("Census mask", censusGrid)
                StereoConfigHandlerRVC2.censusMaskHandler.setMask(censusMask)
            else:
                print("Census mask config is not available in AUTO census kernel mode. Change using the 'c' key")
                StereoConfigHandlerRVC2.config.censusTransform.kernelMask = 0
                StereoConfigHandlerRVC2.censusMaskHandler.destroyWindow()
            print(f"Changing census transform to {nextCensus.name} from {currentCensus.name}")
            StereoConfigHandlerRVC2.config.censusTransform.kernelSize = nextCensus
        elif key == ord("d"):
            StereoConfigHandlerRVC2.newConfig = True
            dispRangeSettings = [dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64, dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_96]
            currentDispRange = StereoConfigHandlerRVC2.config.costMatching.disparityWidth
            nextDispRange = dispRangeSettings[(dispRangeSettings.index(currentDispRange)+1) % len(dispRangeSettings)]
            print(f"Changing disparity range to {nextDispRange.name} from {currentDispRange.name}")
            StereoConfigHandlerRVC2.config.costMatching.disparityWidth = nextDispRange
        elif key == ord("f"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.costMatching.enableCompanding = not StereoConfigHandlerRVC2.config.costMatching.enableCompanding
            state = "on" if StereoConfigHandlerRVC2.config.costMatching.enableCompanding else "off"
            print(f"Companding {state}")
        elif key == ord("v"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.censusTransform.enableMeanMode = not StereoConfigHandlerRVC2.config.censusTransform.enableMeanMode
            state = "on" if StereoConfigHandlerRVC2.config.censusTransform.enableMeanMode else "off"
            print(f"Census transform mean mode {state}")
        elif key == ord("1"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.algorithmControl.enableLeftRightCheck = not StereoConfigHandlerRVC2.config.algorithmControl.enableLeftRightCheck
            state = "on" if StereoConfigHandlerRVC2.config.algorithmControl.enableLeftRightCheck else "off"
            print(f"LR-check {state}")
        elif key == ord("2"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.algorithmControl.enableSubpixel = not StereoConfigHandlerRVC2.config.algorithmControl.enableSubpixel
            state = "on" if StereoConfigHandlerRVC2.config.algorithmControl.enableSubpixel else "off"
            print(f"Subpixel {state}")
        elif key == ord("3"):
            StereoConfigHandlerRVC2.newConfig = True
            StereoConfigHandlerRVC2.config.algorithmControl.enableExtended = not StereoConfigHandlerRVC2.config.algorithmControl.enableExtended
            state = "on" if StereoConfigHandlerRVC2.config.algorithmControl.enableExtended else "off"
            print(f"Extended {state}")

        censusMaskChanged = False
        if StereoConfigHandlerRVC2.censusMaskHandler is not None:
            censusMaskChanged = StereoConfigHandlerRVC2.censusMaskHandler.isChanged()
        if censusMaskChanged:
            StereoConfigHandlerRVC2.config.censusTransform.kernelMask = StereoConfigHandlerRVC2.censusMaskHandler.getMask()
            StereoConfigHandlerRVC2.newConfig = True

        StereoConfigHandlerRVC2.sendConfig(stereoDepthConfigInQueue)

    def sendConfig(stereoDepthConfigInQueue):
        if StereoConfigHandlerRVC2.newConfig:
            StereoConfigHandlerRVC2.newConfig = False
            # configMessage = dai.StereoDepthConfig()
            configMessage = StereoConfigHandlerRVC2.config
            stereoDepthConfigInQueue.send(configMessage)

    def updateDefaultConfig(config):
        StereoConfigHandlerRVC2.config = config

    def registerWindow(stream):
        cv2.namedWindow(stream, cv2.WINDOW_NORMAL)

        StereoConfigHandlerRVC2.trConfidence.append(StereoConfigHandlerRVC2.Trackbar("Disparity confidence", stream, 0, 255, StereoConfigHandlerRVC2.config.costMatching.confidenceThreshold, StereoConfigHandlerRVC2.trackbarConfidence))
        StereoConfigHandlerRVC2.trSigma.append(StereoConfigHandlerRVC2.Trackbar("Bilateral sigma", stream, 0, 100, StereoConfigHandlerRVC2.config.postProcessing.bilateralSigmaValue, StereoConfigHandlerRVC2.trackbarSigma))
        StereoConfigHandlerRVC2.trLrCheck.append(StereoConfigHandlerRVC2.Trackbar("LR-check threshold", stream, 0, 16, StereoConfigHandlerRVC2.config.algorithmControl.leftRightCheckThreshold, StereoConfigHandlerRVC2.trackbarLrCheckThreshold))
        StereoConfigHandlerRVC2.trFractionalBits.append(StereoConfigHandlerRVC2.Trackbar("Subpixel fractional bits", stream, 3, 5, StereoConfigHandlerRVC2.config.algorithmControl.subpixelFractionalBits, StereoConfigHandlerRVC2.trackbarFractionalBits))
        StereoConfigHandlerRVC2.trDisparityShift.append(StereoConfigHandlerRVC2.Trackbar("Disparity shift", stream, 0, 100, StereoConfigHandlerRVC2.config.algorithmControl.disparityShift, StereoConfigHandlerRVC2.trackbarDisparityShift))
        StereoConfigHandlerRVC2.trCenterAlignmentShift.append(StereoConfigHandlerRVC2.Trackbar("Center alignment shift factor", stream, 0, 100, StereoConfigHandlerRVC2.config.algorithmControl.centerAlignmentShiftFactor, StereoConfigHandlerRVC2.trackbarCenterAlignmentShift))
        StereoConfigHandlerRVC2.trInvalidateEdgePixels.append(StereoConfigHandlerRVC2.Trackbar("Invalidate edge pixels", stream, 0, 100, StereoConfigHandlerRVC2.config.algorithmControl.numInvalidateEdgePixels, StereoConfigHandlerRVC2.trackbarInvalidateEdgePixels))
        StereoConfigHandlerRVC2.trLineqAlpha.append(StereoConfigHandlerRVC2.Trackbar("Linear equation alpha", stream, 0, 15, StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.alpha, StereoConfigHandlerRVC2.trackbarLineqAlpha))
        StereoConfigHandlerRVC2.trLineqBeta.append(StereoConfigHandlerRVC2.Trackbar("Linear equation beta", stream, 0, 15, StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.beta, StereoConfigHandlerRVC2.trackbarLineqBeta))
        StereoConfigHandlerRVC2.trLineqThreshold.append(StereoConfigHandlerRVC2.Trackbar("Linear equation threshold", stream, 0, 255, StereoConfigHandlerRVC2.config.costMatching.linearEquationParameters.threshold, StereoConfigHandlerRVC2.trackbarLineqThreshold))
        StereoConfigHandlerRVC2.trCostAggregationP1.append(StereoConfigHandlerRVC2.Trackbar("Cost aggregation P1", stream, 0, 500, StereoConfigHandlerRVC2.config.costAggregation.horizontalPenaltyCostP1, StereoConfigHandlerRVC2.trackbarCostAggregationP1))
        StereoConfigHandlerRVC2.trCostAggregationP2.append(StereoConfigHandlerRVC2.Trackbar("Cost aggregation P2", stream, 0, 500, StereoConfigHandlerRVC2.config.costAggregation.horizontalPenaltyCostP2, StereoConfigHandlerRVC2.trackbarCostAggregationP2))
        StereoConfigHandlerRVC2.trTemporalAlpha.append(StereoConfigHandlerRVC2.Trackbar("Temporal filter alpha", stream, 0, 100, int(StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.alpha*100), StereoConfigHandlerRVC2.trackbarTemporalFilterAlpha))
        StereoConfigHandlerRVC2.trTemporalDelta.append(StereoConfigHandlerRVC2.Trackbar("Temporal filter delta", stream, 0, 100, StereoConfigHandlerRVC2.config.postProcessing.temporalFilter.delta, StereoConfigHandlerRVC2.trackbarTemporalFilterDelta))
        StereoConfigHandlerRVC2.trSpatialAlpha.append(StereoConfigHandlerRVC2.Trackbar("Spatial filter alpha", stream, 0, 100, int(StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.alpha*100), StereoConfigHandlerRVC2.trackbarSpatialFilterAlpha))
        StereoConfigHandlerRVC2.trSpatialDelta.append(StereoConfigHandlerRVC2.Trackbar("Spatial filter delta", stream, 0, 100, StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.delta, StereoConfigHandlerRVC2.trackbarSpatialFilterDelta))
        StereoConfigHandlerRVC2.trSpatialHoleFilling.append(StereoConfigHandlerRVC2.Trackbar("Spatial filter hole filling radius", stream, 0, 16, StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.holeFillingRadius, StereoConfigHandlerRVC2.trackbarSpatialFilterHoleFillingRadius))
        StereoConfigHandlerRVC2.trSpatialNumIterations.append(StereoConfigHandlerRVC2.Trackbar("Spatial filter number of iterations", stream, 0, 4, StereoConfigHandlerRVC2.config.postProcessing.spatialFilter.numIterations, StereoConfigHandlerRVC2.trackbarSpatialFilterNumIterations))
        StereoConfigHandlerRVC2.trThresholdMinRange.append(StereoConfigHandlerRVC2.Trackbar("Threshold filter min range", stream, 0, 65, StereoConfigHandlerRVC2.config.postProcessing.thresholdFilter.minRange, StereoConfigHandlerRVC2.trackbarThresholdMinRange))
        StereoConfigHandlerRVC2.trThresholdMaxRange.append(StereoConfigHandlerRVC2.Trackbar("Threshold filter max range", stream, 0, 65, StereoConfigHandlerRVC2.config.postProcessing.thresholdFilter.maxRange, StereoConfigHandlerRVC2.trackbarThresholdMaxRange))
        StereoConfigHandlerRVC2.trSpeckleRange.append(StereoConfigHandlerRVC2.Trackbar("Speckle filter range", stream, 0, 240, StereoConfigHandlerRVC2.config.postProcessing.speckleFilter.speckleRange, StereoConfigHandlerRVC2.trackbarSpeckleRange))
        StereoConfigHandlerRVC2.trDecimationFactor.append(StereoConfigHandlerRVC2.Trackbar("Decimation factor", stream, 1, 4, StereoConfigHandlerRVC2.config.postProcessing.decimationFilter.decimationFactor, StereoConfigHandlerRVC2.trackbarDecimationFactor))

    def __init__(self, config):
        print("Control median filter using the 'm' key.")
        print("Control census transform kernel size using the 'c' key.")
        print("Control disparity search range using the 'd' key.")
        print("Control disparity companding using the 'f' key.")
        print("Control census transform mean mode using the 'v' key.")
        print("Control depth alignment using the 'a' key.")
        print("Control decimation algorithm using the 'a' key.")
        print("Control temporal persistency mode using the 'r' key.")
        print("Control spatial filter using the 'w' key.")
        print("Control temporal filter using the 't' key.")
        print("Control speckle filter using the 's' key.")
        print("Control left-right check mode using the '1' key.")
        print("Control subpixel mode using the '2' key.")
        print("Control extended mode using the '3' key.")
        if evaluation_mode:
            print("Switch between images using '[' and ']' keys.")

        StereoConfigHandlerRVC2.config = config

        if StereoConfigHandlerRVC2.config.censusTransform.kernelSize != dai.StereoDepthConfig.CensusTransform.KernelSize.AUTO:
            censusMask = StereoConfigHandlerRVC2.config.censusTransform.kernelMask
            censusGridSize = [(5,5), (7,7), (7,9)]
            censusGrid = censusGridSize[StereoConfigHandlerRVC2.config.censusTransform.kernelSize]
            if StereoConfigHandlerRVC2.config.censusTransform.kernelMask == 0:
                censusDefaultMask = [np.uint64(0xA82415), np.uint64(0xAA02A8154055), np.uint64(0x2AA00AA805540155)]
                censusMask = censusDefaultMask[StereoConfigHandlerRVC2.config.censusTransform.kernelSize]
            StereoConfigHandlerRVC2.censusMaskHandler = StereoConfigHandlerRVC2.CensusMaskHandler("Census mask", censusGrid)
            StereoConfigHandlerRVC2.censusMaskHandler.setMask(censusMask)
        else:
            print("Census mask config is not available in AUTO Census kernel mode. Change using the 'c' key")
class StereoConfigHandlerRVC4:

    class Trackbar:
        def __init__(self, trackbarName, windowName, minValue, maxValue, defaultValue, handler):
            self.min = minValue
            self.max = maxValue
            self.windowName = windowName
            self.trackbarName = trackbarName
            cv2.createTrackbar(trackbarName, windowName, minValue, maxValue, handler)
            cv2.setTrackbarPos(trackbarName, windowName, defaultValue)

        def set(self, value):
            if value < self.min:
                value = self.min
                print(f"{self.trackbarName} min value is {self.min}")
            if value > self.max:
                value = self.max
                print(f"{self.trackbarName} max value is {self.max}")
            cv2.setTrackbarPos(self.trackbarName, self.windowName, value)

    newConfig = False
    config = None
    trConfidence = list()
    trLrCheck = list()
    trTemporalAlpha = list()
    trTemporalDelta = list()
    trThresholdMinRange = list()
    trThresholdMaxRange = list()
    trSpeckleRange = list()
    trSpatialAlpha = list()
    trSpatialDelta = list()
    trSpatialHoleFilling = list()
    trSpatialNumIterations = list()
    trDecimationFactor = list()
    trDisparityShift = list()
    trCenterAlignmentShift = list()
    trInvalidateEdgePixels = list()

    def trackbarConfidence(value):
        StereoConfigHandlerRVC4.config.costMatching.confidenceThreshold = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trConfidence:
            tr.set(value)

    def trackbarLrCheckThreshold(value):
        StereoConfigHandlerRVC4.config.algorithmControl.leftRightCheckThreshold = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trLrCheck:
            tr.set(value)

    def trackbarTemporalFilterAlpha(value):
        StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.alpha = value / 100.
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trTemporalAlpha:
            tr.set(value)

    def trackbarTemporalFilterDelta(value):
        StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.delta = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trTemporalDelta:
            tr.set(value)

    def trackbarSpatialFilterAlpha(value):
        StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.alpha = value / 100.
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trSpatialAlpha:
            tr.set(value)

    def trackbarSpatialFilterDelta(value):
        StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.delta = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trSpatialDelta:
            tr.set(value)

    def trackbarSpatialFilterHoleFillingRadius(value):
        StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.holeFillingRadius = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trSpatialHoleFilling:
            tr.set(value)

    def trackbarSpatialFilterNumIterations(value):
        StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.numIterations = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trSpatialNumIterations:
            tr.set(value)

    def trackbarThresholdMinRange(value):
        StereoConfigHandlerRVC4.config.postProcessing.thresholdFilter.minRange = value * 1000
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trThresholdMinRange:
            tr.set(value)

    def trackbarThresholdMaxRange(value):
        StereoConfigHandlerRVC4.config.postProcessing.thresholdFilter.maxRange = value * 1000
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trThresholdMaxRange:
            tr.set(value)

    def trackbarSpeckleRange(value):
        StereoConfigHandlerRVC4.config.postProcessing.speckleFilter.speckleRange = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trSpeckleRange:
            tr.set(value)

    def trackbarDecimationFactor(value):
        StereoConfigHandlerRVC4.config.postProcessing.decimationFilter.decimationFactor = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trDecimationFactor:
            tr.set(value)

    def trackbarDisparityShift(value):
        StereoConfigHandlerRVC4.config.algorithmControl.disparityShift = value
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trDisparityShift:
            tr.set(value)

    def trackbarCenterAlignmentShift(value):
        if StereoConfigHandlerRVC4.config.algorithmControl.depthAlign != dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER:
            print("Center alignment shift factor requires CENTER alignment enabled!")
            return
        StereoConfigHandlerRVC4.config.algorithmControl.centerAlignmentShiftFactor = value / 100.
        print(f"centerAlignmentShiftFactor: {StereoConfigHandlerRVC4.config.algorithmControl.centerAlignmentShiftFactor:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trCenterAlignmentShift:
            tr.set(value)

    def trackbarInvalidateEdgePixels(value):
        StereoConfigHandlerRVC4.config.algorithmControl.numInvalidateEdgePixels = value
        print(f"numInvalidateEdgePixels: {StereoConfigHandlerRVC4.config.algorithmControl.numInvalidateEdgePixels:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.trInvalidateEdgePixels:
            tr.set(value)

    tr_occlusionConfidenceWeight = list()
    def trackbar_occlusionConfidenceWeight(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.occlusionConfidenceWeight = value
        print(f"occlusionConfidenceWeight: {StereoConfigHandlerRVC4.config.confidenceMetrics.occlusionConfidenceWeight:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_occlusionConfidenceWeight:
            tr.set(value)

    tr_motionVectorConfidenceWeight = list()
    def trackbar_motionVectorConfidenceWeight(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceWeight = value
        print(f"motionVectorConfidenceWeight: {StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceWeight:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_motionVectorConfidenceWeight:
            tr.set(value)

    tr_motionVectorConfidenceThreshold = list()
    def trackbar_motionVectorConfidenceThreshold(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceThreshold = value
        print(f"motionVectorConfidenceThreshold: {StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_motionVectorConfidenceThreshold:
            tr.set(value)

    tr_flatnessConfidenceWeight = list()
    def trackbar_flatnessConfidenceWeight(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceWeight = value
        print(f"flatnessConfidenceWeight: {StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceWeight:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_flatnessConfidenceWeight:
            tr.set(value)

    tr_flatnessConfidenceThreshold = list()
    def trackbar_flatnessConfidenceThreshold(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceThreshold = value
        print(f"flatnessConfidenceThreshold: {StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_flatnessConfidenceThreshold:
            tr.set(value)

    tr_flatnessOverride = list()
    def trackbar_flatnessOverride(value):
        StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessOverride = bool(value)
        print(f"flatnessOverride: {StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessOverride:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_flatnessOverride:
            tr.set(value)

    tr_adaptiveMedianFilter_enable = list()
    def trackbar_adaptiveMedianFilter_enable(value):
        StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.enable = bool(value)
        print(f"adaptiveMedianFilter.enable: {StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.enable:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_adaptiveMedianFilter_enable:
            tr.set(value)

    tr_adaptiveMedianFilter_threshold = list()
    def trackbar_adaptiveMedianFilter_threshold(value):
        StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.confidenceThreshold = value
        print(f"adaptiveMedianFilter_threshold: {StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.confidenceThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_adaptiveMedianFilter_threshold:
            tr.set(value)

    tr_noiseThresholdOffset = list()
    def trackbar_noiseThresholdOffset(value):
        StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdOffset = value
        print(f"noiseThresholdOffset: {StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdOffset:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_noiseThresholdOffset:
            tr.set(value)

    tr_noiseThresholdScale = list()
    def trackbar_noiseThresholdScale(value):
        StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdScale = value - 128
        print(f"noiseThresholdScale: {StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdScale:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_noiseThresholdScale:
            tr.set(value)

    tr_p1_enableAdaptive = list()
    def trackbar_p1_enableAdaptive(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.enableAdaptive = bool(value)
        print(f"p1Config.enableAdaptive: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.enableAdaptive:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_enableAdaptive:
            tr.set(value)

    tr_p1_defaultValue = list()
    def trackbar_p1_defaultValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.defaultValue = value
        print(f"p1Config.defaultValue: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.defaultValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_defaultValue:
            tr.set(value)

    tr_p1_edgeValue = list()
    def trackbar_p1_edgeValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeValue = value
        print(f"p1Config.edgeValue: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_edgeValue:
            tr.set(value)

    tr_p1_smoothValue = list()
    def trackbar_p1_smoothValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothValue = value
        print(f"p1Config.smoothValue: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_smoothValue:
            tr.set(value)

    tr_p1_edgeThreshold = list()
    def trackbar_p1_edgeThreshold(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeThreshold = value
        print(f"p1Config.edgeThreshold: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_edgeThreshold:
            tr.set(value)

    tr_p1_smoothThreshold = list()
    def trackbar_p1_smoothThreshold(value):
        StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothThreshold = value
        print(f"p1Config.smoothThreshold: {StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p1_smoothThreshold:
            tr.set(value)

    tr_p2_enableAdaptive = list()
    def trackbar_p2_enableAdaptive(value):
        StereoConfigHandlerRVC4.config.costAggregation.p2Config.enableAdaptive = bool(value)
        print(f"p2Config.enableAdaptive: {StereoConfigHandlerRVC4.config.costAggregation.p2Config.enableAdaptive:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p2_enableAdaptive:
            tr.set(value)

    tr_p2_defaultValue = list()
    def trackbar_p2_defaultValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p2Config.defaultValue = value
        print(f"p2Config.defaultValue: {StereoConfigHandlerRVC4.config.costAggregation.p2Config.defaultValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p2_defaultValue:
            tr.set(value)

    tr_p2_edgeValue = list()
    def trackbar_p2_edgeValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p2Config.edgeValue = value
        print(f"p2Config.edgeValue: {StereoConfigHandlerRVC4.config.costAggregation.p2Config.edgeValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p2_edgeValue:
            tr.set(value)

    tr_p2_smoothValue = list()
    def trackbar_p2_smoothValue(value):
        StereoConfigHandlerRVC4.config.costAggregation.p2Config.smoothValue = value
        print(f"p2Config.smoothValue: {StereoConfigHandlerRVC4.config.costAggregation.p2Config.smoothValue:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_p2_smoothValue:
            tr.set(value)

    tr_holefilling_enable = list()
    def trackbar_holefilling_enable(value):
        StereoConfigHandlerRVC4.config.postProcessing.holeFilling.enable = bool(value)
        print(f"holeFilling.enable: {StereoConfigHandlerRVC4.config.postProcessing.holeFilling.enable:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_holefilling_enable:
            tr.set(value)

    tr_holefilling_highConfidenceThreshold = list()
    def trackbar_holefilling_highConfidenceThreshold(value):
        StereoConfigHandlerRVC4.config.postProcessing.holeFilling.highConfidenceThreshold = value
        print(f"holeFilling.highConfidenceThreshold: {StereoConfigHandlerRVC4.config.postProcessing.holeFilling.highConfidenceThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_holefilling_highConfidenceThreshold:
            tr.set(value)

    tr_holefilling_fillConfidenceThreshold = list()
    def trackbar_holefilling_fillConfidenceThreshold(value):
        StereoConfigHandlerRVC4.config.postProcessing.holeFilling.fillConfidenceThreshold = value
        print(f"holeFilling.fillConfidenceThreshold: {StereoConfigHandlerRVC4.config.postProcessing.holeFilling.fillConfidenceThreshold:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_holefilling_fillConfidenceThreshold:
            tr.set(value)

    tr_holefilling_minValidDisparity = list()
    def trackbar_holefilling_minValidDisparity(value):
        StereoConfigHandlerRVC4.config.postProcessing.holeFilling.minValidDisparity = value
        print(f"holeFilling.minValidDisparity: {StereoConfigHandlerRVC4.config.postProcessing.holeFilling.minValidDisparity:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_holefilling_minValidDisparity:
            tr.set(value)

    tr_holefilling_invalidateDisparities = list()
    def trackbar_holefilling_invalidateDisparities(value):
        StereoConfigHandlerRVC4.config.postProcessing.holeFilling.invalidateDisparities = bool(value)
        print(f"holeFilling.invalidateDisparities: {StereoConfigHandlerRVC4.config.postProcessing.holeFilling.invalidateDisparities:.2f}")
        StereoConfigHandlerRVC4.newConfig = True
        for tr in StereoConfigHandlerRVC4.tr_holefilling_invalidateDisparities:
            tr.set(value)

    def handleKeypress(key, stereoDepthConfigInQueue):
        if key == ord("m"):
            StereoConfigHandlerRVC4.newConfig = True
            medianSettings = [dai.MedianFilter.MEDIAN_OFF, dai.MedianFilter.KERNEL_3x3, dai.MedianFilter.KERNEL_5x5]
            currentMedian = StereoConfigHandlerRVC4.config.postProcessing.median
            nextMedian = medianSettings[(medianSettings.index(currentMedian)+1) % len(medianSettings)]
            print(f"Changing median to {nextMedian.name} from {currentMedian.name}")
            StereoConfigHandlerRVC4.config.postProcessing.median = nextMedian
        if key == ord("w"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.enable = not StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.enable
            state = "on" if StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.enable else "off"
            print(f"Spatial filter {state}")
        if key == ord("t"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.enable = not StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.enable
            state = "on" if StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.enable else "off"
            print(f"Temporal filter {state}")
        if key == ord("s"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.postProcessing.speckleFilter.enable = not StereoConfigHandlerRVC4.config.postProcessing.speckleFilter.enable
            state = "on" if StereoConfigHandlerRVC4.config.postProcessing.speckleFilter.enable else "off"
            print(f"Speckle filter {state}")
        if key == ord("r"):
            StereoConfigHandlerRVC4.newConfig = True
            temporalSettings = [dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_OFF,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_8_OUT_OF_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_OUT_OF_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_2,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_5,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_8,
            dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.PERSISTENCY_INDEFINITELY,
            ]
            currentTemporal = StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.persistencyMode
            nextTemporal = temporalSettings[(temporalSettings.index(currentTemporal)+1) % len(temporalSettings)]
            print(f"Changing temporal persistency to {nextTemporal.name} from {currentTemporal.name}")
            StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.persistencyMode = nextTemporal
        if key == ord("n"):
            StereoConfigHandlerRVC4.newConfig = True
            decimationSettings = [dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING,
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN,
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN,
            ]
            currentDecimation = StereoConfigHandlerRVC4.config.postProcessing.decimationFilter.decimationMode
            nextDecimation = decimationSettings[(decimationSettings.index(currentDecimation)+1) % len(decimationSettings)]
            print(f"Changing decimation mode to {nextDecimation.name} from {currentDecimation.name}")
            StereoConfigHandlerRVC4.config.postProcessing.decimationFilter.decimationMode = nextDecimation
        if key == ord("a"):
            StereoConfigHandlerRVC4.newConfig = True
            alignmentSettings = [dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT,
            dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT,
            dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER,
            ]
            currentAlignment = StereoConfigHandlerRVC4.config.algorithmControl.depthAlign
            nextAlignment = alignmentSettings[(alignmentSettings.index(currentAlignment)+1) % len(alignmentSettings)]
            print(f"Changing alignment mode to {nextAlignment.name} from {currentAlignment.name}")
            StereoConfigHandlerRVC4.config.algorithmControl.depthAlign = nextAlignment
        elif key == ord("1"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.algorithmControl.enableSwLeftRightCheck = not StereoConfigHandlerRVC4.config.algorithmControl.enableSwLeftRightCheck
            state = "on" if StereoConfigHandlerRVC4.config.algorithmControl.enableSwLeftRightCheck else "off"
            print(f"LR-check {state}")
        elif key == ord("4"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.costMatching.enableSwConfidenceThresholding = not StereoConfigHandlerRVC4.config.costMatching.enableSwConfidenceThresholding
            state = "on" if StereoConfigHandlerRVC4.config.costMatching.enableSwConfidenceThresholding else "off"
            print(f"SW confidence thresholding {state}")
        elif key == ord("3"):
            StereoConfigHandlerRVC4.newConfig = True
            StereoConfigHandlerRVC4.config.algorithmControl.enableExtended = not StereoConfigHandlerRVC4.config.algorithmControl.enableExtended
            state = "on" if StereoConfigHandlerRVC4.config.algorithmControl.enableExtended else "off"
            print(f"Extended {state}")

        StereoConfigHandlerRVC4.sendConfig(stereoDepthConfigInQueue)

    def sendConfig(stereoDepthConfigInQueue):
        if StereoConfigHandlerRVC4.newConfig:
            StereoConfigHandlerRVC4.newConfig = False

            # configMessage = dai.StereoDepthConfig()
            configMessage = StereoConfigHandlerRVC4.config
            if configMessage.confidenceMetrics.occlusionConfidenceWeight + StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceWeight + StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceWeight != 32:
                print("Sum of occlusion, motion vector and flatness confidence weights must be 32")

            stereoDepthConfigInQueue.send(configMessage)

    def updateDefaultConfig(config):
        StereoConfigHandlerRVC4.config = config

    def registerWindow(stream):
        cv2.namedWindow(stream, cv2.WINDOW_NORMAL)

        StereoConfigHandlerRVC4.trConfidence.append(StereoConfigHandlerRVC4.Trackbar("Disparity confidence", stream, 0, 255, StereoConfigHandlerRVC4.config.costMatching.confidenceThreshold, StereoConfigHandlerRVC4.trackbarConfidence))
        StereoConfigHandlerRVC4.trLrCheck.append(StereoConfigHandlerRVC4.Trackbar("LR-check threshold", stream, 0, 16, StereoConfigHandlerRVC4.config.algorithmControl.leftRightCheckThreshold, StereoConfigHandlerRVC4.trackbarLrCheckThreshold))
        StereoConfigHandlerRVC4.trDisparityShift.append(StereoConfigHandlerRVC4.Trackbar("Disparity shift", stream, 0, 100, StereoConfigHandlerRVC4.config.algorithmControl.disparityShift, StereoConfigHandlerRVC4.trackbarDisparityShift))
        StereoConfigHandlerRVC4.trCenterAlignmentShift.append(StereoConfigHandlerRVC4.Trackbar("Center alignment shift factor", stream, 0, 100, StereoConfigHandlerRVC4.config.algorithmControl.centerAlignmentShiftFactor, StereoConfigHandlerRVC4.trackbarCenterAlignmentShift))
        StereoConfigHandlerRVC4.trInvalidateEdgePixels.append(StereoConfigHandlerRVC4.Trackbar("Invalidate edge pixels", stream, 0, 100, StereoConfigHandlerRVC4.config.algorithmControl.numInvalidateEdgePixels, StereoConfigHandlerRVC4.trackbarInvalidateEdgePixels))
        StereoConfigHandlerRVC4.trTemporalAlpha.append(StereoConfigHandlerRVC4.Trackbar("Temporal filter alpha", stream, 0, 100, int(StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.alpha*100), StereoConfigHandlerRVC4.trackbarTemporalFilterAlpha))
        StereoConfigHandlerRVC4.trTemporalDelta.append(StereoConfigHandlerRVC4.Trackbar("Temporal filter delta", stream, 0, 100, StereoConfigHandlerRVC4.config.postProcessing.temporalFilter.delta, StereoConfigHandlerRVC4.trackbarTemporalFilterDelta))
        StereoConfigHandlerRVC4.trSpatialAlpha.append(StereoConfigHandlerRVC4.Trackbar("Spatial filter alpha", stream, 0, 100, int(StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.alpha*100), StereoConfigHandlerRVC4.trackbarSpatialFilterAlpha))
        StereoConfigHandlerRVC4.trSpatialDelta.append(StereoConfigHandlerRVC4.Trackbar("Spatial filter delta", stream, 0, 100, StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.delta, StereoConfigHandlerRVC4.trackbarSpatialFilterDelta))
        StereoConfigHandlerRVC4.trSpatialHoleFilling.append(StereoConfigHandlerRVC4.Trackbar("Spatial filter hole filling radius", stream, 0, 16, StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.holeFillingRadius, StereoConfigHandlerRVC4.trackbarSpatialFilterHoleFillingRadius))
        StereoConfigHandlerRVC4.trSpatialNumIterations.append(StereoConfigHandlerRVC4.Trackbar("Spatial filter number of iterations", stream, 0, 4, StereoConfigHandlerRVC4.config.postProcessing.spatialFilter.numIterations, StereoConfigHandlerRVC4.trackbarSpatialFilterNumIterations))
        StereoConfigHandlerRVC4.trThresholdMinRange.append(StereoConfigHandlerRVC4.Trackbar("Threshold filter min range", stream, 0, 65, StereoConfigHandlerRVC4.config.postProcessing.thresholdFilter.minRange, StereoConfigHandlerRVC4.trackbarThresholdMinRange))
        StereoConfigHandlerRVC4.trThresholdMaxRange.append(StereoConfigHandlerRVC4.Trackbar("Threshold filter max range", stream, 0, 65, StereoConfigHandlerRVC4.config.postProcessing.thresholdFilter.maxRange, StereoConfigHandlerRVC4.trackbarThresholdMaxRange))
        StereoConfigHandlerRVC4.trSpeckleRange.append(StereoConfigHandlerRVC4.Trackbar("Speckle filter range", stream, 0, 240, StereoConfigHandlerRVC4.config.postProcessing.speckleFilter.speckleRange, StereoConfigHandlerRVC4.trackbarSpeckleRange))
        StereoConfigHandlerRVC4.trDecimationFactor.append(StereoConfigHandlerRVC4.Trackbar("Decimation factor", stream, 1, 4, StereoConfigHandlerRVC4.config.postProcessing.decimationFilter.decimationFactor, StereoConfigHandlerRVC4.trackbarDecimationFactor))
        StereoConfigHandlerRVC4.tr_occlusionConfidenceWeight.append(StereoConfigHandlerRVC4.Trackbar("Occlusion confidence weight", stream, 0, 32, StereoConfigHandlerRVC4.config.confidenceMetrics.occlusionConfidenceWeight, StereoConfigHandlerRVC4.trackbar_occlusionConfidenceWeight))
        StereoConfigHandlerRVC4.tr_motionVectorConfidenceWeight.append(StereoConfigHandlerRVC4.Trackbar("Motion vector confidence weight", stream, 0, 32, StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceWeight, StereoConfigHandlerRVC4.trackbar_motionVectorConfidenceWeight))
        StereoConfigHandlerRVC4.tr_motionVectorConfidenceThreshold.append(StereoConfigHandlerRVC4.Trackbar("Motion vector confidence threshold", stream, 0, 3, StereoConfigHandlerRVC4.config.confidenceMetrics.motionVectorConfidenceThreshold, StereoConfigHandlerRVC4.trackbar_motionVectorConfidenceThreshold))
        StereoConfigHandlerRVC4.tr_flatnessConfidenceWeight.append(StereoConfigHandlerRVC4.Trackbar("Flatness confidence weight", stream, 0, 32, StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceWeight, StereoConfigHandlerRVC4.trackbar_flatnessConfidenceWeight))
        StereoConfigHandlerRVC4.tr_flatnessConfidenceThreshold.append(StereoConfigHandlerRVC4.Trackbar("Flatness confidence threshold", stream, 1, 7, StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessConfidenceThreshold, StereoConfigHandlerRVC4.trackbar_flatnessConfidenceThreshold))
        StereoConfigHandlerRVC4.tr_flatnessOverride.append(StereoConfigHandlerRVC4.Trackbar("Flatness override enable", stream, 0, 1, int(StereoConfigHandlerRVC4.config.confidenceMetrics.flatnessOverride), StereoConfigHandlerRVC4.trackbar_flatnessOverride))
        StereoConfigHandlerRVC4.tr_adaptiveMedianFilter_enable.append(StereoConfigHandlerRVC4.Trackbar("Adaptive median filter enable", stream, 0, 1, int(StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.enable), StereoConfigHandlerRVC4.trackbar_adaptiveMedianFilter_enable))
        StereoConfigHandlerRVC4.tr_adaptiveMedianFilter_threshold.append(StereoConfigHandlerRVC4.Trackbar("Adaptive median filter threshold", stream, 0, 255, StereoConfigHandlerRVC4.config.postProcessing.adaptiveMedianFilter.confidenceThreshold, StereoConfigHandlerRVC4.trackbar_adaptiveMedianFilter_threshold))
        StereoConfigHandlerRVC4.tr_noiseThresholdOffset.append(StereoConfigHandlerRVC4.Trackbar("Noise threshold offset", stream, 0, 127, StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdOffset, StereoConfigHandlerRVC4.trackbar_noiseThresholdOffset))
        StereoConfigHandlerRVC4.tr_noiseThresholdScale.append(StereoConfigHandlerRVC4.Trackbar("Noise threshold scale (value - 128)", stream, 0, 255, StereoConfigHandlerRVC4.config.censusTransform.noiseThresholdScale + 128, StereoConfigHandlerRVC4.trackbar_noiseThresholdScale))
        StereoConfigHandlerRVC4.tr_p1_enableAdaptive.append(StereoConfigHandlerRVC4.Trackbar("P1 enable adaptive", stream, 0, 1, int(StereoConfigHandlerRVC4.config.costAggregation.p1Config.enableAdaptive), StereoConfigHandlerRVC4.trackbar_p1_enableAdaptive))
        StereoConfigHandlerRVC4.tr_p1_defaultValue.append(StereoConfigHandlerRVC4.Trackbar("P1 default value", stream, 10, 50, StereoConfigHandlerRVC4.config.costAggregation.p1Config.defaultValue, StereoConfigHandlerRVC4.trackbar_p1_defaultValue))
        StereoConfigHandlerRVC4.tr_p1_edgeValue.append(StereoConfigHandlerRVC4.Trackbar("P1 edge value", stream, 10, 50, StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeValue, StereoConfigHandlerRVC4.trackbar_p1_edgeValue))
        StereoConfigHandlerRVC4.tr_p1_smoothValue.append(StereoConfigHandlerRVC4.Trackbar("P1 smooth value", stream, 10, 50, StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothValue, StereoConfigHandlerRVC4.trackbar_p1_smoothValue))
        StereoConfigHandlerRVC4.tr_p1_edgeThreshold.append(StereoConfigHandlerRVC4.Trackbar("P1 edge threshold", stream, 8, 16, StereoConfigHandlerRVC4.config.costAggregation.p1Config.edgeThreshold, StereoConfigHandlerRVC4.trackbar_p1_edgeThreshold))
        StereoConfigHandlerRVC4.tr_p1_smoothThreshold.append(StereoConfigHandlerRVC4.Trackbar("P1 smooth threshold", stream, 2, 12, StereoConfigHandlerRVC4.config.costAggregation.p1Config.smoothThreshold, StereoConfigHandlerRVC4.trackbar_p1_smoothThreshold))
        StereoConfigHandlerRVC4.tr_p2_enableAdaptive.append(StereoConfigHandlerRVC4.Trackbar("P2 enable adaptive", stream, 0, 1, int(StereoConfigHandlerRVC4.config.costAggregation.p2Config.enableAdaptive), StereoConfigHandlerRVC4.trackbar_p2_enableAdaptive))
        StereoConfigHandlerRVC4.tr_p2_defaultValue.append(StereoConfigHandlerRVC4.Trackbar("P2 default value", stream, 20, 100, StereoConfigHandlerRVC4.config.costAggregation.p2Config.defaultValue, StereoConfigHandlerRVC4.trackbar_p2_defaultValue))
        StereoConfigHandlerRVC4.tr_p2_edgeValue.append(StereoConfigHandlerRVC4.Trackbar("P2 edge value", stream, 20, 100, StereoConfigHandlerRVC4.config.costAggregation.p2Config.edgeValue, StereoConfigHandlerRVC4.trackbar_p2_edgeValue))
        StereoConfigHandlerRVC4.tr_p2_smoothValue.append(StereoConfigHandlerRVC4.Trackbar("P2 smooth value", stream, 20, 100, StereoConfigHandlerRVC4.config.costAggregation.p2Config.smoothValue, StereoConfigHandlerRVC4.trackbar_p2_smoothValue))
        StereoConfigHandlerRVC4.tr_holefilling_enable.append(StereoConfigHandlerRVC4.Trackbar("Hole filling enable", stream, 0, 1, int(StereoConfigHandlerRVC4.config.postProcessing.holeFilling.enable), StereoConfigHandlerRVC4.trackbar_holefilling_enable))
        StereoConfigHandlerRVC4.tr_holefilling_highConfidenceThreshold.append(StereoConfigHandlerRVC4.Trackbar("Hole filling high confidence threshold", stream, 0, 255, StereoConfigHandlerRVC4.config.postProcessing.holeFilling.highConfidenceThreshold, StereoConfigHandlerRVC4.trackbar_holefilling_highConfidenceThreshold))
        StereoConfigHandlerRVC4.tr_holefilling_fillConfidenceThreshold.append(StereoConfigHandlerRVC4.Trackbar("Hole filling fill confidence threshold", stream, 0, 255, StereoConfigHandlerRVC4.config.postProcessing.holeFilling.fillConfidenceThreshold, StereoConfigHandlerRVC4.trackbar_holefilling_fillConfidenceThreshold))
        StereoConfigHandlerRVC4.tr_holefilling_minValidDisparity.append(StereoConfigHandlerRVC4.Trackbar("Hole filling min valid disparity", stream, 1, 3, StereoConfigHandlerRVC4.config.postProcessing.holeFilling.minValidDisparity, StereoConfigHandlerRVC4.trackbar_holefilling_minValidDisparity))
        StereoConfigHandlerRVC4.tr_holefilling_invalidateDisparities.append(StereoConfigHandlerRVC4.Trackbar("Hole filling invalidate disparities", stream, 0, 1, int(StereoConfigHandlerRVC4.config.postProcessing.holeFilling.invalidateDisparities), StereoConfigHandlerRVC4.trackbar_holefilling_invalidateDisparities))

    def __init__(self, config):
        print("Control median filter using the 'm' key.")
        print("Control depth alignment using the 'a' key.")
        print("Control decimation algorithm using the 'a' key.")
        print("Control temporal persistency mode using the 'r' key.")
        print("Control spatial filter using the 'w' key.")
        print("Control temporal filter using the 't' key.")
        print("Control speckle filter using the 's' key.")
        print("Control left-right check mode using the '1' key.")
        print("Control extended mode using the '3' key.")
        print("Control SW confidence thresholding using the '4' key.")
        if evaluation_mode:
            print("Switch between images using '[' and ']' keys.")

        StereoConfigHandlerRVC4.config = config


# StereoDepth initial config options.
outDepth = True  # Disparity by default
outConfidenceMap = False  # Output disparity confidence map
outRectified = True   # Output and display rectified streams
lrcheck = True   # Better handling for occlusions
extended = True  # Closer-in minimum depth, disparity range is doubled. Unsupported for now.
subpixel = True   # Better accuracy for longer distance, fractional disparity 32-levels

width = 1280
height = 800

xoutStereoCfg = None

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
stereo = pipeline.create(dai.node.StereoDepth)

monoLeft = stereo.left.createInputQueue()
monoRight = stereo.right.createInputQueue()
xinStereoDepthConfig = stereo.inputConfig.createInputQueue()

xoutDisparity = stereo.disparity.createOutputQueue()
xoutDisparity.setName("disparity")
xoutStereoCfg = stereo.outConfig.createOutputQueue()
xoutStereoCfg.setName("stereoCfg")
xoutLeft = stereo.syncedLeft.createOutputQueue()
xoutLeft.setName("left")
xoutRight = stereo.syncedRight.createOutputQueue()
xoutRight.setName("right")
if outDepth:
    xoutDepth = stereo.depth.createOutputQueue()
    xoutDepth.setName("depth")
if outConfidenceMap:
    xoutConfMap = stereo.confidenceMap.createOutputQueue()
    xoutConfMap.setName("confidenceMap")
if outRectified:
    xoutRectifLeft = stereo.rectifiedLeft.createOutputQueue()
    xoutRectifRight = stereo.rectifiedRight.createOutputQueue()
    xoutRectifLeft.setName("rectifiedLeft")
    xoutRectifRight.setName("rectifiedRight")

if args.debug:
    xoutDebugLrCheckIt1 = stereo.debugDispLrCheckIt1.createOutputQueue()
    xoutDebugLrCheckIt2 = stereo.debugDispLrCheckIt1.createOutputQueue()
    xoutDebugExtLrCheckIt1 = stereo.debugDispLrCheckIt1.createOutputQueue()
    xoutDebugExtLrCheckIt2 = stereo.debugDispLrCheckIt1.createOutputQueue()

    xoutDebugLrCheckIt1.setName("debugLrCheckIt1")
    xoutDebugLrCheckIt2.setName("debugLrCheckIt2")
    xoutDebugExtLrCheckIt1.setName("debugExtLrCheckIt1")
    xoutDebugExtLrCheckIt2.setName("debugExtLrCheckIt2")

if args.dumpdisparitycostvalues:
    xoutDebugCostDump = stereo.debugDispCostDump.createOutputQueue()
    xoutDebugCostDump.setName("debugCostDump")

# Properties
stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)  # KERNEL_7x7 default
stereo.setLeftRightCheck(lrcheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
stereo.initialConfig.setSubpixelFractionalBits(4)

stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64

stereo.initialConfig.setConfidenceThreshold(127)
stereo.initialConfig.setLeftRightCheckThreshold(4)

# Switching depthAlign mode at runtime is not supported while aligning to a specific camera is enabled
stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

# allocates resources for worst case scenario
# allowing runtime switch of stereo modes
stereo.setRuntimeModeSwitch(True)

currentConfig = stereo.initialConfig

platform = pipeline.getDefaultDevice().getPlatform()

if platform == dai.Platform.RVC2:
    StereoConfigHandler = StereoConfigHandlerRVC2
elif platform == dai.Platform.RVC4:
    StereoConfigHandler = StereoConfigHandlerRVC4
    currentConfig.confidenceMetrics = stereo.initialConfig.confidenceMetrics
else:
    StereoConfigHandler = StereoConfigHandlerRVC2

StereoConfigHandler(currentConfig)
StereoConfigHandler.registerWindow("Stereo control panel")

if(args.calibration):
    calibrationHandler = dai.CalibrationHandler(args.calibration)
    pipeline.setCalibrationData(calibrationHandler)
stereo.setInputResolution(width, height)
stereo.setRectification(args.rectify)
baseline = 75
fov = 71.86
focal = width / (2 * math.tan(fov / 2 / 180 * math.pi))

stereo.setBaseline(baseline/10)
stereo.setFocalLength(focal)


def convertToCv2Frame(name, image, config):

    maxDisp = config.getMaxDisparity()
    subpixelLevels = pow(2, config.algorithmControl.subpixelFractionalBits)
    subpixel = config.algorithmControl.enableSubpixel
    dispIntegerLevels = maxDisp if not subpixel else maxDisp / subpixelLevels

    frame = image.getFrame()

    # frame.tofile(name+".raw")

    if name == "depth":
        dispScaleFactor = baseline * focal
        with np.errstate(divide="ignore"):
            frame = dispScaleFactor / frame
            if np.isnan(frame).any() or np.isinf(frame).any():
                frame = np.nan_to_num(frame, nan=0, posinf=0, neginf=0)

        frame = np.clip(frame * 255. / dispIntegerLevels, 0, 255).astype(np.uint8)
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
    elif "confidence_map" in name:
        pass
    elif name == "disparity_cost_dump":
        # frame.tofile(name+".raw")
        pass
    elif "disparity" in name:
        if 1: # Optionally, extend disparity range to better visualize it
            frame = (frame * 255. / maxDisp).astype(np.uint8)
        return frame
        # if 1: # Optionally, apply a color map
        #     frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)

    return frame

class DatasetManager:
    def __init__(self, path):
        self.path = path
        self.index = 0
        self.names = [d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))]
        if len(self.names) == 0:
            raise RuntimeError("No dataset found at {}".format(path))

    def get(self):
        return os.path.join(self.path, self.names[self.index])

    def get_name(self):
        return self.names[self.index]

    def next(self):
        self.index = (self.index + 1) % len(self.names)
        return self.get()

    def prev(self):
        self.index = (self.index - 1) % len(self.names)
        return self.get()


def read_pfm(file):
    file = open(file, "rb")

    color = None
    width = None
    height = None
    scale = None
    endian = None

    header = file.readline().rstrip()
    if header.decode("ascii") == "PF":
        color = True
    elif header.decode("ascii") == "Pf":
        color = False
    else:
        raise Exception("Not a PFM file.")

    dim_match = re.search(r"(\d+)\s(\d+)", file.readline().decode("ascii"))
    if dim_match:
        width, height = map(int, dim_match.groups())
    else:
        raise Exception("Malformed PFM header.")

    scale = float(file.readline().rstrip())
    if scale < 0: # little-endian
        endian = "<"
        scale = -scale
    else:
        endian = ">" # big-endian

    data = np.fromfile(file, endian + "f")
    shape = (height, width, 3) if color else (height, width)
    return np.flip(np.reshape(data, shape), axis=0), scale

def calculate_err_measures(gt_img, oak_img):
    assert gt_img.shape == oak_img.shape

    gt_mask = gt_img != 0
    oak_mask = oak_img != 0
    mask = gt_mask & oak_mask

    gt_img[~gt_mask] = 0.
    oak_img[~mask] = 0.
    err = np.abs(gt_img - oak_img)

    n = np.sum(gt_mask)
    invalid = np.sum(gt_mask & ~oak_mask)

    bad05 = np.sum(mask & (err > 0.5))
    bad1 = np.sum(mask & (err > 1.))
    bad2 = np.sum(mask & (err > 2.))
    bad4 = np.sum(mask & (err > 4.))
    sum_err = np.sum(err[mask])
    sum_sq_err = np.sum(err[mask] ** 2)
    errs = err[mask]

    bad05_p = 100. * bad05 / n
    total_bad05_p = 100. * (bad05 + invalid) / n
    bad1_p = 100. * bad1 / n
    total_bad1_p = 100. * (bad1 + invalid) / n
    bad2_p = 100. * bad2 / n
    total_bad2_p = 100. * (bad2 + invalid) / n
    bad4_p = 100. * bad4 / n
    total_bad4_p = 100. * (bad4 + invalid) / n
    invalid_p = 100. * invalid / n
    if n == invalid:
        avg_err = 0.
        mse = 0.
    else:
        avg_err = sum_err / (n - invalid)
        mse = sum_sq_err / (n - invalid)
    if len(errs) == 0:
        a50 = 0.
        a90 = 0.
        a95 = 0.
        a99 = 0.
    else:
        a50 = np.percentile(errs, 50)
        a90 = np.percentile(errs, 90)
        a95 = np.percentile(errs, 95)
        a99 = np.percentile(errs, 99)

    return {
        "bad0.5": bad05_p,
        "total_bad0.5": total_bad05_p,
        "bad1": bad1_p,
        "total_bad1": total_bad1_p,
        "bad2": bad2_p,
        "total_bad2": total_bad2_p,
        "bad4": bad4_p,
        "total_bad4": total_bad4_p,
        "invalid": invalid_p,
        "avg_err": avg_err,
        "mse": mse,
        "a50": a50,
        "a90": a90,
        "a95": a95,
        "a99": a99
    }

def show_evaluation(img_name, evals):
    cv2.namedWindow("Evaluation", cv2.WINDOW_NORMAL)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 2
    thickness = 3
    color = (0, 0, 0)
    lines = [
        f"Name: {img_name}",
        f"Bad0.5: {evals['bad0.5']:.2f}%",
        f"Total Bad0.5: {evals['total_bad0.5']:.2f}%",
        f"Bad1: {evals['bad1']:.2f}%",
        f"Total Bad1: {evals['total_bad1']:.2f}%",
        f"Bad2: {evals['bad2']:.2f}%",
        f"Total Bad2: {evals['total_bad2']:.2f}%",
        f"Bad4: {evals['bad4']:.2f}%",
        f"Total Bad4: {evals['total_bad4']:.2f}%",
        f"Invalid: {evals['invalid']:.2f}%",
        f"Avg Err: {evals['avg_err']:.2f}",
        f"MSE: {evals['mse']:.2f}",
        f"A50: {evals['a50']:.2f}",
        f"A90: {evals['a90']:.2f}",
        f"A95: {evals['a95']:.2f}",
        f"A99: {evals['a99']:.2f}"
    ]
    sizes = [cv2.getTextSize(line, font, font_scale, thickness) for line in lines]
    sizes = [(size[0][0], size[0][1] + size[1], size[1]) for size in sizes]
    max_width = max([size[0] for size in sizes])
    total_height = sum([size[1] for size in sizes]) + (len(lines) - 1) * thickness
    img = np.ones((total_height + thickness, max_width, 3), dtype=np.uint8) * 255
    y = 0
    for line, size in zip(lines, sizes):
        cv2.putText(img, line, (0, y + size[1] - size[2]), font, font_scale, color, thickness)
        y += size[1] + thickness
    cv2.imshow("Evaluation", img)

def show_debug_disparity(gt_img, oak_img):
    def rescale_img(img):
        img[img == np.inf] = 0.
        img = cv2.resize(img, (1280, 800), interpolation=cv2.INTER_AREA)
        return img.astype(np.uint16)

    gt_img = rescale_img(gt_img)
    oak_img = rescale_img(oak_img)
    maxv = max(gt_img.max(), oak_img.max())
    gt_img = (gt_img * 255. / maxv).astype(np.uint8)
    oak_img = (oak_img * 255. / maxv).astype(np.uint8)
    cv2.imshow("GT", gt_img)
    cv2.imshow("OAK", oak_img)

if evaluation_mode:
    dataset = DatasetManager(args.evaluate)

    # Get the current timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # Create the filename with the timestamp
    filename = f"stereo_middleburry_{'RVC2' if platform == dai.Platform.RVC2 else 'RVC4'}_{timestamp}.csv"
    # Write the dictionary to a CSV file (append mode)
    file_exists = False

    try:
        with open(filename, 'r'):
            file_exists = True
    except FileNotFoundError:
        pass

print("Connecting and starting the pipeline")
# Connect to device and start pipeline
with pipeline:
    pipeline.start()

    stereoDepthConfigInQueue = xinStereoDepthConfig

    inStreamsCameraID = [dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C]
    in_q_list = []
    in_q_list.append(monoLeft)
    in_q_list.append(monoRight)

    # Create a receive queue for each stream
    q_list = []
    q_list.append(xoutLeft)
    q_list.append(xoutRight)
    if outDepth:
        q_list.append(xoutDepth)
    if outConfidenceMap:
        q_list.append(xoutConfMap)
    q_list.append(xoutDisparity)
    if outRectified:
        q_list.append(xoutRectifLeft)
        q_list.append(xoutRectifRight)

    inCfg = xoutStereoCfg

    # Need to set a timestamp for input frames, for the sync stage in Stereo node
    timestamp_ms = 0
    index = 0
    prevQueues = q_list.copy()
    while pipeline.isRunning():
        # Handle input streams, if any
        if in_q_list:
            dataset_size = 1  # Number of image pairs
            frame_interval_ms = 50
            q_names = ["in_left", "in_right"]
            for i, q in enumerate(in_q_list):
                path = os.path.join(dataset.get(), f"im{i}.png") if evaluation_mode else args.dataset + "/" + str(index) + "/" + q_names[i] + ".png"
                data = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                data = cv2.resize(data, (width, height), interpolation = cv2.INTER_AREA)
                data = data.reshape(height*width)
                tstamp = datetime.timedelta(seconds = timestamp_ms // 1000,
                                            milliseconds = timestamp_ms % 1000)
                img = dai.ImgFrame()
                img.setData(data)
                img.setTimestamp(tstamp)
                img.setInstanceNum(inStreamsCameraID[i])
                img.setType(dai.ImgFrame.Type.RAW8)
                img.setWidth(width)
                img.setStride(width)
                img.setHeight(height)
                q.send(img)
                # print("Sent frame: {:25s}".format(path), "timestamp_ms:", timestamp_ms)
            timestamp_ms += frame_interval_ms
            index = (index + 1) % dataset_size
            sleep(frame_interval_ms / 1000)

        gt_disparity = None
        if evaluation_mode:
            # Load GT disparity
            if currentConfig.algorithmControl.depthAlign == dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT:
                gt_disparity = read_pfm(os.path.join(dataset.get(), f"disp0.pfm"))[0]
            else:
                gt_disparity = read_pfm(os.path.join(dataset.get(), f"disp1.pfm"))[0]

        # Handle output streams
        currentConfig = inCfg.get()

        lrCheckEnabled = currentConfig.algorithmControl.enableLeftRightCheck
        extendedEnabled = currentConfig.algorithmControl.enableExtended
        queues = q_list.copy()

        if args.dumpdisparitycostvalues:
            queues.append(xoutDebugCostDump)

        if args.debug:
            q_list_debug = []

            activeDebugStreams = []
            if lrCheckEnabled:
                q_list_debug.append(xoutDebugLrCheckIt1)
                q_list_debug.append(xoutDebugLrCheckIt2)
            if extendedEnabled:
                q_list_debug.append(xoutDebugExtLrCheckIt1)
                if lrCheckEnabled:
                    q_list_debug.append(xoutDebugExtLrCheckIt2)

            queues.extend(q_list_debug)

        def ListDiff(li1, li2):
            return list(set(li1) - set(li2)) + list(set(li2) - set(li1))

        diff = ListDiff(prevQueues, queues)
        for s in diff:
            name = s.getName()
            cv2.destroyWindow(name)
        prevQueues = queues.copy()

        disparity = None
        for q in queues:
            if q.getName() in ["left", "right"]: continue
            data = q.get()
            if q.getName() == "disparity":
                disparity = data.getFrame()
            frame = convertToCv2Frame(q.getName(), data, currentConfig)
            cv2.imshow(q.getName(), frame)

        if disparity is not None and gt_disparity is not None:
            subpixel_bits = 1 << currentConfig.algorithmControl.subpixelFractionalBits
            subpixel_enabled = currentConfig.algorithmControl.enableSubpixel
            width_scale = float(gt_disparity.shape[1]) / float(disparity.shape[1])

            disparity = disparity.astype(np.float32)
            if subpixel_enabled:
                disparity = disparity / subpixel_bits
            disparity = disparity * width_scale
            disparity = cv2.resize(disparity, (gt_disparity.shape[1], gt_disparity.shape[0]), interpolation = cv2.INTER_LINEAR)

            gt_disparity[gt_disparity == np.inf] = 0
            # disparity[disparity == 0.] = np.inf

            show_debug_disparity(gt_disparity, disparity)
            err_vals = calculate_err_measures(gt_disparity, disparity)
            show_evaluation(dataset.get_name(), err_vals)
            err_vals["name"] = dataset.get_name()

        saveToCsvFile = False
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        elif evaluation_mode and key == ord("["):
            dataset.next()
            saveToCsvFile = True
        elif evaluation_mode and key == ord("]"):
            dataset.prev()
            saveToCsvFile = True

        if saveToCsvFile:
            with open(filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write the header only if the file doesn't exist
                if not file_exists:
                    writer.writerow(err_vals.keys())
                    file_exists = True
                # Write the err_vals
                writer.writerow(err_vals.values())

        StereoConfigHandler.handleKeypress(key, stereoDepthConfigInQueue)
