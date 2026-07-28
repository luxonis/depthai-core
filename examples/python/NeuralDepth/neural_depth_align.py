#!/usr/bin/env python3

import cv2
import depthai as dai
import time
from datetime import timedelta
FPS = 25

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C

class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-10:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])

pipeline = dai.Pipeline()

platform = pipeline.getDefaultDevice().getPlatform()

# Define sources and outputs
camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)
left = pipeline.create(dai.node.Camera).build(LEFT_SOCKET)
right = pipeline.create(dai.node.Camera).build(RIGHT_SOCKET)
stereo = pipeline.create(dai.node.NeuralDepth)
sync = pipeline.create(dai.node.Sync)
align = pipeline.create(dai.node.ImageAlign)

sync.setSyncThreshold(timedelta(seconds=1/(2*FPS)))

rgbOut = camRgb.requestOutput(size = (1280, 960), fps = FPS, enableUndistortion=True)
leftOut = left.requestFullResolutionOutput(fps = FPS)
rightOut = right.requestFullResolutionOutput(fps = FPS)
stereo.build(leftOut, rightOut, dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)
# Linking
stereo.depth.link(align.input)
rgbOut.link(align.inputAlignTo)
rgbOut.link(sync.inputs["rgb"])
align.outputAligned.link(sync.inputs["depth_aligned"])

queue = sync.out.createOutputQueue()

rgbWeight = 0.4
depthWeight = 0.6


def updateBlendWeights(percentRgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image

    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percentRgb) / 100.0
    depthWeight = 1.0 - rgbWeight


# Connect to device and start pipeline
with pipeline:
    pipeline.start()

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    windowName = "rgb-depth"

    # Set the window to be resizable and the initial size
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, 1280, 720)
    cv2.createTrackbar(
        "RGB Weight %",
        windowName,
        int(rgbWeight * 100),
        100,
        updateBlendWeights,
    )
    fpsCounter = FPSCounter()
    while True:
        messageGroup = queue.get()
        fpsCounter.tick()
        assert isinstance(messageGroup, dai.MessageGroup)
        frameRgb = messageGroup["rgb"]
        assert isinstance(frameRgb, dai.ImgFrame)
        frameDepth = messageGroup["depth_aligned"]
        assert isinstance(frameDepth, dai.ImgFrame)

        # Blend when both received
        if frameDepth is not None:
            cvFrame = frameRgb.getCvFrame()
            # Colorize the aligned depth
            alignedDepthColorized = dai.colorizeDepthFrame(frameDepth, 500, 12000, useLog=True).getCvFrame()
            # Resize depth to match the rgb frame
            cv2.imshow("Depth aligned", alignedDepthColorized)

            if len(cvFrame.shape) == 2:
                cvFrameUndistorted = cv2.cvtColor(cvFrame, cv2.COLOR_GRAY2BGR)
            # print("RGB Size:", cvFrame.shape)
            # print("Depth Size:", alignedDepthColorized.shape)
            blended = cv2.addWeighted(
                cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0
            )
            cv2.putText(
                blended,
                f"FPS: {fpsCounter.getFps():.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            cv2.imshow(windowName, blended)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
