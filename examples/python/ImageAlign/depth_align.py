#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai
import time
from datetime import timedelta
FPS = 30.0

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

device = dai.Device()

calibrationHandler = device.readCalibration()
rgbDistortion = calibrationHandler.getDistortionCoefficients(RGB_SOCKET)
distortionModel = calibrationHandler.getDistortionModel(RGB_SOCKET)
if distortionModel != dai.CameraModel.Perspective:
    raise RuntimeError("Unsupported distortion model for RGB camera. This example supports only Perspective model.")

pipeline = dai.Pipeline(device)

# Define sources and outputs
camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)
left = pipeline.create(dai.node.Camera).build(LEFT_SOCKET)
right = pipeline.create(dai.node.Camera).build(RIGHT_SOCKET)
stereo = pipeline.create(dai.node.StereoDepth)
sync = pipeline.create(dai.node.Sync)
align = pipeline.create(dai.node.ImageAlign)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(LEFT_SOCKET)

stereo.setExtendedDisparity(True)

sync.setSyncThreshold(timedelta(seconds=1/(2*FPS)))

rgbOut = camRgb.requestOutput(size = (1280, 960))
leftOut = left.requestOutput(size = (640, 400), fps = FPS)
rightOut = right.requestOutput(size = (640, 400), fps = FPS)

# Linking
rgbOut.link(sync.inputs["rgb"])
leftOut.link(stereo.left)
rightOut.link(stereo.right)
stereo.depth.link(align.input)
align.outputAligned.link(sync.inputs["depth_aligned"])
rgbOut.link(align.inputAlignTo)
queue = sync.out.createOutputQueue()

def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    # Log the depth, minDepth and maxDepth
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        # Clip the values to be in the 0-255 range
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Interpolate only valid logDepth values, setting the rest based on the mask
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor


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

        sizeRgb = frameRgb.getData().size
        sizeDepth = frameDepth.getData().size
        # Blend when both received
        if frameDepth is not None:
            cvFrame = frameRgb.getCvFrame()

            # Undistort the rgb frame
            rgbIntrinsics = calibrationHandler.getCameraIntrinsics(RGB_SOCKET, int(cvFrame.shape[1]), int(cvFrame.shape[0]))

            cvFrameUndistorted = cv2.undistort(
                cvFrame,
                np.array(rgbIntrinsics),
                np.array(rgbDistortion),
            )
            # Colorize the aligned depth
            alignedDepthColorized = colorizeDepth(frameDepth.getFrame())
            # Resize depth to match the rgb frame
            cv2.imshow("Depth aligned", alignedDepthColorized)

            if len(cvFrameUndistorted.shape) == 2:
                cvFrameUndistorted = cv2.cvtColor(cvFrameUndistorted, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(
                cvFrameUndistorted, rgbWeight, alignedDepthColorized, depthWeight, 0
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