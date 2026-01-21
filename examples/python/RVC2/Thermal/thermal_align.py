#!/usr/bin/env python3

"""
Due to an issue with our calibration, you might receive the following error when running this script on early release OAK Thermal devices:
```bash
[ImageAlign(4)] [error] Failed to get calibration data: Extrinsic connection between the requested cameraId's doesn't exist. Please recalibrate or modify your calibration data
```
If this happens, please download the calibration data + script from https://drive.google.com/drive/folders/1Q_MZMqWMKDC1eOqVHGPeDO-NJgFmnY5U,
place them into the same folder, connect the camera to the computer and run the script. This will update the
calibration and add required extrinsics between the camera sensors.
"""

import cv2
import depthai as dai
import numpy as np
import time
from datetime import timedelta

FPS = 18

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
COLOR_RESOLUTION = (960, 540)  # (width, height)

class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-100:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        # Calculate the FPS
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])

with dai.Pipeline() as pipeline:
    device = pipeline.getDefaultDevice()

    thermalWidth, thermalHeight = -1, -1
    thermalFound = False
    thermalSocket = None
    for features in device.getConnectedCameraFeatures():
        if dai.CameraSensorType.THERMAL in features.supportedTypes:
            thermalFound = True
            thermalSocket = features.socket
            thermalWidth, thermalHeight = features.width, features.height
            break
    if not thermalFound:
        raise RuntimeError("No thermal camera found!")

    calibrationHandler = device.readCalibration()
    rgbDistortion = calibrationHandler.getDistortionCoefficients(RGB_SOCKET)
    distortionModel = calibrationHandler.getDistortionModel(RGB_SOCKET)
    if distortionModel != dai.CameraModel.Perspective:
        raise RuntimeError("Unsupported distortion model for RGB camera. This example supports only Perspective model.")

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)
    assert thermalSocket is not None
    thermalCam = pipeline.create(dai.node.Thermal).build(thermalSocket, fps=FPS)

    sync = pipeline.create(dai.node.Sync)
    align = pipeline.create(dai.node.ImageAlign)

    camRgbOut = camRgb.requestOutput(COLOR_RESOLUTION, fps=FPS, enableUndistortion=True)

    sync.setSyncThreshold(timedelta(seconds=3 / FPS))

    cfg = align.initialConfig
    staticDepthPlane = cfg.staticDepthPlane

    # Linking
    align.outputAligned.link(sync.inputs["aligned"])
    camRgbOut.link(sync.inputs["rgb"])
    camRgbOut.link(align.inputAlignTo)
    thermalCam.temperature.link(align.input)
    out = sync.out.createOutputQueue()
    cfgIn = align.inputConfig.createInputQueue()


    rgbWeight = 0.4
    thermalWeight = 0.6


    def updateBlendWeights(percentRgb):
        """
        Update the rgb and depth weights used to blend depth/rgb image
        @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
        """
        global thermalWeight
        global rgbWeight
        rgbWeight = float(percentRgb) / 100.0
        thermalWeight = 1.0 - rgbWeight

    def updateDepthPlane(depth):
        global staticDepthPlane
        staticDepthPlane = depth

    pipeline.start()

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    windowName = "rgb-thermal"

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
    cv2.createTrackbar(
        "Static Depth Plane [mm]",
        windowName,
        0,
        2000,
        updateDepthPlane,
    )
    fpsCounter = FPSCounter()
    while True:
        messageGroup = out.get()
        assert isinstance(messageGroup, dai.MessageGroup)
        frameRgb = messageGroup["rgb"]
        assert isinstance(frameRgb, dai.ImgFrame)
        thermalAligned = messageGroup["aligned"]
        assert isinstance(thermalAligned, dai.ImgFrame)
        frameRgbCv = frameRgb.getCvFrame()
        fpsCounter.tick()

        rgbIntrinsics = calibrationHandler.getCameraIntrinsics(RGB_SOCKET, int(frameRgbCv.shape[1]), int(frameRgbCv.shape[0]))

        # Colorize the aligned depth
        thermalFrame = thermalAligned.getData().view(np.float16).reshape((thermalAligned.getHeight(), thermalAligned.getWidth())).astype(np.float32)
        # Create a mask for nan values
        mask = np.isnan(thermalFrame)
        # Replace nan values with a mean for visualization
        thermalFrame[mask] = np.nanmean(thermalFrame)
        thermalFrame = cv2.normalize(thermalFrame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        colormappedFrame = cv2.applyColorMap(thermalFrame, cv2.COLORMAP_MAGMA)
        # Apply the mask back with black pixels (0)
        colormappedFrame[mask] = 0

        blended = cv2.addWeighted(frameRgbCv, rgbWeight, colormappedFrame, thermalWeight, 0)

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
        cv2.imshow("aligned thermal", colormappedFrame)
        cv2.imshow("undistorted rgb", cvFrameUndistorted)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

        cfg.staticDepthPlane = staticDepthPlane
        cfgIn.send(cfg)
