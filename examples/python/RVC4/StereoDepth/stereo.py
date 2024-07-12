#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
from pathlib import Path


# Temporary toy example until the RVC4 devices are calibrated
examplePath = (Path(__file__).parent / ".." / "..").resolve().absolute()
calibJsonFile = examplePath / "models" / "depthai_calib.json"
# Check if file exists otherwise provoke the user to run `python3 examples/python/install_requirements.py`
if not calibJsonFile.exists():
    import sys

    print(
        f"Calibration file not found at: {calibJsonFile}. Please run {sys.executable} {examplePath}/install_requirements.py to get it downloaded."
    )
    exit(1)

calibData = dai.CalibrationHandler(calibJsonFile)

# Create pipeline
pipeline = dai.Pipeline()
pipeline.setCalibrationData(calibData)

monoLeft = pipeline.create(dai.node.Camera)
monoRight = pipeline.create(dai.node.Camera)
stereo = pipeline.create(dai.node.StereoDepth)

# Define sources and outputs
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_A)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_B)

# Linking
monoLeftOut = monoLeft.requestOutput((640, 400), type=dai.ImgFrame.Type.NV12)
monoRightOut = monoRight.requestOutput((640, 400), type=dai.ImgFrame.Type.NV12)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)


stereo.setInputResolution(640, 400)
stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setNumFramesPool(10)
syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
syncedRightQueue = stereo.syncedRight.createOutputQueue()
disparityQueue = stereo.disparity.createOutputQueue()

with pipeline:
    pipeline.setCalibrationData(calibData)
    pipeline.start()
    while pipeline.isRunning():
        leftSynced = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        disparity = disparityQueue.get()
        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)
        cv2.imshow("left", leftSynced.getCvFrame())
        cv2.imshow("right", rightSynced.getCvFrame())
        cv2.imshow("disparity", (disparity.getFrame()).astype(np.uint8))
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
