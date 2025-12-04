#!/usr/bin/env python3
import cv2
import depthai as dai
import argparse
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("calibJsonFile", help="Path to calibration file in json")
args = parser.parse_args()

calibData = dai.CalibrationHandler(args.calibJsonFile)

with dai.Pipeline() as pipeline:
    pipeline.setCalibrationData(calibData)
    # Define sources and output
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    resolution = (640, 480)
    stereo = pipeline.create(dai.node.StereoDepth).build(
        monoLeft.requestOutput(resolution), monoRight.requestOutput(resolution)
    )
    depthQueue = stereo.depth.createOutputQueue()
    pipeline.start()
    while True:
        # blocking call, will wait until a new data has arrived
        inDepth = depthQueue.get()
        frame = inDepth.getFrame()
        # frame is ready to be shown
        cv2.imshow("depth", frame)
        if cv2.waitKey(1) == ord("q"):
            break
