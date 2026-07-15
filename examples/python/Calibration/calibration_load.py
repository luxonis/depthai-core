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
    # The Depth node manages its own stereo cameras and backend internally and
    # uses the calibration data set on the pipeline above.
    resolution = (640, 480)
    depth = pipeline.create(dai.node.Depth).build(
        dai.node.Depth.Algorithm.STEREO, size=resolution
    )
    depthQueue = depth.depth.createOutputQueue()
    pipeline.start()
    while True:
        # blocking call, will wait until a new data has arrived
        inDepth = depthQueue.get()
        frame = inDepth.getFrame()
        # frame is ready to be shown
        cv2.imshow("depth", frame)
        if cv2.waitKey(1) == ord("q"):
            break
