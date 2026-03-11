#!/usr/bin/env python3

import cv2
import depthai as dai
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default="recordings", help="Output path")
args = parser.parse_args()

# Create output directory if it doesn't exist
Path(args.output).mkdir(parents=True, exist_ok=True)

# Create pipeline
with dai.Pipeline(True) as pipeline:
    config = dai.RecordConfig()
    config.outputDir = args.output
    # config.videoEncoding.enabled = True
    # config.videoEncoding.bitrate = 0 # Automatic
    # config.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN

    pipeline.enableHolisticRecord(config)

    # Define source and output
    camA = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camAOut = camA.requestFullResolutionOutput()
    camB = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camBOut = camB.requestFullResolutionOutput()
    camC = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    camCOut = camC.requestFullResolutionOutput()

    viewFinderOut = camA.requestOutput((640, 480))

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    imu.setBatchReportThreshold(100)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncAttempts(0)
    camAOut.link(sync.inputs["camA"])
    camBOut.link(sync.inputs["camB"])
    camCOut.link(sync.inputs["camC"])
    imu.out.link(sync.inputs["imu"])

    viewFinderQueue = viewFinderOut.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    try:
        while pipeline.isRunning():
            frame = viewFinderQueue.get()
            cv2.imshow("video", frame.getCvFrame())
            if cv2.waitKey(1) == ord('q'):
                break
    except KeyboardInterrupt:
        pass

    pipeline.stop()
