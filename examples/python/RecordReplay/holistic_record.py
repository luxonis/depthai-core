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
    # Define source and output
    camA = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camAOut = camA.requestOutput((600, 400))
    camB = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camBOut = camB.requestOutput((600, 400))
    camC = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    camCOut = camC.requestOutput((600, 400))

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    imu.setBatchReportThreshold(100)

    config = dai.RecordConfig()
    config.outputDir = args.output
    config.videoEncoding.enabled = True
    config.videoEncoding.bitrate = 0 # Automatic
    config.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN

    pipeline.enableHolisticRecord(config)

    videoQueueA = camAOut.createOutputQueue()
    videoQueueB = camBOut.createOutputQueue()
    videoQueueC = camCOut.createOutputQueue()
    imuQueue = imu.out.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoInA : dai.ImgFrame = videoQueueA.get()
        videoInB : dai.ImgFrame = videoQueueB.get()
        videoInC : dai.ImgFrame = videoQueueC.get()
        imuData : dai.IMUData = imuQueue.tryGetAll()

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoInA.getCvFrame())
        if imuData:
            for packet in imuData[0].packets:
                print(f"IMU Accelerometer: {packet.acceleroMeter}")
                print(f"IMU Gyroscope: {packet.gyroscope}")

            if cv2.waitKey(1) == ord('q'):
                break
