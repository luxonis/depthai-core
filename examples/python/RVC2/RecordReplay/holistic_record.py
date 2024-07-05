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
    camRgb = pipeline.create(dai.node.ColorCamera)

    # Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setVideoSize(1920, 1080)
    camRgb.setFps(30)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500);
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400);

    config = dai.RecordConfig()
    config.outputDir = args.output;
    config.videoEncoding.enabled = True
    config.videoEncoding.bitrate = 0 # Automatic
    config.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN

    pipeline.enableHolisticRecord(config)

    videoQueue = camRgb.preview.createOutputQueue()
    imuQueue = imu.out.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn : dai.ImgFrame = videoQueue.get()
        imuData : dai.IMUData = imuQueue.get()

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        cv2.imshow("video", videoIn.getCvFrame())

        for packet in imuData.packets:
            print(f"IMU Accelerometer: {packet.acceleroMeter}")
            print(f"IMU Gyroscope: {packet.gyroscope}")

        if cv2.waitKey(1) == ord('q'):
            break
