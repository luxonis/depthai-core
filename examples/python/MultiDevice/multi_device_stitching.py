#!/usr/bin/env python3
import argparse
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or MX id. Repeat for every camera, in left to right order")
parser.add_argument("-n", "--num-devices", type=int, default=2, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(640, 480), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
parser.add_argument("-c", "--continuous", action="store_true", help="Re-estimate the transform on every frame instead of fixing it")
args = parser.parse_args()

if args.device:
    devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device]
else:
    devices = [dai.Device() for _ in range(args.num_devices)]

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    outputs = []
    for device in devices:
        camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A)
        outputs.append(camera.requestOutput(tuple(args.resolution), fps=args.fps))

    stitching = pipeline.create(dai.node.Stitching).build(outputs)
    stitching.setCameraModel(dai.node.Stitching.CameraModel.SPHERICAL)
    stitching.setContinuous(args.continuous)
    # Free running cameras are not hardware synced, so allow a group to span a couple of frame intervals
    stitching.setSyncThreshold(timedelta(seconds=2.0 / args.fps))

    panoramaQueue = stitching.out.createOutputQueue()

    pipeline.start()
    print(f"Stitching {stitching.getNumInputs()} cameras, press q to exit")

    while pipeline.isRunning():
        panorama = panoramaQueue.get()
        cv2.imshow("panorama", panorama.getCvFrame())

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
