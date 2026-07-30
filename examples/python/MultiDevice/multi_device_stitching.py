#!/usr/bin/env python3
"""Panorama of a multi-device rig.

The Stitching node in its default PANORAMA mode registers the images from their content, the way a photo stitcher
does: no calibration is needed, but neighbouring cameras have to overlap and the scene should be far enough away for
a rotation-only model to hold. For a calibrated projection onto a plane instead, see multi_device_planar_stitching.py.
"""
import argparse
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or device id. Repeat for every camera, in left to right order")
parser.add_argument("-n", "--num-devices", type=int, default=2, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-s", "--socket", default="CAM_A", help="Camera socket used on every device")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(640, 480), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
parser.add_argument("-m", "--model", default="SPHERICAL", choices=("SPHERICAL", "CYLINDRICAL", "PINHOLE"), help="Surface the images are warped onto")
parser.add_argument("-c", "--continuous", action="store_true", help="Re-estimate the transform on every frame instead of fixing it")
args = parser.parse_args()

socket = dai.CameraBoardSocket.__members__[args.socket]
devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device] if args.device else [dai.Device() for _ in range(args.num_devices)]

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    outputs = []
    for device in devices:
        camera = pipeline.create(dai.node.Camera, device).build(socket)
        outputs.append(camera.requestOutput(tuple(args.resolution), fps=args.fps))

    stitching = pipeline.create(dai.node.Stitching).build(outputs)
    stitching.setMode(dai.node.Stitching.Mode.PANORAMA)
    # A cylinder keeps straight lines straight in a wide horizontal panorama, a plane suits small fields of view
    stitching.setCameraModel(dai.node.Stitching.CameraModel.__members__[args.model])
    # Registration runs on the first groups and the transform is then reused, unless the cameras move w.r.t. each other
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
