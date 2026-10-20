#!/usr/bin/env python3

import argparse
from datetime import timedelta

import cv2
import depthai as dai

FPS = 10.0

parser = argparse.ArgumentParser(description="Project CAM_B and CAM_C onto a plane")
parser.add_argument("--device-ip", help="Device IP address (default: auto-discover)")
args = parser.parse_args()

device = dai.Device(dai.DeviceInfo(args.device_ip)) if args.device_ip else dai.Device()
with dai.Pipeline(device) as pipeline:
    outputs = []
    for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
        camera = pipeline.create(dai.node.Camera).build(socket, sensorFps=FPS)
        outputs.append(camera.requestOutput((640, 400), fps=FPS))

    stitching = pipeline.create(dai.beta.node.Stitching).build(outputs)
    stitching.setMode(dai.beta.node.Stitching.Mode.PLANAR_PROJECTION)
    stitching.setPlane(
        dai.Point3f(0, 0, 130),   # A point on the plane, in centimetres
        dai.Point3f(0, 1, 1),    # Plane normal
        dai.LengthUnit.CENTIMETER,
    )
    stitching.setMaxRange(2.0, dai.LengthUnit.METER)
    stitching.setMaxViewSize(1280, 720)
    stitching.setSyncThreshold(timedelta(seconds=2.0 / FPS))
    output = stitching.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        cv2.imshow("CAM_B + CAM_C planar projection", output.get().getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
