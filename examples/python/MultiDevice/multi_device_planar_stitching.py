#!/usr/bin/env python3
"""Bird's eye view of a multi-device rig.

Every camera stream is re-expressed in the frame of the reference camera with CoordinateFrameTransform, using the rig
calibration estimated by MultiDeviceCalibration (and saved to json), and then projected onto a plane - typically the
ground - by the Stitching node in PLANAR_PROJECTION mode.
"""
import argparse
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--calibration", type=Path, required=True, help="Calibration json, as written by CalibrationHandler.eepromToJsonFile()")
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or MX id. The first one holds the reference camera")
parser.add_argument("-n", "--num-devices", type=int, default=2, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-s", "--socket", default="CAM_A", help="Camera socket used on every device")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(640, 400), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
parser.add_argument("--plane-point", type=float, nargs=3, default=(0.0, 150.0, 0.0), help="A point of the plane, in cm in the reference camera frame")
parser.add_argument("--plane-normal", type=float, nargs=3, default=(0.0, -1.0, 0.0), help="Normal of the plane, in the reference camera frame")
parser.add_argument("--range", type=float, default=1000.0, help="How far from the cameras the plane is still rendered, in cm")
parser.add_argument("--view-size", type=int, nargs=2, default=(1280, 1280), help="Upper bound on the size of the computed view")
args = parser.parse_args()

socket = dai.CameraBoardSocket.__members__[args.socket]
devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device] if args.device else [dai.Device() for _ in range(args.num_devices)]
# The plane and the rendered view are expressed in the frame of this camera
reference = dai.CoordinateFrame(devices[0].getDeviceId(), socket)

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    # Poses between the devices, so that the streams can be brought into a single frame
    pipeline.setMultiDeviceCalibration(dai.CalibrationHandler(args.calibration))

    outputs = []
    for device in devices:
        camera = pipeline.create(dai.node.Camera, device).build(socket)
        outputs.append(camera.requestOutput(tuple(args.resolution), fps=args.fps))

    # Metadata only: the images keep their pixels, but their extrinsics now all point at the reference camera
    unified = pipeline.create(dai.node.CoordinateFrameTransform).build(outputs, reference)

    stitching = pipeline.create(dai.node.Stitching).build([unified.outputs[f"output{index}"] for index in range(len(outputs))])
    stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
    stitching.setPlane(dai.Point3f(*args.plane_point), dai.Point3f(*args.plane_normal))
    stitching.setMaxRange(args.range)
    stitching.setMaxViewSize(*args.view_size)
    # Leaving the view unset renders the part of the plane the cameras actually see; setView() places the virtual
    # camera by hand instead, e.g.
    # stitching.setView(dai.node.Stitching.VirtualCamera.lookAt(position=..., target=..., up=..., hFovDegrees=90.0, width=1280, height=1280))
    stitching.setSyncThreshold(timedelta(seconds=2.0 / args.fps))

    projectedQueue = stitching.out.createOutputQueue()

    pipeline.start()
    print(f"Projecting {stitching.getNumInputs()} cameras onto the plane, press q to exit")

    while pipeline.isRunning():
        projected = projectedQueue.get()
        cv2.imshow("planar stitch", projected.getCvFrame())

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
