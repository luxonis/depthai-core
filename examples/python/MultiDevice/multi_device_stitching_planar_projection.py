#!/usr/bin/env python3

import argparse
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai

FPS = 10.0

parser = argparse.ArgumentParser(description="Project cameras from multiple devices onto a plane")
parser.add_argument("--devices", nargs="+", required=True, help="Two or more device IDs or IPs; one camera per device")
parser.add_argument(
    "--socket", choices=[f"CAM_{c}" for c in "ABCDEFGHIJ"], default="CAM_A",
    help="Camera used on each device with --devices (default: CAM_A)",
)
parser.add_argument("--calibration", type=Path, required=True, help="Cross-device calibration JSON")
parser.add_argument(
    "--plane-point", type=float, nargs=3, default=[0, 0, 130], metavar=("X", "Y", "Z"),
    help="Point on the plane in the common calibration origin, in centimetres",
)
parser.add_argument(
    "--plane-normal", type=float, nargs=3, default=[0, 1, 1], metavar=("X", "Y", "Z"),
    help="Plane normal in the common calibration origin",
)
args = parser.parse_args()
if len(args.devices) < 2:
    parser.error("--devices requires at least two devices")
if not hasattr(getattr(dai, "beta", None), "MultiDeviceCalibrationHandler"):
    parser.error("loading cross-device calibration requires a build with beta support")

# Load and validate the graph before connecting to any devices.
calibration = dai.beta.MultiDeviceCalibrationHandler(args.calibration)
with dai.Pipeline(False) as pipeline:
    pipeline.setMultiDeviceCalibration(calibration.getGraph())
    pipeline.setStopOnDeviceLoss(True)
    outputs = []
    socket = getattr(dai.CameraBoardSocket, args.socket)
    for deviceId in args.devices:
        info = dai.DeviceInfo(deviceId)
        device = pipeline.addDevice(info)
        print(f"Using device {device.getDeviceId()}")
        camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=FPS)
        outputs.append(camera.requestOutput((640, 400), fps=FPS))

    stitching = pipeline.create(dai.node.Stitching).build(outputs)
    stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
    stitching.setPlane(
        dai.Point3f(*args.plane_point),  # In the shared calibration origin, in centimetres
        dai.Point3f(*args.plane_normal),
        dai.LengthUnit.CENTIMETER,
    )
    stitching.setMaxRange(2.0, dai.LengthUnit.METER)
    stitching.setMaxViewSize(1280, 720)
    stitching.setSyncThreshold(timedelta(seconds=2.0 / FPS))
    output = stitching.out.createOutputQueue()

    pipeline.start()
    cv2.namedWindow("Calibrated planar projection", cv2.WINDOW_AUTOSIZE)
    try:
        while pipeline.isRunning():
            message = output.tryGet()
            if message is not None:
                cv2.imshow("Calibrated planar projection", message.getCvFrame())
            if cv2.waitKey(1) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
