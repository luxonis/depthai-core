#!/usr/bin/env python3

import argparse
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai

FPS = 30.0
WINDOW_NAME = "Calibrated panorama"

parser = argparse.ArgumentParser(description="Compose a calibrated panorama from multiple devices")
parser.add_argument("--devices", nargs="+", required=True, help="Two or more device IDs or IPs; one camera per device")
parser.add_argument(
    "--socket", choices=[f"CAM_{c}" for c in "ABCDEFGHIJ"], default="CAM_A",
    help="Camera used on each device with --devices (default: CAM_A)",
)
parser.add_argument("--calibration", type=Path, required=True, help="Cross-device calibration JSON")
parser.add_argument(
    "--projection", choices=["Perspective", "Cylindrical", "Equirectangular"], default="Cylindrical",
    help="Panorama surface (default: Cylindrical, suitable for a camera ring)",
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
        # Calibrated composition needs undistorted inputs.
        outputs.append(camera.requestOutput((640, 400), fps=FPS, enableUndistortion=True))

    stitching = pipeline.create(dai.node.Stitching).build(outputs)
    stitching.setMode(dai.node.Stitching.Mode.PANORAMA)
    stitching.setCameraModel(getattr(dai.CameraModel, args.projection))
    # Uncomment to trade seam quality for throughput.
    # stitching.setSeamFinder(dai.node.Stitching.SeamFinder.NONE)
    # Uncomment to register the images visually instead of using the calibration.
    # stitching.setUseInputCalibration(False)
    stitching.setMaxPanoramaSize(2000, 1000)
    stitching.setSyncThreshold(timedelta(seconds=2.0 / FPS))
    output = stitching.out.createOutputQueue()

    pipeline.start()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    try:
        while pipeline.isRunning():
            message = output.tryGet()
            if message is not None:
                cv2.imshow(WINDOW_NAME, message.getCvFrame())
            if cv2.waitKey(1) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
