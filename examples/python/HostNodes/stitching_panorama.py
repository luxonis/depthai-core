#!/usr/bin/env python3

import argparse
from datetime import timedelta

import cv2
import depthai as dai

FPS = 30.0
WINDOW_NAME = "CAM_B + CAM_C panorama"

parser = argparse.ArgumentParser(description="Stitch CAM_B and CAM_C into a panorama")
parser.add_argument("--device-ip", help="Device IP address (default: auto-discover)")
args = parser.parse_args()

device = dai.Device(dai.DeviceInfo(args.device_ip)) if args.device_ip else dai.Device()
with dai.Pipeline(device) as pipeline:
    outputs = []
    for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
        camera = pipeline.create(dai.node.Camera).build(socket, sensorFps=FPS)
        outputs.append(camera.requestOutput((640, 400), fps=FPS))

    stitching = pipeline.create(dai.beta.node.Stitching).build(outputs)
    stitching.setMode(dai.beta.node.Stitching.Mode.PANORAMA)
    stitching.setCameraModel(dai.beta.node.Stitching.CameraModel.PINHOLE)
    # Uncomment to trade seam quality for throughput.
    # stitching.setSeamFinder(dai.beta.node.Stitching.SeamFinder.NONE)
    stitching.setPanoConfidenceThreshold(0.3)
    stitching.setContinuous(False)
    stitching.setEstimationFrames(10)
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
