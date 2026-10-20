#!/usr/bin/env python3
"""Device-to-device link in ONE dai.Pipeline.

A Camera on device A is linked directly to an ImageManip running on device B.
The pipeline inserts the host relay automatically at build time (visible as one
info log line); no manual forwarding is needed.
"""
import sys

import cv2
import depthai as dai

if len(sys.argv) >= 3:
    deviceInfos = [dai.DeviceInfo(sys.argv[1]), dai.DeviceInfo(sys.argv[2])]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

with dai.Pipeline(False) as pipeline:
    deviceA = pipeline.addDevice(deviceInfos[0])
    deviceB = pipeline.addDevice(deviceInfos[1])

    camera = pipeline.create(dai.node.Camera, deviceA).build(dai.CameraBoardSocket.CAM_A)
    manip = pipeline.create(dai.node.ImageManip, deviceB)
    manip.initialConfig.setOutputSize(320, 200)

    # Cross-device link: relayed through the host automatically
    camera.requestOutput((640, 400)).link(manip.inputImage)

    queue = manip.out.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        frame = queue.get()
        if frame is None:
            continue
        cv2.imshow("device_to_device_relay", frame.getCvFrame())
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
