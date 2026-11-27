#!/usr/bin/env python3
"""Device-to-device link in ONE dai.Pipeline.

A Camera on device A is linked directly to an ImageManip running on device B.
At build time the pipeline sends the frames directly from A to B when both are
RVC4 devices that reach each other, and relays them through the host otherwise.
One info log line per link tells which it is, and why; no manual forwarding is
needed. Pass --relay to always relay through the host, to compare.

Usage: device_to_device_relay.py [deviceA deviceB] [--relay]
"""
import sys

import cv2
import depthai as dai

forceRelay = "--relay" in sys.argv
args = [arg for arg in sys.argv[1:] if arg != "--relay"]
if len(args) >= 2:
    deviceInfos = [dai.DeviceInfo(args[0]), dai.DeviceInfo(args[1])]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

with dai.Pipeline(False) as pipeline:
    deviceA = pipeline.addDevice(deviceInfos[0])
    deviceB = pipeline.addDevice(deviceInfos[1])
    if forceRelay:
        pipeline.setCrossDeviceLinkMode(dai.Pipeline.CrossDeviceLinkMode.HOST_RELAY)

    camera = pipeline.create(dai.node.Camera, deviceA).build(dai.CameraBoardSocket.CAM_A)
    manip = pipeline.create(dai.node.ImageManip, deviceB)
    manip.initialConfig.setOutputSize(320, 200)

    # Cross-device link: direct between the devices, or relayed through the host
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
