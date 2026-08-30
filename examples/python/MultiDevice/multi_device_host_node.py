#!/usr/bin/env python3
"""A custom host node consuming streams from several devices in ONE dai.Pipeline.

The node discovers which device feeds each of its inputs with
Input.getSourceDevice() (resolved at pipeline build) and composes a mosaic.
It keeps running when a device disappears: the lost device's tile freezes and is
labeled OFFLINE while the other tiles keep updating (partial operation).
"""
import os
import sys
import time

import cv2
import depthai as dai


class MosaicNode(dai.node.ThreadedHostNode):
    def __init__(self, pipeline):
        super().__init__()
        self.pipeline = pipeline
        self.inputsByName = {}
        self.output = self.createOutput()

    def addStream(self, name):
        inp = self.createInput(name=name, blocking=False, queueSize=4)
        self.inputsByName[name] = inp
        return inp

    def run(self):
        # Which device produces each input - valid after pipeline build
        sources = {name: inp.getSourceDevice() for name, inp in self.inputsByName.items()}
        latest = {}
        while self.mainLoop():
            anyNew = False
            for name, inp in self.inputsByName.items():
                frame = inp.tryGet()
                if frame is not None:
                    latest[name] = frame.getCvFrame()
                    anyNew = True
            if not anyNew:
                time.sleep(0.005)
                continue

            tiles = []
            for name in sorted(latest):
                tile = latest[name].copy()
                device = sources.get(name)
                label = name
                if device is not None and self.pipeline.getDeviceState(device) != dai.DeviceState.RUNNING:
                    label += " [OFFLINE]"
                    tile = cv2.cvtColor(cv2.cvtColor(tile, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
                cv2.putText(tile, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 127, 255), 2, cv2.LINE_AA)
                tiles.append(tile)

            mosaic = cv2.hconcat(tiles)
            outFrame = dai.ImgFrame()
            outFrame.setCvFrame(mosaic, dai.ImgFrame.Type.BGR888i)
            self.output.send(outFrame)


if len(sys.argv) >= 2:
    deviceInfos = [dai.DeviceInfo(arg) for arg in sys.argv[1:]]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

with dai.Pipeline(False) as pipeline:
    mosaic = MosaicNode(pipeline)

    for info in deviceInfos:
        device = pipeline.addDevice(info)
        camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A)
        camera.requestOutput((640, 400)).link(mosaic.addStream(device.getDeviceId()))

    queue = mosaic.output.createOutputQueue()
    pipeline.start()

    display = bool(os.environ.get("DISPLAY"))
    while pipeline.isRunning():
        frame = queue.get()
        if frame is None:
            continue
        if display:
            cv2.imshow("multi_device_host_node", frame.getCvFrame())
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            print(f"Mosaic frame {frame.getWidth()}x{frame.getHeight()}")
