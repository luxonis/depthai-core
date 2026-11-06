#!/usr/bin/env python3
"""A custom host node consuming streams from several devices in ONE dai.Pipeline.

The node discovers which device feeds each of its inputs with
Input.getSourceDevice() (resolved at pipeline build) and composes a mosaic.
Every tile shows its frame timestamp; the mosaic shows the max diff between the
newest and the oldest timestamp of the frames shown together.
It keeps running when a device disappears: the lost device's tile freezes and is
labeled OFFLINE while the other tiles keep updating (partial operation).
With --stop-on-device-loss the pipeline stops instead when any device is lost for good.
"""
import argparse
import time

import cv2
import depthai as dai


class MosaicNode(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
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
                    latest[name] = (frame.getCvFrame(), frame.getTimestamp())
                    anyNew = True
            if not anyNew:
                time.sleep(0.005)
                continue

            tiles = []
            liveTimestamps = []
            for name in sorted(latest):
                image, timestamp = latest[name]
                tile = image.copy()
                device = sources.get(name)
                label = f"{name}  {timestamp.total_seconds():.3f} s"
                if device is not None and device.getDeviceState() != dai.DeviceState.RUNNING:
                    label += " [OFFLINE]"
                    tile = cv2.cvtColor(cv2.cvtColor(tile, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
                else:
                    liveTimestamps.append(timestamp)
                cv2.putText(tile, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 127, 255), 2, cv2.LINE_AA)
                tiles.append(tile)

            mosaic = cv2.hconcat(tiles)
            if len(liveTimestamps) >= 2:
                diffMs = (max(liveTimestamps) - min(liveTimestamps)).total_seconds() * 1e3
                cv2.putText(mosaic, f"max diff = {diffMs:.2f} ms", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            outFrame = dai.ImgFrame()
            outFrame.setCvFrame(mosaic, dai.ImgFrame.Type.BGR888i)
            self.output.send(outFrame)


parser = argparse.ArgumentParser()
parser.add_argument("devices", nargs="*", help="Device ids or IPs (default: all available devices)")
parser.add_argument("--stop-on-device-loss", action="store_true", help="Stop the pipeline when any device is lost for good")
args = parser.parse_args()

if args.devices:
    deviceInfos = [dai.DeviceInfo(arg) for arg in args.devices]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

with dai.Pipeline(False) as pipeline:
    # Off by default (partial operation)
    pipeline.setStopOnDeviceLoss(args.stop_on_device_loss)
    mosaic = pipeline.create(MosaicNode)

    for info in deviceInfos:
        device = pipeline.addDevice(info)
        camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A)
        camera.requestOutput((1280, 800)).link(mosaic.addStream(device.getDeviceId()))

    queue = mosaic.output.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        try:
            frame = queue.get()
        except dai.MessageQueue.QueueException:
            # The pipeline stopped itself after a device loss
            print("Pipeline stopped - a device was lost")
            break
        if frame is None:
            continue
        cv2.imshow("multi_device_stream", frame.getCvFrame())
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
