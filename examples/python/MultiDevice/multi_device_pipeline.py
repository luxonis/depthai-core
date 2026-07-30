#!/usr/bin/env python3
"""One pipeline, several devices.

A single `dai.Pipeline` can drive any number of devices: pass the device to `pipeline.create()` and that node runs on
it, while host nodes see the streams of all of them. Every frame carries the coordinate frame it was taken in
(device id + socket), which is what makes it possible to combine streams of different devices later on.
"""
import argparse
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or device id. Repeat for every device")
parser.add_argument("-n", "--num-devices", type=int, default=2, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-s", "--socket", action="append", default=None, help="Sockets to use on every device, e.g. CAM_B. Defaults to CAM_A")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(640, 400), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
args = parser.parse_args()

sockets = [dai.CameraBoardSocket.__members__[name] for name in (args.socket or ["CAM_A"])]
devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device] if args.device else [dai.Device() for _ in range(args.num_devices)]

# createImplicitDevice=False keeps the pipeline from grabbing a device of its own - the nodes bring their own
with dai.Pipeline(createImplicitDevice=False) as pipeline:
    sync = pipeline.create(dai.node.Sync)
    sync.setRunOnHost(True)
    # Devices are not hardware synced here, so let a group span a couple of frame intervals
    sync.setSyncThreshold(timedelta(seconds=2.0 / args.fps))

    for device in devices:
        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            camera.requestOutput(tuple(args.resolution), fps=args.fps).link(sync.inputs[f"{device.getDeviceId()}_{socket.name}"])

    groupQueue = sync.out.createOutputQueue()

    pipeline.start()
    print(f"Streaming {len(devices) * len(sockets)} cameras of {len(devices)} devices, press q to exit")

    while pipeline.isRunning():
        group: dai.MessageGroup = groupQueue.get()
        frames = []
        for name in sorted(group.getMessageNames()):
            frame: dai.ImgFrame = group[name]
            # Whose camera the pixels come from, as depthai tracks it through the pipeline
            reference = frame.getTransformation().getExtrinsics().getReferenceFrame()
            image = frame.getCvFrame()
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
            cv2.putText(image, f"{reference}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            frames.append(image)

        cv2.imshow("multi device", cv2.hconcat(frames))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
