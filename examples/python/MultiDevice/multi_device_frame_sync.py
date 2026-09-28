#!/usr/bin/env python3
"""Frame synchronization across several devices in ONE dai.Pipeline.

Every device's cameras are created in the same pipeline with an explicit device
(pipeline.create(dai.node.Camera, device)) and link directly into one host Sync
node - no per-device pipelines, no manual queue pumping between them.

Hardware sync is configured the same way as before:
  --external-sync  FSYNC wiring: the master device strobes, slaves lock to it
  --ptp-sync       PTP: cameras timestamp on the PTP-synchronized system clock
"""
import argparse
import os
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--fps", type=float, default=30.0, help="Target FPS")
parser.add_argument("-d", "--devices", nargs="+", default=[], help="Device ids or IPs")
parser.add_argument("-t", "--sync-threshold-sec", type=float, default=1e-3, help="Sync threshold in seconds")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--external-sync", action="store_true", help="Use FSYNC wiring")
group.add_argument("--ptp-sync", action="store_true", help="Use PTP time sync")
args = parser.parse_args()

if args.devices:
    deviceInfos = [dai.DeviceInfo(d) for d in args.devices]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

# One pipeline; the first added device becomes the master (default device)
pipeline = dai.Pipeline(False)

sync = pipeline.create(dai.node.Sync)
sync.setRunOnHost(True)
sync.setSyncThreshold(timedelta(seconds=args.sync_threshold_sec))

inputNames = []
for info in deviceInfos:
    device = pipeline.addDevice(info)
    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError("This example supports only the RVC4 platform!")

    role = None
    if args.external_sync:
        role = device.getExternalFrameSyncRole()
        if role == dai.ExternalFrameSyncRole.MASTER:
            device.setExternalStrobeEnable(True)
            print(f"{device.getDeviceId()} is FSYNC master")
        else:
            print(f"{device.getDeviceId()} is FSYNC slave")

    for socket in device.getConnectedCameras():
        if args.ptp_sync or role == dai.ExternalFrameSyncRole.MASTER:
            cam = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=args.fps)
        else:
            # FSYNC slaves lock to the master's strobe
            cam = pipeline.create(dai.node.Camera, device).build(socket)
        if args.ptp_sync:
            cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
        name = f"{device.getDeviceId()}_{socket.name}"
        cam.requestOutput((640, 400), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH).link(sync.inputs[name])
        inputNames.append(name)

queue = sync.out.createOutputQueue()
pipeline.start()

display = bool(os.environ.get("DISPLAY"))
while pipeline.isRunning():
    group_msg = queue.get()
    if group_msg is None:
        continue
    deltaMs = group_msg.getIntervalNs() / 1e6
    print(f"Synced group of {group_msg.getNumMessages()} frames, timestamp spread {deltaMs:.2f} ms")

    if display:
        frames = []
        for name in inputNames:
            frame = group_msg[name]
            img = frame.getCvFrame()
            cv2.putText(img, name, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 127, 255), 2, cv2.LINE_AA)
            frames.append(img)
        combined = cv2.hconcat(frames)
        cv2.putText(combined, f"delta = {deltaMs:.2f} ms", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("multi_device_frame_sync", combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

pipeline.stop()
