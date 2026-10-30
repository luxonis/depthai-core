#!/usr/bin/env python3
"""Frame synchronization across several devices in ONE dai.Pipeline.

Every device's CAM_A camera is created in the same pipeline with an explicit device
(pipeline.create(dai.node.Camera, device)) and links directly into one host Sync
node - no per-device pipelines, no manual queue pumping between them.

Hardware sync is configured the same way as before:
  --external-sync  FSYNC wiring: the master device strobes, slaves lock to it
  --ptp-sync       PTP: cameras timestamp on the PTP-synchronized system clock
Without either option the cameras run free and only the host Sync node pairs the
frames by host timestamp (software sync, no special wiring or network setup).

Every tile shows its frame timestamp; the mosaic shows the max diff between the
newest and the oldest timestamp of the synced group.
"""
import argparse
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--fps", type=float, default=30.0, help="Target FPS")
parser.add_argument("-d", "--devices", nargs="+", default=[], help="Device ids or IPs")
parser.add_argument(
    "-t",
    "--sync-threshold-sec",
    type=float,
    default=None,
    help="Sync threshold in seconds (default: 1 ms with hardware sync, 20 ms with host sync)",
)
group = parser.add_mutually_exclusive_group()
group.add_argument("--external-sync", action="store_true", help="Use FSYNC wiring")
group.add_argument("--ptp-sync", action="store_true", help="Use PTP time sync")
args = parser.parse_args()

hostSyncOnly = not (args.external_sync or args.ptp_sync)
if args.sync_threshold_sec is None:
    args.sync_threshold_sec = 20e-3 if hostSyncOnly else 1e-3

if args.devices:
    deviceInfos = [dai.DeviceInfo(d) for d in args.devices]
else:
    deviceInfos = dai.Device.getAllAvailableDevices()
if len(deviceInfos) < 2:
    print("At least two devices are required for this example.")
    raise SystemExit(0)

# One pipeline; the first added device becomes the master (default device)
with dai.Pipeline(False) as pipeline:

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

        if not args.external_sync or role == dai.ExternalFrameSyncRole.MASTER:
            cam = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A, sensorFps=args.fps)
        else:
            # FSYNC slaves lock to the master's strobe
            cam = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A)
        if args.ptp_sync:
            cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
        name = device.getDeviceId()
        cam.requestOutput((1280, 800), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH).link(sync.inputs[name])
        inputNames.append(name)

    queue = sync.out.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        group_msg = queue.get()
        if group_msg is None:
            continue
        # Newest minus oldest frame timestamp of the group
        maxDiffMs = group_msg.getIntervalNs() / 1e6

        frames = []
        for name in inputNames:
            frame = group_msg[name]
            img = frame.getCvFrame()
            label = f"{name}  {frame.getTimestamp().total_seconds():.3f} s"
            cv2.putText(img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 127, 255), 2, cv2.LINE_AA)
            frames.append(img)
        combined = cv2.hconcat(frames)
        cv2.putText(combined, f"max diff = {maxDiffMs:.2f} ms", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("multi_device_frame_sync", combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
