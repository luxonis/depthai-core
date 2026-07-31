#!/usr/bin/env python3
"""Record the synchronized streams of a multi-device rig for offline replay.

The recording is self-contained: besides the per-camera video + metadata it stores every device's factory calibration
and, when given, the rig calibration json. That is everything MultiDeviceCalibration and the calibrated Stitching node
need, so the recorded session can be replayed (multi_device_replay.py) without the devices being connected.

Layout written to the output directory:
    <deviceId>_<SOCKET>.avi / .mcap   one pair per camera stream
    <deviceId>_calibration.json       factory calibration of every device
    rig.json                          the rig calibration, copied from --calibration when given
    session.json                      manifest describing the streams
"""
import argparse
import json
import time
from pathlib import Path

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", type=Path, required=True, help="Directory the recording is written to")
parser.add_argument("-d", "--device", action="append", help="Device to use, by IP or MX id. The first one holds the reference camera")
parser.add_argument("-n", "--num-devices", type=int, default=3, help="Number of devices to discover when none are given explicitly")
parser.add_argument("-s", "--socket", action="append", help="Camera sockets to record on every device (default CAM_B and CAM_C)")
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(1280, 800), help="Resolution requested from every camera")
parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate requested from every camera")
parser.add_argument("-t", "--seconds", type=float, default=6.0, help="How long to record")
parser.add_argument("-c", "--calibration", type=Path, help="Rig calibration json to store with the recording")
args = parser.parse_args()

sockets = [dai.CameraBoardSocket.__members__[name] for name in (args.socket or ["CAM_B", "CAM_C"])]
devices = [dai.Device(dai.DeviceInfo(name)) for name in args.device] if args.device else [dai.Device() for _ in range(args.num_devices)]
args.output.mkdir(parents=True, exist_ok=True)

reference = {"deviceId": devices[0].getDeviceId(), "socket": sockets[0].name}
streams = []

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    for device in devices:
        deviceId = device.getDeviceId()
        # The factory calibration travels with the recording so the geometry can be reconstructed without the device
        device.readCalibration().eepromToJsonFile(str(args.output / f"{deviceId}_calibration.json"))
        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            output = camera.requestOutput(tuple(args.resolution), dai.ImgFrame.Type.NV12, fps=args.fps)

            name = f"{deviceId}_{socket.name}"
            record = pipeline.create(dai.node.RecordVideo)
            record.setRecordVideoFile(args.output / f"{name}.avi")
            record.setRecordMetadataFile(args.output / f"{name}.mcap")
            output.link(record.input)
            streams.append({"deviceId": deviceId, "socket": socket.name, "video": f"{name}.avi", "metadata": f"{name}.mcap"})

    if args.calibration is not None:
        # Keep the rig with the recording so a replay reproduces the whole geometry
        (args.output / "rig.json").write_text(args.calibration.read_text())

    manifest = {
        "reference": reference,
        "resolution": list(args.resolution),
        "fps": args.fps,
        "devices": [device.getDeviceId() for device in devices],
        "streams": streams,
        "hasRig": args.calibration is not None,
    }
    (args.output / "session.json").write_text(json.dumps(manifest, indent=2))

    pipeline.start()
    print(f"Recording {len(streams)} streams of {len(devices)} devices to {args.output} for {args.seconds}s", flush=True)
    deadline = time.monotonic() + args.seconds
    while pipeline.isRunning() and time.monotonic() < deadline:
        time.sleep(0.1)

print(f"Wrote recording to {args.output}", flush=True)
