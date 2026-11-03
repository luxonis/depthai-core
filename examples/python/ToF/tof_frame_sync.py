#!/usr/bin/env python3
"""ToF with hardware FSYNC and frame-sync-id staggering (multiple OAK ToF devices).

Run one instance per device. Give every device a different frame-sync-id, so their
ToF illumination is staggered in time and the devices do not interfere.
On OAK-D-SR-PoE the ToF sensor can only receive FSYNC, so the device wired as
FSYNC master must also run a stereo sensor (CAM_B) to generate the pulse: pass --generator.

  python3 tof_frame_sync.py --device 10.0.0.10 --sync-id 0 --generator
  python3 tof_frame_sync.py --device 10.0.0.11 --sync-id 1
"""

import argparse

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("--device", default=None, help="Device IP or ID (default: first available)")
parser.add_argument("--sync-id", type=int, default=0, help="Frame sync slot ID, unique per device")
parser.add_argument("--generator", action="store_true", help="Also run CAM_B to generate FSYNC (FSYNC master)")
parser.add_argument("--fps", type=float, default=10.0)
args = parser.parse_args()

device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
with dai.Pipeline(device) as pipeline:
    tof = pipeline.create(dai.node.ToF).build(dai.CameraBoardSocket.AUTO, dai.ToFConfig.Profile.MID_RANGE, args.fps)
    tof.initialControl.setFrameSyncId(args.sync_id)

    if args.generator:
        # The sensor runs at the ToF fps and generates FSYNC. Its frames are not needed, but v3 requires
        # a consumer: request a native-resolution output (no ImageManip) at 1 fps (almost no bandwidth).
        generator = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=args.fps)
        generatorQueue = generator.requestFullResolutionOutput(fps=1).createOutputQueue(maxSize=1, blocking=False)

    depthQueue = tof.depth.createOutputQueue()
    pipeline.start()
    prev = None
    while pipeline.isRunning():
        depth = depthQueue.get()
        ts = depth.getTimestamp().total_seconds()
        if prev is not None:
            print(f"sync-id {args.sync_id}: depth seq {depth.getSequenceNum()}, host ts {ts:.6f} s, interval {(ts - prev) * 1000:.1f} ms")
        prev = ts
