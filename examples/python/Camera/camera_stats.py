#!/usr/bin/env python3
"""Poll two physical sensors. See examples/CameraStats.md for counter coverage and deltas."""

import argparse
import time

import depthai as dai

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("first", choices=[f"CAM_{letter}" for letter in "ABCDEFGHIJ"])
parser.add_argument("second", choices=[f"CAM_{letter}" for letter in "ABCDEFGHIJ"])
parser.add_argument("seconds", type=float, nargs="?", default=30)
parser.add_argument("--device", help="RVC4 device ID or IP address; otherwise discover a connected RVC4")
parser.add_argument("--verbose", action="store_true", help="Also print the individual CSL cause counters")
args = parser.parse_args()
if args.first == args.second:
    parser.error("Specify two different physical sockets")
if args.seconds <= 0:
    parser.error("Duration must be positive")
sockets = {getattr(dai.CameraBoardSocket, args.first), getattr(dai.CameraBoardSocket, args.second)}
counters = [
    "completedCaptures",
    "halDeviceErrors",
    "halRequestErrors",
    "halResultErrors",
    "halBufferErrors",
    "cslErrorEvents",
    "linkStalledEvents",
    "csidFatalEvents",
    "csidFifoOverflowEvents",
    "csidRecoveryOverflowEvents",
    "csidLaneFifoOverflowEvents",
    "csidPixelCountMismatchEvents",
    "csidPacketHeaderCorruptionEvents",
    "csidMissingPacketHeaderEvents",
    "csidSensorSwitchingEvents",
    "csidUnboundedFrameEvents",
    "icpNoMemoryEvents",
    "icpSystemFailureEvents",
    "csidMissingEotEvents",
    "csidPayloadCorruptionEvents",
    "sensorStreamOffFailureEvents",
]
if not args.verbose:
    counters = counters[:6]
previous = {}
deviceInfo = dai.DeviceInfo(args.device) if args.device else dai.DeviceInfo()
deviceInfo.platform = dai.XLinkPlatform.X_LINK_RVC4
pipeline = dai.Pipeline(dai.Device(deviceInfo))
with pipeline:
    device = pipeline.getDefaultDevice()
    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError("This example requires an RVC4 device; use --device to select one")
    print(f"Using RVC4 {device.getDeviceId()}", flush=True)
    connected = {camera.socket for camera in device.getConnectedCameraFeatures()}
    if not sockets.issubset(connected):
        raise RuntimeError(f"Requested sockets are not connected. Available: {sorted(socket.name for socket in connected)}")
    queues = [
        pipeline.create(dai.node.Camera).build(socket, sensorFps=15.0)
        .requestOutput((320, 240), fps=15.0).createOutputQueue(1, False)
        for socket in sockets
    ]
    pipeline.start()
    end = time.monotonic() + args.seconds
    nextPoll = time.monotonic()
    while pipeline.isRunning() and time.monotonic() < end:
        for queue in queues:
            queue.tryGet()
        if time.monotonic() >= nextPoll:
            for current in device.getCameraStats():
                if current.socket == dai.CameraBoardSocket.AUTO:
                    print(f"cameraId={current.cameraId}: statistics unavailable; physical socket mapping is unverified", end="\n\n", flush=True)
                    continue
                if current.socket not in sockets:
                    continue
                old = previous.get(current.socket)
                # Counter presence determines validity, including HAL errors in UNAVAILABLE snapshots.
                sameSession = (
                    old is not None
                    and bool(current.sessionId)
                    and old.sessionId == current.sessionId
                    and current.timestampNs > old.timestampNs
                )
                if old is None or old.sessionId != current.sessionId:
                    print(f"{current.socket.name}: new session {current.sessionId}", flush=True)
                values = []
                for name in counters:
                    value = getattr(current, name)
                    text = f"{name}={value if value is not None else 'unavailable'}"
                    csl = name != "completedCaptures" and not name.startswith("hal")
                    if (
                        sameSession and value is not None and getattr(old, name) is not None
                        and (
                            not csl
                            or (
                                current.cslTimestampNs is not None and old.cslTimestampNs is not None
                                and current.cslTimestampNs > old.cslTimestampNs
                            )
                        )
                    ):
                        delta = (value - getattr(old, name)) & ((1 << 64) - 1)
                        text += f" (+{delta})"
                        if name == "completedCaptures":
                            text += f" {delta * 1e9 / (current.timestampNs - old.timestampNs):.1f} fps"
                    values.append(text)
                print(f"{current.socket.name} {current.status.name}: " + ", ".join(values), end="\n\n", flush=True)
                previous[current.socket] = current
            nextPoll += 1
        time.sleep(0.005)
