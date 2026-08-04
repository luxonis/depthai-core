#!/usr/bin/env python3
"""Calibrate the physical three-device CAM_B/C rig and print the result."""
import argparse
import math
import time
from contextlib import ExitStack
from datetime import timedelta
from pathlib import Path

import depthai as dai


DEFAULT_DEVICES = ("10.11.0.128", "10.11.0.48", "10.11.103.135")
STEREO_SOCKETS = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--device",
        action="append",
        help="Device IP or MX ID. Repeat in physical rig order. Defaults to the checked lab IPs.",
    )
    parser.add_argument("--rig", type=Path, default=Path(".codex-tmp/multidevice_calibration/rig_feature_tracks.json"))
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 800))
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--warmup-frames", type=int, default=15, help="Drop this many startup frames before calibration.")
    parser.add_argument("--settle", type=float, default=1.0, help="Minimum warm-up seconds after the camera pipeline starts.")
    return parser.parse_args()


def socket_name(socket):
    return getattr(socket, "name", str(socket).split(".")[-1])


def ensure_feature_tracks_available():
    method_enum = getattr(dai.node.MultiDeviceCalibration, "Method", None)
    if method_enum is None or not hasattr(method_enum, "FEATURE_TRACKS"):
        raise SystemExit("The loaded depthai module does not expose MultiDeviceCalibration.Method.FEATURE_TRACKS")


def require_cam_bc(devices):
    required = set(STEREO_SOCKETS)
    for device in devices:
        missing = required - set(device.getConnectedCameras())
        if missing:
            missing_names = ", ".join(socket_name(socket) for socket in sorted(missing, key=socket_name))
            raise SystemExit(f"Device {device.getDeviceId()} is missing {missing_names}")


def transform_distance_cm(transform):
    return math.sqrt(sum(transform[row][3] * transform[row][3] for row in range(3)))


def print_camera_center_distances(handler, frames):
    if len(frames) < 2:
        return

    print("CAM_B center distances:", flush=True)
    for index, frame_a in enumerate(frames):
        for frame_b in frames[index + 1 :]:
            transform = handler.getTransform(frame_a, frame_b, dai.LengthUnit.CENTIMETER)
            print(f"  {frame_a} <-> {frame_b}: {transform_distance_cm(transform):.1f} cm", flush=True)


def missing_connected_frames(handler, frames):
    if not frames:
        return []
    base = frames[0]
    return [frame for frame in frames[1:] if not handler.canTransform(frame, base)]


def print_result(result, rig_path, expected_frames):
    print(f"passed={result.passed} confidence={result.dataConfidence:.3f} info={result.info!r}", flush=True)
    if not result.passed:
        return False

    handler = dai.MultiDeviceCalibrationHandler(result.calibration)
    for edge in result.calibration.edges:
        transform = handler.getTransform(edge.from_, edge.to, dai.LengthUnit.CENTIMETER)
        print(f"{edge.from_} -> {edge.to} (cm):", flush=True)
        for row in transform:
            print("  " + " ".join(f"{value:9.3f}" for value in row), flush=True)

    missing = missing_connected_frames(handler, expected_frames)
    if missing:
        print(
            "Calibration did not connect all requested CAM_B frames: "
            + ", ".join(str(frame) for frame in missing),
            flush=True,
        )
        return False

    print_camera_center_distances(handler, expected_frames)

    rig_path.parent.mkdir(parents=True, exist_ok=True)
    handler.toJsonFile(str(rig_path))
    print(f"Rig written to {rig_path}", flush=True)
    return True


def warmup_seconds(args):
    if args.fps <= 0.0:
        return args.settle
    return max(args.settle, max(0, args.warmup_frames) / args.fps)


def build_calibration_sources(pipeline, devices, args):
    sources = []
    gates = []
    for device in devices:
        for socket in STEREO_SOCKETS:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            output = camera.requestOutput(tuple(args.resolution), fps=args.fps)
            gate = pipeline.create(dai.node.Gate, device)
            gate.initialConfig = dai.GateControl.closeGate()
            output.link(gate.input)
            sources.append((dai.CoordinateFrame(device.getDeviceId(), socket), gate.output))
            gates.append(gate)
    return sources, gates


def main():
    args = parse_args()
    ensure_feature_tracks_available()
    device_names = args.device or list(DEFAULT_DEVICES)

    expected_frames = []
    with ExitStack() as stack:
        devices = [stack.enter_context(dai.Device(dai.DeviceInfo(name))) for name in device_names]
        require_cam_bc(devices)
        expected_frames = [dai.CoordinateFrame(device.getDeviceId(), dai.CameraBoardSocket.CAM_B) for device in devices]

        print("Connected devices:", flush=True)
        for name, device in zip(device_names, devices):
            connected = ", ".join(socket_name(socket) for socket in device.getConnectedCameras())
            print(f"  {name} -> {device.getDeviceId()} ({connected})", flush=True)

        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            sources, gates = build_calibration_sources(pipeline, devices, args)
            calibration = pipeline.create(dai.node.MultiDeviceCalibration).build(sources)
            calibration.setMethod(dai.node.MultiDeviceCalibration.Method.FEATURE_TRACKS)
            calibration.setSampleCount(args.samples)
            calibration.sync.setSyncThreshold(timedelta(seconds=max(0.5, 3.0 / args.fps)))
            rig_queue = calibration.rigCalibration.createOutputQueue()
            gate_queues = [gate.inputControl.createInputQueue() for gate in gates]

            pipeline.start()
            seconds = warmup_seconds(args)
            print(f"Warming up calibration: dropping startup frames for {seconds:.1f}s.", flush=True)
            if seconds > 0.0:
                time.sleep(seconds)
            for queue in gate_queues:
                queue.send(dai.GateControl.openGate())
            print(f"Calibrating from CAM_B/C using {args.samples} synchronized samples.", flush=True)
            result = rig_queue.get()

    if not print_result(result, args.rig, expected_frames):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
