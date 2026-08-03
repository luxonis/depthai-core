#!/usr/bin/env python3
"""Calibrate and stitch the physical three-device CAM_B/C setup.

This is a higher-level version of the small multi-device examples:
  1. calibrate the rig from CAM_B and CAM_C on every device,
  2. save the rig json,
  3. load that rig and run calibrated planar stitching.

The default devices are the current lab setup:
  10.11.1.218, 10.11.0.48, 10.11.103.135

On poc/multi-device-stitching, use --method dcl and provide one --guess per
device after the first. On the smart calibration branch, --method
feature-tracks uses the CAM_B/C stereo pairs directly and does not need
guesses.
"""
import argparse
import math
import time
from contextlib import ExitStack
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai


DEFAULT_DEVICES = ("10.11.1.218", "10.11.0.48", "10.11.103.135")
STEREO_SOCKETS = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
ALLOWED_SOCKET_NAMES = ("CAM_B", "CAM_C")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--device",
        action="append",
        help="Device IP or MX ID. Repeat in physical rig order. Defaults to the three checked lab IPs.",
    )
    parser.add_argument("--mode", choices=("all", "calibrate", "stitch"), default="all")
    parser.add_argument("--method", choices=("dcl", "feature-tracks"), default="dcl")
    parser.add_argument("-o", "--output-dir", type=Path, default=Path(".codex-tmp/multidevice_physical"))
    parser.add_argument("--rig", type=Path, help="Rig json to write/read. Defaults to <output-dir>/rig_cam_bc.json")
    parser.add_argument("--stitched-output", type=Path, help="Save the first stitched frame here and exit")
    parser.add_argument("--headless", action="store_true", help="Do not open OpenCV windows; save one stitched frame and exit")
    parser.add_argument("--samples", type=int, default=20, help="Synchronized image sets used for calibration")
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 800), help="Camera output resolution")
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument(
        "-g",
        "--guess",
        action="append",
        nargs=4,
        type=float,
        metavar=("YAW", "X", "Y", "Z"),
        help="DCL initial pose of each non-reference device w.r.t. the first CAM_B: yaw degrees, x/y/z cm.",
    )
    parser.add_argument(
        "--performance-mode",
        choices=[mode.name for mode in dai.DynamicCalibrationControl.PerformanceMode.__members__.values()],
        help="DCL performance mode. If omitted, the node default is used; on the smart branch this preserves auto-strategy.",
    )
    parser.add_argument(
        "--no-estimate-scale",
        action="store_true",
        help="Disable scene-based inter-device scale recovery when the API is available.",
    )
    parser.add_argument(
        "--disable-auto-strategy",
        action="store_true",
        help="Disable smart branch DCL auto-strategy when the API is available.",
    )
    parser.add_argument(
        "--yaw-sweep",
        type=float,
        nargs="+",
        help="Smart branch DCL yaw offsets, in degrees, tried around each initial guess.",
    )
    parser.add_argument(
        "--stitch-socket",
        action="append",
        choices=ALLOWED_SOCKET_NAMES,
        help="Socket to feed into planar stitching. Repeat for CAM_B and CAM_C. Defaults to CAM_B.",
    )
    parser.add_argument("--plane-point", type=float, nargs=3, default=(0.0, 150.0, 0.0), help="Point on the plane, cm in reference CAM_B frame")
    parser.add_argument("--plane-normal", type=float, nargs=3, default=(0.0, -1.0, 0.0), help="Plane normal in reference CAM_B frame")
    parser.add_argument("--range", type=float, default=600.0, help="Maximum rendered range from cameras, in cm")
    parser.add_argument("--view-size", type=int, nargs=2, default=(1400, 1400), help="Maximum auto-computed output view size")
    return parser.parse_args()


def socket_name(socket):
    return getattr(socket, "name", str(socket).split(".")[-1])


def require_cam_bc(devices):
    missing = []
    required = set(STEREO_SOCKETS)
    for device in devices:
        connected = set(device.getConnectedCameras())
        absent = required - connected
        if absent:
            missing.append(f"{device.getDeviceId()}: {', '.join(socket_name(socket) for socket in sorted(absent, key=socket_name))}")
    if missing:
        raise SystemExit("CAM_B/C are required on every device; missing " + "; ".join(missing))


def pose_from_yaw(yaw_degrees, x, y, z):
    yaw = math.radians(yaw_degrees)
    extrinsics = dai.Extrinsics()
    extrinsics.setTransformationMatrix(
        [
            [math.cos(yaw), 0.0, math.sin(yaw), x],
            [0.0, 1.0, 0.0, y],
            [-math.sin(yaw), 0.0, math.cos(yaw), z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dai.LengthUnit.CENTIMETER,
    )
    return extrinsics


def ensure_feature_tracks_available():
    method_enum = getattr(dai.node.MultiDeviceCalibration, "Method", None)
    if method_enum is None or not hasattr(method_enum, "FEATURE_TRACKS"):
        raise SystemExit(
            "--method feature-tracks needs the smart calibration branch rebuilt into the Python bindings; "
            "the loaded depthai module does not expose MultiDeviceCalibration.Method.FEATURE_TRACKS."
        )


def build_camera_outputs(pipeline, devices, sockets, resolution, fps):
    sources = []
    outputs = []
    for device in devices:
        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            output = camera.requestOutput(tuple(resolution), fps=fps)
            frame = dai.CoordinateFrame(device.getDeviceId(), socket)
            sources.append((frame, output))
            outputs.append(output)
    return sources, outputs


def run_calibration(devices, args, rig_path):
    if args.method == "dcl":
        guesses = args.guess or []
        if len(guesses) != len(devices) - 1:
            raise SystemExit(
                f"--method dcl needs {len(devices) - 1} --guess values, one per device after the first. "
                "Use --method feature-tracks on the smart branch to calibrate without guesses."
            )
    else:
        ensure_feature_tracks_available()

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        sources, outputs = build_camera_outputs(pipeline, devices, STEREO_SOCKETS, args.resolution, args.fps)
        calibration = pipeline.create(dai.node.MultiDeviceCalibration).build(sources)
        calibration.setSampleCount(args.samples)

        if args.method == "feature-tracks":
            calibration.setMethod(dai.node.MultiDeviceCalibration.Method.FEATURE_TRACKS)
        else:
            reference = dai.CoordinateFrame(devices[0].getDeviceId(), dai.CameraBoardSocket.CAM_B)
            for device, guess in zip(devices[1:], args.guess or []):
                source = dai.CoordinateFrame(device.getDeviceId(), dai.CameraBoardSocket.CAM_B)
                calibration.setInitialGuess(source, reference, pose_from_yaw(*guess))

        if args.performance_mode is not None:
            calibration.setPerformanceMode(dai.DynamicCalibrationControl.PerformanceMode.__members__[args.performance_mode])
        if args.no_estimate_scale and hasattr(calibration, "setEstimateInterDeviceScale"):
            calibration.setEstimateInterDeviceScale(False)
        if args.disable_auto_strategy and hasattr(calibration, "setAutoStrategy"):
            calibration.setAutoStrategy(False)
        if args.yaw_sweep is not None:
            if not hasattr(calibration, "setGuessYawSweep"):
                raise SystemExit("--yaw-sweep needs the smart calibration branch API")
            calibration.setGuessYawSweep(args.yaw_sweep)

        calibration.sync.setSyncThreshold(timedelta(seconds=max(0.5, 3.0 / args.fps)))
        rig_queue = calibration.rigCalibration.createOutputQueue()
        preview_queues = [] if args.headless else [output.createOutputQueue(maxSize=1, blocking=False) for output in outputs]

        pipeline.start()
        print(
            f"Calibrating {len(devices)} devices from CAM_B/C using {args.method}, "
            f"{args.samples} synchronized sets. Keep the shared scene static and textured.",
            flush=True,
        )

        while pipeline.isRunning():
            if preview_queues:
                preview = []
                for queue in preview_queues:
                    message = queue.tryGet()
                    if message is None:
                        continue
                    image = message.getCvFrame()
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
                    preview.append(cv2.resize(image, (320, 200)))
                if preview:
                    cv2.imshow("CAM_B/C calibration inputs", cv2.hconcat(preview))

            result = rig_queue.tryGet()
            if result is not None:
                print(f"passed={result.passed} confidence={result.dataConfidence:.3f} info={result.info!r}", flush=True)
                if not result.passed:
                    return False

                handler = dai.MultiDeviceCalibrationHandler(result.calibration)
                for edge in result.calibration.edges:
                    translation = [round(row[3], 1) for row in handler.getTransform(edge.from_, edge.to)[:3]]
                    print(f"  {edge.from_} -> {edge.to}: {translation} cm", flush=True)
                handler.toJsonFile(str(rig_path))
                print(f"Rig written to {rig_path}", flush=True)
                return True

            if not args.headless and cv2.waitKey(1) & 0xFF == ord("q"):
                return False

        return False


def run_stitching(devices, args, rig_path):
    if not rig_path.exists():
        raise SystemExit(f"Rig file does not exist: {rig_path}")

    stitch_socket_names = args.stitch_socket or ["CAM_B"]
    stitch_sockets = [dai.CameraBoardSocket.__members__[name] for name in stitch_socket_names]
    output_path = args.stitched_output
    if output_path is None and args.headless:
        output_path = args.output_dir / "planar_stitch_cam_bc.png"

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        pipeline.setMultiDeviceCalibration(dai.MultiDeviceCalibrationHandler(str(rig_path)))
        reference = dai.CoordinateFrame(devices[0].getDeviceId(), stitch_sockets[0])
        _, outputs = build_camera_outputs(pipeline, devices, stitch_sockets, args.resolution, args.fps)

        unified = pipeline.create(dai.node.CoordinateFrameTransform).build(outputs, reference)
        stitching_inputs = [unified.outputs[f"output{index}"] for index in range(len(outputs))]
        stitching = pipeline.create(dai.node.Stitching).build(stitching_inputs)
        stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
        stitching.setPlane(dai.Point3f(*args.plane_point), dai.Point3f(*args.plane_normal))
        stitching.setMaxRange(args.range)
        stitching.setMaxViewSize(*args.view_size)
        stitching.setSyncThreshold(timedelta(seconds=max(0.5, 3.0 / args.fps)))

        stitched_queue = stitching.out.createOutputQueue()
        pipeline.start()
        print(f"Planar stitching {len(outputs)} CAM_B/C streams with rig {rig_path}", flush=True)

        while pipeline.isRunning():
            stitched = stitched_queue.get().getCvFrame()
            if output_path is not None:
                output_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(output_path), stitched)
                print(f"Saved stitched frame {stitched.shape[1]}x{stitched.shape[0]} to {output_path}", flush=True)
                return True

            cv2.imshow("CAM_B/C planar stitch", stitched)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return True

        return False


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rig_path = args.rig or (args.output_dir / "rig_cam_bc.json")

    device_names = args.device or list(DEFAULT_DEVICES)
    with ExitStack() as stack:
        devices = [stack.enter_context(dai.Device(dai.DeviceInfo(name))) for name in device_names]
        print("Connected devices:", flush=True)
        for name, device in zip(device_names, devices):
            connected = ", ".join(socket_name(socket) for socket in device.getConnectedCameras())
            print(f"  {name} -> {device.getDeviceId()} ({connected})", flush=True)
        require_cam_bc(devices)

        ok = True
        if args.mode in ("all", "calibrate"):
            ok = run_calibration(devices, args, rig_path)
        if ok and args.mode in ("all", "stitch"):
            ok = run_stitching(devices, args, rig_path)

    cv2.destroyAllWindows()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
