#!/usr/bin/env python3
"""Load one recorded stereo pair and save its depth as an ImgFrame .dai file."""

import argparse
from datetime import timedelta
from pathlib import Path

import depthai as dai


def apply_calibration(frame, calibration, socket, origin, device_id):
    transformation = frame.getTransformation()
    source_width, source_height = transformation.getSourceSize()
    if not source_width or not source_height:
        source_width, source_height = frame.getWidth(), frame.getHeight()
        transformation = dai.ImgTransformation(source_width, source_height)
    # Keep the recorded crop/resize, updating the source camera calibration.
    transformation.setIntrinsicMatrix(calibration.getCameraIntrinsics(socket, source_width, source_height))
    transformation.setDistortionModel(calibration.getDistortionModel(socket))
    transformation.setDistortionCoefficients(calibration.getDistortionCoefficients(socket))
    pose = dai.Extrinsics(calibration.getCameraExtrinsics(socket, origin), origin)
    pose.toDeviceId = device_id
    transformation.setExtrinsics(pose)
    frame.setTransformation(transformation)
    frame.setInstanceNum(int(socket))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path, help="Left ImgFrame .dai file")
    parser.add_argument("right", type=Path, help="Right ImgFrame .dai file")
    parser.add_argument("calibration", type=Path, help="CalibrationHandler JSON file")
    parser.add_argument("output", type=Path, nargs="?", default=Path("depth.dai"))
    parser.add_argument("--device", help="Device ID or IP address; otherwise select an available device")
    parser.add_argument("--backend", choices=["AUTO", "HW", "GPU", "GPU_CPU"], default="AUTO")
    parser.add_argument("--left-socket", choices=dai.CameraBoardSocket.__members__, default="CAM_B")
    parser.add_argument("--right-socket", choices=dai.CameraBoardSocket.__members__, default="CAM_C")
    parser.add_argument("--timeout", type=float, default=30, help="Depth receive timeout in seconds")
    args = parser.parse_args()

    left, right = dai.ImgFrame(), dai.ImgFrame()
    left.load(args.left)
    right.load(args.right)
    if (left.getWidth(), left.getHeight()) != (right.getWidth(), right.getHeight()):
        parser.error("Left and right frame dimensions must match")
    if args.left_socket == args.right_socket:
        parser.error("Left and right camera sockets must differ")
    if args.timeout <= 0:
        parser.error("Timeout must be positive")
    calibration = dai.CalibrationHandler(str(args.calibration))
    left_socket = dai.CameraBoardSocket.__members__[args.left_socket]
    right_socket = dai.CameraBoardSocket.__members__[args.right_socket]
    origin = left.getTransformation().getExtrinsics().toCameraSocket
    if origin == dai.CameraBoardSocket.AUTO:
        origin = left_socket

    device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
    with device, dai.Pipeline(device) as pipeline:
        pipeline.setCalibrationData(calibration)
        for frame, socket in ((left, left_socket), (right, right_socket)):
            apply_calibration(frame, calibration, socket, origin, device.getDeviceId())
            # Replay this pair together regardless of its original capture times.
            frame.setTimestampDevice(left.getTimestampDevice())
            frame.setTimestamp(left.getTimestamp())
            frame.setSequenceNum(left.getSequenceNum())

        stereo = pipeline.create(dai.node.StereoDepth)
        if hasattr(stereo, "setBackend"):
            stereo.setBackend(dai.node.StereoDepth.Backend.__members__[args.backend])
        elif args.backend not in ("AUTO", "HW"):
            parser.error("This SDK does not support selecting a GPU stereo backend")
        stereo.setRectification(True)
        stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        left_queue = stereo.left.createInputQueue()
        right_queue = stereo.right.createInputQueue()
        depth_queue = stereo.depth.createOutputQueue()
        pipeline.start()
        left_queue.send(left)
        right_queue.send(right)
        depth = depth_queue.get(timeout=timedelta(seconds=args.timeout))
        if depth is None:
            raise TimeoutError("Timed out waiting for StereoDepth output")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        depth.save(args.output)
        print(f"Saved {depth.getWidth()}x{depth.getHeight()} {depth.getType()} depth to {args.output}")


if __name__ == "__main__":
    main()
