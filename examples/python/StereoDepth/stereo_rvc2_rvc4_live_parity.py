#!/usr/bin/env python3

import argparse
import os

import cv2
import depthai as dai
import numpy as np


SIZE = (1280, 800)


def parse_args():
    parser = argparse.ArgumentParser(description="Compare live RVC2 stereo disparity with the RVC2-compatible RVC4 backend")
    parser.add_argument("--rvc2-device", help="RVC2 device name or ID; auto-selected when exactly one is available")
    parser.add_argument(
        "--rvc4-device",
        default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"),
        help="RVC4 IP, name, or ID; auto-selected when exactly one is available",
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--frames", type=int, default=100, help="number of frames to compare; zero runs until q")
    parser.add_argument("--disparity-64", action="store_true", help="compare the 64-disparity RVC2-compatible profiles")
    lr_check = parser.add_mutually_exclusive_group()
    lr_check.add_argument("--lr-check", dest="lr_check", action="store_true", help="enable left-right checking (default)")
    lr_check.add_argument("--no-lr-check", dest="lr_check", action="store_false", help="disable left-right checking")
    parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4), help="decimation factor for RVC2 stereo (default: 1)")
    parser.set_defaults(lr_check=True)
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.frames < 0:
        parser.error("--frames cannot be negative")
    return args


def select_device(platform, requested, label, pick_first=False):
    devices = [info for info in dai.Device.getAllAvailableDevices() if info.platform == platform]
    if requested:
        devices = [info for info in devices if requested in (info.name, info.deviceId, info.getDeviceId())]
    elif pick_first:
        devices.sort(key=lambda info: (info.state == dai.XLinkDeviceState.X_LINK_GATE_SETUP, info.name or "", info.deviceId))
    if not devices or (len(devices) != 1 and not pick_first):
        available = ", ".join(f"{info.name} ({info.deviceId})" for info in devices) or "none"
        raise RuntimeError(f"Expected exactly one {label} device, found: {available}")
    return devices[0]


def create_rvc2_pipeline(device_info, fps, disparity_64, lr_check, decimation=None):
    pipeline = dai.Pipeline(dai.Device(device_info))
    pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)

    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    if disparity_64:
        stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    stereo.setLeftRightCheck(lr_check)
    if decimation is not None:
        stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = decimation
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    # Crop invalid rectification borders which only RVC2 knows to mask in disparity.
    stereo.setAlphaScaling(-5)
    left.requestOutput(SIZE, fps=fps).link(stereo.left)
    right.requestOutput(SIZE, fps=fps).link(stereo.right)

    return (
        pipeline,
        stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=True),
        stereo.rectifiedRight.createOutputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
    )


def create_rvc4_pipeline(device_info, disparity_64, lr_check, decimation=None):
    pipeline = dai.Pipeline(dai.Device(device_info))
    stereo = pipeline.create(dai.node.StereoDepth)
    backend = (
        dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT_64
        if disparity_64
        else dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT
    )
    stereo.setStereoBackend(backend)
    stereo.setLeftRightCheck(lr_check)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)  # The RVC2 frames are already rectified.
    if decimation is not None:
        stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = decimation


    return (
        pipeline,
        stereo.left.createInputQueue(maxSize=4, blocking=True),
        stereo.right.createInputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
    )


def colorize(disparity, maximum):
    normalized = np.clip(disparity.astype(np.float32) * (255.0 / maximum), 0, 255).astype(np.uint8)
    result = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    result[disparity == 0] = 0
    return result


def show_comparison(rvc2, rvc4, difference, exact_percent, disparity_64, lr_check):
    maximum = max(int(rvc2.max()), int(rvc4.max()), 1)
    absolute_difference = np.abs(difference)
    profile = f"{'64' if disparity_64 else '96'} disparities, LR {'on' if lr_check else 'off'}"
    panels = [
        (colorize(rvc2, maximum), f"RVC2 DEFAULT ({profile})"),
        (colorize(rvc4, maximum), f"RVC4 DSP_RVC2_DEFAULT ({profile})"),
        (colorize(absolute_difference, max(int(absolute_difference.max()), 1)), "absolute difference"),
    ]
    for panel, title in panels:
        cv2.putText(panel, title, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    comparison = np.hstack([cv2.resize(panel, (1280, 800), interpolation=cv2.INTER_AREA) for panel, _ in panels])
    cv2.putText(
        comparison,
        f"exact match: {exact_percent:.8f}%",
        (12, comparison.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("RVC2 / RVC4 stereo parity", comparison)


def main():
    args = parse_args()
    rvc2_info = select_device(dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    rvc4_info = select_device(dai.XLinkPlatform.X_LINK_RVC4, args.rvc4_device, "RVC4", pick_first=True)
    print(f"RVC2: {rvc2_info.name} ({rvc2_info.deviceId})")
    print(f"RVC4: {rvc4_info.name} ({rvc4_info.deviceId})")

    rvc2_pipeline, rvc2_left, rvc2_right, rvc2_disparity = create_rvc2_pipeline(rvc2_info, args.fps, args.disparity_64, args.lr_check, args.decimation)
    rvc4_pipeline, rvc4_left, rvc4_right, rvc4_disparity = create_rvc4_pipeline(rvc4_info, args.disparity_64, args.lr_check, args.decimation)
    with rvc2_pipeline, rvc4_pipeline:
        rvc4_pipeline.start()
        rvc2_pipeline.start()

        frame_count = 0
        total_pixels = 0
        total_mismatches = 0
        while rvc2_pipeline.isRunning() and rvc4_pipeline.isRunning():
            rvc2_disparity_message = rvc2_disparity.get()
            left = rvc2_left.get()
            right = rvc2_right.get()
            sequence = rvc2_disparity_message.getSequenceNum()
            if sequence != left.getSequenceNum() or sequence != right.getSequenceNum():
                raise RuntimeError("RVC2 rectified and disparity streams are out of sync")

            # Forward the exact frames used by RVC2 stereo into the RVC4 pipeline.
            rvc4_left.send(left)
            rvc4_right.send(right)
            rvc4_disparity_message = rvc4_disparity.get()
            if rvc4_disparity_message.getSequenceNum() != sequence:
                raise RuntimeError("RVC4 disparity stream is out of sync")

            rvc2_frame = rvc2_disparity_message.getFrame()
            rvc4_frame = rvc4_disparity_message.getFrame()
            difference = rvc4_frame.astype(np.int32) - rvc2_frame.astype(np.int32)
            mismatches = int(np.count_nonzero(difference))
            total_mismatches += mismatches
            total_pixels += difference.size
            frame_count += 1
            exact_percent = (difference.size - mismatches) * 100 / difference.size
            print(f"sequence {sequence}: {exact_percent:.8f}% exact, {mismatches} mismatches")

            if not args.no_display:
                show_comparison(rvc2_frame, rvc4_frame, difference, exact_percent, args.disparity_64, args.lr_check)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if args.frames and frame_count >= args.frames:
                break

    cv2.destroyAllWindows()
    if total_pixels == 0:
        raise RuntimeError("No frames were compared")
    exact_percent = (total_pixels - total_mismatches) * 100 / total_pixels
    print(f"Compared {frame_count} frames: {exact_percent:.8f}% exact, {total_mismatches} mismatches")


if __name__ == "__main__":
    main()
