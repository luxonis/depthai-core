#!/usr/bin/env python3

import argparse
import datetime
import os
from collections import Counter
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


SIZE = (1280, 800)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture live RVC2 rectified frames, replay them through RVC2 and RVC4, and compare disparity bit-for-bit"
    )
    parser.add_argument("--rvc2-device", help="RVC2 device name or ID; auto-selected when exactly one is available")
    parser.add_argument(
        "--rvc4-device",
        default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"),
        help="RVC4 IP, name, or ID; auto-selected when exactly one is available",
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--alpha", type=float, default=-5.0, help="RVC2 rectification alpha scaling (default: -5)")
    parser.add_argument("--frames", type=int, default=100, help="number of frames to capture and compare")
    parser.add_argument("--disparity-64", action="store_true", help="compare the 64-disparity RVC2-compatible profiles")
    lr_check = parser.add_mutually_exclusive_group()
    lr_check.add_argument("--lr-check", dest="lr_check", action="store_true", help="enable left-right checking (default)")
    lr_check.add_argument("--no-lr-check", dest="lr_check", action="store_false", help="disable left-right checking")
    parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4), help="decimation factor for RVC2 stereo (default: 1)")
    parser.add_argument("--speckle-range", type=int, help="override the speckle component-size threshold on both devices")
    parser.add_argument(
        "--disable-filter",
        action="append",
        choices=("median", "speckle", "spatial", "temporal", "threshold"),
        default=[],
        help="disable one post-processing filter on both devices; may be repeated",
    )
    parser.add_argument("--raw", action="store_true", help="disable post-processing filters on both devices")
    parser.set_defaults(lr_check=True)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--confidence-diagnostics",
        action="store_true",
        help="connect RVC2 confidence output and report its values at mismatches",
    )
    parser.add_argument("--dump-dir", type=Path, help="save one comparison frame as NumPy arrays for offline analysis")
    parser.add_argument("--dump-sequence", type=int, help="RVC2 sequence number to save with --dump-dir")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.dump_sequence is not None and args.dump_dir is None:
        parser.error("--dump-sequence requires --dump-dir")
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


def configure_post_processing(stereo, raw, decimation, speckle_range=None, disabled_filters=()):
    config = stereo.initialConfig
    if raw:
        config.postProcessing.decimationFilter.decimationFactor = 1
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.spatialFilter.enable = False
        config.postProcessing.temporalFilter.enable = False
        config.postProcessing.holeFilling.enable = False
        config.postProcessing.adaptiveMedianFilter.enable = False
        config.postProcessing.thresholdFilter.minRange = 0
        config.postProcessing.thresholdFilter.maxRange = 65535
    if decimation is not None:
        config.postProcessing.decimationFilter.decimationFactor = decimation
    if speckle_range is not None:
        config.postProcessing.speckleFilter.speckleRange = speckle_range
    if "median" in disabled_filters:
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    if "speckle" in disabled_filters:
        config.postProcessing.speckleFilter.enable = False
    if "spatial" in disabled_filters:
        config.postProcessing.spatialFilter.enable = False
    if "temporal" in disabled_filters:
        config.postProcessing.temporalFilter.enable = False
    if "threshold" in disabled_filters:
        config.postProcessing.thresholdFilter.minRange = 0
        config.postProcessing.thresholdFilter.maxRange = 65535


def create_capture_pipeline(
    device_info, fps, disparity_64, lr_check, raw, alpha, decimation=None, speckle_range=None, disabled_filters=()
):
    pipeline = dai.Pipeline(dai.Device(device_info))
    pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)

    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    if disparity_64:
        stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    stereo.setLeftRightCheck(lr_check)
    configure_post_processing(stereo, raw, decimation, speckle_range, disabled_filters)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    # Crop invalid rectification borders which only RVC2 knows to mask in disparity.
    stereo.setAlphaScaling(alpha)
    left.requestOutput(SIZE, fps=fps).link(stereo.left)
    right.requestOutput(SIZE, fps=fps).link(stereo.right)

    return (
        pipeline,
        stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=True),
        stereo.rectifiedRight.createOutputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
    )


def create_replay_pipeline(
    device_info,
    platform,
    disparity_64,
    lr_check,
    raw,
    output_confidence=False,
    decimation=None,
    speckle_range=None,
    disabled_filters=(),
):
    pipeline = dai.Pipeline(dai.Device(device_info))
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    if platform == dai.XLinkPlatform.X_LINK_RVC4:
        backend = (
            dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT_64
            if disparity_64
            else dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT
        )
        stereo.setStereoBackend(backend)
    elif disparity_64:
        stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    stereo.setLeftRightCheck(lr_check)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)  # The RVC2 frames are already rectified.
    configure_post_processing(stereo, raw, decimation, speckle_range, disabled_filters)

    return (
        pipeline,
        stereo.left.createInputQueue(maxSize=4, blocking=True),
        stereo.right.createInputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
        stereo.confidenceMap.createOutputQueue(maxSize=4, blocking=True) if output_confidence else None,
    )


def replay_frame(image, transformation, sequence, socket):
    message = dai.ImgFrame()
    message.setData(image.reshape(-1))
    message.setTimestamp(datetime.timedelta(milliseconds=sequence * 50))
    message.setSequenceNum(sequence)
    message.setInstanceNum(socket)
    message.setType(dai.ImgFrame.Type.RAW8)
    message.setWidth(image.shape[1])
    message.setHeight(image.shape[0])
    message.setStride(image.shape[1])
    message.setTransformation(transformation)
    return message


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
    comparison = np.hstack([panel for panel, _ in panels])
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
    if args.dump_dir is not None:
        args.dump_dir.mkdir(parents=True, exist_ok=True)
    rvc2_info = select_device(dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    rvc4_info = select_device(dai.XLinkPlatform.X_LINK_RVC4, args.rvc4_device, "RVC4", pick_first=True)
    print(f"RVC2: {rvc2_info.name} ({rvc2_info.deviceId})")
    print(f"RVC4: {rvc4_info.name} ({rvc4_info.deviceId})")

    capture_pipeline, capture_left, capture_right, capture_disparity = create_capture_pipeline(
        rvc2_info,
        args.fps,
        args.disparity_64,
        args.lr_check,
        args.raw,
        args.alpha,
        args.decimation,
        args.speckle_range,
        args.disable_filter,
    )
    captured = []
    print(f"Capturing {args.frames} synchronized rectified pairs on RVC2...")
    with capture_pipeline:
        capture_pipeline.start()
        for index in range(args.frames):
            disparity_message = capture_disparity.get()
            left = capture_left.get()
            right = capture_right.get()
            sequence = disparity_message.getSequenceNum()
            if sequence != left.getSequenceNum() or sequence != right.getSequenceNum():
                raise RuntimeError("RVC2 rectified and disparity streams are out of sync")
            captured.append(
                (
                    left.getFrame().copy(),
                    right.getFrame().copy(),
                    left.getTransformation(),
                    right.getTransformation(),
                )
            )
            print(f"captured sequence {sequence} ({index + 1}/{args.frames})")

    left_transformation = captured[0][2]
    right_transformation = captured[0][3]
    intrinsics = np.asarray(left_transformation.getIntrinsicMatrix())
    print(f"RVC2 rectified focal length: {intrinsics[0, 0]:.6f} px")
    try:
        extrinsics = np.asarray(
            left_transformation.getExtrinsicsTransformationMatrixTo(
                right_transformation, False, dai.LengthUnit.MILLIMETER
            )
        )
        baseline = float(np.linalg.norm(extrinsics[:3, 3]))
        print(f"RVC2 stereo baseline: {baseline:.6f} mm")
    except RuntimeError as error:
        print(f"RVC2 stereo baseline unavailable from frame metadata: {error}")

    rvc2_pipeline, rvc2_left, rvc2_right, rvc2_disparity, rvc2_confidence = create_replay_pipeline(
        rvc2_info,
        dai.XLinkPlatform.X_LINK_MYRIAD_X,
        args.disparity_64,
        args.lr_check,
        args.raw,
        args.confidence_diagnostics,
        args.decimation,
        args.speckle_range,
        args.disable_filter,
    )
    rvc4_pipeline, rvc4_left, rvc4_right, rvc4_disparity, _ = create_replay_pipeline(
        rvc4_info,
        dai.XLinkPlatform.X_LINK_RVC4,
        args.disparity_64,
        args.lr_check,
        args.raw,
        False,
        args.decimation,
        args.speckle_range,
        args.disable_filter,
    )
    with rvc2_pipeline, rvc4_pipeline:
        rvc4_pipeline.start()
        rvc2_pipeline.start()

        frame_count = 0
        total_pixels = 0
        total_mismatches = 0
        difference_histogram = Counter()
        mismatch_coordinates = Counter()
        rvc2_zero_mismatches = 0
        rvc4_zero_mismatches = 0
        nonzero_mismatches = 0
        mismatch_confidence = Counter()
        for sequence, (left_image, right_image, left_transformation, right_transformation) in enumerate(captured):
            rvc2_left.send(replay_frame(left_image, left_transformation, sequence, dai.CameraBoardSocket.CAM_B))
            rvc2_right.send(replay_frame(right_image, right_transformation, sequence, dai.CameraBoardSocket.CAM_C))
            rvc4_left.send(replay_frame(left_image, left_transformation, sequence, dai.CameraBoardSocket.CAM_B))
            rvc4_right.send(replay_frame(right_image, right_transformation, sequence, dai.CameraBoardSocket.CAM_C))
            rvc2_disparity_message = rvc2_disparity.get()
            rvc2_confidence_message = rvc2_confidence.get() if rvc2_confidence is not None else None
            rvc4_disparity_message = rvc4_disparity.get()
            if (
                rvc2_disparity_message.getSequenceNum() != sequence
                or (rvc2_confidence_message is not None and rvc2_confidence_message.getSequenceNum() != sequence)
                or rvc4_disparity_message.getSequenceNum() != sequence
            ):
                raise RuntimeError("Replay disparity streams are out of sync")

            rvc2_frame = rvc2_disparity_message.getFrame()
            rvc4_frame = rvc4_disparity_message.getFrame()
            confidence_frame = rvc2_confidence_message.getFrame() if rvc2_confidence_message is not None else None
            difference = rvc4_frame.astype(np.int32) - rvc2_frame.astype(np.int32)
            mismatches = int(np.count_nonzero(difference))
            mismatch_mask = difference != 0
            values, counts = np.unique(difference[mismatch_mask], return_counts=True)
            difference_histogram.update(dict(zip(values.tolist(), counts.tolist())))
            mismatch_y, mismatch_x = np.nonzero(mismatch_mask)
            mismatch_coordinates.update(zip(mismatch_x.tolist(), mismatch_y.tolist()))
            if confidence_frame is not None:
                mismatch_confidence.update(confidence_frame[mismatch_mask].tolist())
            rvc2_zero_mismatches += int(np.count_nonzero((rvc2_frame == 0) & (rvc4_frame != 0)))
            rvc4_zero_mismatches += int(np.count_nonzero((rvc2_frame != 0) & (rvc4_frame == 0)))
            nonzero_mismatches += int(np.count_nonzero((rvc2_frame != 0) & (rvc4_frame != 0) & mismatch_mask))
            total_mismatches += mismatches
            total_pixels += difference.size
            frame_count += 1
            exact_percent = (difference.size - mismatches) * 100 / difference.size
            print(f"sequence {sequence}: {exact_percent:.8f}% exact, {mismatches} mismatches")

            should_dump = args.dump_dir is not None and (
                args.dump_sequence == sequence or (args.dump_sequence is None and mismatches and not any(args.dump_dir.iterdir()))
            )
            if should_dump:
                prefix = args.dump_dir / f"sequence_{sequence}"
                np.save(f"{prefix}_left.npy", left_image)
                np.save(f"{prefix}_right.npy", right_image)
                np.save(f"{prefix}_rvc2.npy", rvc2_frame)
                np.save(f"{prefix}_rvc4.npy", rvc4_frame)
                np.save(f"{prefix}_difference.npy", difference)
                print(f"Saved sequence {sequence} comparison to {args.dump_dir}")

            if not args.no_display:
                show_comparison(rvc2_frame, rvc4_frame, difference, exact_percent, args.disparity_64, args.lr_check)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

    cv2.destroyAllWindows()
    if total_pixels == 0:
        raise RuntimeError("No frames were compared")
    exact_percent = (total_pixels - total_mismatches) * 100 / total_pixels
    print(f"Compared {frame_count} frames: {exact_percent:.8f}% exact, {total_mismatches} mismatches")
    print(
        "Mismatch classes: "
        f"RVC2 zero={rvc2_zero_mismatches}, RVC4 zero={rvc4_zero_mismatches}, both nonzero={nonzero_mismatches}"
    )
    print(f"Most common signed differences: {difference_histogram.most_common(16)}")
    print(f"Most common mismatch coordinates: {mismatch_coordinates.most_common(16)}")
    if args.confidence_diagnostics:
        print(f"RVC2 confidence at mismatches: {mismatch_confidence.most_common(16)}")


if __name__ == "__main__":
    main()
