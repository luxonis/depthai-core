#!/usr/bin/env python3

"""Replay rectified stereo pairs through every RVC2 preset on RVC2 and RVC4."""

import argparse
import datetime
from pathlib import Path

import depthai as dai
import numpy as np


SIZE = (1280, 800)
PRESETS = (
    "FAST_ACCURACY",
    "FAST_DENSITY",
    "DEFAULT",
    "FACE",
    "HIGH_DETAIL",
    "ROBOTICS",
    "DENSITY",
    "ACCURACY",
)


def select_device(platform, requested):
    devices = [device for device in dai.Device.getAllAvailableDevices() if device.platform == platform]
    devices = [
        device
        for device in devices
        if requested is None or requested in (device.name, device.deviceId, device.getDeviceId())
    ]
    if len(devices) != 1:
        raise RuntimeError(f"Expected one {platform.name} device matching {requested!r}, found {devices}")
    return devices[0]


def load_pairs(directory, count):
    def sequence(path):
        return int(path.stem.split("_")[1])

    pairs = []
    for left_path in sorted(directory.glob("sequence_*_left.npy"), key=sequence)[:count]:
        number = sequence(left_path)
        right_path = directory / f"sequence_{number}_right.npy"
        if not right_path.exists():
            raise RuntimeError(f"Missing replay input {right_path}")
        pairs.append((np.load(left_path), np.load(right_path)))
    if not pairs:
        raise RuntimeError(f"No sequence_*_left/right.npy pairs found in {directory}")
    return pairs


def frame_message(image, sequence, socket):
    message = dai.ImgFrame()
    message.setData(image.reshape(-1))
    message.setTimestamp(datetime.timedelta(milliseconds=sequence * 50))
    message.setSequenceNum(sequence)
    message.setInstanceNum(socket)
    message.setType(dai.ImgFrame.Type.RAW8)
    message.setWidth(image.shape[1])
    message.setHeight(image.shape[0])
    message.setStride(image.shape[1])
    return message


def configure_filters(stereo, decimation, raw, disabled_filters):
    config = stereo.initialConfig
    config.postProcessing.decimationFilter.decimationFactor = decimation
    if raw:
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.spatialFilter.enable = False
        config.postProcessing.temporalFilter.enable = False
        config.postProcessing.thresholdFilter.minRange = 0
        config.postProcessing.thresholdFilter.maxRange = 65535
    else:
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


def create_pipeline(device_info, preset_name, decimation, raw, disabled_filters):
    pipeline = dai.Pipeline(dai.Device(device_info))
    stereo = pipeline.create(dai.node.StereoDepth)
    preset = getattr(dai.node.StereoDepth.PresetMode, preset_name)
    if device_info.platform == dai.XLinkPlatform.X_LINK_RVC4:
        stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT)
    stereo.setDefaultProfilePreset(preset)
    configure_filters(stereo, decimation, raw, disabled_filters)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)
    return (
        pipeline,
        stereo.left.createInputQueue(maxSize=4, blocking=True),
        stereo.right.createInputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
    )


def compare_profile(rvc2, rvc4, pairs, preset, decimation, raw, disabled_filters, save_dir):
    p2, left2, right2, output2 = create_pipeline(rvc2, preset, decimation, raw, disabled_filters)
    p4, left4, right4, output4 = create_pipeline(rvc4, preset, decimation, raw, disabled_filters)
    pixels = mismatches = maximum_difference = 0
    reference_range = [65535, 0]
    candidate_range = [65535, 0]
    with p2, p4:
        p4.start()
        p2.start()
        for sequence, (left, right) in enumerate(pairs):
            for queue, image, socket in (
                (left2, left, dai.CameraBoardSocket.CAM_B),
                (right2, right, dai.CameraBoardSocket.CAM_C),
                (left4, left, dai.CameraBoardSocket.CAM_B),
                (right4, right, dai.CameraBoardSocket.CAM_C),
            ):
                queue.send(frame_message(image, sequence, socket))
            reference = output2.get().getFrame()
            candidate = output4.get().getFrame()
            if save_dir is not None:
                save_dir.mkdir(parents=True, exist_ok=True)
                np.save(save_dir / f"{preset}_{sequence:03d}_rvc2.npy", reference)
                np.save(save_dir / f"{preset}_{sequence:03d}_rvc4.npy", candidate)
            if reference.shape != candidate.shape:
                raise RuntimeError(f"{preset}: shape mismatch RVC2={reference.shape}, RVC4={candidate.shape}")
            difference = candidate.astype(np.int32) - reference.astype(np.int32)
            pixels += difference.size
            mismatches += int(np.count_nonzero(difference))
            maximum_difference = max(maximum_difference, int(np.abs(difference).max()))
            reference_range = [min(reference_range[0], int(reference.min())), max(reference_range[1], int(reference.max()))]
            candidate_range = [min(candidate_range[0], int(candidate.min())), max(candidate_range[1], int(candidate.max()))]
    exact = 100.0 * (pixels - mismatches) / pixels
    print(
        f"{preset}: {exact:.8f}% exact, mismatches={mismatches}/{pixels}, max_abs={maximum_difference}, "
        f"ranges=RVC2{reference_range} RVC4{candidate_range}",
        flush=True,
    )
    return mismatches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rvc2-device")
    parser.add_argument("--rvc4-device")
    parser.add_argument("--replay-dir", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=21)
    parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4), default=1)
    parser.add_argument("--preset", action="append", choices=PRESETS)
    parser.add_argument("--raw", action="store_true", help="disable all public post-processing filters")
    parser.add_argument(
        "--disable-filter",
        action="append",
        choices=("median", "speckle", "spatial", "temporal", "threshold"),
        default=[],
        help="disable one filter on both devices; repeat to isolate preset stages",
    )
    parser.add_argument("--save-dir", type=Path, help="save each RVC2/RVC4 disparity frame as NPY")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    rvc2 = select_device(dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device)
    rvc4 = select_device(dai.XLinkPlatform.X_LINK_RVC4, args.rvc4_device)
    pairs = load_pairs(args.replay_dir, args.frames)
    presets = args.preset or PRESETS
    print(
        f"Testing {len(presets)} RVC2 presets, {len(pairs)} frames each, "
        f"native disparity widths, decimation={args.decimation}, raw={args.raw}, "
        f"disabled_filters={args.disable_filter}"
    )
    mismatches = sum(
        compare_profile(
            rvc2,
            rvc4,
            pairs,
            preset,
            args.decimation,
            args.raw,
            args.disable_filter,
            args.save_dir,
        )
        for preset in presets
    )
    if mismatches:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
