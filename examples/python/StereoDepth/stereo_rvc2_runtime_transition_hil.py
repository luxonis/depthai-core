#!/usr/bin/env python3
"""Bit-compare RVC2 preset transitions on an actual RVC2 and RVC4.

Unlike the full parity matrix, this test keeps one StereoDepth pipeline alive
while runtime configs move between extended and full-resolution execution
paths. It catches stale or missing scratch buffers that isolated preset runs
cannot expose.
"""

import argparse

import depthai as dai
import numpy as np

from stereo_rvc2_rvc4_live_parity import (
    PRESETS,
    SIZE,
    capture_rectified_pairs,
    clone_config,
    find_device,
    get_with_deadline,
    make_frame,
)


TRANSITION_CYCLE = ("face", "fast-accuracy", "default")
OUTPUTS = ("disparity", "depth")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rvc2-device", required=True, help="RVC2 name or MXID")
    parser.add_argument("--rvc4-device", required=True, help="RVC4 name or IP address")
    parser.add_argument("--width", action="append", type=int, choices=(64, 96), default=[])
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--frames", type=int, default=4, help="compared frames per transition segment")
    parser.add_argument("--settle-frames", type=int, default=2, help="discarded frames after each config change")
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--capture-fps", type=float, default=10.0)
    parser.add_argument("--alpha", type=float, default=-5.0, help="RVC2 rectification alpha")
    args = parser.parse_args()
    args.width = args.width or [64, 96]
    if args.cycles < 1 or args.frames < 1 or args.settle_frames < 1 or args.warmup < 1 or args.capture_fps <= 0:
        parser.error("cycles, frames, settle-frames, warmup, and capture-fps must be positive")
    return args


def transition_sequence(cycles):
    sequence = ["face"]
    for _ in range(cycles):
        sequence.extend(("fast-accuracy", "default", "face"))
    return sequence


def disparity_width(width):
    return getattr(dai.StereoDepthConfig.CostMatching.DisparityWidth, f"DISPARITY_{width}")


def create_preset_config(pipeline, platform, preset, width):
    """Create one preset from defaults; presets are overlays, not resets."""
    template = pipeline.create(dai.node.StereoDepth)
    if platform == dai.XLinkPlatform.X_LINK_RVC4:
        template.setStereoBackend(dai.StereoDepthProperties.StereoBackend.RVC2)
    template.setDefaultProfilePreset(PRESETS[preset])
    template.initialConfig.costMatching.disparityWidth = disparity_width(width)
    config = clone_config(template.initialConfig)
    pipeline.remove(template)
    return config


def config_fingerprint(config):
    algorithm = config.algorithmControl
    post = config.postProcessing
    return (
        config.costMatching.disparityWidth,
        algorithm.enableExtended,
        algorithm.enableLeftRightCheck,
        algorithm.enableSubpixel,
        algorithm.subpixelFractionalBits,
        config.costMatching.confidenceThreshold,
        algorithm.leftRightCheckThreshold,
        post.median,
        post.decimationFilter.decimationFactor,
        post.speckleFilter.enable,
        post.spatialFilter.enable,
        post.temporalFilter.enable,
        post.thresholdFilter.minRange,
        post.thresholdFilter.maxRange,
    )


def create_pipeline(device_info, platform, width, calibration):
    pipeline = dai.Pipeline(dai.Device(device_info))
    configs = {
        preset: create_preset_config(pipeline, platform, preset, width)
        for preset in TRANSITION_CYCLE
    }

    # FACE is the initial extended path. Starting extended also lets the actual
    # RVC2 allocate everything it needs before the test later re-enters FACE.
    stereo = pipeline.create(dai.node.StereoDepth)
    if platform == dai.XLinkPlatform.X_LINK_RVC4:
        stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.RVC2)
    stereo.setDefaultProfilePreset(PRESETS["face"])
    stereo.initialConfig.costMatching.disparityWidth = disparity_width(width)
    stereo.setRuntimeModeSwitch(True)
    if platform == dai.XLinkPlatform.X_LINK_RVC4:
        stereo.setBaseline(calibration["baseline_cm"])
        stereo.setFocalLength(calibration["focal_length_px"])
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)

    return (
        pipeline,
        stereo.left.createInputQueue(maxSize=1, blocking=True),
        stereo.right.createInputQueue(maxSize=1, blocking=True),
        stereo.inputConfig.createInputQueue(maxSize=1, blocking=True),
        {name: getattr(stereo, name).createOutputQueue(maxSize=1, blocking=True) for name in OUTPUTS},
        stereo.outConfig.createOutputQueue(maxSize=1, blocking=True),
        configs,
    )


def send_pair(left_queue, right_queue, pair, sequence):
    left_queue.send(
        make_frame(
            pair.left,
            pair.left_transformation,
            pair.timing["left"],
            sequence,
            dai.CameraBoardSocket.CAM_B,
        )
    )
    right_queue.send(
        make_frame(
            pair.right,
            pair.right_transformation,
            pair.timing["right"],
            sequence,
            dai.CameraBoardSocket.CAM_C,
        )
    )


def run_transitions(device_info, platform, pairs, width, sequence, warmup, settle_frames, frames, calibration):
    pipeline, left_queue, right_queue, config_queue, output_queues, out_config_queue, configs = create_pipeline(
        device_info, platform, width, calibration
    )
    results = []
    pair_index = 0
    with pipeline:
        pipeline.start()
        for _ in range(warmup):
            send_pair(left_queue, right_queue, pairs[pair_index], pair_index)
            for name, queue in output_queues.items():
                output = get_with_deadline(queue, f"{platform.name} D{width} warmup {name}")
                if output.getSequenceNum() != pair_index:
                    raise RuntimeError(f"{platform.name} D{width} warmup sequence mismatch")
            output_config = get_with_deadline(out_config_queue, f"{platform.name} D{width} warmup config")
            if config_fingerprint(output_config) != config_fingerprint(configs["face"]):
                raise RuntimeError(f"{platform.name} D{width} warmup did not report FACE config")
            pair_index += 1

        for segment, preset in enumerate(sequence):
            if segment:
                config_queue.send(configs[preset])
            outputs = {name: [] for name in OUTPUTS}
            for frame_index in range(settle_frames + frames):
                send_pair(left_queue, right_queue, pairs[pair_index], pair_index)
                for name, queue in output_queues.items():
                    output = get_with_deadline(
                        queue,
                        f"{platform.name} D{width} segment {segment} {preset} {name}",
                    )
                    if output.getSequenceNum() != pair_index:
                        raise RuntimeError(
                            f"{platform.name} D{width} segment {segment} {name} sequence "
                            f"{output.getSequenceNum()} != {pair_index}"
                        )
                    if frame_index >= settle_frames:
                        outputs[name].append(output.getFrame().copy())
                output_config = get_with_deadline(
                    out_config_queue,
                    f"{platform.name} D{width} segment {segment} {preset} config",
                )
                if config_fingerprint(output_config) != config_fingerprint(configs[preset]):
                    raise RuntimeError(
                        f"{platform.name} D{width} segment {segment} did not report {preset} config"
                    )
                pair_index += 1
            results.append(outputs)
    return results


def compare_results(width, sequence, rvc2_results, rvc4_results):
    failures = []
    compared = 0
    for segment, preset in enumerate(sequence):
        summaries = []
        for output_name in OUTPUTS:
            rvc2_frames = rvc2_results[segment][output_name]
            rvc4_frames = rvc4_results[segment][output_name]
            if len(rvc2_frames) != len(rvc4_frames):
                failures.append(f"D{width} segment {segment} {preset} {output_name}: frame-count mismatch")
                continue
            mismatches = 0
            pixels = 0
            for rvc2_frame, rvc4_frame in zip(rvc2_frames, rvc4_frames):
                if rvc2_frame.shape != rvc4_frame.shape or rvc2_frame.dtype != rvc4_frame.dtype:
                    failures.append(f"D{width} segment {segment} {preset} {output_name}: shape/dtype mismatch")
                    continue
                mismatches += int(np.count_nonzero(rvc2_frame != rvc4_frame))
                pixels += rvc2_frame.size
            compared += pixels
            summaries.append(f"{output_name}={mismatches}/{pixels}")
            if mismatches:
                failures.append(f"D{width} segment {segment} {preset}: {mismatches} {output_name} pixels differ")
        print(f"D{width} segment={segment} preset={preset}: " + ", ".join(summaries))
    return failures, compared


def main():
    args = parse_args()
    rvc2 = find_device(args.rvc2_device, dai.XLinkPlatform.X_LINK_MYRIAD_X)
    rvc4 = find_device(args.rvc4_device, dai.XLinkPlatform.X_LINK_RVC4)
    sequence = transition_sequence(args.cycles)
    pair_count = args.warmup + len(sequence) * (args.settle_frames + args.frames)
    pairs, calibration = capture_rectified_pairs(rvc2, pair_count, args.capture_fps, args.alpha)
    print(f"RVC2={rvc2.name} ({rvc2.deviceId}) RVC4={rvc4.name} ({rvc4.deviceId})")

    failures = []
    compared = 0
    for width in args.width:
        rvc2_results = run_transitions(
            rvc2, dai.XLinkPlatform.X_LINK_MYRIAD_X, pairs, width, sequence,
            args.warmup, args.settle_frames, args.frames, calibration
        )
        rvc4_results = run_transitions(
            rvc4, dai.XLinkPlatform.X_LINK_RVC4, pairs, width, sequence,
            args.warmup, args.settle_frames, args.frames, calibration
        )
        width_failures, width_compared = compare_results(width, sequence, rvc2_results, rvc4_results)
        failures.extend(width_failures)
        compared += width_compared

    if failures:
        raise RuntimeError("Runtime transition parity failed:\n  " + "\n  ".join(failures))
    print(f"PASS: runtime preset transitions are exact across {compared} compared values")


if __name__ == "__main__":
    main()
