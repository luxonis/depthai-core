#!/usr/bin/env python3

import argparse
import time

import depthai as dai


CAMERA_SOCKETS = {
    "CAM_A": dai.CameraBoardSocket.CAM_A,
    "CAM_B": dai.CameraBoardSocket.CAM_B,
    "CAM_C": dai.CameraBoardSocket.CAM_C,
    "CAM_D": dai.CameraBoardSocket.CAM_D,
}
DEFAULT_CAMERAS = ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cameras", nargs="+", default=DEFAULT_CAMERAS, choices=DEFAULT_CAMERAS)
    parser.add_argument("--main-camera", default="CAM_B", choices=DEFAULT_CAMERAS)
    parser.add_argument("--frames", type=int, default=400)
    parser.add_argument("--edge-exclusion", type=int, default=10)
    parser.add_argument(
        "--disable-fsync",
        action="store_true",
        help="Disable camera frame sync via camera.initialControl.setFrameSyncMode(OFF).",
    )
    parser.add_argument(
        "--non-raw",
        action="store_true",
        help="Use camera.requestFullResolutionOutput() instead of the raw stream.",
    )
    parser.add_argument("--verbose", action="store_true", help="Log per-frame timestamps and sequence numbers.")
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Keep collecting and emitting a new sync summary every --frames frames per camera.",
    )
    return parser.parse_args()


def unique_preserve_order(items: list[str]) -> list[str]:
    unique_items: list[str] = []
    seen: set[str] = set()
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        unique_items.append(item)
    return unique_items


def main() -> None:
    args = parse_args()
    camera_names = unique_preserve_order(args.cameras)
    if args.main_camera not in camera_names:
        raise SystemExit(f"--main-camera {args.main_camera} must be included in --cameras")
    if args.frames <= 0:
        raise SystemExit("--frames must be greater than 0")

    pipeline = dai.Pipeline()
    script = pipeline.create(dai.node.Script)
    done_queue = script.outputs["done"].createOutputQueue()

    stream_label_suffix = "FULL_RES" if args.non_raw else "RAW"
    camera_labels = {name: f"{name}_{stream_label_suffix}" for name in camera_names}
    for camera_name in camera_names:
        camera = pipeline.create(dai.node.Camera).build(CAMERA_SOCKETS[camera_name])
        if args.disable_fsync:
            camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.OFF)
        if args.non_raw:
            camera.requestFullResolutionOutput().link(script.inputs[camera_name])
        else:
            camera.raw.link(script.inputs[camera_name])

    script.setScript(
        f"""
CAMERAS = {camera_names!r}
CAMERA_LABELS = {camera_labels!r}
MAIN_CAMERA = {args.main_camera!r}
TARGET_FRAMES = {args.frames}
EDGE_EXCLUSION = {args.edge_exclusion}
VERBOSE = {args.verbose}
LOOP = {args.loop}
MATCH_WINDOW_1US = 1
MATCH_WINDOW_1MS_US = 1000


def timestamp_to_us(timestamp):
    return timestamp.days * 86400000000 + timestamp.seconds * 1000000 + timestamp.microseconds


def log_frame(name, frame_idx, frame):
    if not VERBOSE:
        return

    node.warn(
        name
        + " frame_idx="
        + str(frame_idx)
        + " seq="
        + str(frame.getSequenceNum())
        + " ts="
        + str(frame.getTimestamp())
        + " ts_device="
        + str(frame.getTimestampDevice())
    )


def has_match(timestamp_us, other_timestamps_us, match_window_us):
    for other_timestamp_us in other_timestamps_us:
        delta_us = timestamp_us - other_timestamp_us
        if delta_us < 0:
            delta_us = -delta_us

        if delta_us <= match_window_us:
            return True

    return False


def report_matches(name, timestamps_us, other_timestamps_us, match_window_us, should_log=True):
    matched = 0
    unmatched = 0
    ignored_edge_misses = 0

    for frame_idx in range(len(timestamps_us)):
        frame_has_match = has_match(timestamps_us[frame_idx], other_timestamps_us, match_window_us)

        if frame_has_match:
            matched += 1
        elif frame_idx < EDGE_EXCLUSION or frame_idx >= len(timestamps_us) - EDGE_EXCLUSION:
            ignored_edge_misses += 1
        else:
            unmatched += 1

    return matched, unmatched, ignored_edge_misses


def count_matching_pairs(left_timestamps_us, right_timestamps_us, match_window_us):
    left_idx = 0
    right_idx = 0
    matches = 0

    while left_idx < len(left_timestamps_us) and right_idx < len(right_timestamps_us):
        delta_us = left_timestamps_us[left_idx] - right_timestamps_us[right_idx]

        if delta_us < 0:
            delta_abs_us = -delta_us
        else:
            delta_abs_us = delta_us

        if delta_abs_us <= match_window_us:
            matches += 1
            left_idx += 1
            right_idx += 1
        elif delta_us < 0:
            left_idx += 1
        else:
            right_idx += 1

    return matches


def absolute_us(delta_us):
    if delta_us < 0:
        return -delta_us
    return delta_us


def collect_nearest_pair_deltas(left_timestamps_us, right_timestamps_us):
    if len(left_timestamps_us) == 0 or len(right_timestamps_us) == 0:
        return []

    deltas_us = []
    right_idx = 0

    for left_timestamp_us in left_timestamps_us:
        while (
            right_idx + 1 < len(right_timestamps_us)
            and absolute_us(right_timestamps_us[right_idx + 1] - left_timestamp_us)
            <= absolute_us(right_timestamps_us[right_idx] - left_timestamp_us)
        ):
            right_idx += 1

        deltas_us.append(right_timestamps_us[right_idx] - left_timestamp_us)

    return deltas_us


def median_value(values):
    if len(values) == 0:
        return 0

    sorted_values = values[:]
    sorted_values.sort()
    middle_idx = len(sorted_values) // 2

    if len(sorted_values) % 2 == 1:
        return sorted_values[middle_idx]

    return (sorted_values[middle_idx - 1] + sorted_values[middle_idx]) // 2


def overlap_duration_us(left_timestamps_us, right_timestamps_us):
    if len(left_timestamps_us) < 2 or len(right_timestamps_us) < 2:
        return 0

    overlap_start_us = left_timestamps_us[0]
    if right_timestamps_us[0] > overlap_start_us:
        overlap_start_us = right_timestamps_us[0]

    overlap_end_us = left_timestamps_us[-1]
    if right_timestamps_us[-1] < overlap_end_us:
        overlap_end_us = right_timestamps_us[-1]

    if overlap_end_us <= overlap_start_us:
        return 0

    return overlap_end_us - overlap_start_us


def calculate_synced_fps(left_timestamps_us, right_timestamps_us, matching_pairs):
    shared_duration_us = overlap_duration_us(left_timestamps_us, right_timestamps_us)
    if shared_duration_us <= 0:
        return 0.0

    return matching_pairs * 1000000.0 / shared_duration_us


def format_float(value):
    return "{{:.3f}}".format(value)


def summarize_pair(left_name, left_timestamps_us, right_name, right_timestamps_us):
    matching_pairs_1us = count_matching_pairs(left_timestamps_us, right_timestamps_us, MATCH_WINDOW_1US)
    matching_pairs_1ms = count_matching_pairs(left_timestamps_us, right_timestamps_us, MATCH_WINDOW_1MS_US)
    matched_left, unmatched_left, ignored_left = report_matches(
        left_name + "_vs_" + right_name, left_timestamps_us, right_timestamps_us, MATCH_WINDOW_1US
    )
    matched_right, unmatched_right, ignored_right = report_matches(
        right_name + "_vs_" + left_name, right_timestamps_us, left_timestamps_us, MATCH_WINDOW_1US
    )
    matched_left_1ms, unmatched_left_1ms, ignored_left_1ms = report_matches(
        left_name + "_vs_" + right_name, left_timestamps_us, right_timestamps_us, MATCH_WINDOW_1MS_US, False
    )
    matched_right_1ms, unmatched_right_1ms, ignored_right_1ms = report_matches(
        right_name + "_vs_" + left_name, right_timestamps_us, left_timestamps_us, MATCH_WINDOW_1MS_US, False
    )
    pair_deltas_us = collect_nearest_pair_deltas(left_timestamps_us, right_timestamps_us)
    median_delta_us = median_value(pair_deltas_us)
    min_delta_us = min(pair_deltas_us)
    max_delta_us = max(pair_deltas_us)
    synced_fps = calculate_synced_fps(left_timestamps_us, right_timestamps_us, matching_pairs_1ms)

    node.warn(
        "pair="
        + left_name
        + "_to_"
        + right_name
        + " matching_pairs_within_1us="
        + str(matching_pairs_1us)
        + " matched_frames_within_1us="
        + str(matched_left + matched_right)
        + " unmatched_frames_within_1us="
        + str(unmatched_left + unmatched_right)
        + " ignored_edge_misses_within_1us="
        + str(ignored_left + ignored_right)
        + " matching_pairs_within_1ms="
        + str(matching_pairs_1ms)
        + " synced_fps="
        + format_float(synced_fps)
        + " matched_frames_within_1ms="
        + str(matched_left_1ms + matched_right_1ms)
        + " unmatched_frames_within_1ms="
        + str(unmatched_left_1ms + unmatched_right_1ms)
        + " ignored_edge_misses_within_1ms="
        + str(ignored_left_1ms + ignored_right_1ms)
        + " median_right_minus_left_us="
        + str(median_delta_us)
        + " min_right_minus_left_us="
        + str(min_delta_us)
        + " max_right_minus_left_us="
        + str(max_delta_us)
    )


def create_timestamp_buckets():
    timestamps_us = {{}}
    for camera_name in CAMERAS:
        timestamps_us[camera_name] = []
    return timestamps_us


camera_timestamps_us = create_timestamp_buckets()

while True:
    collected_all_frames = True

    for camera_name in CAMERAS:
        if len(camera_timestamps_us[camera_name]) >= TARGET_FRAMES:
            continue

        collected_all_frames = False
        frame = node.inputs[camera_name].tryGet()
        if frame is None:
            continue

        log_frame(CAMERA_LABELS[camera_name], len(camera_timestamps_us[camera_name]), frame)
        camera_timestamps_us[camera_name].append(timestamp_to_us(frame.getTimestampDevice()))

    if not collected_all_frames:
        continue

    collected_summary = "main_camera=" + CAMERA_LABELS[MAIN_CAMERA]
    for camera_name in CAMERAS:
        collected_summary += " collected_" + CAMERA_LABELS[camera_name].lower() + "=" + str(len(camera_timestamps_us[camera_name]))
    node.warn(collected_summary)

    for camera_name in CAMERAS:
        if camera_name == MAIN_CAMERA:
            continue
        summarize_pair(
            CAMERA_LABELS[camera_name],
            camera_timestamps_us[camera_name],
            CAMERA_LABELS[MAIN_CAMERA],
            camera_timestamps_us[MAIN_CAMERA],
        )

    node.outputs["done"].send(Buffer(1))
    if not LOOP:
        break

    camera_timestamps_us = create_timestamp_buckets()
"""
    )

    pipeline.start()
    stream_description = "full-resolution" if args.non_raw else "raw"

    if args.loop:
        print(
            "Collecting "
            f"{args.frames} {stream_description} frames from {', '.join(camera_names)} "
            f"and recomputing sync stats against {args.main_camera} every batch. Press Ctrl+C to stop."
        )
    else:
        print(
            "Collecting "
            f"{args.frames} {stream_description} frames from {', '.join(camera_names)} "
            f"and comparing every non-main camera to {args.main_camera}."
        )

    try:
        while True:
            if done_queue.tryGet() is None:
                time.sleep(0.1)
                continue

            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(0.2)
        pipeline.stop()


if __name__ == "__main__":
    main()
