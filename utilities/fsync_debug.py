#!/usr/bin/env python3

import time

import depthai as dai


pipeline = dai.Pipeline()

cam_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
cam_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
script = pipeline.create(dai.node.Script)
done_queue = script.outputs["done"].createOutputQueue()

#cam_b.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.OFF)
#cam_c.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.OFF)

cam_b.requestFullResolutionOutput().link(script.inputs["cam_b"])
cam_c.requestFullResolutionOutput().link(script.inputs["cam_c"])

script.setScript(
    """
    TARGET_FRAMES = 400
    EDGE_EXCLUSION = 10
    MATCH_WINDOW_US = 1


    def timestamp_to_us(timestamp):
        return timestamp.days * 86400000000 + timestamp.seconds * 1000000 + timestamp.microseconds


    def log_frame(name, frame_idx, frame):
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


    def has_match(timestamp_us, other_timestamps_us):
        for other_timestamp_us in other_timestamps_us:
            delta_us = timestamp_us - other_timestamp_us
            if delta_us < 0:
                delta_us = -delta_us

            if delta_us <= MATCH_WINDOW_US:
                return True

        return False


    def report_matches(name, timestamps_us, other_timestamps_us):
        matched = 0
        unmatched = 0
        ignored_edge_misses = 0

        for frame_idx in range(len(timestamps_us)):
            frame_has_match = has_match(timestamps_us[frame_idx], other_timestamps_us)

            if frame_has_match:
                matched += 1
            elif frame_idx < EDGE_EXCLUSION or frame_idx >= len(timestamps_us) - EDGE_EXCLUSION:
                ignored_edge_misses += 1
            else:
                unmatched += 1

        node.warn(
            name
            + " matched_frames="
            + str(matched)
            + " unmatched_frames="
            + str(unmatched)
            + " ignored_edge_misses="
            + str(ignored_edge_misses)
        )

        return matched, unmatched, ignored_edge_misses


    def count_matching_pairs(left_timestamps_us, right_timestamps_us):
        left_idx = 0
        right_idx = 0
        matches = 0

        while left_idx < len(left_timestamps_us) and right_idx < len(right_timestamps_us):
            delta_us = left_timestamps_us[left_idx] - right_timestamps_us[right_idx]

            if delta_us < 0:
                delta_abs_us = -delta_us
            else:
                delta_abs_us = delta_us

            if delta_abs_us <= MATCH_WINDOW_US:
                matches += 1
                left_idx += 1
                right_idx += 1
            elif delta_us < 0:
                left_idx += 1
            else:
                right_idx += 1

        return matches


    cam_b_timestamps_us = []
    cam_c_timestamps_us = []

    while len(cam_b_timestamps_us) < TARGET_FRAMES or len(cam_c_timestamps_us) < TARGET_FRAMES:
        frame_b = node.inputs["cam_b"].tryGet()
        if frame_b is not None and len(cam_b_timestamps_us) < TARGET_FRAMES:
            log_frame("CAM_B", len(cam_b_timestamps_us), frame_b)
            cam_b_timestamps_us.append(timestamp_to_us(frame_b.getTimestampDevice()))

        frame_c = node.inputs["cam_c"].tryGet()
        if frame_c is not None and len(cam_c_timestamps_us) < TARGET_FRAMES:
            log_frame("CAM_C", len(cam_c_timestamps_us), frame_c)
            cam_c_timestamps_us.append(timestamp_to_us(frame_c.getTimestampDevice()))

    matching_pairs = count_matching_pairs(cam_b_timestamps_us, cam_c_timestamps_us)
    matched_b, unmatched_b, ignored_b = report_matches("CAM_B", cam_b_timestamps_us, cam_c_timestamps_us)
    matched_c, unmatched_c, ignored_c = report_matches("CAM_C", cam_c_timestamps_us, cam_b_timestamps_us)

    node.warn(
        "matching_pairs_within_1us="
        + str(matching_pairs)
        + " collected_cam_b="
        + str(len(cam_b_timestamps_us))
        + " collected_cam_c="
        + str(len(cam_c_timestamps_us))
    )
    node.warn(
        "summary matched_frames="
        + str(matched_b + matched_c)
        + " unmatched_frames="
        + str(unmatched_b + unmatched_c)
        + " ignored_edge_misses="
        + str(ignored_b + ignored_c)
        + " ignored_edge_frames_per_camera="
        + str(EDGE_EXCLUSION)
    )

    node.outputs["done"].send(Buffer(1))
"""
)

pipeline.start()

print("Collecting 400 frames per camera and logging timestamp match statistics from the Script node.")
done_queue.get()
time.sleep(0.2)
pipeline.stop()
