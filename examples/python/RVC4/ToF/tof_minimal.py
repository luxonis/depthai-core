#!/usr/bin/env python3
"""Minimal RVC4 ToF example with auto-created Camera (no manual rawInput wiring)."""

import cv2
import depthai as dai
import numpy as np


def colorize_depth(frame_depth: np.ndarray) -> np.ndarray:
    invalid_mask = frame_depth == 0
    try:
        min_depth = np.percentile(frame_depth[frame_depth != 0], 3)
        max_depth = np.percentile(frame_depth[frame_depth != 0], 95)
        log_depth = np.log(frame_depth, where=frame_depth != 0)
        log_min_depth = np.log(min_depth)
        log_max_depth = np.log(max_depth)
        if log_max_depth <= log_min_depth:
            # Valid depths collapsed to a single value; avoid divide-by-zero normalization.
            log_max_depth = log_min_depth + 1.0
        np.nan_to_num(log_depth, copy=False, nan=log_min_depth)
        log_depth = np.clip(log_depth, log_min_depth, log_max_depth)
        depth_color = np.interp(log_depth, (log_min_depth, log_max_depth), (0, 255))
        depth_color = depth_color.astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_color, cv2.COLORMAP_JET)
        depth_color[invalid_mask] = 0
        return depth_color
    except IndexError:
        return np.zeros((frame_depth.shape[0], frame_depth.shape[1], 3), dtype=np.uint8)


def main() -> None:
    pipeline = dai.Pipeline()

    tof = pipeline.create(dai.node.ToF)
    tof.build(
        dai.CameraBoardSocket.AUTO,
        sensorMode=dai.ToFSensorMode.F3_FULL,
        fps=10,
        preset=dai.ToFPreset.MID_RANGE,
    )

    depth_queue = tof.depth.createOutputQueue()

    with pipeline as p:
        p.start()
        if tof.getCamera() is not None:
            print(f"Auto-created Camera on socket {tof.getBoardSocket()}")
        while p.isRunning():
            depth_frame = depth_queue.tryGet()
            if depth_frame is None:
                continue
            depth_image = colorize_depth(depth_frame.getFrame())
            cv2.imshow("depth", depth_image)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
