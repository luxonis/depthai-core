#!/usr/bin/env python3
"""Focused-depth detector deployment example with a per-frame time budget.

Crops are processed largest-first within a budget of one frame period (1000/fps ms).
For each crop the controller picks the smallest Nano/S/M model that fits both the crop
size and the remaining budget (downgrading toward Nano when time is short), using
per-model cost estimates fitted to the on-device benchmark in luxonis/depthai-core#1912
(Nano ~49 ms, S ~57 ms, M ~68 ms per inference). Remaining crops are skipped once the
budget is exhausted, so the node stays within the frame period.
"""

import argparse
import os
import time

import depthai as dai
import focused_depth_viz as viz
import numpy as np


def save_ply(path, xyz):
    """Save valid (Z>0) XYZ points as an ASCII PLY; returns the point count."""
    pts = xyz[xyz[:, 2] > 0]
    with open(path, "w") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {len(pts)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        np.savetxt(handle, pts, fmt="%.4f")
    return len(pts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov6-nano")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=30.0, help="Target focused-depth FPS budget")
    parser.add_argument("--ply-dir", default=None, help="If set, save each frame's point cloud as PLY here")
    parser.add_argument("--headless", action="store_true", help="Disable depth visualization")
    parser.add_argument("--max-depth", type=float, default=5000.0, help="Visualization depth clip in millimeters")
    args = parser.parse_args()

    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detector = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
    detector.setConfidenceThreshold(0.5)
    depth = pipeline.create(dai.node.Depth)
    models = [
        dai.DeviceModelZoo.NEURAL_DEPTH_NANO,
        dai.DeviceModelZoo.NEURAL_DEPTH_SMALL,
        dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM,
    ]
    depth.setFocusModels(models)
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.ALL)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.TIME_BUDGET)
    depth.build(args.fps)
    detector.out.link(depth.inputDetections)

    # Render the focused depth as a point cloud (PointCloud runs on host by default).
    point_cloud = pipeline.create(dai.node.PointCloud)
    point_cloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)
    depth.focusedDepth.link(point_cloud.inputDepth)

    detections = detector.out.createOutputQueue(maxSize=4, blocking=False)
    output = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)
    pcd_queue = point_cloud.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
    debug_queue = depth.focusDebug.createOutputQueue(maxSize=4, blocking=False)

    if args.ply_dir:
        os.makedirs(args.ply_dir, exist_ok=True)

    with pipeline:
        pipeline.start()
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print("Configured focus models: NANO=384x240, SMALL=480x300, MEDIUM=576x360")
        print(f"Focus mode: detector / ALL / TIME_BUDGET / target_fps={args.fps:.1f} / models=NANO,SMALL,MEDIUM")
        start = time.monotonic()
        for index in range(args.frames):
            frame = output.get()
            pcd = pcd_queue.get()
            detection_message = detections.tryGet()
            count = len(detection_message.detections) if detection_message else 0
            elapsed = time.monotonic() - start
            valid = int(np.count_nonzero(frame.getFrame()))
            xyz = pcd.getPoints()
            points = int(np.count_nonzero(xyz[:, 2] > 0))
            if args.ply_dir:
                save_ply(os.path.join(args.ply_dir, f"frame_{index:03d}.ply"), xyz)
            summary = (
                f"frame={index:03d} detections={count} valid={valid} points={points} "
                f"Z=[{pcd.getMinZ():.2f},{pcd.getMaxZ():.2f}] "
                f"time_ms={elapsed * 1000:.1f} fps={(index + 1) / elapsed:.2f}"
            )
            debug = viz.read_debug(debug_queue)
            print(summary)
            print(f"debug: {debug}")
            if not viz.show(
                "focused depth",
                frame.getFrame(),
                args.max_depth,
                [summary, debug[:120]],
                args.headless,
            ):
                break


if __name__ == "__main__":
    main()
