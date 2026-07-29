#!/usr/bin/env python3
"""Focused-depth ROI deployment example (host-published detections)."""

import argparse
import os
import threading
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


def detections(roi):
    det = dai.ImgDetection()
    det.label = 0
    det.confidence = 1.0
    det.xmin, det.ymin, det.xmax, det.ymax = roi
    message = dai.ImgDetections()
    message.detections = [det]
    message.setTimestamp(dai.Clock.now())
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--roi", default="0.35,0.30,0.65,0.70")
    parser.add_argument("--ply-dir", default=None, help="If set, save each frame's point cloud as PLY here")
    parser.add_argument("--headless", action="store_true", help="Disable depth visualization")
    parser.add_argument("--max-depth", type=float, default=5000.0, help="Visualization depth clip in millimeters")
    args = parser.parse_args()
    roi = tuple(float(value) for value in args.roi.split(","))
    if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
        parser.error("--roi must be xmin,ymin,xmax,ymax in normalized coordinates")

    pipeline = dai.Pipeline()
    depth = pipeline.create(dai.node.Depth)
    depth.setFocusModels([dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM])
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.ALL)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.SINGLE_TIER_PER_FRAME)
    depth.build(args.fps)

    # Connect the host detection input first: accessing depth.focusedDepth below triggers the
    # focused backend build, which is only wired when inputDetections is already connected.
    roi_queue = depth.inputDetections.createInputQueue()

    # Render the focused depth as a point cloud. PointCloud runs on host by default, so it does not
    # add a device XLink stream to the focused pipeline; it reads the depth frame's intrinsics.
    point_cloud = pipeline.create(dai.node.PointCloud)
    point_cloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)
    depth.focusedDepth.link(point_cloud.inputDepth)

    output_queue = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)
    pcd_queue = point_cloud.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
    debug_queue = depth.focusDebug.createOutputQueue(maxSize=4, blocking=False)

    if args.ply_dir:
        os.makedirs(args.ply_dir, exist_ok=True)

    stop_event = threading.Event()

    def publish_roi():
        while not stop_event.is_set():
            try:
                roi_queue.send(detections(roi))
            except Exception:
                return
            stop_event.wait(0.015)

    with pipeline:
        pipeline.start()
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print("Focus mode: ROI / ALL / SINGLE_TIER_PER_FRAME / model=NEURAL_DEPTH_MEDIUM")
        publisher = threading.Thread(target=publish_roi, daemon=True)
        publisher.start()
        try:
            start = time.monotonic()
            for index in range(args.frames):
                frame = output_queue.get()
                pcd = pcd_queue.get()
                elapsed = time.monotonic() - start
                valid = int(np.count_nonzero(frame.getFrame()))
                xyz = pcd.getPoints()
                points = int(np.count_nonzero(xyz[:, 2] > 0))
                if args.ply_dir:
                    save_ply(os.path.join(args.ply_dir, f"frame_{index:03d}.ply"), xyz)
                summary = (
                    f"frame={index:03d} valid={valid} points={points} "
                    f"Z=[{pcd.getMinZ():.2f},{pcd.getMaxZ():.2f}] "
                    f"time_ms={elapsed * 1000:.1f} fps={(index + 1) / elapsed:.2f}"
                )
                debug = viz.read_debug(debug_queue, block=True)
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
        finally:
            stop_event.set()
            publisher.join(timeout=2)
            viz.close_windows()


if __name__ == "__main__":
    main()
