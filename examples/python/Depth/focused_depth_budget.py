#!/usr/bin/env python3
"""Focused-depth detector deployment example with a per-frame time budget.

Crops are processed largest-first within a budget of one frame period (1000/fps ms).
For each crop the controller picks the smallest Nano/S/M model that fits both the crop
size and the remaining budget (downgrading toward Nano when time is short), using
per-model cost estimates fitted to the on-device benchmark in luxonis/depthai-core#1912
(Nano ~49 ms, S ~57 ms, M ~68 ms per inference). Remaining crops are skipped once the
budget is exhausted, so the node stays within the frame period.

Images, detections, focused depth and the point cloud are streamed to the DepthAI
Visualizer; open the printed http://localhost:<httpPort> URL in a browser.
"""

import argparse
import os

import depthai as dai
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


def read_debug(debug_queue):
    """Return the latest focusDebug trace string, or '' if none is pending."""
    message = debug_queue.tryGet()
    if message is None:
        return ""
    return bytes(message.getData()).decode("utf-8", "replace")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov6-nano")
    parser.add_argument("--fps", type=float, default=30.0, help="Target focused-depth FPS budget")
    parser.add_argument("--frames", type=int, default=0, help="Stop after N frames (0 = run until 'q')")
    parser.add_argument("--ply-dir", default=None, help="If set, save each frame's point cloud as PLY here")
    parser.add_argument("--webSocketPort", type=int, default=8765)
    parser.add_argument("--httpPort", type=int, default=8082)
    args = parser.parse_args()

    remote = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
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

    debug_queue = depth.focusDebug.createOutputQueue(maxSize=4, blocking=False)
    pcd_queue = point_cloud.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

    remote.addTopic("images", detector.passthrough, "img")
    remote.addTopic("detections", detector.out, "img")
    remote.addTopic("depth", depth.focusedDepth, "img")
    remote.addTopic("pointcloud", point_cloud.outputPointCloud, "3d")

    if args.ply_dir:
        os.makedirs(args.ply_dir, exist_ok=True)

    with pipeline:
        pipeline.start()
        remote.registerPipeline(pipeline)
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print("Configured focus models: NANO=384x240, SMALL=480x300, MEDIUM=576x360")
        print(f"Focus mode: detector / ALL / TIME_BUDGET / target_fps={args.fps:.1f} / models=NANO,SMALL,MEDIUM")
        print(f"Visualizer running at http://localhost:{args.httpPort}")
        frames_done = 0
        while pipeline.isRunning():
            if remote.waitKey(1) == ord("q"):
                break
            debug = read_debug(debug_queue)
            if not debug:
                continue
            print(f"frame={frames_done:03d} debug: {debug}")
            if args.ply_dir:
                pcd = pcd_queue.tryGet()
                if pcd is not None:
                    save_ply(os.path.join(args.ply_dir, f"frame_{frames_done:03d}.ply"), pcd.getPoints())
            frames_done += 1
            if args.frames and frames_done >= args.frames:
                break


if __name__ == "__main__":
    main()
