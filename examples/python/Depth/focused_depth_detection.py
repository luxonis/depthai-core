#!/usr/bin/env python3
"""Focused-depth detector deployment example (largest detection only).

Images, detections, focused depth and the point cloud are streamed to the DepthAI Visualizer;
open the printed http://localhost:<httpPort> URL in a browser.
"""

import argparse
import os
from urllib.parse import quote

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
    parser.add_argument("--frames", type=int, default=0, help="Stop after N frames (0 = run until 'q')")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--ply-dir", default=None, help="If set, save each frame's point cloud as PLY here")
    parser.add_argument("--webSocketPort", type=int, default=8765, help="Visualizer websocket port")
    parser.add_argument("--httpPort", type=int, default=8082, help="Visualizer HTTP port")
    args = parser.parse_args()

    remote = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    color_out = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.RGB888i, enableUndistortion=True, fps=args.fps)
    detector = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
    detector.setConfidenceThreshold(0.5)
    depth = pipeline.create(dai.node.Depth)
    depth.setFocusModels([dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM])
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.LARGEST)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.SINGLE_TIER_PER_FRAME)
    depth.build(args.fps)
    detector.out.link(depth.inputDetections)

    align = pipeline.create(dai.node.ImageAlign)
    align.setRunOnHost(True)
    depth.focusedDepth.link(align.input)
    color_out.link(align.inputAlignTo)

    point_cloud = pipeline.create(dai.node.PointCloud)
    point_cloud.useCPUMT(4)
    point_cloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)
    align.outputAligned.link(point_cloud.inputDepth)
    color_out.link(point_cloud.inputColor)

    debug_queue = depth.focusDebug.createOutputQueue(maxSize=4, blocking=False)
    pcd_queue = point_cloud.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

    remote.addTopic("images", color_out, "img")
    remote.addTopic("detections", detector.out, "img")
    remote.addTopic("depth", align.outputAligned, "img")
    remote.addTopic("pcl", point_cloud.outputPointCloud, "common")

    if args.ply_dir:
        os.makedirs(args.ply_dir, exist_ok=True)

    with pipeline:
        pipeline.start()
        remote.registerPipeline(pipeline)
        ws_url = quote(f"ws://localhost:{args.webSocketPort}", safe="")
        visualizer_url = f"http://localhost:{args.httpPort}?ws_url={ws_url}"
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print(f"Focus mode: detector / LARGEST / SINGLE_TIER_PER_FRAME / model={args.model}")
        print(f"Visualizer running at {visualizer_url}")
        frames_done = 0
        while pipeline.isRunning():
            if remote.waitKey(1) == ord("q"):
                break
            debug = read_debug(debug_queue)
            if not debug:
                continue
            print(f"frame={frames_done:03d} debug: {debug}")
            if args.ply_dir:
                pcd_messages = pcd_queue.tryGetAll()
                if pcd_messages:
                    save_ply(os.path.join(args.ply_dir, f"frame_{frames_done:03d}.ply"), pcd_messages[-1].getPoints())
            frames_done += 1
            if args.frames and frames_done >= args.frames:
                break


if __name__ == "__main__":
    main()
