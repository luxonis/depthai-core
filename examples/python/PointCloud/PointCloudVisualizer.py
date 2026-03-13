#!/usr/bin/env python3
"""
PointCloud Visualizer

Live 3D visualization of stereo-depth point clouds using Open3D.
Press Q in the Open3D window (or Ctrl-C in the terminal) to quit.
"""

import time

import cv2
import numpy as np

try:
    import open3d as o3d
except ImportError:
    raise SystemExit("Please install open3d: pip install open3d")

import depthai as dai


def colorize_depth(frame: np.ndarray) -> np.ndarray:
    """Normalize a uint16 depth frame and apply a colormap for display."""
    downscaled = frame[::4]
    non_zero = downscaled[downscaled != 0]
    if non_zero.size == 0:
        min_d, max_d = 0, 1
    else:
        min_d = np.percentile(non_zero, 1)
        max_d = np.percentile(downscaled, 99)
    colored = np.interp(frame, (min_d, max_d), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(colored, cv2.COLORMAP_HOT)


def main() -> None:
    print("PointCloud Visualizer")
    print("=====================")
    print("Connecting to device...")

    device = dai.Device()
    print(f"Device: {device.getDeviceName()}  (ID: {device.getDeviceId()})\n")

    with dai.Pipeline(device) as pipeline:
        # ── Camera + StereoDepth ──────────────────────────────────────
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.requestOutput((640, 400)).link(stereo.left)
        right.requestOutput((640, 400)).link(stereo.right)

        # ── PointCloud node ───────────────────────────────────────────
        pc = pipeline.create(dai.node.PointCloud)
        pc.setRunOnHost(True)
        pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        stereo.depth.link(pc.inputDepth)

        queue = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        # ── Open3D setup ──────────────────────────────────────────────
        vis = o3d.visualization.Visualizer()
        vis.create_window("PointCloud Visualizer")
        pcd = o3d.geometry.PointCloud()
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        vis.add_geometry(coord)
        first = True

        # ── Start pipeline & wait for auto-exposure ───────────────────
        pipeline.start()
        print("Waiting for auto-exposure to settle...")
        time.sleep(1)
        queue.tryGetAll()  # drain stale frames
        q_depth.tryGetAll()

        print("Streaming... Press Q in the Open3D window to quit.")

        try:
            while True:
                pcl_data = queue.tryGet()

                if pcl_data is not None:
                    points = pcl_data.getPoints()
                    if len(points) > 0:
                        pcd.points = o3d.utility.Vector3dVector(points)
                        if first:
                            vis.add_geometry(pcd)
                            first = False
                        else:
                            vis.update_geometry(pcd)

                # Show colorized depth in an OpenCV window
                depth_msg = q_depth.tryGet()
                if depth_msg is not None:
                    cv2.imshow("Depth", colorize_depth(depth_msg.getCvFrame()))

                if cv2.waitKey(1) == ord("q"):
                    break
                if not vis.poll_events():
                    break
                vis.update_renderer()

        except KeyboardInterrupt:
            print("\nStopping...")

        vis.destroy_window()
        cv2.destroyAllWindows()

    print("Done.")


if __name__ == "__main__":
    main()
