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


def colorizeDepth(frame: np.ndarray) -> np.ndarray:
    """Normalize a uint16 depth frame and apply a colormap for display."""
    downscaled = frame[::4, ::4]
    nonZero = downscaled[downscaled != 0]
    if nonZero.size == 0:
        minD, maxD = 0, 1
    else:
        minD = np.percentile(nonZero, 1)
        maxD = np.percentile(nonZero, 99)
    colored = np.interp(frame, (minD, maxD), (0, 255)).astype(np.uint8)
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
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.requestOutput((640, 400)).link(stereo.left)
        right.requestOutput((640, 400)).link(stereo.right)

        # Align depth to color camera
        platform = pipeline.getDefaultDevice().getPlatform()
        colorOut = color.requestOutput(
            (640, 400), type=dai.ImgFrame.Type.RGB888i,
            resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True,
        )
        if platform == dai.Platform.RVC4:
            align = pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            colorOut.link(align.inputAlignTo)
            alignedDepth = align.outputAligned
        else:
            colorOut.link(stereo.inputAlignTo)
            alignedDepth = stereo.depth

        # ── PointCloud node ───────────────────────────────────────────
        pc = pipeline.create(dai.node.PointCloud)
        pc.setRunOnHost(True)
        pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        alignedDepth.link(pc.inputDepth)
        colorOut.link(pc.inputColor)

        queue = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
        qDepth = pc.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)

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
        qDepth.tryGetAll()

        print("Streaming... Press Q in the Open3D window to quit.")

        try:
            while True:
                pclData = queue.tryGet()

                if pclData is not None:
                    if pclData.isColor():
                        points, colors = pclData.getPointsRGB()
                        if len(points) > 0:
                            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                            pcd.colors = o3d.utility.Vector3dVector(
                                np.delete(colors / 255.0, 3, 1).astype(np.float64)
                            )
                    else:
                        points = pclData.getPoints()
                        if len(points) > 0:
                            pcd.points = o3d.utility.Vector3dVector(points)
                            pcd.colors = o3d.utility.Vector3dVector()

                    if len(pcd.points) > 0:
                        if first:
                            vis.add_geometry(pcd)
                            ctr = vis.get_view_control()
                            ctr.set_front([0, 0, -1])
                            ctr.set_up([0, -1, 0])
                            ctr.set_lookat([0, 0, 1])
                            ctr.set_zoom(0.3)
                            first = False
                        else:
                            vis.update_geometry(pcd)

                # Show colorized depth in an OpenCV window
                depthMsg = qDepth.tryGet()
                if depthMsg is not None:
                    cv2.imshow("Depth", colorizeDepth(depthMsg.getCvFrame()))

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
