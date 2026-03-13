#!/usr/bin/env python3
"""
PointCloud Node Showcase

Demonstrates all capabilities of the PointCloud node using a single pipeline
with multiple PointCloud nodes, each configured differently.  Stereo depth
fans out to all of them simultaneously.

Features shown:
  1. Sparse point cloud       – METER, via initialConfig.setLengthUnit()
  2. Organized point cloud    – MILLIMETER, via initialConfig.setOrganized(True)
  3. Camera-to-camera transform – setTargetCoordinateSystem(CameraBoardSocket)
  4. Custom 4×4 transform     – 90° Z rotation via initialConfig + passthrough


Coordinate-system transforms
----------------------------
The PointCloud node can transform output points into three kinds of
coordinate frames via ``setTargetCoordinateSystem()``:

  A) **Camera board socket** – uses extrinsic calibration between cameras.
     ``pc.setTargetCoordinateSystem(dai.CameraBoardSocket.<SOCKET>)``

     Available sockets:
       AUTO, CAM_A, CAM_B, CAM_C, CAM_D, CAM_E, CAM_F, CAM_G, CAM_H,
       CAM_I, CAM_J

     Optional flag ``useSpecTranslation`` (default False):
       When True the node uses nominal/spec translation between cameras
       instead of per-unit calibration data.

  B) **Housing coordinate system** – uses housing calibration stored on the
     device (position of mounting points, IMU, camera fronts, etc.).
     ``pc.setTargetCoordinateSystem(dai.HousingCoordinateSystem.<CS>)``

     Available coordinate systems:
       AUTO,
       CAM_A … CAM_J       – camera origins on the housing
       FRONT_CAM_A … FRONT_CAM_J – points in front of each camera
       VESA_A … VESA_J     – VESA mounting-point origins
       IMU                 – IMU sensor origin

     Optional flag ``useSpecTranslation`` (default True):
       When True the node uses nominal/spec housing coordinates;
       when False it uses per-unit calibrated values.

  C) **Custom 4×4 matrix** – applied via ``initialConfig`` or the runtime
     ``inputConfig`` queue.
     ``pc.initialConfig.setTransformationMatrix([[...], ...])``
     This can also be combined with (A) or (B); the custom matrix is
     applied *after* the calibration-based transform.
"""

import time
import os
# os.environ["DEPTHAI_DEVICE_RVC4_FWP"] = "/home/tomas/code/depthai-device-kb/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz"
import depthai as dai


# ---------------------------------------------------------------------------
# Print helpers
# ---------------------------------------------------------------------------
def print_header(title: str) -> None:
    print(f"\n╔══════════════════════════════════════════════╗")
    print(f"║  {title:<44s}║")
    print(f"╚══════════════════════════════════════════════╝")


def print_point_cloud_info(pcd: dai.PointCloudData, frame_num: int) -> None:
    points = pcd.getPoints()
    print(f"\n--- Frame {frame_num} ---")
    print(f"  Points       : {len(points)}")
    print(f"  Width×Height : {pcd.getWidth()} × {pcd.getHeight()}")
    print(f"  Organized    : {'yes' if pcd.isOrganized() else 'no'}")
    print(f"  Color        : {'yes' if pcd.isColor() else 'no'}")
    print(f"  Bounding box :"
          f"  X [{pcd.getMinX()}, {pcd.getMaxX()}]"
          f"  Y [{pcd.getMinY()}, {pcd.getMaxY()}]"
          f"  Z [{pcd.getMinZ()}, {pcd.getMaxZ()}]")


# ===========================================================================
NUM_FRAMES = 3


def main() -> None:
    print("PointCloud Node Showcase")
    print("========================")
    print("Connecting to device...")

    # device = dai.Device()
    # print(f"Device: {device.getDeviceName()}  (ID: {device.getDeviceId()})\n")
    device_ip = "10.11.0.29"
    device_info = dai.DeviceInfo(device_ip)
    device = dai.Device(device_info)

    # ------------------------------------------------------------------
    # Single pipeline – shared Camera + StereoDepth, multiple PointCloud
    # nodes configured differently.
    # ------------------------------------------------------------------
    with dai.Pipeline(device) as pipeline:
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.requestFullResolutionOutput().link(stereo.left)
        right.requestFullResolutionOutput().link(stereo.right)

        # ── 1. Sparse point cloud  (METER, multi-threaded, large pool) ────
        pc_sparse = pipeline.create(dai.node.PointCloud)
        pc_sparse.setRunOnHost(False)
        pc_sparse.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        stereo.depth.link(pc_sparse.inputDepth)
        q_sparse = pc_sparse.outputPointCloud.createOutputQueue()

        # ── 2. Organized point cloud  (MILLIMETER, single-threaded) ───────
        pc_organized = pipeline.create(dai.node.PointCloud)
        pc_organized.setRunOnHost(False)
        pc_organized.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pc_organized.initialConfig.setOrganized(True)
        stereo.depth.link(pc_organized.inputDepth)
        q_organized = pc_organized.outputPointCloud.createOutputQueue()

        # ── 3. Transform pointcloud into another device's coordinate system ───
        pc_cam = pipeline.create(dai.node.PointCloud)
        pc_cam.setRunOnHost(False)
        pc_cam.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pc_cam.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_A)
        # Or transform to a housing coordinate system instead, e.g.:
        # pc_cam.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        stereo.depth.link(pc_cam.inputDepth)
        q_cam = pc_cam.outputPointCloud.createOutputQueue()

        # ── 4. Custom 4×4 transform  (90° Z rotation) + passthrough ──────
        pc_custom = pipeline.create(dai.node.PointCloud)
        pc_custom.setRunOnHost(False)
        pc_custom.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pc_custom.useCPU()
        transform = [
            [0.0, -1.0, 0.0, 0.0],
            [1.0,  0.0, 0.0, 0.0],
            [0.0,  0.0, 1.0, 0.0],
            [0.0,  0.0, 0.0, 1.0],
        ]
        pc_custom.initialConfig.setTransformationMatrix(transform)
        stereo.depth.link(pc_custom.inputDepth)
        q_custom = pc_custom.outputPointCloud.createOutputQueue()
        q_depth = pc_custom.passthroughDepth.createOutputQueue()

        # Note: Housing coordinate system transform is also available, e.g.:
        #   pc.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        # See the docstring at the top of this file for all available
        # CameraBoardSocket and HousingCoordinateSystem values.

        sparse_frames = []
        organized_frames = []
        cam_frames = []
        custom_frames = []
        depth_frames = []

        pipeline.start()

        # Wait for auto-exposure to settle and stereo depth to stabilize
        print("Waiting for auto-exposure to settle...")
        time.sleep(1)

        # Drain stale frames that arrived during warm-up
        q_sparse.tryGetAll()
        q_organized.tryGetAll()
        q_cam.tryGetAll()
        q_custom.tryGetAll()
        q_depth.tryGetAll()

        for _ in range(NUM_FRAMES):
            sparse_frames.append(q_sparse.get())
            organized_frames.append(q_organized.get())
            cam_frames.append(q_cam.get())
            custom_frames.append(q_custom.get())
            depth_frames.append(q_depth.get())

    # ------------------------------------------------------------------
    # Display results grouped by feature
    # ------------------------------------------------------------------

    # 1 ── Sparse point cloud
    print_header("1. Basic sparse point cloud")
    print("  Config: METER")
    for i, pcd in enumerate(sparse_frames):
        print_point_cloud_info(pcd, i)

    # 2 ── Organized point cloud
    print_header("2. Organized point cloud")
    print("  Config: MILLIMETER, initialConfig.setOrganized(True)")
    for i, pcd in enumerate(organized_frames):
        print_point_cloud_info(pcd, i)

    # 3 ── Transform pointcloud into another device's coordinate system
    print_header("3. Camera-to-camera transform")
    print("  Config: setTargetCoordinateSystem(CAM_A)")
    for i, pcd in enumerate(cam_frames):
        print_point_cloud_info(pcd, i)

    # 4 ── Custom transform + passthrough depth
    print_header("4. Custom transform matrix + passthrough")
    print("  Config: 90° Z rotation via initialConfig")
    for i, (pcd, depth) in enumerate(zip(custom_frames, depth_frames)):
        print_point_cloud_info(pcd, i)
        print(f"  Depth frame  : {depth.getWidth()} × {depth.getHeight()}")

    print("\nAll demos completed.")


if __name__ == "__main__":
    main()
