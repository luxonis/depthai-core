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


def print_point_cloud_info(pcd: dai.PointCloudData, frameNum: int) -> None:
    points = pcd.getPoints()
    print(f"\n--- Frame {frameNum} ---")
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
    deviceIp = "10.11.0.29"
    deviceInfo = dai.DeviceInfo(deviceIp)
    device = dai.Device(deviceInfo)

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
        pcSparse = pipeline.create(dai.node.PointCloud)
        pcSparse.setRunOnHost(False)
        pcSparse.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        stereo.depth.link(pcSparse.inputDepth)
        qSparse = pcSparse.outputPointCloud.createOutputQueue()

        # ── 2. Organized point cloud  (MILLIMETER, single-threaded) ───────
        pcOrganized = pipeline.create(dai.node.PointCloud)
        pcOrganized.setRunOnHost(False)
        pcOrganized.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcOrganized.initialConfig.setOrganized(True)
        stereo.depth.link(pcOrganized.inputDepth)
        qOrganized = pcOrganized.outputPointCloud.createOutputQueue()

        # ── 3. Transform pointcloud into another device's coordinate system ───
        pcCam = pipeline.create(dai.node.PointCloud)
        pcCam.setRunOnHost(False)
        pcCam.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcCam.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_A)
        # Or transform to a housing coordinate system instead, e.g.:
        # pcCam.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        stereo.depth.link(pcCam.inputDepth)
        qCam = pcCam.outputPointCloud.createOutputQueue()

        # ── 4. Custom 4×4 transform  (90° Z rotation) + passthrough ──────
        pcCustom = pipeline.create(dai.node.PointCloud)
        pcCustom.setRunOnHost(False)
        pcCustom.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcCustom.useCPU()
        transform = [
            [0.0, -1.0, 0.0, 0.0],
            [1.0,  0.0, 0.0, 0.0],
            [0.0,  0.0, 1.0, 0.0],
            [0.0,  0.0, 0.0, 1.0],
        ]
        pcCustom.initialConfig.setTransformationMatrix(transform)
        stereo.depth.link(pcCustom.inputDepth)
        qCustom = pcCustom.outputPointCloud.createOutputQueue()
        qDepth = pcCustom.passthroughDepth.createOutputQueue()

        # Note: Housing coordinate system transform is also available, e.g.:
        #   pc.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        # See the docstring at the top of this file for all available
        # CameraBoardSocket and HousingCoordinateSystem values.

        sparseFrames = []
        organizedFrames = []
        camFrames = []
        customFrames = []
        depthFrames = []

        pipeline.start()

        # Wait for auto-exposure to settle and stereo depth to stabilize
        print("Waiting for auto-exposure to settle...")
        time.sleep(1)

        # Drain stale frames that arrived during warm-up
        qSparse.tryGetAll()
        qOrganized.tryGetAll()
        qCam.tryGetAll()
        qCustom.tryGetAll()
        qDepth.tryGetAll()

        for _ in range(NUM_FRAMES):
            sparseFrames.append(qSparse.get())
            organizedFrames.append(qOrganized.get())
            camFrames.append(qCam.get())
            customFrames.append(qCustom.get())
            depthFrames.append(qDepth.get())

    # ------------------------------------------------------------------
    # Display results grouped by feature
    # ------------------------------------------------------------------

    # 1 ── Sparse point cloud
    print_header("1. Basic sparse point cloud")
    print("  Config: METER")
    for i, pcd in enumerate(sparseFrames):
        print_point_cloud_info(pcd, i)

    # 2 ── Organized point cloud
    print_header("2. Organized point cloud")
    print("  Config: MILLIMETER, initialConfig.setOrganized(True)")
    for i, pcd in enumerate(organizedFrames):
        print_point_cloud_info(pcd, i)

    # 3 ── Transform pointcloud into another device's coordinate system
    print_header("3. Camera-to-camera transform")
    print("  Config: setTargetCoordinateSystem(CAM_A)")
    for i, pcd in enumerate(camFrames):
        print_point_cloud_info(pcd, i)

    # 4 ── Custom transform + passthrough depth
    print_header("4. Custom transform matrix + passthrough")
    print("  Config: 90° Z rotation via initialConfig")
    for i, (pcd, depth) in enumerate(zip(customFrames, depthFrames)):
        print_point_cloud_info(pcd, i)
        print(f"  Depth frame  : {depth.getWidth()} × {depth.getHeight()}")

    print("\nAll demos completed.")


if __name__ == "__main__":
    main()
