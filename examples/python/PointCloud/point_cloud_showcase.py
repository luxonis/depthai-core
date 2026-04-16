#!/usr/bin/env python3
"""
PointCloud Node Showcase

Demonstrates filtered/organized output, camera-to-camera transforms,
housing coordinate system transforms, and custom 4x4 matrix transforms.

See examples/cpp/PointCloud/README.md for full documentation.
"""

import time
import depthai as dai


# ---------------------------------------------------------------------------
# Print helpers
# ---------------------------------------------------------------------------
def print_header(title: str) -> None:
    print("\n╔══════════════════════════════════════════════╗")
    print(f"║  {title:<44s}║")
    print("╚══════════════════════════════════════════════╝")


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

    device = dai.Device()
    print(f"Device: {device.getDeviceName()}  (ID: {device.getDeviceId()})\n")

    # ------------------------------------------------------------------
    # Single pipeline – shared Camera + StereoDepth, multiple PointCloud
    # nodes configured differently.
    # ------------------------------------------------------------------
    with dai.Pipeline(device) as pipeline:
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.requestFullResolutionOutput().link(stereo.left)
        right.requestFullResolutionOutput().link(stereo.right)

        # ── 1. Filtered point cloud  (METER) ────
        pcSparse = pipeline.create(dai.node.PointCloud)
        pcSparse.setRunOnHost(True)
        pcSparse.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        stereo.depth.link(pcSparse.inputDepth)
        qSparse = pcSparse.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        # ── 2. Organized point cloud  (MILLIMETER) ───────
        pcOrganized = pipeline.create(dai.node.PointCloud)
        pcOrganized.setRunOnHost(True)
        pcOrganized.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcOrganized.initialConfig.setOrganized(True)
        stereo.depth.link(pcOrganized.inputDepth)
        qOrganized = pcOrganized.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        # ── 3. Transform pointcloud into another device's coordinate system ───
        pcCam = pipeline.create(dai.node.PointCloud)
        pcCam.setRunOnHost(True)
        pcCam.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcCam.initialConfig.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_A)
        # Or transform to a housing coordinate system instead, e.g.:
        # pcCam.initialConfig.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        stereo.depth.link(pcCam.inputDepth)
        qCam = pcCam.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        # ── 4. Custom 4×4 transform  (90° Z rotation) + passthrough ──────
        pcCustom = pipeline.create(dai.node.PointCloud)
        pcCustom.setRunOnHost(True)
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
        qCustom = pcCustom.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
        qDepth = pcCustom.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)

        # ── 5. Colorized point cloud (aligned RGB from color camera) ─────
        pcColorized = pipeline.create(dai.node.PointCloud)
        pcColorized.setRunOnHost(True)
        pcColorized.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        colorOut = color.requestOutput(
            (640, 400), type=dai.ImgFrame.Type.RGB888i,
            resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True,
        )
        platform = pipeline.getDefaultDevice().getPlatform()
        if platform == dai.Platform.RVC4:
            imageAlign = pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(imageAlign.input)
            colorOut.link(imageAlign.inputAlignTo)
            imageAlign.outputAligned.link(pcColorized.inputDepth)
        else:
            colorOut.link(stereo.inputAlignTo)
            stereo.depth.link(pcColorized.inputDepth)
        colorOut.link(pcColorized.inputColor)
        qColorized = pcColorized.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        # Note: Housing coordinate system transform is also available, e.g.:
        #   pc.initialConfig.setTargetCoordinateSystem(dai.HousingCoordinateSystem.VESA_A)
        # See the docstring at the top of this file for all available
        # CameraBoardSocket and HousingCoordinateSystem values.

        sparseFrames = []
        organizedFrames = []
        camFrames = []
        customFrames = []
        depthFrames = []
        colorizedFrames = []

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
        qColorized.tryGetAll()

        for _ in range(NUM_FRAMES):
            sparseFrames.append(qSparse.get())
            organizedFrames.append(qOrganized.get())
            camFrames.append(qCam.get())
            customFrames.append(qCustom.get())
            depthFrames.append(qDepth.get())
            colorizedFrames.append(qColorized.get())

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
    for i, (pcd, depth) in enumerate(zip(customFrames, depthFrames, strict=True)):
        print_point_cloud_info(pcd, i)
        print(f"  Depth frame  : {depth.getWidth()} × {depth.getHeight()}")

    # 5 ── Colorized point cloud
    print_header("5. Colorized point cloud (RGB)")
    print("  Config: METER, aligned color camera linked to inputColor")
    for i, pcd in enumerate(colorizedFrames):
        print_point_cloud_info(pcd, i)

    print("\nAll demos completed.")


if __name__ == "__main__":
    main()
