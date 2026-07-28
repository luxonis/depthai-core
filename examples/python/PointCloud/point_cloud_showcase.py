#!/usr/bin/env python3
"""
PointCloud Node Showcase

Demonstrates filtered/organized output, camera-to-camera transforms,
housing coordinate system transforms, and custom 4x4 matrix transforms.

See examples/cpp/PointCloud/README.md for full documentation.
"""

import time
import depthai as dai


def printHeader(title: str) -> None:
    print("\n╔══════════════════════════════════════════════╗")
    print(f"║  {title:<44s}║")
    print("╚══════════════════════════════════════════════╝")


def printPointCloudInfo(pcd: dai.PointCloudData, frameNum: int) -> None:
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


NUM_FRAMES = 3


def main() -> None:
    print("PointCloud Node Showcase")
    print("========================")
    print("Connecting to device...")

    device = dai.Device()
    print(f"Device: {device.getDeviceName()}  (ID: {device.getDeviceId()})\n")

    with dai.Pipeline(device) as pipeline:
        colorSocket = dai.CameraBoardSocket.CAM_A
        for features in device.getConnectedCameraFeatures():
            if dai.CameraSensorType.COLOR in features.supportedTypes:
                colorSocket = features.socket
                break
        color = pipeline.create(dai.node.Camera).build(colorSocket)
        colorOut = color.requestOutput(
            (640, 400), type=dai.ImgFrame.Type.RGB888i,
            resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True,
        )

        depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (640, 400))
        depth.setAlignTo(colorOut)

        pcSparse = pipeline.create(dai.node.PointCloud)
        pcSparse.setRunOnHost(True)
        pcSparse.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        depth.depth.link(pcSparse.inputDepth)
        qSparse = pcSparse.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        pcOrganized = pipeline.create(dai.node.PointCloud)
        pcOrganized.setRunOnHost(True)
        pcOrganized.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcOrganized.initialConfig.setOrganized(True)
        depth.depth.link(pcOrganized.inputDepth)
        qOrganized = pcOrganized.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

        pcCam = pipeline.create(dai.node.PointCloud)
        pcCam.setRunOnHost(True)
        pcCam.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        pcCam.initialConfig.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_A)
        depth.depth.link(pcCam.inputDepth)
        qCam = pcCam.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

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
        depth.depth.link(pcCustom.inputDepth)
        qCustom = pcCustom.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
        qDepth = pcCustom.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)

        pcColorized = pipeline.create(dai.node.PointCloud)
        pcColorized.setRunOnHost(True)
        pcColorized.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        depth.depth.link(pcColorized.inputDepth)
        colorOut.link(pcColorized.inputColor)
        qColorized = pcColorized.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)


        sparseFrames = []
        organizedFrames = []
        camFrames = []
        customFrames = []
        depthFrames = []
        colorizedFrames = []

        pipeline.start()

        print("Waiting for auto-exposure to settle...")
        time.sleep(1)

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


    printHeader("1. Basic sparse point cloud")
    print("  Config: METER")
    for i, pcd in enumerate(sparseFrames):
        printPointCloudInfo(pcd, i)

    printHeader("2. Organized point cloud")
    print("  Config: MILLIMETER, initialConfig.setOrganized(True)")
    for i, pcd in enumerate(organizedFrames):
        printPointCloudInfo(pcd, i)

    printHeader("3. Camera-to-camera transform")
    print("  Config: setTargetCoordinateSystem(CAM_A)")
    for i, pcd in enumerate(camFrames):
        printPointCloudInfo(pcd, i)

    printHeader("4. Custom transform matrix + passthrough")
    print("  Config: 90° Z rotation via initialConfig")
    if len(customFrames) != len(depthFrames):
        raise RuntimeError("customFrames and depthFrames must have the same length")
    for i, (pcd, depth) in enumerate(zip(customFrames, depthFrames)):
        printPointCloudInfo(pcd, i)
        print(f"  Depth frame  : {depth.getWidth()} × {depth.getHeight()}")

    printHeader("5. Colorized point cloud (RGB)")
    print("  Config: METER, aligned color camera linked to inputColor")
    for i, pcd in enumerate(colorizedFrames):
        printPointCloudInfo(pcd, i)

    print("\nAll demos completed.")


if __name__ == "__main__":
    main()
