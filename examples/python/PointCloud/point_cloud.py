#!/usr/bin/env python3
"""Minimal PointCloud example: colorized point cloud from stereo depth + RGB."""

import depthai as dai

pipeline = dai.Pipeline()

# Color camera
color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

# Color output aligned to depth
colorOut = color.requestOutput((640, 400), type=dai.ImgFrame.Type.RGB888i,
                               resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

# Unified Depth node. It manages its own stereo cameras and backend, and aligns
# depth to the color output internally via setAlignTo (no ImageAlign node needed).
depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (640, 400))
depth.setAlignTo(colorOut)

# Point cloud
pc = pipeline.create(dai.node.PointCloud)
pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)

depth.depth.link(pc.inputDepth)
colorOut.link(pc.inputColor)

q = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        pcd = q.get()
        if pcd.isColor():
            xyz, rgba = pcd.getPointsRGB()
            numPoints = len(xyz)
        else:
            xyz = pcd.getPoints()
            numPoints = len(xyz)
        print(f"Points: {numPoints}, {pcd.getWidth()}x{pcd.getHeight()}, "
              f"color={pcd.isColor()}, Z=[{pcd.getMinZ():.2f}, {pcd.getMaxZ():.2f}]")
