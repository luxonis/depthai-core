#!/usr/bin/env python3
"""Minimal PointCloud example: colorized point cloud from stereo depth + RGB."""

import depthai as dai

pipeline = dai.Pipeline()

# Cameras
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

# Stereo depth
stereo = pipeline.create(dai.node.StereoDepth)
left.requestFullResolutionOutput().link(stereo.left)
right.requestFullResolutionOutput().link(stereo.right)

# Color output aligned to depth
colorOut = color.requestOutput((640, 400), type=dai.ImgFrame.Type.RGB888i,
                               resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

# Point cloud
pc = pipeline.create(dai.node.PointCloud)
pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)

# Align depth to color on RVC4
platform = pipeline.getDefaultDevice().getPlatform()
if platform == dai.Platform.RVC4:
    imageAlign = pipeline.create(dai.node.ImageAlign)
    stereo.depth.link(imageAlign.input)
    colorOut.link(imageAlign.inputAlignTo)
    imageAlign.outputAligned.link(pc.inputDepth)
else:
    colorOut.link(stereo.inputAlignTo)
    stereo.depth.link(pc.inputDepth)

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
