#!/usr/bin/env python3

import depthai as dai


with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build()
    queue = camera.requestOutput((640, 400)).createOutputQueue()
    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (640, 400))
    point_cloud = pipeline.create(dai.node.PointCloud)
    depth.depth.link(point_cloud.inputDepth)
    point_cloud_queue = point_cloud.outputPointCloud.createOutputQueue()

    pipeline.start()
    queue.get().save("./frame.dai")
    point_cloud_queue.get().save("./pointcloud.dai")

frame = dai.ImgFrame()
frame.load("./frame.dai")
print(f"Loaded frame: {frame.getWidth()}x{frame.getHeight()}")

point_cloud = dai.PointCloudData()
point_cloud.load("./pointcloud.dai")
print(f"Loaded point cloud: {len(point_cloud.getPoints())} points")
