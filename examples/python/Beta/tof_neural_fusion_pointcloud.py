#!/usr/bin/env python3
"""Fused ToF and neural-depth point cloud in the DepthAI Visualizer.

Open http://localhost:8082 after starting this script.
"""

import cv2
import depthai as dai


class DepthColorizer(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.output = self.createOutput()

    def run(self):
        while self.isRunning():
            colorized = dai.utility.colorizeDepthFrame(self.input.get())
            colorized.setCvFrame(cv2.cvtColor(colorized.getCvFrame(), cv2.COLOR_BGR2RGB), dai.ImgFrame.Type.RGB888i)
            self.output.send(colorized)


with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
    fusion = pipeline.create(dai.beta.node.ToFStereoFusion).build(left, right)
    fusion.initialConfig.confidenceThreshold = 0.3

    colorizer = pipeline.create(DepthColorizer)
    fusion.depth.link(colorizer.input)

    pointcloud = pipeline.create(dai.node.PointCloud)
    pointcloud.setRunOnHost(True)
    fusion.depth.link(pointcloud.inputDepth)
    colorizer.output.link(pointcloud.inputColor)

    remote = dai.RemoteConnection()
    remote.addTopic("pcl", pointcloud.outputPointCloud, "3d")

    pipeline.start()
    remote.registerPipeline(pipeline)
    print("Visualizer running at http://localhost:8082")

    while pipeline.isRunning():
        if remote.waitKey(1) == ord("q"):
            break
