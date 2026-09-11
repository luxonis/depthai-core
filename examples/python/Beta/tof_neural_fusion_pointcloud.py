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
    fusion.initialConfig.confidenceThreshold = 0.0
    fusion.neuralDepth.initialConfig.setConfidenceThreshold(0)

    colorizer = pipeline.create(DepthColorizer)
    fusion.depth.link(colorizer.input)

    pointcloud = pipeline.create(dai.node.PointCloud)
    pointcloud.setRunOnHost(True)
    fusion.depth.link(pointcloud.inputDepth)
    colorizer.output.link(pointcloud.inputColor)

    remote = dai.RemoteConnection()
    remote.addTopic("pcl", pointcloud.outputPointCloud, "3d")

    fusion_config_queue = fusion.inputConfig.createInputQueue()
    neural_config_queue = fusion.neuralDepth.inputConfig.createInputQueue()
    tof_config_queue = fusion.tof.tofBaseNode.inputConfig.createInputQueue()

    def set_fusion_confidence(value):
        fusion.initialConfig.confidenceThreshold = value / 100.0
        fusion_config_queue.send(fusion.initialConfig)

    def set_neural_confidence(value):
        fusion.neuralDepth.initialConfig.setConfidenceThreshold(value)
        neural_config_queue.send(fusion.neuralDepth.initialConfig)

    def set_temporal_filter(value):
        fusion.tof.tofBaseNode.initialConfig.enablePhaseShuffleTemporalFilter = bool(value)
        tof_config_queue.send(fusion.tof.tofBaseNode.initialConfig)

    pipeline.start()
    cv2.namedWindow("Confidence thresholds", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Fusion confidence (%)", "Confidence thresholds", 0, 100, set_fusion_confidence)
    cv2.createTrackbar("Neural confidence", "Confidence thresholds", 0, 255, set_neural_confidence)
    cv2.createTrackbar("Temporal filter", "Confidence thresholds",
                       int(fusion.tof.tofBaseNode.initialConfig.enablePhaseShuffleTemporalFilter), 1, set_temporal_filter)
    remote.registerPipeline(pipeline)
    print("Visualizer running at http://localhost:8082")

    while pipeline.isRunning():
        if cv2.waitKey(1) == ord("q") or remote.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
