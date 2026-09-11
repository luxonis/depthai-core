#!/usr/bin/env python3
"""Fused ToF and neural depth."""

import cv2
import depthai as dai


with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
    fusion = pipeline.create(dai.beta.node.ToFStereoFusion).build(left, right)
    fusion.neuralDepth.initialConfig.setConfidenceThreshold(0)
    fusion.initialConfig.confidenceThreshold = 0.0

    benchmark = pipeline.create(dai.node.BenchmarkIn)
    benchmark.sendReportEveryNMessages(30)
    benchmark.logReportsAsWarnings(True)
    fusion.depth.link(benchmark.input)

    depth_queue = fusion.depth.createOutputQueue()

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
    cv2.namedWindow("Fused depth", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Fusion confidence (%)", "Fused depth", 0, 100, set_fusion_confidence)
    cv2.createTrackbar("Neural confidence", "Fused depth", 0, 255, set_neural_confidence)
    cv2.createTrackbar("Temporal filter", "Fused depth",
                       int(fusion.tof.tofBaseNode.initialConfig.enablePhaseShuffleTemporalFilter), 1, set_temporal_filter)
    while pipeline.isRunning():
        depth = depth_queue.get()
        cv2.imshow("Fused depth", dai.utility.colorizeDepthFrame(depth).getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
