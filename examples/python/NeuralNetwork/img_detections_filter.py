#!/usr/bin/env python3
"""Keep confident person detections from one camera; press q to exit."""
import argparse
from datetime import timedelta

import cv2
import depthai as dai

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model", default="yolov6-nano", help="Model zoo slug using COCO labels")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=10)
    network = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model), fps=10)
    detections = pipeline.create(dai.node.ImgDetectionsFilter)
    detections.initialConfig.labelsToKeep = [0]  # COCO person
    detections.initialConfig.setConfidenceRange(0.6)
    detections.initialConfig.maxDetections = 10
    network.out.link(detections.inputs["cam"])

    display = pipeline.create(dai.node.Sync)
    display.setRunOnHost(True)
    display.setSyncThreshold(timedelta(milliseconds=30))
    network.passthrough.link(display.inputs["image"])
    detections.out.link(display.inputs["detections"])
    queue = display.out.createOutputQueue()
    pipeline.start()
    while pipeline.isRunning():
        group = queue.tryGet()
        if group is not None:
            frame = group["image"].getCvFrame()
            for detection in group["detections"].detections:
                box = detection.getBoundingBox().denormalize(frame.shape[1], frame.shape[0])
                points = box.getPoints()
                for a, b in zip(points, points[1:] + points[:1]):
                    cv2.line(frame, (round(a.x), round(a.y)), (round(b.x), round(b.y)), (0, 255, 0), 2)
            cv2.imshow("Filtered people", frame)
        if cv2.waitKey(1) == ord("q"):
            break
    cv2.destroyAllWindows()
