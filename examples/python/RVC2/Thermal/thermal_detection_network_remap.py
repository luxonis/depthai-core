#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    labelMap = detectionNetwork.getClasses()
    thermal = pipeline.create(dai.node.Thermal).build()

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()
    qTemp = thermal.color.createOutputQueue()

    pipeline.start()

    def displayFrame(name: str, frame: dai.ImgFrame, imgDetections: dai.ImgDetections):
        color = (0, 255, 0)
        assert imgDetections.getTransformation() is not None
        cvFrame = frame.getCvFrame()
        for detection in imgDetections.detections:
            # Get the shape of the frame from which the detections originated for denormalization
            normShape = imgDetections.getTransformation().getSize()
            # Create rotated rectangle to remap
            # Here we use an intermediate dai.Rect to create a dai.RotatedRect to simplify construction and denormalization
            rotRect = dai.RotatedRect(dai.Rect(dai.Point2f(detection.xmin, detection.ymin), dai.Point2f(detection.xmax, detection.ymax)).denormalize(normShape[0], normShape[1]), 0)
            # Remap the detection rectangle to target frame
            remapped = imgDetections.getTransformation().remapRectTo(frame.getTransformation(), rotRect)
            # Remapped rectangle could be rotated, so we get the bounding box
            bbox = [int(l) for l in remapped.getOuterRect()]
            cv2.putText(
                cvFrame,
                labelMap[detection.label],
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.putText(
                cvFrame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.rectangle(cvFrame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # Show the frame
        cv2.imshow(name, cvFrame)

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        inTemp: dai.ImgFrame = qTemp.get()
        hasRgb = inRgb is not None
        hasTemp = inTemp is not None
        hasDet = inDet is not None
        if hasRgb:
            displayFrame("rgb", inRgb, inDet)
        if hasTemp:
            displayFrame("temp", inTemp, inDet)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

