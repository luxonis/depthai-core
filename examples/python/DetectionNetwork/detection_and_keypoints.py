#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

device = dai.Device()
fps = 30
model_name = "luxonis/yolov8-large-pose-estimation:coco-640x352"
if device.getPlatform() == dai.Platform.RVC2:
    model_name = "luxonis/yolov8-nano-pose-estimation:coco-512x288"
    fps = 12

# Create pipeline
with dai.Pipeline(device) as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(sensorFps=fps)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription(model_name))
    labelMap = detectionNetwork.getClasses()

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()

    pipeline.start()

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame):
        color = (255, 0, 0)
        for detection in detections:
            bbox = frameNorm(
                frame,
                (detection.xmin, detection.ymin, detection.xmax, detection.ymax),
            )
            cv2.putText(
                frame,
                detection.labelName,
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.putText(
                frame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)

            keypoints = detection.getKeypoints()
            for keypoint in keypoints:
                keypoint_pos = frameNorm(
                    frame,
                    (keypoint.imageCoordinates.x, keypoint.imageCoordinates.y),
                )
                cv2.circle(frame, (keypoint_pos[0], keypoint_pos[1]), 3, (0, 255, 0), -1)
                if keypoint.labelName != "":
                    cv2.putText(
                        frame,
                        keypoint.labelName,
                        (keypoint_pos[0] + 5, keypoint_pos[1] - 5),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.4,
                        (0, 255, 0),
                    )

            for edge in detection.getEdges():
                kp1 = keypoints[edge[0]]
                kp2 = keypoints[edge[1]]
                kp1_pos = frameNorm(
                    frame,
                    (kp1.imageCoordinates.x, kp1.imageCoordinates.y),
                )
                kp2_pos = frameNorm(
                    frame,
                    (kp2.imageCoordinates.x, kp2.imageCoordinates.y),
                )
                cv2.line(frame, (kp1_pos[0], kp1_pos[1]), (kp2_pos[0], kp2_pos[1]), (0, 255, 0), 2)
        # Show the frame
        cv2.imshow(name, frame)


    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            cv2.putText(
                frame,
                "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.4,
                color2,
            )

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if frame is not None:
            displayFrame("rgb", frame)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break