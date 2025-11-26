#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

model_name = "luxonis/yolov8-instance-segmentation-large:coco-640x480"
setRunOnHost = False
device = dai.Device()
if device.getPlatform() == dai.Platform.RVC2:
    model_name = "luxonis/yolov8-instance-segmentation-nano:coco-512x288"
    setRunOnHost = True

# Create pipeline
with dai.Pipeline(device) as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()

    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription(model_name))
    detectionNetwork.detectionParser.setRunOnHost(setRunOnHost)
    labelMap = detectionNetwork.getClasses()
    assert labelMap is not None
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

    def displayFrame(frame):
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
                0.7,
                255,
            )
            cv2.putText(
                frame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.7,
                255,
            )
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        return frame

    filtered_label = -1
    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            pipeline.stop()
            break

        if inRgb is not None:
            frame = inRgb.getCvFrame()

            side_panel = np.ones((frame.shape[0], 400, 3), dtype=np.uint8) * 255
            if inDet is not None:
                detections = inDet.detections
                counter += 1

                labels = sorted(list(set(detection.label for detection in detections)))

                label_maps = [labelMap[l] for l in labels]
                cv2.putText(side_panel, "Press index to filter by class:", (10, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)
                for i, label in enumerate(label_maps):
                    cv2.putText(
                        side_panel,
                        f"{i + 1} - {label}",
                        (10, 40 + i * 20),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.7,
                        (0, 0, 0),
                        1
                    )
                cv2.putText(
                    side_panel,
                    "0 - Show all",
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.7,
                    (0, 0, 0),
                    1
                )

                if key == ord('0') :
                    print("Showing all labels")
                    filtered_label = -1
                elif key > ord('0') and key <= ord('9'):
                    int_key = key - ord('1')
                    if len(labels) > int_key:
                        print(f"Showing only: {labelMap[labels[int_key]]}")
                        filtered_label = labels[int_key]

                width = inDet.getSegmentationMaskWidth()
                height = inDet.getSegmentationMaskHeight()

                segmentation_mask = cv2.Mat(np.zeros((height, width), dtype=np.uint8))
                if filtered_label == -1:
                    segmentation_mask = inDet.getCvSegmentationMask()
                else:
                    segmentation_mask = inDet.getCvSegmentationMaskByClass(filtered_label)
                    detections = [det for det in detections if det.label == filtered_label]

            if segmentation_mask is not None:
                scaled_mask = segmentation_mask.copy()
                scaled_mask[segmentation_mask != 255] = segmentation_mask[segmentation_mask != 255] * 25 # scale for better visualization
                colored_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
                colored_mask[segmentation_mask == 255] = frame[segmentation_mask == 255]
                frame = cv2.addWeighted(frame, 0.7, colored_mask, 0.3, 0)

            cv2.putText(
                frame,
                "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.4,
                color2,
            )

        if frame is not None:
            frame = displayFrame(frame)
            concatenated_frame = cv2.hconcat([frame, side_panel])
            cv2.imshow("Segmentations", concatenated_frame)
