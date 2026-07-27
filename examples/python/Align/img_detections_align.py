#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

SOURCE_SIZE = (1280, 600)
MASK_COLOR_SCALE = 37
TARGET_COLOR = (0, 200, 255)

def getLabel(detection) -> str:
    if detection.labelName:
        return detection.labelName
    return str(detection.label)

def overlayDetectionsMsg(frame: np.ndarray, detectionsMsg: dai.ImgDetections) -> np.ndarray:
    segmentationMask = detectionsMsg.getCvSegmentationMask()
    output = frame.copy()
    if segmentationMask is not None:
        scaledMask = np.zeros_like(segmentationMask, dtype=np.uint8)
        validMask = segmentationMask != 255
        scaledMask[validMask] = ((segmentationMask[validMask].astype(np.uint16) * MASK_COLOR_SCALE) % 256).astype(np.uint8)
        coloredMask = cv2.applyColorMap(scaledMask, cv2.COLORMAP_JET)
        coloredMask[~validMask] = output[~validMask]

        output = cv2.addWeighted(output, 0.7, coloredMask, 0.3, 0)

    for detection in detectionsMsg.detections:
        bbox = detection.getBoundingBox().denormalize(frame.shape[1], frame.shape[0])
        points = np.array([[int(point.x), int(point.y)] for point in bbox.getPoints()], dtype=np.int32)
        anchor = tuple(points[0])

        cv2.polylines(output, [points], isClosed=True, color=TARGET_COLOR, thickness=2)
        cv2.putText(output, getLabel(detection), (anchor[0] + 8, anchor[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, TARGET_COLOR)
        cv2.putText(
            output,
            f"{int(detection.confidence * 100)}%",
            (anchor[0] + 8, anchor[1] + 40),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            TARGET_COLOR,
        )
    return output

device = dai.Device()
if device.getPlatform() == dai.Platform.RVC2:
    raise RuntimeError("Align is not supported on the RVC2 platform.")

fps = 30.0
modelName = "luxonis/yolov8-instance-segmentation-large:coco-640x480"
capability = dai.ImgFrameCapability()
capability.resizeMode = dai.ImgResizeMode.STRETCH
capability.enableUndistortion = False

with dai.Pipeline(device) as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=fps)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(modelName), capability)

    alignToSource = camera.requestOutput(
        size=SOURCE_SIZE,
        fps=fps,
        resizeMode=dai.ImgResizeMode.CROP,
        enableUndistortion=True,
    )

    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.addRotateDeg(30.0)
    manip.initialConfig.setOutputSize(800, 600)
    manip.initialConfig.setBackgroundColor(255, 255, 255)
    manip.setMaxOutputFrameSize(800 * 600 * 3)
    alignToSource.link(manip.inputImage)

    align = pipeline.create(dai.node.Align)
    detectionNetwork.out.link(align.input)
    manip.out.link(align.inputAlignTo)

    sourceFrame = detectionNetwork.passthrough.createOutputQueue()
    alignToFrame = manip.out.createOutputQueue()
    alignedDetections = align.outputAligned.createOutputQueue()
    sourceDetections = detectionNetwork.out.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        sourceFrameMsg = sourceFrame.get()
        alignToFrameMsg  = alignToFrame.get()
        sourceDetectionsMsg = sourceDetections.get()
        alignedDetectionsMsg = alignedDetections.get()

        assert isinstance(sourceFrameMsg, dai.ImgFrame)
        assert isinstance(alignToFrameMsg, dai.ImgFrame)
        assert isinstance(sourceDetectionsMsg, dai.ImgDetections)
        assert isinstance(alignedDetectionsMsg, dai.ImgDetections)

        sourceImg = overlayDetectionsMsg(sourceFrameMsg.getCvFrame(), sourceDetectionsMsg)
        alignedImg = overlayDetectionsMsg(alignToFrameMsg.getCvFrame(), alignedDetectionsMsg)

        cv2.imshow("Source frame", sourceImg)
        cv2.imshow("AlignTo frame", alignedImg)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

cv2.destroyAllWindows()
