#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        depthFrameColor[invalidMask] = 0
    except IndexError:
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor

with dai.Pipeline() as pipeline:
    colorSocket = dai.CameraBoardSocket.CAM_A
    for features in pipeline.getDefaultDevice().getConnectedCameraFeatures():
        if dai.CameraSensorType.COLOR in features.supportedTypes:
            colorSocket = features.socket
            break
    cameraNode = pipeline.create(dai.node.Camera).build(colorSocket)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    labelMap = detectionNetwork.getClasses()
    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (1280, 720))

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()
    qDepth = depth.depth.createOutputQueue()

    pipeline.start()

    def displayFrame(name: str, frame: dai.ImgFrame, imgDetections: dai.ImgDetections):
        color = (0, 255, 0)
        assert imgDetections.getTransformation() is not None
        cvFrame = frame.getFrame() if frame.getType() == dai.ImgFrame.Type.RAW16 else frame.getCvFrame()
        if(frame.getType() == dai.ImgFrame.Type.RAW16):
            cvFrame = colorizeDepth(cvFrame)
        for detection in imgDetections.detections:
            normShape = imgDetections.getTransformation().getSize()

            rotRect = dai.RotatedRect(dai.Rect(dai.Point2f(detection.xmin, detection.ymin), dai.Point2f(detection.xmax, detection.ymax)).denormalize(normShape[0], normShape[1]), 0)
            remapped = imgDetections.getTransformation().remapRectTo(frame.getTransformation(), rotRect)
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
        cv2.imshow(name, cvFrame)

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        inDepth: dai.ImgFrame = qDepth.get()
        hasRgb = inRgb is not None
        hasDepth = inDepth is not None
        hasDet = inDet is not None
        if hasRgb:
            displayFrame("rgb", inRgb, inDet)
        if hasDepth:
            displayFrame("depth", inDepth, inDet)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

