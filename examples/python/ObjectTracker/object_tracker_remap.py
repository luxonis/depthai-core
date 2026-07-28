#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.zeros_like(frameDepth, dtype=np.float32)
        np.log(frameDepth, where=frameDepth != 0, out=logDepth)
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
    objectTracker = pipeline.create(dai.node.ObjectTracker)
    labelMap = detectionNetwork.getClasses()
    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (1280, 720))

    detectionNetwork.out.link(objectTracker.inputDetections)
    detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qTrack = objectTracker.out.createOutputQueue()
    qDepth = depth.depth.createOutputQueue()

    pipeline.start()

    def displayFrame(name: str, frame: dai.ImgFrame, tracklets: dai.Tracklets):
        color = (0, 255, 0)
        assert tracklets.getTransformation() is not None
        cvFrame = frame.getFrame() if frame.getType() == dai.ImgFrame.Type.RAW16 else frame.getCvFrame()
        if(frame.getType() == dai.ImgFrame.Type.RAW16):
            cvFrame = colorizeDepth(cvFrame)
        for tracklet in tracklets.tracklets:
            normShape = tracklets.getTransformation().getSize()

            rotRect = dai.RotatedRect(tracklet.roi.denormalize(normShape[0], normShape[1]), 0)
            remapped = tracklets.getTransformation().remapRectTo(frame.getTransformation(), rotRect)
            bbox = [int(l) for l in remapped.getOuterRect()]
            cv2.putText(
                cvFrame,
                labelMap[tracklet.label],
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.putText(
                cvFrame,
                f"{int(tracklet.srcImgDetection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.rectangle(cvFrame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        cv2.imshow(name, cvFrame)

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inTrack: dai.Tracklets = qTrack.get()
        inDepth: dai.ImgFrame = qDepth.get()
        hasRgb = inRgb is not None
        hasDepth = inDepth is not None
        hasTrack = inTrack is not None
        if hasRgb:
            displayFrame("rgb", inRgb, inTrack)
        if hasDepth:
            displayFrame("depth", inDepth, inTrack)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
