#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    labelMap = detectionNetwork.getClasses()
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    # Linking
    monoLeftOut = monoLeft.requestOutput((1280, 720), type=dai.ImgFrame.Type.NV12)
    monoRightOut = monoRight.requestOutput((1280, 720), type=dai.ImgFrame.Type.NV12)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()
    qDepth = stereo.disparity.createOutputQueue()

    pipeline.start()

    def displayFrame(name: str, frame: dai.ImgFrame, imgDetections: dai.ImgDetections):
        color = (0, 255, 0)
        assert imgDetections.getTransformation() is not None
        cvFrame = frame.getFrame() if frame.getType() == dai.ImgFrame.Type.RAW16 else frame.getCvFrame()
        if(frame.getType() == dai.ImgFrame.Type.RAW16):
            cvFrame = (cvFrame * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
            cvFrame = cv2.applyColorMap(cvFrame, cv2.COLORMAP_JET)
        for detection in imgDetections.detections:
            # Get the shape of the frame from which the detections originated for denormalization
            normShape = imgDetections.getTransformation().getSize()

            # Create rotated rectangle to remap
            rotRect = dai.RotatedRect(dai.Rect(dai.Point2f(xmin, ymin), dai.Point2f(xmax, ymax)).denormalize(normShape[0], normShape[1]), 0)
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

