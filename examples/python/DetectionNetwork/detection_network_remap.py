#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline(dai.Device("10.12.103.170")) as pipeline:
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

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def denormalize(shape, bbox):
        return (int(bbox[0] * shape[0]), int(bbox[1] * shape[1]), int(bbox[2] * shape[0]), int(bbox[3] * shape[1]))

    def displayFrame(name: str, frame: dai.ImgFrame, imgDetections: dai.ImgDetections):
        color = (255, 0, 0)
        assert imgDetections.getTransformation() is not None
        for detection in imgDetections.detections:
            (xmin, ymin, xmax, ymax) = denormalize((frame.getWidth(), frame.getHeight()), (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            rotRect = dai.RotatedRect()
            rotRect.center.x = xmin + (xmax - xmin) / 2
            rotRect.center.y = ymin + (ymax - ymin) / 2
            rotRect.size.width = xmax - xmin
            rotRect.size.height = ymax - ymin
            remapped = imgDetections.getTransformation().remapRectTo(frame.getTransformation(), rotRect)
            bbox = remapped.getOuterRect()
            cv2.putText(
                frame,
                labelMap[detection.label],
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
        # Show the frame
        cv2.imshow(name, frame)

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        inDepth: dai.ImgFrame = qDepth.get()
        hasRgb = inRgb is not None
        hasDepth = inDepth is not None
        hasDet = inDet is not None
        if hasRgb:
            displayFrame("rgb", inRgb, inDet.detections)
        if hasDepth:
            displayFrame("depth", inDepth, inDet.detections)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

