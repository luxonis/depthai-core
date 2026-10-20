#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    colorSockets = pipeline.getDefaultDevice().getConnectedCameras(dai.CameraSensorType.COLOR)
    colorSocket = colorSockets[0] if colorSockets else dai.CameraBoardSocket.CAM_A
    cameraNode = pipeline.create(dai.node.Camera).build(colorSocket)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    objectTracker = pipeline.create(dai.node.ObjectTracker)
    labelMap = detectionNetwork.getClasses()
    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO)

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
        if(frame.getType() == dai.ImgFrame.Type.RAW16):
            cvFrame = dai.utility.colorizeDepthFrame(frame).getCvFrame()
        else:
            cvFrame = frame.getCvFrame()
        for tracklet in tracklets.tracklets:
            # Get the shape of the frame from which the detections originated for denormalization
            normShape = tracklets.getTransformation().getSize()

            # Create rotated rectangle to remap
            # Here we use an intermediate dai.Rect to create a dai.RotatedRect to simplify construction and denormalization
            rotRect = dai.RotatedRect(tracklet.roi.denormalize(normShape[0], normShape[1]), 0)
            # Remap the detection rectangle to target frame
            remapped = tracklets.getTransformation().remapRectTo(frame.getTransformation(), rotRect)
            # Remapped rectangle could be rotated, so we get the bounding box
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
        # Show the frame
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
