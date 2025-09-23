#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    # Log the depth, minDepth and maxDepth
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        # Clip the values to be in the 0-255 range
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Interpolate only valid logDepth values, setting the rest based on the mask
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    objectTracker = pipeline.create(dai.node.ObjectTracker)
    labelMap = detectionNetwork.getClasses()
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    # Linking
    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    detectionNetwork.out.link(objectTracker.inputDetections)
    detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)


    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qTrack = objectTracker.out.createOutputQueue()
    qDepth = stereo.disparity.createOutputQueue()

    pipeline.start()

    def displayFrame(name: str, frame: dai.ImgFrame, tracklets: dai.Tracklets):
        color = (0, 255, 0)
        assert tracklets.getTransformation() is not None
        cvFrame = frame.getFrame() if frame.getType() == dai.ImgFrame.Type.RAW16 else frame.getCvFrame()
        if(frame.getType() == dai.ImgFrame.Type.RAW16):
            cvFrame = colorizeDepth(cvFrame)
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

