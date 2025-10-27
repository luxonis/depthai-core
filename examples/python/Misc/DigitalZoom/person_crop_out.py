#!/usr/bin/env python3

import cv2
import depthai as dai
import time

# This example shows how to crop out the biggest person from the maximum resolution of the camera and setting the autofocus and autoexposure region to the same ROI.


def getBiggestPerson(imgDetections: dai.ImgDetections):
    biggestDetection = None
    biggestDetectionSize = 0
    for detection in imgDetections.detections:
        if detection.label == 0: # Person
            size = (detection.xmax - detection.xmin) * (detection.ymax - detection.ymin)
            if size > biggestDetectionSize:
                biggestDetection = detection
                biggestDetectionSize = size
    return biggestDetection


def displayFrame(name: str, frame: dai.ImgFrame, imgDetections: dai.ImgDetections, labelMap: dict):
    color = (0, 255, 0)
    assert imgDetections.getTransformation() is not None
    cvFrame = frame.getFrame() if frame.getType() == dai.ImgFrame.Type.RAW16 else frame.getCvFrame()
    for detection in imgDetections.detections:
        # Get the shape of the frame from which the detections originated for denormalization
        normShape = imgDetections.getTransformation().getSize()

        # Create rotated rectangle to remap
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


def transformDetectionToSource(imgDetections: dai.ImgDetections, detection: dai.ImgDetection):
    normShape = imgDetections.getTransformation().getSize()
    sourceSize = imgDetections.getTransformation().getSourceSize()
    width, height = sourceSize
    rotRect = dai.RotatedRect(dai.Rect(dai.Point2f(detection.xmin, detection.ymin), dai.Point2f(detection.xmax, detection.ymax)).denormalize(normShape[0], normShape[1]), 0)
    rotRect = imgDetections.getTransformation().invTransformRect(rotRect)
    outerRect = rotRect.getOuterRect()

    firstPoint = dai.Point2f(max(0, min(outerRect[0], width)), max(0, min(outerRect[1], height)))
    secondPoint = dai.Point2f(max(0, min(outerRect[2], width)), max(0, min(outerRect[3], height)))
    return dai.Rect(firstPoint, secondPoint)


device = dai.Device()
modelPath = dai.getModelFromZoo(dai.NNModelDescription("yolov6-nano", platform=device.getPlatformAsString()))
modelArchive = dai.NNArchive(modelPath)
inputSize = modelArchive.getInputSize()
type = modelArchive.getConfig().model.inputs[0].preprocessing.daiType

if type:
    try:
        frameType = getattr(dai.ImgFrame.Type, type)
    except AttributeError:
        type = None

if not type:
    if device.getPlatform() == dai.Platform.RVC2:
        frameType = dai.ImgFrame.Type.BGR888p
    else:
        frameType = dai.ImgFrame.Type.BGR888i

# Create pipeline
with dai.Pipeline(device) as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    cameraControlQueue = cam.inputControl.createInputQueue()
    fullResStream = cam.requestFullResolutionOutput(useHighestResolution=True)

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setOutputSize(inputSize[0], inputSize[1])
    imageManip.initialConfig.setFrameType(frameType)

    fullResStream.link(imageManip.inputImage)

    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(imageManip.out, modelArchive)
    labelMap = detectionNetwork.getClasses()

    imageManipCropOut = pipeline.create(dai.node.ImageManip)
    imageManipCropOut.setMaxOutputFrameSize(round(1000*1000*1.5)+300)
    imageManipCropOut.initialConfig.setOutputSize(800, 800)
    imageManipCropOut.inputImage.setBlocking(False)
    imageManipCropOut.inputImage.setMaxSize(1)

    imageManipConfigQueue = imageManipCropOut.inputConfig.createInputQueue()
    imageManipCropOutQueue = imageManipCropOut.out.createOutputQueue()
    fullResStream.link(imageManipCropOut.inputImage)

    videoQueue = detectionNetwork.passthrough.createOutputQueue()
    detectionQueue = detectionNetwork.out.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    lastTimeToAutoFocus = time.time()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        detections = detectionQueue.get()
        biggestDetection = getBiggestPerson(detections)
        if biggestDetection:
            sourceRect = transformDetectionToSource(detections, biggestDetection)
            configQueue = dai.ImageManipConfig()
            configQueue.addCrop(sourceRect, False)
            configQueue.setOutputSize(800, 800, dai.ImageManipConfig.ResizeMode.LETTERBOX)
            imageManipConfigQueue.send(configQueue)

            configCamera = dai.CameraControl()
            configCamera.setAutoExposureRegion(int(sourceRect.x), int(sourceRect.y), int(sourceRect.width), int(sourceRect.height))
            if(time.time() - lastTimeToAutoFocus > 5):
                lastTimeToAutoFocus = time.time()
                configCamera.setAutoFocusRegion(int(sourceRect.x), int(sourceRect.y), int(sourceRect.width), int(sourceRect.height))
            cameraControlQueue.send(configCamera)
        imageManipCropOutFrame = imageManipCropOutQueue.tryGet()
        if imageManipCropOutFrame is not None:
            assert isinstance(imageManipCropOutFrame, dai.ImgFrame)
            cv2.imshow("Cropped out frame", imageManipCropOutFrame.getCvFrame())
        assert isinstance(videoIn, dai.ImgFrame)
        assert isinstance(detections, dai.ImgDetections)
        displayFrame("Full view video", videoIn, detections, labelMap)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
