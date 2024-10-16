#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
class ImageAnnotationsGenerator(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.output = self.createOutput()

    def run(self):
        while self.isRunning():
            nnData = self.input.get()
            if nnData is not None:
                detections = nnData.detections
                imgAnnt = dai.ImageAnnotations()
                imgAnnt.setTimestamp(nnData.getTimestamp())
                annotation = dai.ImageAnnotation()
                annotation.points = [dai.PointsAnnotation()] * len(detections)
                annotation.texts = [dai.TextAnnotation()] * len(detections)
                for i in range(len(detections)):
                    # create ImageAnnotations message
                    annotation.points[i].type = dai.PointsAnnotationType.LINE_STRIP
                    annotation.points[i].points = [
                        dai.Point2f(detections[i].xmin, detections[i].ymin),
                        dai.Point2f(detections[i].xmax, detections[i].ymin),
                        dai.Point2f(detections[i].xmax, detections[i].ymax),
                        dai.Point2f(detections[i].xmin, detections[i].ymax),
                    ]
                    outlineColor = dai.Color(0.5, 0.5, 0.5, 1.0)
                    annotation.points[i].outlineColor = outlineColor

                    fillColor = dai.Color(0.5, 0.5, 0.5, 0.5)
                    annotation.points[i].fillColor = fillColor
                    annotation.points[i].thickness = 2.0
                    annotation.texts[i].position = dai.Point2f(detections[i].xmin, detections[i].ymin)
                    annotation.texts[i].text = f"{detections[i].label} ({int(detections[i].confidence * 100)}%)"
                    annotation.texts[i].fontSize = 0.5
                    textColor = dai.Color(0.5, 0.5, 0.5, 1.0)
                    annotation.texts[i].textColor = textColor
                    backgroundColor = dai.Color(0.5, 0.5, 0.5, 1.0)
                    annotation.texts[i].backgroundColor = backgroundColor
                    imgAnnt.annotations = [annotation]
                print(f"Annotation ts: {imgAnnt.getTimestamp()}")
                self.output.send(imgAnnt)


remoteConnector = dai.RemoteConnection()
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        cameraNode, dai.NNModelDescription("yolov6-nano")
    )

    imageAnnotationsGenerator = pipeline.create(ImageAnnotationsGenerator)
    outputToEncode = cameraNode.requestOutput((1920, 1440), type=dai.ImgFrame.Type.NV12)
    h264Encoder = pipeline.create(dai.node.VideoEncoder)
    h264Encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H264_MAIN)
    outputToEncode.link(h264Encoder.input)

    mjpegEncoder = pipeline.create(dai.node.VideoEncoder)
    mjpegEncoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.MJPEG)
    outputToEncode.link(mjpegEncoder.input)

    detectionNetwork.out.link(imageAnnotationsGenerator.input)
    labelMap = detectionNetwork.getClasses()

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()
    # Add the remote connector topics
    remoteConnector.addTopic("rawVideo", outputToEncode, "testGroup")
    remoteConnector.addTopic("h264", h264Encoder.out, "testGroup")
    remoteConnector.addTopic("mjpeg", mjpegEncoder.out, "specialMjpegGroup")
    remoteConnector.addTopic("detections", detectionNetwork.out, "testGroup")
    remoteConnector.addTopic("annotations", imageAnnotationsGenerator.output, "testGroup")
    testInput = remoteConnector.addTopic("testInput", "testGroup")
    encoderQueue = mjpegEncoder.out.createOutputQueue()

    # Register the pipeline with the remote connector
    # remoteConnector.registerPipeline(pipeline)
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
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
        if(key != -1):
            print("Got key from the remote connection: ", key)
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        encdoedMessage = encoderQueue.get()
        testInput.send(encdoedMessage)
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

