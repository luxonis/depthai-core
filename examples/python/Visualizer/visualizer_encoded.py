#!/usr/bin/env python3

import depthai as dai
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)

args = parser.parse_args()

remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
ENCODER_PROFILE = dai.VideoEncoderProperties.Profile.MJPEG
class ImgAnnotationsGenerator(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.inputDet = self.createInput()
        self.output = self.createOutput()

    def setLabelMap(self, labelMap):
        self.labelMap = labelMap
    def run(self):
        while self.isRunning():
            nnData = self.inputDet.get()
            detections = nnData.detections
            imgAnnt = dai.ImgAnnotations()
            imgAnnt.setTimestamp(nnData.getTimestamp())
            annotation = dai.ImgAnnotation()
            for detection in detections:
                pointsAnnotation = dai.PointsAnnotation()
                pointsAnnotation.type = dai.PointsAnnotationType.LINE_STRIP
                pointsAnnotation.points = dai.VectorPoint2f([
                    dai.Point2f(detection.xmin, detection.ymin),
                    dai.Point2f(detection.xmax, detection.ymin),
                    dai.Point2f(detection.xmax, detection.ymax),
                    dai.Point2f(detection.xmin, detection.ymax),
                ])
                outlineColor = dai.Color(1.0, 0.5, 0.5, 1.0)
                pointsAnnotation.outlineColor = outlineColor
                fillColor = dai.Color(0.5, 1.0, 0.5, 0.5)
                pointsAnnotation.fillColor = fillColor
                pointsAnnotation.thickness = 2.0
                text = dai.TextAnnotation()
                text.position = dai.Point2f(detection.xmin, detection.ymin)
                text.text = f"{self.labelMap[detection.label]} {int(detection.confidence * 100)}%"
                text.fontSize = 50.5
                textColor = dai.Color(0.5, 0.5, 1.0, 1.0)
                text.textColor = textColor
                backgroundColor = dai.Color(1.0, 1.0, 0.5, 1.0)
                text.backgroundColor = backgroundColor
                annotation.points.append(pointsAnnotation)
                annotation.texts.append(text)

            imgAnnt.annotations.append(annotation)
            self.output.send(imgAnnt)

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        cameraNode, dai.NNModelDescription("yolov6-nano")
    )
    imageAnnotationsGenerator = pipeline.create(ImgAnnotationsGenerator)
    outputToEncode = cameraNode.requestOutput((1920, 1440), type=dai.ImgFrame.Type.NV12)
    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(30, ENCODER_PROFILE)
    outputToEncode.link(encoder.input)


    detectionNetwork.out.link(imageAnnotationsGenerator.inputDet)
    labelMap = detectionNetwork.getClasses()
    imageAnnotationsGenerator.setLabelMap(labelMap)

    # Add the remote connector topics
    remoteConnector.addTopic("encoded", encoder.out, "images")
    remoteConnector.addTopic("detections", detectionNetwork.out, "images")
    remoteConnector.addTopic("annotations", imageAnnotationsGenerator.output, "images")

    pipeline.start()

    # Register the pipeline with the remote connector
    remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        if remoteConnector.waitKey(1) == ord("q"):
            pipeline.stop()
            break

