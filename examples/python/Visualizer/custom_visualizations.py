#!/usr/bin/env python3

import depthai as dai

class ImgDetectionsExtended(dai.ImgDetections):
    def __init__(self, detections: dai.ImgDetections):
        dai.ImgDetections.__init__(self)
        self.detections = detections.detections

    # The function can return dai.ImgAnnotations or dai.ImgFrame
    def getVisualizationMessage(self):
        detections = self.detections
        imgAnnt = dai.ImgAnnotations()
        # Setting the timestamp is important, as the visualizer uses it to synchronize the data
        imgAnnt.setTimestamp(self.getTimestamp())
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
            text.text = f"Test annotation"
            text.fontSize = 50.5
            textColor = dai.Color(0.5, 0.5, 1.0, 1.0)
            text.textColor = textColor
            backgroundColor = dai.Color(1.0, 1.0, 0.5, 1.0)
            text.backgroundColor = backgroundColor
            annotation.points.append(pointsAnnotation)
            annotation.texts.append(text)

        imgAnnt.annotations.append(annotation)
        return imgAnnt

class ImgAnnotationsGenerator(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.inputDet = self.createInput()
        self.output = self.createOutput()

    def run(self):
        while self.isRunning():
            nnData = self.inputDet.get()
            extended = ImgDetectionsExtended(nnData)
            # Setting the timestamp is important, as the visualizer uses it to synchronize the data
            extended.setTimestamp(nnData.getTimestamp())
            self.output.send(extended)

remoteConnector = dai.RemoteConnection()

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        cameraNode, dai.NNModelDescription("yolov6-nano")
    )
    imageAnnotationsGenerator = pipeline.create(ImgAnnotationsGenerator)
    outputToVisualize = cameraNode.requestOutput((640,480), type=dai.ImgFrame.Type.NV12)
    detectionNetwork.out.link(imageAnnotationsGenerator.inputDet)

    # Add the remote connector topics
    remoteConnector.addTopic("encoded", outputToVisualize, "images")
    remoteConnector.addTopic("detections", detectionNetwork.out, "images")
    remoteConnector.addTopic("annotations", imageAnnotationsGenerator.output, "images")

    pipeline.start()

    # Register the pipeline with the remote connector
    remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        if remoteConnector.waitKey(1) == ord("q"):
            pipeline.stop()
            break

