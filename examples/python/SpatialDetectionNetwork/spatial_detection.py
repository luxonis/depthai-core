#!/usr/bin/env python3

import cv2
import depthai as dai

fps = 20
modelDescription = dai.NNModelDescription("yolov6-nano")

class SpatialVisualizer(dai.node.HostNode):
    def __init__(self):
        dai.node.HostNode.__init__(self)
        self.sendProcessingToPipeline(True)
    def build(self, depth:dai.Node.Output, detections: dai.Node.Output, rgb: dai.Node.Output):
        self.link_args(depth, detections, rgb) # Must match the inputs to the process method

    def process(self, depthPreview, detections, rgbPreview):
        rgbPreview = rgbPreview.getCvFrame()
        depthFrameColor = self.processDepthFrame(depthPreview)
        self.displayResults(rgbPreview, depthFrameColor, detections.detections)

    def processDepthFrame(self, depthFrame):
        return dai.utility.colorizeDepthFrame(depthFrame, colormap=cv2.COLORMAP_HOT).getCvFrame()

    def displayResults(self, rgbFrame, depthFrameColor, detections):
        height, width, _ = rgbFrame.shape
        for detection in detections:
            self.drawBoundingBoxes(depthFrameColor, detection)
            self.drawDetections(rgbFrame, detection, width, height)

        cv2.imshow("Depth frame", depthFrameColor)
        cv2.imshow("Color frame", rgbFrame)
        if cv2.waitKey(1) == ord('q'):
            self.stopPipeline()

    def drawBoundingBoxes(self, depthFrameColor, detection):
        roiData = detection.boundingBoxMapping
        roi = roiData.roi
        roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
        topLeft = roi.topLeft()
        bottomRight = roi.bottomRight()
        cv2.rectangle(depthFrameColor, (int(topLeft.x), int(topLeft.y)), (int(bottomRight.x), int(bottomRight.y)), (255, 255, 255), 1)

    def drawDetections(self, frame, detection, frameWidth, frameHeight):
        x1 = int(detection.xmin * frameWidth)
        x2 = int(detection.xmax * frameWidth)
        y1 = int(detection.ymin * frameHeight)
        y2 = int(detection.ymax * frameHeight)
        label = detection.labelName
        color = (255, 255, 255)
        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, "{:.2f}".format(detection.confidence * 100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

# Creates the pipeline and a default device implicitly
with dai.Pipeline() as p:
    # Define sources and outputs
    colorSockets = p.getDefaultDevice().getConnectedCameras(dai.CameraSensorType.COLOR)
    colorSocket = colorSockets[0] if colorSockets else dai.CameraBoardSocket.CAM_A
    camRgb = p.create(dai.node.Camera).build(colorSocket, sensorFps=fps)
    depth = p.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, fps, (640, 400))

    spatialDetectionNetwork = p.create(dai.node.SpatialDetectionNetwork).build(
        camRgb, depth, modelDescription
    )
    visualizer = p.create(SpatialVisualizer)

    spatialDetectionNetwork.spatialLocationCalculator.initialConfig.setSegmentationPassthrough(False)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    visualizer.build(
        spatialDetectionNetwork.passthroughDepth,
        spatialDetectionNetwork.out,
        spatialDetectionNetwork.passthrough,
    )

    print("Starting pipeline")

    p.run()
