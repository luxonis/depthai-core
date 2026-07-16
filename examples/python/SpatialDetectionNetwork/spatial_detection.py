#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

fps = 20
# For better results on OAK4, use a segmentation model like "luxonis/yolov8-instance-segmentation-large:coco-640x480"
# for depth estimation over the objects mask instead of the full bounding box.
modelDescription = dai.NNModelDescription("yolov6-nano")

class SpatialVisualizer(dai.node.HostNode):
    def __init__(self):
        dai.node.HostNode.__init__(self)
        self.sendProcessingToPipeline(True)
    def build(self, depth:dai.Node.Output, detections: dai.Node.Output, rgb: dai.Node.Output):
        self.link_args(depth, detections, rgb) # Must match the inputs to the process method

    def process(self, depthPreview, detections, rgbPreview):
        depthPreview = depthPreview.getCvFrame()
        rgbPreview = rgbPreview.getCvFrame()
        depthFrameColor = self.processDepthFrame(depthPreview)
        self.displayResults(rgbPreview, depthFrameColor, detections.detections)

    def processDepthFrame(self, depthFrame):
        depthDownscaled = depthFrame[::4]
        if np.all(depthDownscaled == 0):
            minDepth = 0
        else:
            minDepth = np.percentile(depthDownscaled[depthDownscaled != 0], 1)
        maxDepth = np.percentile(depthDownscaled, 99)
        depthFrameColor = np.interp(depthFrame, (minDepth, maxDepth), (0, 255)).astype(np.uint8)
        return cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

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
    camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=fps)
    # The Depth node manages its own stereo cameras and backend internally, so
    # no explicit left/right cameras are needed. SpatialDetectionNetwork aligns
    # the depth to the color camera internally.
    depth = p.create(dai.node.Depth).build(fps=fps)

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
