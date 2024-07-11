#!/usr/bin/env python3

import depthai as dai
import cv2
import time
import numpy as np

def drawFeatures(frame, features):
        pointColor = (0, 0, 255)
        greenColor = (0, 255, 0)
        circleRadius = 2
        for feature in features:
            # descriptor = feature.descriptor
            # print(descriptor)
            if feature.age < 2:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, greenColor, -1, cv2.LINE_AA, 0)
            else:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, pointColor, -1, cv2.LINE_AA, 0)

class HostCamera(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        # Create a VideoCapture object
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            pipeline.stop()
            raise RuntimeError("Error: Couldn't open host camera")
        while self.isRunning():
            # frame = depthImg
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            imgFrame = dai.ImgFrame()
            imgFrame.setData(frame)
            imgFrame.setWidth(frame.shape[1])
            imgFrame.setHeight(frame.shape[0])
            imgFrame.setType(dai.ImgFrame.Type.GRAY8)
            # Send the message
            self.output.send(imgFrame)
            # Wait for the next frame
            time.sleep(0.1)

# Create pipeline
info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3


initialThreshold = 1000
initialRobustness = 100

def on_trackbar(val):
    cfg = dai.FeatureTrackerConfigRvc4()

    cfg.setHarrisCornerDetectorThreshold(cv2.getTrackbarPos('harris_score','Features'))
    cfg.setHarrisCornerDetectorRobustness(cv2.getTrackbarPos('robustness','Features'))
    cfg.setNumMaxFeatures(cv2.getTrackbarPos('numMaxFeatures', 'Features'))

    cfg.setMotionEstimator(motionEstimator)

    inputConfigQueue.send(cfg)

cv2.namedWindow('HostCamera')
cv2.namedWindow('Features')

# create trackbars threshold and robustness change
cv2.createTrackbar('harris_score','Features',2000,25000, on_trackbar)
cv2.createTrackbar('robustness','Features',100,127, on_trackbar)
cv2.createTrackbar('numMaxFeatures','Features',256,1024, on_trackbar)


with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        cnt = 0
        hostCamera = pipeline.create(HostCamera)
        camQueue = hostCamera.output.createOutputQueue()
        
        featureTrackerLeft = pipeline.create(dai.node.FeatureTrackerRvc4)
       
        featureTrackerLeft.initialConfig.setHarrisCornerDetectorThreshold(initialThreshold)
        featureTrackerLeft.initialConfig.setHarrisCornerDetectorRobustness(initialRobustness)
        featureTrackerLeft.initialConfig.setCornerDetector(dai.FeatureTrackerConfigRvc4.CornerDetector.Type.HARRIS)
        featureTrackerLeft.initialConfig.setNumMaxFeatures(256)


        outputFeaturePassthroughQueue = featureTrackerLeft.passthroughInputImage.createOutputQueue()
        outputFeatureQueue = featureTrackerLeft.outputFeatures.createOutputQueue()

        hostCamera.output.link(featureTrackerLeft.inputImage)

        inputConfigQueue = featureTrackerLeft.inputConfig.createInputQueue()
        motionEstimator = dai.FeatureTrackerConfigRvc4.MotionEstimator()
        motionEstimator.enable = False

        thresholds = dai.FeatureTrackerConfigRvc4.CornerDetector.Thresholds()

        pipeline.start()
        while pipeline.isRunning():
            trackedFeaturesLeft = outputFeatureQueue.get().trackedFeatures

            image : dai.ImgFrame = camQueue.get()
            cv2.imshow("HostCamera", image.getCvFrame())

            outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

            passthroughImage = outputPassthroughImage.getCvFrame()
            passthroughImage = cv2.cvtColor(passthroughImage, cv2.COLOR_GRAY2BGR)


            drawFeatures(passthroughImage, trackedFeaturesLeft)

            # Show the frame
            cv2.imshow("Features", passthroughImage)
            key = cv2.waitKey(1)
      
            if key == ord('q'):
                break
            elif key == ord('m'):
                cfg = dai.FeatureTrackerConfigRvc4()
                cfg.setHarrisCornerDetectorThreshold(cv2.getTrackbarPos('harris_score','Features'))
                cfg.setHarrisCornerDetectorRobustness(cv2.getTrackbarPos('robustness','Features'))
                cfg.setNumMaxFeatures(cv2.getTrackbarPos('numMaxFeatures', 'Features'))
                if motionEstimator.enable == False:
                    motionEstimator.enable = True
                    cfg.setMotionEstimator(motionEstimator)
                    print("Enabling motionEstimator")
                else:
                    motionEstimator.enable = False
                    cfg.setMotionEstimator(motionEstimator)
                    print("Disabling motionEstimator")

                inputConfigQueue.send(cfg)
