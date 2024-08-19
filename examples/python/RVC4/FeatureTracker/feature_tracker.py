import numpy as np
import cv2
import time
from collections import deque
import depthai as dai
from time import monotonic
from pathlib import Path

examplesRoot = Path(__file__).parent / Path('../../').resolve()
models = examplesRoot / Path('models')
videoPath = models / Path('construction_vest.mp4')
  
class FeatureTrackerDrawer:

    lineColor = (200, 0, 200)
    pointColor = (0, 0, 255)
    circleRadius = 2
    maxTrackedFeaturesPathLength = 30
    # for how many frames the feature is tracked
    trackedFeaturesPathLength = 10

    trackedIDs = None
    trackedFeaturesPath = None

    def onTrackBar(self, val):
        FeatureTrackerDrawer.trackedFeaturesPathLength = val
        pass

    def trackFeaturePath(self, features):

        newTrackedIDs = set()
        for currentFeature in features:
            currentID = currentFeature.id
            newTrackedIDs.add(currentID)

            if currentID not in self.trackedFeaturesPath:
                self.trackedFeaturesPath[currentID] = deque()

            path = self.trackedFeaturesPath[currentID]

            path.append(currentFeature.position)
            while(len(path) > max(1, FeatureTrackerDrawer.trackedFeaturesPathLength)):
                path.popleft()

            self.trackedFeaturesPath[currentID] = path

        featuresToRemove = set()
        for oldId in self.trackedIDs:
            if oldId not in newTrackedIDs:
                featuresToRemove.add(oldId)

        for id in featuresToRemove:
            self.trackedFeaturesPath.pop(id)

        self.trackedIDs = newTrackedIDs

    def drawFeatures(self, img):

        cv2.setTrackbarPos(self.trackbarName, self.windowName, FeatureTrackerDrawer.trackedFeaturesPathLength)

        for featurePath in self.trackedFeaturesPath.values():
            path = featurePath

            for j in range(len(path) - 1):
                src = (int(path[j].x), int(path[j].y))
                dst = (int(path[j + 1].x), int(path[j + 1].y))
                cv2.line(img, src, dst, self.lineColor, 1, cv2.LINE_AA, 0)
            j = len(path) - 1
            cv2.circle(img, (int(path[j].x), int(path[j].y)), self.circleRadius, self.pointColor, -1, cv2.LINE_AA, 0)

    def __init__(self, trackbarName, windowName):
        self.trackbarName = trackbarName
        self.windowName = windowName
        cv2.namedWindow(windowName)
        cv2.createTrackbar(trackbarName, windowName, FeatureTrackerDrawer.trackedFeaturesPathLength, FeatureTrackerDrawer.maxTrackedFeaturesPathLength, self.onTrackBar)
        self.trackedIDs = set()
        self.trackedFeaturesPath = dict()
  
print("Press 'm' to enable/disable motion estimation!")


def on_trackbar(val):
    cfg = dai.FeatureTrackerConfig()
    cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')
    cornerDetector.robustness = cv2.getTrackbarPos('robustness', 'Features')

    thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')
    cornerDetector.thresholds = thresholds

    cfg.setCornerDetector(cornerDetector)
    cfg.setMotionEstimator(motionEstimator)

    inputConfigQueue.send(cfg)

cv2.namedWindow('Features')

# create trackbars threshold change
cv2.createTrackbar('harris_score','Features',20000,25000, on_trackbar)
cv2.createTrackbar('numMaxFeatures','Features',256,1024, on_trackbar)
cv2.createTrackbar('robustness','Features',0, 127, on_trackbar)

# Create pipeline
with dai.Pipeline() as pipeline:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(videoPath)
    replay.setSize(640, 640)
    replay.setOutFrameType(dai.ImgFrame.Type.GRAY8)

    featureTrackerLeft = pipeline.create(dai.node.FeatureTracker)

    featureTrackerLeft.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTrackerLeft.initialConfig.setMotionEstimator(False)

    motionEstimator = dai.FeatureTrackerConfig.MotionEstimator()
    motionEstimator.enable = True
    featureTrackerLeft.initialConfig.setMotionEstimator(motionEstimator)

    cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector.numMaxFeatures = 256
    cornerDetector.robustness = 0

    outputFeaturePassthroughQueue = featureTrackerLeft.passthroughInputImage.createOutputQueue()
    outputFeatureQueue = featureTrackerLeft.outputFeatures.createOutputQueue()

    replay.out.link(featureTrackerLeft.inputImage)

    inputConfigQueue = featureTrackerLeft.inputConfig.createInputQueue()

    thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')

    cornerDetector.thresholds = thresholds
    featureTrackerLeft.initialConfig.setCornerDetector(cornerDetector)

    leftWindowName = "Features"
    leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName)

    pipeline.start()
    while pipeline.isRunning():
        outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

        passthroughImage = outputPassthroughImage.getCvFrame()
        leftFrame = cv2.cvtColor(passthroughImage, cv2.COLOR_GRAY2BGR)

        trackedFeaturesLeft = outputFeatureQueue.get().trackedFeatures


        leftFeatureDrawer.trackFeaturePath(trackedFeaturesLeft)
        leftFeatureDrawer.drawFeatures(leftFrame)

        # Show the frame
        cv2.imshow(leftWindowName, leftFrame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('m'):
            cfg = dai.FeatureTrackerConfig()
            cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
            cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')
            cornerDetector.robustness = cv2.getTrackbarPos('robustness', 'Features')

            thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
            thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')
            cornerDetector.thresholds = thresholds

            cfg.setCornerDetector(cornerDetector)
            cfg.setMotionEstimator(motionEstimator)

            if motionEstimator.enable == False:
                motionEstimator.enable = True
                cfg.setMotionEstimator(motionEstimator)
                print("Enabling motionEstimator")
            else:
                motionEstimator.enable = False
                cfg.setMotionEstimator(motionEstimator)
                print("Disabling motionEstimator")

            inputConfigQueue.send(cfg)

