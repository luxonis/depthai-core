import numpy as np
import cv2
import time
from collections import deque
import depthai as dai
from time import monotonic

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

cv2.namedWindow('Features')

# create trackbars threshold and robustness change
cv2.createTrackbar('harris_score','Features',2000,25000, on_trackbar)
cv2.createTrackbar('robustness','Features',100,127, on_trackbar)
cv2.createTrackbar('numMaxFeatures','Features',256,1024, on_trackbar)

# Connect to device and start pipeline
with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        hostCamera = pipeline.create(HostCamera)
        camQueue = hostCamera.output.createOutputQueue()

        featureTrackerLeft = pipeline.create(dai.node.FeatureTrackerRvc4)

        featureTrackerLeft.initialConfig.setHarrisCornerDetectorThreshold(initialThreshold)
        featureTrackerLeft.initialConfig.setHarrisCornerDetectorRobustness(initialRobustness)
        featureTrackerLeft.initialConfig.setCornerDetector(dai.FeatureTrackerConfigRvc4.CornerDetector.Type.HARRIS)
        featureTrackerLeft.initialConfig.setNumMaxFeatures(256)
        
        # Disable optical flow
        featureTrackerLeft.initialConfig.setMotionEstimator(True)
        
        # variable to keep track of state
        motionEstimator = dai.FeatureTrackerConfigRvc4.MotionEstimator()
        motionEstimator.enable = False

        outputFeaturePassthroughQueue = featureTrackerLeft.passthroughInputImage.createOutputQueue()
        outputFeatureQueue = featureTrackerLeft.outputFeatures.createOutputQueue()

        hostCamera.output.link(featureTrackerLeft.inputImage)

        # config = featureTrackerLeft.initialConfig
        inputConfigQueue = featureTrackerLeft.inputConfig.createInputQueue()

        leftWindowName = "Features"
        leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName)
    
        pipeline.start()
        while pipeline.isRunning():
            inImage : dai.ImgFrame = camQueue.get()
            cv2.imshow("frame", inImage.getCvFrame())


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
  
