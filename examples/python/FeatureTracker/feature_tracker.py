import cv2
from collections import deque
import depthai as dai

class FeatureTrackerDrawer:
    lineColor = (200, 0, 200)
    pointColor = (0, 0, 255)
    circleRadius = 2
    maxTrackedFeaturesPathLength = 30
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

inputConfigQueue = None
def on_trackbar(val):
    try:
        cfg = dai.FeatureTrackerConfig()
        cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
        cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')
        cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures

        thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
        thresholds.initialValue = cv2.getTrackbarPos('harrisScore','Features')
        cornerDetector.thresholds = thresholds
    except cv2.error as e:
        pass

    cfg.setCornerDetector(cornerDetector)
    if inputConfigQueue:
        inputConfigQueue.send(cfg)

cv2.namedWindow('Features', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Features', 1080, 800)

cv2.createTrackbar('harrisScore','Features',20000,25000, on_trackbar)
cv2.createTrackbar('numMaxFeatures','Features',256,1024, on_trackbar)

# Create pipeline
with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build()
    camOutput = camera.requestOutput((640, 640), dai.ImgFrame.Type.NV12)
    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
    camOutput.link(manip.inputImage)

    featureTracker = pipeline.create(dai.node.FeatureTracker)

    featureTracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker.initialConfig.setMotionEstimator(False)
    featureTracker.initialConfig.setNumTargetFeatures(256)

    motionEstimator = dai.FeatureTrackerConfig.MotionEstimator()
    motionEstimator.enable = True
    featureTracker.initialConfig.setMotionEstimator(motionEstimator)

    cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector.numMaxFeatures = 256
    cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures

    # RVC2 specific setting to allow for more features
    featureTracker.setHardwareResources(2,2)

    outputFeaturePassthroughQueue = camOutput.createOutputQueue()
    outputFeatureQueue = featureTracker.outputFeatures.createOutputQueue()

    manip.out.link(featureTracker.inputImage)

    inputConfigQueue = featureTracker.inputConfig.createInputQueue()

    thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds.initialValue = cv2.getTrackbarPos('harrisScore','Features')

    cornerDetector.thresholds = thresholds
    featureTracker.initialConfig.setCornerDetector(cornerDetector)

    leftWindowName = "Features"
    leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName)

    pipeline.start()
    while pipeline.isRunning():
        outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

        passthroughImage = outputPassthroughImage.getCvFrame()
        trackedFeaturesLeft = outputFeatureQueue.get().trackedFeatures


        leftFeatureDrawer.trackFeaturePath(trackedFeaturesLeft)
        leftFeatureDrawer.drawFeatures(passthroughImage)

        # Show the frame
        cv2.imshow(leftWindowName, passthroughImage)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('m'):
            cfg = dai.FeatureTrackerConfig()
            cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
            cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')
            cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures

            thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
            thresholds.initialValue = cv2.getTrackbarPos('harrisScore','Features')
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
