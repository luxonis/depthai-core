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


class CameraMotionEstimator:
    def __init__(self, filter_weight=0.5, motion_threshold=0.01, rotation_threshold=0.05):
        self.last_avg_flow = np.array([0.0, 0.0])
        self.filter_weight = filter_weight
        self.motion_threshold = motion_threshold
        self.rotation_threshold = rotation_threshold

    def estimate_motion(self, feature_paths):
        most_prominent_motion = "Camera Staying Still"
        max_magnitude = 0.0
        avg_flow = np.array([0.0, 0.0])
        total_rotation = 0.0
        vanishing_point = np.array([0.0, 0.0])
        num_features = len(feature_paths)

        print(f"Number of features: {num_features}")

        if num_features == 0:
            return most_prominent_motion, vanishing_point

        for path in feature_paths.values():
            if len(path) >= 2:
                src = np.array([path[-2].x, path[-2].y])
                dst = np.array([path[-1].x, path[-1].y])
                avg_flow += dst - src
                motion_vector = dst + (dst - src)
                vanishing_point += motion_vector
                rotation = np.arctan2(dst[1] - src[1], dst[0] - src[0])
                total_rotation += rotation

        avg_flow /= num_features
        avg_rotation = total_rotation / num_features
        vanishing_point /= num_features

        print(f"Average Flow: {avg_flow}")
        print(f"Average Rotation: {avg_rotation}")

        avg_flow = (self.filter_weight * self.last_avg_flow +
                    (1 - self.filter_weight) * avg_flow)
        self.last_avg_flow = avg_flow

        flow_magnitude = np.linalg.norm(avg_flow)
        rotation_magnitude = abs(avg_rotation)

        if flow_magnitude > max_magnitude and flow_magnitude > self.motion_threshold:
            if abs(avg_flow[0]) > abs(avg_flow[1]):
                most_prominent_motion = 'Right' if avg_flow[0] < 0 else 'Left'
            else:
                most_prominent_motion = 'Down' if avg_flow[1] < 0 else 'Up'
            max_magnitude = flow_magnitude

        if rotation_magnitude > max_magnitude and rotation_magnitude > self.rotation_threshold:
            most_prominent_motion = 'Rotating'

        return most_prominent_motion, vanishing_point


class FeatureTrackerDrawer:

    lineColor = (200, 0, 200)
    pointColor = (0, 0, 255)
    vanishingPointColor = (255, 0, 255)  # Violet color for vanishing point
    circleRadius = 2
    maxTrackedFeaturesPathLength = 30
    trackedFeaturesPathLength = 10

    trackedIDs = None
    trackedFeaturesPath = None

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

    def drawVanishingPoint(self, img, vanishing_point):
        cv2.circle(img, (int(vanishing_point[0]), int(vanishing_point[1])), self.circleRadius, self.vanishingPointColor, -1, cv2.LINE_AA, 0)

    # Define color mapping for directions
    direction_colors = {
        "Up": (0, 255, 255),  # Yellow
        "Down": (0, 255, 0),  # Green
        "Left": (255, 0, 0),  # Blue
        "Right": (0, 0, 255), # Red
    }

    def drawFeatures(self, img, vanishing_point=None, prominent_motion=None):

        # Get the appropriate point color based on the prominent motion
        if prominent_motion in self.direction_colors:
            point_color = self.direction_colors[prominent_motion]
        else:
            point_color = self.pointColor

        for featurePath in self.trackedFeaturesPath.values():
            path = featurePath

            for j in range(len(path) - 1):
                src = (int(path[j].x), int(path[j].y))
                dst = (int(path[j + 1].x), int(path[j + 1].y))
                cv2.line(img, src, dst, point_color, 1, cv2.LINE_AA, 0)

            j = len(path) - 1
            cv2.circle(img, (int(path[j].x), int(path[j].y)), self.circleRadius, point_color, -1, cv2.LINE_AA, 0)

        # Draw the direction text on the image
        if prominent_motion:
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2
            text_size = cv2.getTextSize(prominent_motion, font, font_scale, font_thickness)[0]
            text_x = (img.shape[1] - text_size[0]) // 2
            text_y = text_size[1] + 20  # 20 pixels from the top

            # Get the appropriate color based on the prominent motion
            text_color = self.direction_colors.get(prominent_motion, (255, 255, 255))  # Default to white

            # Draw the text
            cv2.putText(img, prominent_motion, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)


        # Draw vanishing point if provided
        if vanishing_point is not None:
            self.drawVanishingPoint(img, vanishing_point)

    def __init__(self, windowName):
        self.windowName = windowName
        cv2.namedWindow(windowName)
        self.trackedIDs = set()
        self.trackedFeaturesPath = dict()

# Connect over ADB
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
    cfg.setMotionEstimator(True)

    inputConfigQueue.send(cfg)


cv2.namedWindow('HostCamera')
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

        # Enable optical flow
        featureTrackerLeft.initialConfig.setMotionEstimator(True)

        outputFeaturePassthroughQueue = featureTrackerLeft.passthroughInputImage.createOutputQueue()
        outputFeatureQueue = featureTrackerLeft.outputFeatures.createOutputQueue()

        hostCamera.output.link(featureTrackerLeft.inputImage)

        inputConfigQueue = featureTrackerLeft.inputConfig.createInputQueue()
        cfg = dai.FeatureTrackerConfig()

        left_window_name = "Features"
        left_feature_drawer = FeatureTrackerDrawer(left_window_name)
        camera_estimator_left = CameraMotionEstimator(filter_weight=0.5, motion_threshold=0.3, rotation_threshold=0.5)

        pipeline.start()
        while pipeline.isRunning():

            inImage : dai.ImgFrame = camQueue.get()
            cv2.imshow("HostCamera", inImage.getCvFrame())

            outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

            passthroughImage = outputPassthroughImage.getCvFrame()
            leftFrame = cv2.cvtColor(passthroughImage, cv2.COLOR_GRAY2BGR)

            trackedFeaturesLeft = outputFeatureQueue.get().trackedFeatures


            motions_left, vanishing_pt_left = camera_estimator_left.estimate_motion(left_feature_drawer.trackedFeaturesPath)

            left_feature_drawer.trackFeaturePath(trackedFeaturesLeft)
            left_feature_drawer.drawFeatures(leftFrame, vanishing_pt_left, motions_left)

            print("Motions:", motions_left)
            cv2.imshow(left_window_name, leftFrame)

            if cv2.waitKey(1) == ord('q'):
                break