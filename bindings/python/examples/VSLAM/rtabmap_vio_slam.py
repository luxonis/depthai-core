import time
import depthai as dai
from rerun_node import RerunNode

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    featureTracker = p.create(dai.node.FeatureTracker)
    odom = p.create(dai.node.RTABMapVIO).build()
    slam = p.create(dai.node.RTABMapSLAM).build()

    params = {"RGBD/CreateOccupancyGrid": "true",
              "Grid/3D": "true",
              "Rtabmap/SaveWMState": "true"}
    slam.setParams(params)

    rerunViewer = RerunNode()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    featureTracker.setHardwareResources(1,2)
    featureTracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker.initialConfig.setNumTargetFeatures(1000)
    featureTracker.initialConfig.setMotionEstimator(False)
    featureTracker.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
    left.setCamera("left")
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setFps(fps)
    right.setCamera("right")
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setFps(fps)

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)


    # Linking

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    featureTracker.passthroughInputImage.link(odom.rect)
    stereo.rectifiedLeft.link(featureTracker.inputImage)
    stereo.depth.link(odom.depth)
    imu.out.link(odom.imu)
    featureTracker.outputFeatures.link(odom.features)

    odom.transform.link(slam.odom)
    odom.passthroughRect.link(slam.rect)
    odom.passthroughDepth.link(slam.depth)

    slam.transform.link(rerunViewer.inputTrans)
    slam.passthroughRect.link(rerunViewer.inputImg)
    slam.occupancyGridMap.link(rerunViewer.inputGrid)
    p.start()
    while p.isRunning():
        time.sleep(1)