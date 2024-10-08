import depthai as dai
import numpy as np
import cv2
import time

FPS = 15

remoteConnector = dai.RemoteConnector()


with dai.Pipeline() as pipeline:

    # Create nodes
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)
    pointcloud = pipeline.create(dai.node.PointCloud)

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)
    depth.depth.link(pointcloud.inputDepth)

    # Get pointcloud queue
    pointcloudQueue = pointcloud.outputPointCloud.createOutputQueue()

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoLeft.setFps(FPS)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")
    monoRight.setFps(FPS)

    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    depth.setLeftRightCheck(True)
    depth.setExtendedDisparity(False)
    depth.setSubpixel(True)
    depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    depth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout

    depth.initialConfig.postProcessing.speckleFilter.enable = True
    depth.initialConfig.postProcessing.speckleFilter.speckleRange = 60
    depth.initialConfig.postProcessing.temporalFilter.enable = True
    depth.initialConfig.postProcessing.spatialFilter.holeFillingRadius = 2
    depth.initialConfig.postProcessing.spatialFilter.numIterations = 1
    depth.initialConfig.postProcessing.thresholdFilter.minRange = 700  # mm
    depth.initialConfig.postProcessing.thresholdFilter.maxRange = 4000  # mm
    depth.initialConfig.censusTransform.enableMeanMode = True
    depth.initialConfig.costMatching.linearEquationParameters.alpha = 0
    depth.initialConfig.costMatching.linearEquationParameters.beta = 2

    remoteConnector.addTopic("pointcloud", pointcloud.outputPointCloud, "testGroup")

    pipeline.start()
    remoteConnector.registerPipeline(pipeline)
    
    while pipeline.isRunning():
        time.sleep(1)

        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

        print("running")
        q = pointcloudQueue.get()
        print(q)    