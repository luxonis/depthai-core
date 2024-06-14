import time
import depthai as dai
from rerun_node import RerunNode
# Create pipeline

with dai.Pipeline() as p:
    fps = 60
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO).build()

    rerunViewer = RerunNode()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    left.setCamera("left")
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setFps(fps)
    right.setCamera("right")
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setFps(fps)

    # Linking

    left.out.link(odom.left)
    right.out.link(odom.right)
    imu.out.link(odom.imu)
    odom.passthrough.link(rerunViewer.inputImg)
    odom.transform.link(rerunViewer.inputTrans)
    p.start()
    while p.isRunning():
        time.sleep(1)