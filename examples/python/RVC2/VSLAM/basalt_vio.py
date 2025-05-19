import signal
import time
import depthai as dai
from rerun_node import RerunNode
# Create pipeline

with dai.Pipeline() as p:
    fps = 60
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)

    rerunViewer = RerunNode()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
    imu.out.link(odom.imu)
    odom.passthrough.link(rerunViewer.inputImg)
    odom.transform.link(rerunViewer.inputTrans)
    p.start()
    while p.isRunning():
        time.sleep(0.01)
