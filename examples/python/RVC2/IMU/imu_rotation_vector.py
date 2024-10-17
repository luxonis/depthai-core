#!/usr/bin/env python3
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 50)
    # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    imuQueue = imu.out.createOutputQueue(maxSize=50, blocking=False)

    pipeline.start()

    while pipeline.isRunning():
        imuData : dai.IMUData = imuQueue.get()
        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            rotationVector = imuPacket.rotationVector
            imuF = "{:.06f}"
            print(f"Rotation Vector: i: {imuF.format(rotationVector.i)} j: {imuF.format(rotationVector.j)} k: {imuF.format(rotationVector.k)} real: {imuF.format(rotationVector.real)}")