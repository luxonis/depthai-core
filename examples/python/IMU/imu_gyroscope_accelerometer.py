#!/usr/bin/env python3
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)

    # enable ACCELEROMETER_RAW at 500 hz rate
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 480)
    # enable GYROSCOPE_RAW at 400 hz rate
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # if lower or equal to batchReportThreshold then the sending is always blocking on device
    # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(10)

    imuQueue = imu.out.createOutputQueue(maxSize=50, blocking=False)

    pipeline.start()
    baseTs = None
    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    while pipeline.isRunning():
        imuData = imuQueue.get()
        assert isinstance(imuData, dai.IMUData)
        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope

            acceleroTs = acceleroValues.getTimestamp()
            gyroTs = gyroValues.getTimestamp()

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            print(f"Accelerometer timestamp: {acceleroTs}")
            print(f"Latency [ms]: {dai.Clock.now() - acceleroValues.getTimestamp()}")
            print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            print(f"Gyroscope timestamp: {gyroTs}")
            print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")