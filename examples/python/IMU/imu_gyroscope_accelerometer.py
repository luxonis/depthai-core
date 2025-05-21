#!/usr/bin/env python3
import depthai as dai
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque

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

    time_window = 100
    time_vals = deque(maxlen=time_window)
    accel_x_vals = deque(maxlen=time_window)
    accel_y_vals = deque(maxlen=time_window)
    accel_z_vals = deque(maxlen=time_window)
    gyro_x_vals = deque(maxlen=time_window)
    gyro_y_vals = deque(maxlen=time_window)
    gyro_z_vals = deque(maxlen=time_window)
    
    plt.ion()
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))
    while pipeline.isRunning():
        imuData = imuQueue.get()
        assert isinstance(imuData, dai.IMUData)
        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope
            accel = imuPacket.acceleroMeter
            gyro = imuPacket.gyroscope

            acx =  accel.x
            acy =  accel.y
            acz =  accel.z
            gx =  gyro.x
            gy =  gyro.y
            gz =  gyro.z

            acceleroTs = acceleroValues.getTimestamp()
            gyroTs = gyroValues.getTimestamp()

            imuF = "{:.06f}"
            tsF  = "{:.03f}"
            timestamp = time.time()
            time_vals.append(timestamp)
            accel_x_vals.append(acx)
            accel_y_vals.append(acy)
            accel_z_vals.append(acz)
            gyro_x_vals.append(gx)
            gyro_y_vals.append(gy)
            gyro_z_vals.append(gz)

            ax[0].cla()
            ax[0].plot(time_vals, accel_x_vals, label="Accel X", color='r')
            ax[0].plot(time_vals, accel_y_vals, label="Accel Y", color='g')
            ax[0].plot(time_vals, accel_z_vals, label="Accel Z", color='b')
            ax[0].set_title("Accelerometer Data")
            ax[0].legend()
            ax[0].grid()
            
            ax[1].cla()
            ax[1].plot(time_vals, gyro_x_vals, label="Gyro X", color='r')
            ax[1].plot(time_vals, gyro_y_vals, label="Gyro Y", color='g')
            ax[1].plot(time_vals, gyro_z_vals, label="Gyro Z", color='b')
            ax[1].set_title("Gyroscope Data")
            ax[1].legend()
            ax[1].grid()
            
            plt.pause(0.01)

    plt.ioff()
    plt.show()
