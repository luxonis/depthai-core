#!/usr/bin/env python3
from collections import deque
import math
import time

import depthai as dai


windowSeconds = 10.0
printUpdateHz = 4.0
yzSwapRotation = [
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
]


def quaternionToEulerXYZ(i: float, j: float, k: float, real: float) -> tuple[float, float, float]:
    sinrCosp = 2.0 * (real * i + j * k)
    cosrCosp = 1.0 - 2.0 * (i * i + j * j)
    x = math.degrees(math.atan2(sinrCosp, cosrCosp))

    sinp = 2.0 * (real * j - k * i)
    sinp = max(-1.0, min(1.0, sinp))
    y = math.degrees(math.asin(sinp))

    sinyCosp = 2.0 * (real * k + i * j)
    cosyCosp = 1.0 - 2.0 * (j * j + k * k)
    z = math.degrees(math.atan2(sinyCosp, cosyCosp))

    return x, y, z


def resolveImuExtrinsicsDestination(eepromData: dai.EepromData) -> dai.CameraBoardSocket:
    socket = eepromData.imuExtrinsics.toCameraSocket
    if socket != dai.CameraBoardSocket.AUTO:
        return socket

    if dai.CameraBoardSocket.CAM_A in eepromData.cameraData:
        return dai.CameraBoardSocket.CAM_A

    if eepromData.cameraData:
        return next(iter(eepromData.cameraData))

    return dai.CameraBoardSocket.CAM_A


def rotationVectorSupported(device: dai.Device) -> bool:
    return not device.getConnectedIMU().startswith("BMI")


def windowSpans(samples: deque[tuple[float, float, float, float, float, float, float, float, float]]) -> tuple[float, float, float]:
    xValues = [sample[1] for sample in samples]
    yValues = [sample[2] for sample in samples]
    zValues = [sample[3] for sample in samples]
    return (
        max(xValues) - min(xValues),
        max(yValues) - min(yValues),
        max(zValues) - min(zValues),
    )


with dai.Pipeline() as pipeline:
    device = pipeline.getDefaultDevice()
    imuName = device.getConnectedIMU()
    print(f"IMU type: {imuName}, firmware version: {device.getIMUFirmwareVersion()}")
    if not rotationVectorSupported(device):
        print(f"Device HW does not support ROTATION_VECTOR. Connected IMU: {imuName} (BMI IMU detected).")
        raise SystemExit(0)

    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)

    # Enable ROTATION_VECTOR at 200 hz rate
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 200)
    # It's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    # Above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    # Maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # If lower or equal to batchReportThreshold then the sending is always blocking on device
    # Useful to reduce device's CPU load and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(10)

    imuQueue = imu.out.createOutputQueue(maxSize=50, blocking=False)
    calibration = device.readCalibration()
    eepromData = calibration.getEepromData()
    imuExtrinsics = eepromData.imuExtrinsics

    destinationSocket = resolveImuExtrinsicsDestination(eepromData)
    translation = [imuExtrinsics.translation.x, imuExtrinsics.translation.y, imuExtrinsics.translation.z]
    specTranslation = [imuExtrinsics.specTranslation.x, imuExtrinsics.specTranslation.y, imuExtrinsics.specTranslation.z]

    calibration.setImuExtrinsics(destinationSocket, yzSwapRotation, translation, specTranslation)
    device.setCalibration(calibration)

    pipeline.start()
    startTime = time.monotonic()
    lastPrintTime = 0.0
    samples: deque[tuple[float, float, float, float, float, float, float, float, float]] = deque()

    print(f"Applied runtime IMU extrinsics Y/Z swap relative to {destinationSocket.name}.")
    print(f"Rotation matrix: {yzSwapRotation}")
    print(f"Rotation vector stream started on IMU {imuName}.")
    print("Move the device around each axis and watch the rolling XYZ angle spans respond.")
    print("Expected use: X/Y/Z spans should match the physical axis you rotate around, without swapped signs.")

    while pipeline.isRunning():
        try:
            imuData = imuQueue.get()
        except KeyboardInterrupt:
            break

        assert isinstance(imuData, dai.IMUData)

        for imuPacket in imuData.packets:
            rotationVector = imuPacket.rotationVector
            xAngle, yAngle, zAngle = quaternionToEulerXYZ(
                rotationVector.i,
                rotationVector.j,
                rotationVector.k,
                rotationVector.real,
            )

            sampleTime = time.monotonic() - startTime
            samples.append(
                (
                    sampleTime,
                    xAngle,
                    yAngle,
                    zAngle,
                    rotationVector.i,
                    rotationVector.j,
                    rotationVector.k,
                    rotationVector.real,
                    rotationVector.rotationVectorAccuracy,
                )
            )

            while samples and sampleTime - samples[0][0] > windowSeconds:
                samples.popleft()

        currentTime = time.monotonic()
        if samples and currentTime - lastPrintTime >= 1.0 / printUpdateHz:
            latest = samples[-1]
            xSpan, ySpan, zSpan = windowSpans(samples)
            print(
                f"Window {windowSeconds:4.1f}s span [deg]: x={xSpan:7.2f} y={ySpan:7.2f} z={zSpan:7.2f} | "
                f"latest xyz=[{latest[1]:7.2f}, {latest[2]:7.2f}, {latest[3]:7.2f}] "
                f"quat=[{latest[4]: .5f}, {latest[5]: .5f}, {latest[6]: .5f}, {latest[7]: .5f}] "
                f"acc={latest[8]:.5f} rad"
            )
            lastPrintTime = currentTime
