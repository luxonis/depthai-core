import time

import cv2
import numpy as np

import depthai as dai


def print_coverage(coverage: dai.CoverageData) -> None:
    print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
    print(f"Data Acquired       = {coverage.dataAcquired} / 100 [%]")


def format_socket(socket: dai.CameraBoardSocket) -> str:
    return socket.name if hasattr(socket, "name") else str(socket)


def print_calibration_result(result: dai.DynamicCalibrationResult) -> None:
    print(f"Dynamic calibration status: {result.info}")
    calibration_data = result.calibrationData
    if calibration_data is None:
        return

    print("Successfully calibrated")
    print(f"Data confidence: {calibration_data.dataConfidence:.3f}")

    quality = calibration_data.calibrationDifference
    rotation_change = np.array(quality.rotationChange, dtype=float)
    print(f"Mean Sampson error achievable = {quality.sampsonErrorNew:.3f} px")
    print(f"Mean Sampson error current    = {quality.sampsonErrorCurrent:.3f} px")

    if len(quality.depthErrorDifference) >= 4:
        print(
            "Theoretical Depth Error Difference "
            f"@1m:{quality.depthErrorDifference[0]:.2f}%, "
            f"2m:{quality.depthErrorDifference[1]:.2f}%, "
            f"5m:{quality.depthErrorDifference[2]:.2f}%, "
            f"10m:{quality.depthErrorDifference[3]:.2f}%"
        )

    for socket_pair, rotation_delta in quality.pairwiseRotationDifference.items():
        socket_a, socket_b = socket_pair
        formatted_delta = ", ".join(f"{value:.3f}" for value in rotation_delta)
        print(f"Pairwise rotation difference {format_socket(socket_a)} -> {format_socket(socket_b)}: [{formatted_delta}]")


with dai.Pipeline() as pipeline:
    rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    rgbOut = rgb.requestOutput((1280, 800), fps=30)
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    dynCalib = pipeline.create(dai.node.DynamicCalibration)
    rgbOut.link(dynCalib.rgb)
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)

    rgbQueue = rgbOut.createOutputQueue()
    leftQueue = monoLeftOut.createOutputQueue()
    rightQueue = monoRightOut.createOutputQueue()
    coverageQueue = dynCalib.coverageOutput.createOutputQueue()
    calibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
    inputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    time.sleep(1)

    inputControl.send(
        dai.DynamicCalibrationControl.setPerformanceMode(
            dai.DynamicCalibrationControl.PerformanceMode.OPTIMIZE_PERFORMANCE
        )
    )
    inputControl.send(dai.DynamicCalibrationControl.startCalibration())

    while pipeline.isRunning():
        cv2.imshow("rgb", rgbQueue.get().getCvFrame())
        cv2.imshow("left", leftQueue.get().getCvFrame())
        cv2.imshow("right", rightQueue.get().getCvFrame())

        coverage = coverageQueue.tryGet()
        if coverage is not None:
            print_coverage(coverage)

        calibration_result = calibrationQueue.tryGet()
        if calibration_result is not None:
            print_calibration_result(calibration_result)
            if calibration_result.calibrationData is not None:
                inputControl.send(
                    dai.DynamicCalibrationControl.applyCalibration(
                        calibration_result.calibrationData.newCalibration,
                        flash=False
                    )
                )
                break

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
