import depthai as dai
import numpy as np
import time
import cv2

# ---------- Pipeline definition ----------
with dai.Pipeline() as pipeline:
    # Cameras
    monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # Full-res NV12 outputs
    monoLeftOut  = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    # Initialize the DynamicCalibration node
    dynCalib = pipeline.create(dai.node.DynamicCalibration)

    # Link the cameras to the DynamicCalibration
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)

    # Initialize the command output queues for calibration and coverage
    dynCalibCalibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
    dynCalibCoverageQueue    = dynCalib.coverageOutput.createOutputQueue()

    # Initialize the command input queue
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    time.sleep(1) # wait for auto exposure to settle

    # Set performance mode
    dynCalibInputControl.send(
        dai.DynamicCalibrationControl.setPerformanceMode(
            dai.DynamicCalibrationControl.OPTIMIZE_PERFORMANCE
        )
    )

    # Start periodic calibration
    dynCalibInputControl.send(
        dai.DynamicCalibrationControl.startCalibration()
    )

    while pipeline.isRunning():
        # --- Coverage (non-blocking) ---
        coverage = dynCalibCoverageQueue.tryGet()
        if coverage is not None:
            print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
            print(f"Data Acquired       = {coverage.dataAcquired} / 100 [%]")

        # --- Calibration result (non-blocking) ---
        dynCalibrationResult = dynCalibCalibrationQueue.tryGet()
        calibrationData = dynCalibrationResult.calibrationData if dynCalibrationResult is not None else None

        if dynCalibrationResult is not None:
            print(f"Dynamic calibration status: {dynCalibrationResult.info}")

        # --- Apply calibration if available, print quality deltas, then reset+continue ---
        if calibrationData:
            print("Successfully calibrated")
            # Apply to device
            dynCalibInputControl.send(
                dai.DynamicCalibrationControl.applyCalibration(calibrationData.newCalibration)
            )

            q = calibrationData.calibrationDifference
            rotDiff = float(np.sqrt(q.rotationChange[0]**2 +
                                    q.rotationChange[1]**2 +
                                    q.rotationChange[2]**2))
            print(f"Rotation difference: || r_current - r_new || = {rotDiff:.2f} deg")
            print(f"Mean Sampson error achievable = {q.sampsonErrorNew:.3f} px")
            print(f"Mean Sampson error current    = {q.sampsonErrorCurrent:.3f} px")
            print("Theoretical Depth Error Difference "
                  f"@1m:{q.depthErrorDifference[0]:.2f}%, "
                  f"2m:{q.depthErrorDifference[1]:.2f}%, "
                  f"5m:{q.depthErrorDifference[2]:.2f}%, "
                  f"10m:{q.depthErrorDifference[3]:.2f}%")

            # Reset accumulators and continue periodic calibration
            dynCalibInputControl.send(
                dai.DynamicCalibrationControl.resetData()
            )
            dynCalibInputControl.send(
                dai.DynamicCalibrationControl.startCalibration()
            )
