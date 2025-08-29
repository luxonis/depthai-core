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
    monoLeftOut  = monoLeft.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)
    monoRightOut = monoRight.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)

    # Dynamic calibration
    dynCalib = pipeline.create(dai.node.DynamicCalibration)
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)

    # Stereo (for disparity + synced previews)
    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    # Output queues
    syncedLeftQueue  = stereo.syncedLeft.createOutputQueue()
    syncedRightQueue = stereo.syncedRight.createOutputQueue()
    disparityQueue   = stereo.disparity.createOutputQueue()

    # Dynamic-calibration outputs
    dynCalibCalibrationQueue = dynCalib.calibrationOutput.createOutputQueue()
    dynCalibCoverageQueue    = dynCalib.coverageOutput.createOutputQueue()

    # IMPORTANT: create input queue with the correct datatype
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    maxDisparity = 1.0

    pipeline.start()
    time.sleep(1)  # let autoexposure settle

    # Start periodic calibration
    dynCalibInputControl.send(
        dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.StartCalibrationCommand())
    )

    while pipeline.isRunning():
        leftSynced  = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        disparity   = disparityQueue.get()

        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)

        cv2.imshow("left", leftSynced.getCvFrame())
        cv2.imshow("right", rightSynced.getCvFrame())

        # --- Disparity visualization ---
        npDisparity = disparity.getFrame()
        curMax = float(np.max(npDisparity))
        if curMax > 0:
            maxDisparity = max(maxDisparity, curMax)

        # Normalize to [0,255] and colorize; keep zero-disparity as black
        denom = maxDisparity if maxDisparity > 0 else 1.0
        normalized = (npDisparity / denom * 255.0).astype(np.uint8)
        colorizedDisparity = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        colorizedDisparity[normalized == 0] = (0, 0, 0)
        cv2.imshow("disparity", colorizedDisparity)

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
                dai.DynamicCalibrationControl(
                    dai.DynamicCalibrationControl.ApplyCalibrationCommand(calibrationData.newCalibration)
                )
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
                dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.ResetDataCommand())
            )
            dynCalibInputControl.send(
                dai.DynamicCalibrationControl(dai.DynamicCalibrationControl.StartCalibrationCommand())
            )

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
