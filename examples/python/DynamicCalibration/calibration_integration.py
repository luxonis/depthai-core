import depthai as dai
import numpy as np
import time
import cv2


def print_metrics(metrics: dai.CalibrationQualityData) -> None:
    for socket_pair, rotation in metrics.pairwiseRotationDifference.items():
        print(f"Pairwise rotation difference {socket_pair} = {rotation} deg")
    print(f"Mean Sampson error achievable = {metrics.sampsonErrorNew:.3f} px")
    print(f"Mean Sampson error current    = {metrics.sampsonErrorCurrent:.3f} px")

# ---------- Pipeline definition ----------

with dai.Pipeline() as pipeline:
    # Create camera nodes
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # Request full resolution NV12 outputs
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    # Initialize the DynamicCalibration node
    dynCalib = pipeline.create(dai.node.DynamicCalibration)

    # Link the cameras to the DynamicCalibration
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)

    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
    syncedRightQueue = stereo.syncedRight.createOutputQueue()
    disparityQueue = stereo.disparity.createOutputQueue()

    # Initialize the command output queues for coverage and calibration output
    dynCalibCoverageQueue = dynCalib.coverageOutput.createOutputQueue()
    dynCalibCalibrationQueue = dynCalib.calibrationOutput.createOutputQueue()

    # Initialize the command input queue
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.getCalibration())

    # Setup the colormap for visualization
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
    maxDisparity = 1

    pipeline.start()
    time.sleep(1)  # wait for auto exposure to settle
    start = time.time()

    while pipeline.isRunning():

        leftSynced = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        disparity = disparityQueue.get()

        cv2.imshow("left", leftSynced.getCvFrame())
        cv2.imshow("right", rightSynced.getCvFrame())

        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(
            ((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap
        )

        coverage = dynCalibCoverageQueue.tryGet()
        if coverage is not None:
            print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
            print(f"Data Acquired       = {coverage.dataAcquired} / 100 [%]")

        # Run one-shot calibration every 3 seconds and inspect metrics from calibrationOutput.
        if np.abs(time.time() - start) > 3:
            dynCalibInputControl.send(dai.DynamicCalibrationControl.loadImage())
            dynCalibInputControl.send(dai.DynamicCalibrationControl.calibrate(False))
            start = time.time()

        dynCalibrationResult = dynCalibCalibrationQueue.tryGet()
        if dynCalibrationResult is not None:
            print(f"Dynamic calibration status: {dynCalibrationResult.info}")
            calibrationData = dynCalibrationResult.calibrationData
        else:
            calibrationData = None

        if calibrationData:
            quality = calibrationData.calibrationDifference
            print("Successfully evaluated metrics from calibration output")
            print_metrics(quality)

            if np.abs(quality.sampsonErrorNew - quality.sampsonErrorCurrent) > 0.05:
                print("Applying new calibration")
                dynCalibInputControl.send(dai.DynamicCalibrationControl.applyCalibration(calibrationData.newCalibration))

            dynCalibInputControl.send(dai.DynamicCalibrationControl.resetData())

        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break
