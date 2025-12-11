import depthai as dai
import numpy as np
import time
import cv2
import argparse
from datetime import datetime

    # Helper for logging
def log_quality(tag, rx, ry, rz, norm, sam_new, sam_cur):
    timestamp = datetime.now().isoformat()

    if args.log:
        # TXT
        txt_log.write(
            f"[{timestamp}] {tag}: XYZ=({rx:.6f},{ry:.6f},{rz:.6f}), "
            f"‖ΔR‖={norm:.6f}, sampson_new={sam_new:.6f}, sampson_current={sam_cur:.6f}\n"
        )
        txt_log.flush()

        # CSV
        csv_log.write(
            f"{timestamp},{tag},{rx:.6f},{ry:.6f},{rz:.6f},{norm:.6f},{sam_new:.6f},{sam_cur:.6f}\n"
        )
        csv_log.flush()

# ---------------- ARGPARSE ----------------
parser = argparse.ArgumentParser()
parser.add_argument("--log", action="store_true",
                    help="Enable logging to TXT and CSV")
args = parser.parse_args()

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

    # Initialize the command output queues for coverage, calibration quality and output
    dynCalibCoverageQueue = dynCalib.coverageOutput.createOutputQueue()
    dynCalibQualityQueue = dynCalib.qualityOutput.createOutputQueue()
    dynCalibCalibrationQueue = dynCalib.calibrationOutput.createOutputQueue()

    # Initialize the command input queue
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    # Setup the colormap for visualization
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
    maxDisparity = 1

    pipeline.start()
    time.sleep(1)  # wait for auto exposure to settle
    start = time.time()
    # Optional loggers
    if args.log:
        txt_log = open(f"dynamic_calibration_log_{device.getDeviceId()}_{datetime.now().isoformat()}.txt", "a")
        csv_log = open(f"dynamic_calibration_log_{device.getDeviceId()}_{datetime.now().isoformat()}.csv", "a")

        # CSV header if empty
        if csv_log.tell() == 0:
            csv_log.write("timestamp,type,rx_deg,ry_deg,rz_deg,rot_norm,sampson_new,sampson_current\n")
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

        # Waiting for the coverage output

        # Coverage output
        coverage = dynCalibCoverageQueue.tryGet()
        """if coverage is not None:
            print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
            print(f"Data Acquired = {coverage.dataAcquired}% / 100 [%]")"""

        # Run quality check command every 3 seconds
        if np.abs(time.time() - start) > 2:
            dynCalibInputControl.send(dai.DynamicCalibrationControl.loadImage())
            dynCalibInputControl.send(dai.DynamicCalibrationControl.calibrationQuality(False))
            start = time.time()

        # Wait for the calibration result
        dynQualityResult = dynCalibQualityQueue.tryGet()
        if dynQualityResult is not None:
            print(f"Dynamic calibration status: {dynQualityResult.info}")

        # If the calibration is successfully returned apply it to the device
        if dynQualityResult is not None and dynQualityResult.qualityData:
            calibrationData = dynQualityResult.qualityData
            rx, ry, rz = calibrationData.rotationChange
            rot_norm = np.sqrt(rx*rx + ry*ry + rz*rz)
            print("Successfully evaluated Quality")
            print(f"Rotation XYZ: X={rx:.4f}°, Y={ry:.4f}°, Z={rz:.4f}°")
            print(f"‖ΔR‖ = {rot_norm:.4f}°")
            quality = calibrationData
            """
            print(
                "Rotation difference: || r_current - r_new || = "
                f"{np.sqrt(quality.rotationChange[0]**2 + quality.rotationChange[1]**2 + quality.rotationChange[2]**2)} deg"
            )
            print(
                f"Mean Sampson error achievable = {quality.sampsonErrorNew} px \n"
                f"Mean Sampson error current = {quality.sampsonErrorCurrent} px"
            )
            print(f"Theoretical Depth Error Difference @1m:{quality.depthErrorDifference[0]:.2f}%, 2m:{quality.depthErrorDifference[1]:.2f}%, 5m:{quality.depthErrorDifference[2]:.2f}%, 10m:{quality.depthErrorDifference[3]:.2f}%")
            """
            log_quality("QUALITY", rx, ry, rz, rot_norm, calibrationData.sampsonErrorNew, calibrationData.sampsonErrorCurrent)
            dynCalibInputControl.send(dai.DynamicCalibrationControl.resetData())

            # example of usage of sampson error as main information for when calibration is needed (higher than 0.05px difference)
            if np.abs(quality.sampsonErrorNew - quality.sampsonErrorCurrent) > 0.05:
                print("Start recalibration process")
                dynCalibInputControl.send(dai.DynamicCalibrationControl.startCalibration())


        # Wait for the calibration result
        dynCalibrationResult = dynCalibCalibrationQueue.tryGet()
        if dynCalibrationResult is not None:
            print(f"Dynamic calibration status: {dynCalibrationResult.info}")
            calibrationData = dynCalibrationResult.calibrationData
        else:
            calibrationData = None

        # If the calibration is successfully returned apply it to the device
        if calibrationData:
            print("Successfully calibrated")
            print(f"New calibration: {calibrationData.newCalibration}")

            dynCalibInputControl.send(dai.DynamicCalibrationControl.applyCalibration(calibrationData.newCalibration))
            quality = calibrationData.calibrationDifference
            """print(
                "Rotation difference: || r_current - r_new || = "
                f"{np.sqrt(quality.rotationChange[0]**2 + quality.rotationChange[1]**2 + quality.rotationChange[2]**2)} deg"
            )
            print(
                f"Mean Sampson error achievable = {quality.sampsonErrorNew} px \n"
                f"Mean Sampson error current = {quality.sampsonErrorCurrent} px"
            )
            print(f"Theoretical Depth Error Difference @1m:{quality.depthErrorDifference[0]:.2f}%, 2m:{quality.depthErrorDifference[1]:.2f}%, 5m:{quality.depthErrorDifference[2]:.2f}%, 10m:{quality.depthErrorDifference[3]:.2f}%")
            """
            rx, ry, rz = quality.rotationChange
            rot_norm = np.sqrt(rx*rx + ry*ry + rz*rz)
            print(f"Rotation XYZ: X={rx:.4f}°, Y={ry:.4f}°, Z={rz:.4f}°")
            print(f"‖ΔR‖ = {rot_norm:.4f}°")

            log_quality("CALIBRATION", rx, ry, rz, rot_norm, quality.sampsonErrorNew, quality.sampsonErrorCurrent)
            dynCalibInputControl.send(dai.DynamicCalibrationControl.resetData())

        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break
# Close files on exit
if args.log:
    txt_log.close()
    csv_log.close()
