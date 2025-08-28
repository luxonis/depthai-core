import depthai as dai
import numpy as np
import time
import cv2

# ---------- Pipeline definition ----------

with dai.Pipeline() as pipeline:
    # Create camera nodes
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    
    # Request full resolution NV12 outputs
    monoLeftOut = monoLeft.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)
    monoRightOut = monoRight.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)
    
    # Dynamic calibration node
    dynCalib = pipeline.create(dai.node.DynamicCalibration)
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)
    
    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
    syncedRightQueue = stereo.syncedRight.createOutputQueue()
    disparityQueue = stereo.disparity.createOutputQueue()
    
    # I/O queues
    dynCalibCalibrbationQueue = dynCalib.calibrationOutput.createOutputQueue()
    dynCalibCoverageQueue = dynCalib.coverageOutput.createOutputQueue()
    
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()
    
    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
    maxDisparity = 1

    pipeline.start()
    time.sleep(1) # wait for autoexposure to settle

    #Command to start the calibration after pipeline has started
    dynCalibInputControl.send(dai.StartCalibrationCommand())

    while pipeline.isRunning():

        leftSynced = syncedLeftQueue.get()
        rightSynced = syncedRightQueue.get()
        disparity = disparityQueue.get()

        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        assert isinstance(disparity, dai.ImgFrame)

        cv2.imshow("left", leftSynced.getCvFrame())
        cv2.imshow("right", rightSynced.getCvFrame())

        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)

        # Waiting for the coverage output

        # Coverage output 
        coverage = dynCalibCoverageQueue.tryGet()
        if coverage is not None:
            print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
            print(f"Data Acquired = {coverage.dataAcquired}% / 100 [%]")

        # wait for the calibration result 
        dynCalibrationResult = dynCalibCalibrbationQueue.tryGet()
        if dynCalibrationResult is not None:
            print(f"Dynamic calibration status: {dynCalibrationResult.info}")
            calibrationData = dynCalibrationResult.calibrationData
        else:
            calibrationData = None

        # if the calibration is succesfully returned apply it to the device

        if calibrationData:
            print("Succesfully calibrated")
            print(f"New calibration: {calibrationData.newCalibration}")

            dynCalibInputControl.send(dai.ApplyCalibrationCommand(calibrationData.newCalibration))
            quality = calibrationData.calibrationDifference
            print(
                "Rotation dofference: || r_current - r_new || = "
                f"{np.sqrt(quality.rotationChange[0]**2 + quality.rotationChange[1]**2 + quality.rotationChange[2]**2)} deg"
            ) 
            print(
                f"Mean Sampson error achievable = {quality.sampsonErrorNew} px \n"
                f"Mean Sampson error current = {quality.sampsonErrorCurrent} px"
            )
            print(f"Theoretical Depth Error Difference @1m:{quality.depthErrorDifference[0]:.2f}%, 2m:{quality.depthErrorDifference[1]:.2f}%, 5m:{quality.depthErrorDifference[2]:.2f}%, 10m:{quality.depthErrorDifference[3]:.2f}%")
            dynCalibInputControl.send(dai.ResetDataCommand())
            dynCalibInputControl.send(dai.StartCalibrationCommand())

        cv2.imshow("disparity", colorizedDisparity)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
