import depthai as dai
import numpy as np
import time
import cv2

# ---------- Pipeline definition ----------
with dai.Pipeline() as pipeline:
    # Create camera nodes
    monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # Request full resolution NV12 outputs
    monoLeftOut  = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    # Initialize the DynamicCalibration node
    dynCalib = pipeline.create(dai.node.DynamicCalibration)

    # Link the cameras to the DynamicCalibration
    monoLeftOut.link(dynCalib.left)
    monoRightOut.link(dynCalib.right)

    # Initialize the command output queues for coverage and calibration quality
    dynCalibCoverageQueue = dynCalib.coverageOutput.createOutputQueue()

    # Initialize the command input queue
    dynCalibInputControl = dynCalib.inputControl.createInputQueue()

    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    time.sleep(1) # wait for auto exposure to settle

    dynCalibInputControl.send(dai.DynamicCalibrationControl.loadImage())
    while pipeline.isRunning():
        # --- Load one frame into calibration & read coverage
        coverage = dynCalibCoverageQueue.get()
        if coverage is not None:
            print(f"2D Spatial Coverage = {coverage.meanCoverage} / 100 [%]")
            print(f"Data Acquired       = {coverage.dataAcquired} / 100 [%]")
            print(f"Coverage matrix for image A: {coverage.coveragePerCellA}")
            print(f"Coverage matrix for image B: {coverage.coveragePerCellB}")
            dynCalibInputControl.send(dai.DynamicCalibrationControl.loadImage())