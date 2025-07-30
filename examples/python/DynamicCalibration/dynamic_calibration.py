#!/usr/bin/env python3

import depthai as dai
import cv2
import numpy as np

# ---------- Pipeline definition ----------
pipeline = dai.Pipeline()

# Create camera nodes
cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# Request full resolution NV12 outputs
left_out = cam_left.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)
right_out = cam_right.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)

# Stereo node
stereo = pipeline.create(dai.node.StereoDepth)
left_out.link(stereo.left)
right_out.link(stereo.right)

# Dynamic calibration node
dyn_calib = pipeline.create(dai.node.DynamicCalibration)
left_out.link(dyn_calib.left)
right_out.link(dyn_calib.right)

# Output queues
left_xout = stereo.syncedLeft.createOutputQueue()
right_xout = stereo.syncedRight.createOutputQueue()
disp_xout = stereo.disparity.createOutputQueue()

calibration_output = dyn_calib.calibrationOutput.createOutputQueue()
coverage_output = dyn_calib.coverageOutput.createOutputQueue()
quality_output = dyn_calib.qualityOutput.createOutputQueue()

command_input = dyn_calib.commandInput.createInputQueue()
initial_config_input = dyn_calib.configInput.createInputQueue()
 
# set config
config = dai.DynamicCalibrationConfig()
config.recalibrationMode = dai.RecalibrationMode.DEFAULT
config.performanceMode = dai.PerformanceMode.OPTIMIZE_PERFORMANCE
config.loadImagePeriod = 0.5
config.calibrationPeriod = 5.0
initial_config_input.send(config)

device = pipeline.getDefaultDevice()
calibration = device.readCalibration()
device.setCalibration(calibration)
# ---------- Device and runtime loop ----------
pipeline.start()

while pipeline.isRunning():
    max_disp = stereo.initialConfig.getMaxDisparity()

    in_left = left_xout.get()
    in_right = right_xout.get()
    in_disp = disp_xout.get()

    disp_frame = in_disp.getFrame()
    disp_vis = (disp_frame * (255.0 / max_disp)).astype(np.uint8)
    disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
    key = cv2.waitKey(1)
    cv2.imshow("Disparity", disp_vis)
    coverage = coverage_output.tryGet()
    if coverage:
        print(coverage.coveragePerCellA)
        print(coverage.coveragePerCellB)
        print(coverage.meanCoverage)

    calibration_data = calibration_output.tryGet()
    if calibration_data:
        if calibration_data.calibration:
            print("Found new calibration")
            oldCalibration = calibration
            calibration = calibration_data.calibration
        else:
            print(calibration_data.info)

    if key == ord('q'):
        break

    if key == ord('r'):
        print("Recalibrating ...")
        command_input.send(dai.StartRecalibrationCommand())

    if key == ord('l'):
        print("Loading image ... ")
        command_input.send(dai.LoadImageCommand())

    if key == ord('n'):
        print("Applying new calibration ... ")
        command_input.send(dai.ApplyCalibrationCommand(calibration))

    if key == ord('p'):
        print("Applying previous calibration ... ")
        if oldCalibration:
            command_input.send(dai.ApplyCalibrationCommand(oldCalibration))
            calibration = oldCalibration

    if key == ord('c'):
        print("Checking quality ... ")
        command_input.send(dai.CheckQualityCommand())
        quality = quality_output.get()
        if quality.data:
            print(quality.data.rotationChange[0])
        
