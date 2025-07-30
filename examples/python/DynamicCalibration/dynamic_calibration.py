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
config.loadImageFrequency = 0.5
config.calibrationFrequency = 5.0
initial_config_input.send(config)

device  = pipeline.getDefaultDevice()
device.setCalibration(device.readCalibration())
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

    if key == ord('q'):
        break

    if key == ord('r'):
        print("Recalibrating ...")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.START_RECALIBRATION
        command_input.send(configMessage)
        calibration = calibration_output.get()
        print(calibration.info)

    if key == ord('l'):
        print("Start loading data ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.START_LOADING_IMAGES
        command_input.send(configMessage)

    if key == ord('s'):
        print("Stop loading data ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.STOP_LOADING_IMAGES
        command_input.send(configMessage)

    if key == ord('n'):
        print("Applying new calibration ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.APPLY_NEW_CALIBRATION
        command_input.send(configMessage)

    if key == ord('p'):
        print("Applying previous calibration ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.APPLY_PREVIOUS_CALIBRATION
        command_input.send(configMessage)

    if key == ord('i'):
        print("Applying initial calibration ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.APPLY_INITIAL_CALIBRATION
        command_input.send(configMessage)

    if key == ord('c'):
        print("Checking quality ... ")
        configMessage = dai.DynamicCalibrationCommand()
        configMessage.calibrationCommand = dai.Command.START_CALIBRATION_QUALITY_CHECK
        command_input.send(configMessage)
        quality = quality_output.get()
        if quality.data:
            print(quality.data.rotationChange[0])
        
