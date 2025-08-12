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
config.performanceMode = dai.PerformanceMode.OPTIMIZE_PERFORMANCE
config.loadImagePeriod = 0.5
initial_config_input.send(config)

device = pipeline.getDefaultDevice()

calibration = device.readCalibration()
new_calibration = None 
old_calibration = None 

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

    calibration_result = calibration_output.tryGet()
    if calibration_result:
        if calibration_result.calibrationData:
            print("Found new calibration")
            new_calibration = calibration_result.calibrationData.newCalibration
            print(new_calibration)
        else:
            print(calibration_result.info)

    if key == ord('q'):
        break

    if key == ord('r'):
        print("Recalibrating ...")
        command_input.send(dai.StartRecalibrationCommand())

    if key == ord('l'):
        print("Loading image ... ")
        command_input.send(dai.LoadImageCommand())

    if key == ord('n'):
        if new_calibration:
            print("Applying new calibration ... ")
            command_input.send(dai.ApplyCalibrationCommand(new_calibration))
            old_calibration = calibration
            calibration = new_calibration
            new_calibration = None

    if key == ord('p'):
        if old_calibration:
            print("Applying previous calibration ... ")
            command_input.send(dai.ApplyCalibrationCommand(old_calibration))
            new_calibration = calibration
            calibration = old_calibration
            old_calibration = None

    if key == ord('c'):
        print("Checking quality ... ")
        command_input.send(dai.LoadImageCommand())
        coverage = coverage_output.tryGet()
        command_input.send(dai.CalibrationQualityCommand(force=True))
        quality = quality_output.get()
        if quality.data:
            print(
                "|| r_current - r_new || = "
                f"{np.sqrt(quality.data.rotationChange[0]**2 + quality.data.rotationChange[1]**2 + quality.data.rotationChange[2]**2)} deg"
            ) 
            print(
                f"mean Sampson error achievable = {quality.data.sampsonErrorNew} px \n"
                f"mean Sampson error current = {quality.data.sampsonErrorCurrent} px"
            ) 
        
