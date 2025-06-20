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
left_xout = left_out.createOutputQueue()
right_xout = right_out.createOutputQueue()
disp_xout = stereo.disparity.createOutputQueue()
dyncal_out = dyn_calib.outputCalibrationResults.createOutputQueue()
input_config = dyn_calib.inputConfig.createInputQueue()

# ---------- Device and runtime loop ----------
pipeline.start()
device  = pipeline.getDefaultDevice()
calibNew = device.readCalibration()
calibOld = device.readCalibration()

with pipeline:
    max_disp = stereo.initialConfig.getMaxDisparity()

    while True:
        in_left = left_xout.tryGet()
        in_right = right_xout.tryGet()
        in_disp = disp_xout.tryGet()

        if in_disp:
            assert isinstance(in_disp, dai.ImgFrame)
            disp_frame = in_disp.getFrame()
            disp_vis = (disp_frame * (255.0 / max_disp)).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            cv2.imshow("Disparity", disp_vis)

        if in_left:
            assert isinstance(in_left, dai.ImgFrame)
            cv2.imshow("Left", in_left.getCvFrame())

        if in_right:
            assert isinstance(in_right, dai.ImgFrame)
            cv2.imshow("Right", in_right.getCvFrame())

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        configMessage = dai.DynamicCalibrationConfig()
        if key == ord('c'):
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_CALIBRATION_QUALITY_CHECK
            input_config.send(configMessage)
        elif key == ord('r'):
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_RECALIBRATION
            input_config.send(configMessage)
        elif key == ord("n"):
            dyn_calib.setNewCalibration(calibNew)
        elif key == ord("o"):
            dyn_calib.setNewCalibration(calibOld)

        calibration_result = dyncal_out.tryGet()
        if calibration_result is None:
            continue

        # Print quality + calibration status
        qual_result = calibration_result.quality
        calib_result = calibration_result.calibration

        if qual_result.valid:
            print(f"[QUALITY] Score: {qual_result.value:.2f} | Info: {qual_result.info}")
        if calib_result.valid:
            calibNew = calib_result.getCalibration()
            print(f"[CALIB] Info: {calib_result.info}")
