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
continious = True
if continious:
    dyn_calib.setContiniousMode()
    dyn_calib.setPerformanceMode(dai.DynamicCalibrationConfig.AlgorithmControl.PerformanceMode.OPTIMIZE_PEFRORMACE)
    dyn_calib.setTimeFrequency(2)
dyn_calib.setPerformanceMode(dai.DynamicCalibrationConfig.AlgorithmControl.PerformanceMode.OPTIMIZE_SPEED)
left_out.link(dyn_calib.left)
right_out.link(dyn_calib.right)

# Output queues
left_xout = left_out.createOutputQueue()
right_xout = right_out.createOutputQueue()
disp_xout = stereo.disparity.createOutputQueue()
dyncal_out = dyn_calib.outputCalibrationResults.createOutputQueue()
input_config = dyn_calib.inputConfig.createInputQueue()
device  = pipeline.getDefaultDevice()
calibNew = device.readCalibration()
calibOld = device.readCalibration()
device.setCalibration(calibOld)
# ---------- Device and runtime loop ----------
pipeline.start()
import time
start = time.time()
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
        if key == ord('c'):
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_CALIBRATION_QUALITY_CHECK
            input_config.send(configMessage)
            print("Sending command for calibQualityCheck")
        elif key == ord('r'):
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_RECALIBRATION
            input_config.send(configMessage)
            print("Sending command for recalibration")
        elif key == ord('f'):
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_FORCE_CALIBRATION_QUALITY_CHECK
            input_config.send(configMessage)
            print("Sending command for forced calibQualityCheck")
        elif key == ord('p'):
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_FORCE_RECALIBRATION
            input_config.send(configMessage)
            print("Sending command for forced recalibration")

        elif key == ord("n"):
            device.setCalibration(calibNew) # TODO DCL: implement this

        elif key == ord("o"):
            device.setCalibration(calibOld)
        
        calibration_result = dyncal_out.tryGet()
        if calibration_result is not None:
            dyn_result = calibration_result
            calib_result = dyn_result.newCalibration

            if calib_result is not None and getattr(calib_result, 'calibHandler', None) is not None and not continious:
                calibNew = calib_result.calibHandler
                device.setCalibration(calibNew)
                print("Applying new calibration.")

            if dyn_result.calibOverallQuality is not None:
                overall_quality = dyn_result.calibOverallQuality
                report = getattr(overall_quality, 'report', None)

                mean_coverage = report.coverageQuality.meanCoverage if report and report.coverageQuality else None
                if mean_coverage is not None:
                    print(f"Got calibCheck. Coverage quality = {mean_coverage}")

                if report is not None and getattr(report, 'calibrationQuality', None) is not None:
                    calib_quality = report.calibrationQuality

                    rotation_change = getattr(calib_quality, 'rotationChange', [])
                    depth_accuracy = getattr(calib_quality, 'depthErrorDifference', [])

                    print("Rotation change (as float):", ' '.join(f"{float(val):.3f}" for val in rotation_change))
                    print("Depth accuracy changes (as float):", ' '.join(f"{float(val):.3f}" for val in depth_accuracy))

