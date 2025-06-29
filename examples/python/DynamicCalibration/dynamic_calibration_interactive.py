#!/usr/bin/env python3

import depthai as dai
import cv2
import numpy as np
from utils import draw_health_bar, draw_progress_bar_with_percentage, overlay_coverage_on_gray, print_final_calibration_results

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
dyn_calib.setPerformanceMode(dai.DynamicCalibrationConfig.AlgorithmControl.PerformanceMode.OPTIMIZE_PERFORMANCE)
left_out.link(dyn_calib.left)
right_out.link(dyn_calib.right)

# Output queues
left_xout = left_out.createOutputQueue()
right_xout = right_out.createOutputQueue()
disp_xout = stereo.disparity.createOutputQueue()
dyncal_out = dyn_calib.outputCalibrationResults.createOutputQueue()
input_config = dyn_calib.inputConfig.createInputQueue()
device  = pipeline.getDefaultDevice()
calibNew = device.readFactoryCalibration()
calibOld = device.readFactoryCalibration()
device.setCalibration(calibOld)
# ---------- Device and runtime loop ----------
pipeline.start()
import time
start = time.time()
leftFrame = None
rightFrame = None
coverage_matrix = None
progress = 0.0
depthDiff = []
displayTimer = time.time()
text = ""
state = ""
print(f"<<< -----------------------------|Introduction|------------------------------->>>")
print("Key commands:")
print("[c] → Calibration quality check")
print("[r] → Recalibrate")
print("[a] → Force calibration check")
print("[d] → Force recalibrate")
print("[n] → Apply new calibration")
print("[o] → Apply old calibration")
print("[l] → Flash new calibration")
print("[k] → Flash old calibration")
print("[q] → Quit")
print("<<< -----------------------------|Start the pipeline!|------------------------->>>")
with pipeline:
    max_disp = stereo.initialConfig.getMaxDisparity()

    while True:
        in_left = left_xout.get()
        in_right = right_xout.get()
        in_disp = disp_xout.get()
        if in_disp:
            assert isinstance(in_disp, dai.ImgFrame)
            disp_frame = in_disp.getFrame()
            disp_vis = (disp_frame * (255.0 / max_disp)).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            cv2.imshow("Disparity", disp_vis)

        if in_left:
            assert isinstance(in_left, dai.ImgFrame)
            leftFrame = in_left.getCvFrame()

        if in_right:
            assert isinstance(in_right, dai.ImgFrame)
            rightFrame = in_right.getCvFrame()

        if depthDiff != [] and np.abs(displayTimer - time.time()) < 5:
            leftFrame = draw_health_bar(leftFrame, depthDiff, display_text = f"{text} Health Bad of Depth Difference Error")
            rightFrame = draw_health_bar(rightFrame, depthDiff, display_text = f"{text} Health Bar of Depth Difference Error")
            cv2.imshow("Left", leftFrame)
            cv2.imshow("Right", rightFrame)
            key = cv2.waitKey(1)
        else:
            depthDiff = []
        if coverage_matrix is not None and depthDiff == []:
            leftFrame = overlay_coverage_on_gray(leftFrame, coverage_matrix, progress)

        calibration_result = dyncal_out.tryGet()

        # Optional info
        if getattr(calibration_result, 'info', None):
            print(f"Intermediate Information: {calibration_result.info}")

        # Apply new calibration if present
        calib_result = getattr(calibration_result, 'newCalibration', None)
        if calib_result and getattr(calib_result, 'calibHandler', None):
            calibNew = calib_result.calibHandler
            device.setCalibration(calibNew)
            print("Applying new calibration.")

        # Get calibration quality report
        overall_quality = getattr(calibration_result, 'calibOverallQuality', None)
        report = getattr(overall_quality, 'report', None) if overall_quality else None
        calib_quality = getattr(report, 'calibrationQuality', None) if report else None

        # Extract and display data if present
        if report and report.coverageQuality and report.coverageQuality.coveragePerCellA:
            coverage_matrix = report.coverageQuality.coveragePerCellA
            progress = report.dataAquired

        # Reset and display the calibrationQuality once done
        if calib_quality:
            coverage_matrix = None
            progress = 0.0
            dataAquired = 0.0
            displayTimer = time.time()
            depthDiff = getattr(calib_quality, 'depthErrorDifference', []).copy()
            print_final_calibration_results(calib_quality, state)

        if leftFrame is not None:
            cv2.imshow("Left", leftFrame)
        if rightFrame is not None:
            cv2.imshow("Right", rightFrame)

        key = cv2.waitKey(1)

        if key == ord('q'):
            break
        if key == ord('c'):
            depthDiff = []
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_CALIBRATION_QUALITY_CHECK
            input_config.send(configMessage)
            state = "Calibration check"
            print("Sending command for calibQualityCheck")
        elif key == ord('r'):
            depthDiff = []
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_RECALIBRATION
            input_config.send(configMessage)
            state = "Recalibration"
            print("Sending command for recalibration")
        elif key == ord('a'):
            depthDiff = []
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_FORCE_CALIBRATION_QUALITY_CHECK
            input_config.send(configMessage)
            state = "Calibration check"
            print("Sending command for forced calibQualityCheck")
        elif key == ord('d'):
            depthDiff = []
            configMessage = dai.DynamicCalibrationConfig()
            configMessage.calibrationCommand = dai.DynamicCalibrationConfig.CalibrationCommand.START_FORCE_RECALIBRATION
            input_config.send(configMessage)
            state = "Recalibration"
            print("Sending command for forced recalibration")

        elif key == ord("n"):
            print("Device setting new calibration")
            device.setCalibration(calibNew)

        elif key == ord("o"):
            print("Device setting old calibration")
            device.setCalibration(calibOld)

        elif key == ord("l"):
            print("Device flasing new calibration")
            device.flashCalibration(calibNew)

        elif key == ord("k"):
            print("Device flasing old calibration")
            device.flashCalibration(calibOld)
