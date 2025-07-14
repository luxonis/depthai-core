#!/usr/bin/env python3

import depthai as dai
import cv2
import numpy as np
from utils import *

mouse_coords = (-1, -1)
def on_mouse_disparity(event, x, y, flags, param):
    global mouse_coords
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_coords = (x, y)

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
depth_xout = stereo.depth.createOutputQueue()
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
print("[s] → Flash new calibration")
print("[k] → Flash old calibration")
print("[q] → Quit")
print("<<< -----------------------------|Start the pipeline!|------------------------->>>")
cv2.namedWindow("MasterFrame")
cv2.setMouseCallback("MasterFrame", on_mouse_disparity)

display = False
with pipeline:
    max_disp = stereo.initialConfig.getMaxDisparity()

    while True:
        in_left = left_xout.get()
        in_right = right_xout.get()
        in_disp = disp_xout.get()
        in_depth = depth_xout.get()
        fourthFrame = np.zeros((800, 1280, 3), dtype=np.uint8)
        if in_disp:
            assert isinstance(in_disp, dai.ImgFrame)
            disp_frame = in_disp.getFrame()
            disp_vis = (disp_frame * (255.0 / max_disp)).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        if in_depth:
            assert isinstance(in_depth, dai.ImgFrame)
            depth_frame = in_depth.getFrame()

        if in_left:
            assert isinstance(in_left, dai.ImgFrame)
            leftFrame = in_left.getCvFrame()

        if in_right:
            assert isinstance(in_right, dai.ImgFrame)
            rightFrame = in_right.getCvFrame()
        
        masterFrame = update_master_frame(leftFrame, rightFrame, disp_vis, fourthFrame)

        if depthDiff != [] and display:
            if state == "Recalibration":
               leftFrame = draw_recalibration_message(leftFrame, depthDiff,rotationDiff)
               rightFrame = draw_recalibration_message(rightFrame, depthDiff,rotationDiff)
               fourthFrame = draw_recalibration_message(fourthFrame, depthDiff,rotationDiff)
               masterFrame = update_master_frame(leftFrame, rightFrame, disp_vis, fourthFrame)
            else:
                fourthFrame = draw_health_bar(fourthFrame, depthDiff,rotationDiff, display_text = f"{text} Health Bad of Depth Difference Error")
                masterFrame = update_master_frame(leftFrame, rightFrame, disp_vis, fourthFrame)
            key_end = cv2.waitKey(1)
            if key_end != -1:  # -1 means no key was pressed
                display = False
        else:
            depthDiff = []
        if coverage_matrix is not None and depthDiff == []:
            leftFrame = overlay_coverage_on_gray(leftFrame, coverage_matrix, progress)
            rightFrame = overlay_coverage_on_gray(rightFrame, coverage_matrix, progress)
            masterFrame = update_master_frame(leftFrame, rightFrame, disp_vis, fourthFrame)
        elif not display:
            draw_key_commands(fourthFrame)
            masterFrame = update_master_frame(leftFrame, rightFrame, disp_vis, fourthFrame)

        disp_x_start, disp_y_start = 0, 400
        disp_x_end, disp_y_end = 640, 800
        if (disp_x_start <= mouse_coords[0] < disp_x_end and
            disp_y_start <= mouse_coords[1] < disp_y_end and in_disp):
            local_x = mouse_coords[0] # disp_vis x offset
            local_y = mouse_coords[1] - 400  # disp_vis y offset
            if 0 <= local_x < depth_frame.shape[1] and 0 <= local_y < depth_frame.shape[0]:
                depth_val = depth_frame[int(local_y * 2), int(local_x * 2)] / 1000.0  # mm → meters
                display_text = f"Depth: {depth_val:.2f}m"
                text_pos = (mouse_coords[0] + 10, mouse_coords[1] + 10)

                cv2.putText(masterFrame, display_text, text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
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
            rotationDiff = getattr(calib_quality, 'rotationChange', []).copy()
            print_final_calibration_results(calib_quality, state)
            display = True

        if leftFrame is not None and rightFrame is not None:
            cv2.imshow("MasterFrame", masterFrame)

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
            print("Device applying new calibration")
            device.setCalibration(calibNew)
            print("New calibration applied successfully")

        elif key == ord("o"):
            print("Device applying old calibration")
            device.setCalibration(calibOld)
            print("Old calibration applied successfully")

        elif key == ord("s"):
            print("Device flasing new calibration")
            device.flashCalibration(calibNew)
            print("New calibration flashed successfully")

        elif key == ord("k"):
            print("Device flasing old calibration")
            device.flashCalibration(calibOld)
            print("Old calibration flashed successfully")
        
        elif key == ord("f"):
            print("Device flashing factory calibration")
            device.flashCalibration(device.readFactoryCalibration())
            print("Factory calibration flashed successfully.")
