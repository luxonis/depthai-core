"""
Example how to manually capture frames and perform recalibration.
This example skips all checks and is provided for illustration purposes only.
"""

import depthai as dai
import time

# ---------- Pipeline definition ----------
pipeline = dai.Pipeline()

# Create camera nodes
cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# Request full resolution NV12 outputs
left_out = cam_left.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)
right_out = cam_right.requestFullResolutionOutput(dai.ImgFrame.Type.NV12)

# Dynamic calibration node
dyn_calib = pipeline.create(dai.node.DynamicCalibration)
left_out.link(dyn_calib.left)
right_out.link(dyn_calib.right)

# O/I queues
calibration_output = dyn_calib.calibrationOutput.createOutputQueue()
coverage_output = dyn_calib.coverageOutput.createOutputQueue()

initial_config_input = dyn_calib.inputConfig.createInputQueue()
command_input = dyn_calib.inputControl.createInputQueue()

device = pipeline.getDefaultDevice()
device.setCalibration(device.readCalibration())

pipeline.start()
time.sleep(1) # wait for autoexposure to settle

# start loading the collecting data
command_input.send(dai.LoadImageCommand())
coverage = coverage_output.get()
command_input.send(dai.RecalibrateCommand(force=True))
calibration_result = calibration_output.get()
calibration_data = calibration_result.calibrationData
if calibration_data:
    command_input.send(dai.ApplyCalibrationCommand(calibration_data.newCalibration))
    print("Succesfully recalibrated")
else:
    print(calibration_result.info)

pipeline.stop()
pipeline.wait()
