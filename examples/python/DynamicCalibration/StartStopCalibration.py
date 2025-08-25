import depthai as dai
import numpy as np
import time

# ---------- Pipeline definition ----------


with dai.Pipeline() as pipeline:
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
    
    stereo = pipeline.create(dai.node.StereoDepth)
    left_out.link(stereo.left)
    right_out.link(stereo.right)
    left_xout = stereo.syncedLeft.createOutputQueue()
    right_xout = stereo.syncedRight.createOutputQueue()
    disp_xout = stereo.disparity.createOutputQueue()
    
    # O/I queues
    calibration_output = dyn_calib.calibrationOutput.createOutputQueue()
    coverage_output = dyn_calib.coverageOutput.createOutputQueue()
    
    initial_config_input = dyn_calib.inputConfig.createInputQueue()
    command_input = dyn_calib.inputControl.createInputQueue()
    
    # start loading the collecting data
    
    iteration = 0
    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    time.sleep(1) # wait for autoexposure to settle

    command_input.send(dai.StartRecalibrationCommand(performanceMode=dai.PerformanceMode.OPTIMIZE_PERFORMANCE))
    calibration_result = calibration_output.get()
    command_input.send(dai.StopRecalibrationCommand())
    calibration_result = calibration_output.tryGet()
    time.sleep(4)
    calibration_result_null = calibration_output.tryGet()
    assert calibration_result_null is None

