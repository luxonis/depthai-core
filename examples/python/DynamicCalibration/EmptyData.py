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
    quality_output = dyn_calib.qualityOutput.createOutputQueue()
    
    initial_config_input = dyn_calib.configInput.createInputQueue()
    command_input = dyn_calib.commandInput.createInputQueue()
    
    # start loading the collecting data
    
    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    pipeline.start()
    time.sleep(1) # wait for autoexposure to settle

    command_input.send(dai.RecalibrateCommand(performanceMode=dai.PerformanceMode.OPTIMIZE_PERFORMANCE))
    calibration_result = calibration_output.get()
    print(calibration_result.info)
    assert calibration_result.calibrationData  is None

    command_input.send(dai.RecalibrateCommand(force=True))
    calibration_result = calibration_output.get()
    print(calibration_result.info)
    assert calibration_result.calibrationData  is None

    command_input.send(dai.CalibrationQualityCommand(force=True))
    quality = quality_output.get()
    print(quality.info)
    assert quality.data is None

    command_input.send(dai.CalibrationQualityCommand(force=False))
    quality = quality_output.get()
    print(quality.info)
    assert quality.data is None
