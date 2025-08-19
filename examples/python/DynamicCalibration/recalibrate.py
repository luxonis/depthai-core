import depthai as dai
import numpy as np
import time
import json
import os

folder = "data/sessionXYZ/"
number_of_saved_pics = 5

os.makedirs(folder, exist_ok=True)

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
    
    initial_config_input = dyn_calib.configInput.createInputQueue()
    command_input = dyn_calib.commandInput.createInputQueue()
    
    # start loading the collecting data
    
    iteration = 0
    device = pipeline.getDefaultDevice()
    device.setCalibration(device.readCalibration())

    with open(f"{folder}calibration_before.json", "w") as f:
        json.dump(device.readCalibration().eepromToJson(), f, indent=4)

    pipeline.start()
    time.sleep(1) # wait for autoexposure to settle
    command_input.send(dai.StartRecalibrationCommand(performanceMode=dai.PerformanceMode.OPTIMIZE_PERFORMANCE))
    while pipeline.isRunning():
        iteration += 1
        print(f"Iteration {iteration} ... ")
        # wait for a coverage data
        coverage = coverage_output.get()
        print(f"Coverage = {coverage.meanCoverage}.")
        # wait for the calibration result 
        calibration_result = calibration_output.get()
        calibration_data = calibration_result.calibrationData
        # if the calibration is succesfully returned apply it to the device
        if calibration_data:
            for i in range(number_of_saved_pics):
                in_left = left_xout.get()
                in_right = right_xout.get()
                np.save(f"{folder}img_left_{i}.npy", in_left.getCvFrame())
                np.save(f"{folder}img_right_{i}.npy", in_right.getCvFrame())
                
            with open(f"{folder}calibration_after.json", "w") as f:
                json.dump(calibration_data.newCalibration.eepromToJson(), f, indent=4)
            print("Succesfully recalibrated")
            command_input.send(dai.ApplyCalibrationCommand(calibration_data.newCalibration))
            break
        else:
            print(calibration_result.info)


# pipeline.stop()
# pipeline.wait()
