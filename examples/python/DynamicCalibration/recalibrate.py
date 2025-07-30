import depthai as dai

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

# set config
config = dai.DynamicCalibrationConfig()
config.recalibrationMode = dai.RecalibrationMode.DEFAULT
config.performanceMode = dai.PerformanceMode.OPTIMIZE_PERFORMANCE
config.loadImagePeriod = 0.5 # Period of loading images in sec.

# O/I queues
calibration_output = dyn_calib.calibrationOutput.createOutputQueue()
coverage_output = dyn_calib.coverageOutput.createOutputQueue()

initial_config_input = dyn_calib.configInput.createInputQueue()
command_input = dyn_calib.commandInput.createInputQueue()

initial_config_input.send(config)

device = pipeline.getDefaultDevice()
device.setCalibration(device.readCalibration())

pipeline.start()
time.sleep(1) # wait for autoexposure to settle

# start loading the collecting data
command_input.send(dai.StartRecalibrationCommand())

iteration = 0
while pipeline.isRunning():
    iteration += 1
    print(f"Iteration {iteration} ... ")
    # wait for a coverage data
    coverage = coverage_output.get()
    print(f"Coverage = {coverage.meanCoverage}.")
    # wait for the calibration result 
    calibration_data = calibration_output.get()
    calibration = calibration_data.calibration
    # if the calibration is succesfully returned apply it to the device
    if calibration:
        command_input.send(dai.ApplyCalibrationCommand(calibration))
        print("Succesfully recalibrated")
        break
    else:
        print(calibration_data.info)
