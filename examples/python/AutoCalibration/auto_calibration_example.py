import cv2 as cv
import numpy as np
import depthai as dai


def showDepth(depthFrame, windowName="Depth", minDistance=500, maxDistance=5000,
               colormap=cv.COLORMAP_TURBO, useLog=False):
    """
    Nicely visualize a depth map.

    Args:
        depthFrame (np.ndarray): Depth frame (in millimeters).
        window_name (str): OpenCV window name.
        minDistance (int): Minimum depth to display (in mm).
        maxDistance (int): Maximum depth to display (in mm).
        colormap (int): OpenCV colormap (e.g., cv.COLORMAP_JET, COLORMAP_TURBO, etc.).
        use_log (bool): Apply logarithmic scaling for better visual contrast.
    """
    # Convert to float for processing
    depthFrame = depthFrame.astype(np.float32)

    # Optionally apply log scaling
    if useLog:
        depthFrame = np.log(depthFrame + 1)

    # Clip to defined range (avoid far-out values)
    depthFrame = np.uint8(np.clip(depthFrame, minDistance, maxDistance) / maxDistance * 255)

    # Apply color map
    depthColor = cv.applyColorMap(depthFrame, colormap)

    # Show in a window
    cv.imshow(windowName, depthColor)


def botchCalibration(device : dai.Device):
    calibrationHandler = device.readCalibration()
    T = calibrationHandler.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
    t = [-7.5, 0, 0]
    R = np.eye(3)
    calibrationHandler.setCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, R, t, t)
    device.setCalibration(calibrationHandler)


# Create pipeline
with dai.Pipeline() as pipeline:
    device = pipeline.getDefaultDevice()
    botchCalibration(device)

    camLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    dcWorker = pipeline.create(dai.node.AutoCalibration).build(camLeft, camRight)
    dcWorker.initialConfig.maxIterations = 2
    dcWorker.initialConfig.sleepingTime = 10
    dcWorker.initialConfig.flashCalibration = False
    dcWorker.initialConfig.mode = dai.AutoCalibrationConfig.CONTINUOUS  # ON_START
    dcWorker.initialConfig.validationSetSize = 5
    dcWorker.initialConfig.dataConfidenceThreshold = 0.7
    workerOutputQueue = dcWorker.output.createOutputQueue()

    videoQueueLeft = camLeft.requestOutput((1280, 800), fps=30)
    videoQueueRight = camRight.requestOutput((1280, 800), fps=30)

    videoQueueLeft.link(stereo.left)
    videoQueueRight.link(stereo.right)

    stereoOut = stereo.depth.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        workerOutput = workerOutputQueue.tryGet()
        if workerOutput is not None:
            if workerOutput.passed:
                print("Passed")
                print(f"dataConfidence = {workerOutput.dataConfidence}")
                print(f"calibrationConfidence = {workerOutput.calibrationConfidence}")
            else:
                print("Did not pass")

        depth = stereoOut.get()
        showDepth(
            depth.getCvFrame(),
            windowName="Depth",
            minDistance=500,
            maxDistance=5000,
            colormap=cv.COLORMAP_TURBO,
            useLog=False
        )

        if cv.waitKey(1) == ord("q"):
            break

    pipeline.stop()
