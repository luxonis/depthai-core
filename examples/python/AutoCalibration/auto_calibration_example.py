import cv2 as cv
import numpy as np
import depthai as dai
import warnings


def showDepth(depthFrame, windowName="Depth", minDistance=500, maxDistance=5000,
               colormap=cv.COLORMAP_TURBO, useLog=False):
    """
    Nicely visualize a depth map.

    Args:
        depthFrame (np.ndarray): Depth frame (in millimeters).
        windowName (str): OpenCV window name.
        minDistance (int): Minimum depth to display (in mm).
        maxDistance (int): Maximum depth to display (in mm).
        colormap (int): OpenCV colormap (e.g., cv.COLORMAP_JET, COLORMAP_TURBO, etc.).
        useLog (bool): Apply logarithmic scaling for better visual contrast.
    """
    if maxDistance <= minDistance:
        warnings.warn(
            f"Invalid distance range: maxDistance ({maxDistance}) <= minDistance ({minDistance})",
            stacklevel=2,
        )
        return

    # Convert to float for processing
    depthFrame = depthFrame.astype(np.float32)

    # Optionally apply log scaling
    if useLog:
        depthFrame = np.log(depthFrame + 1)
        minDistance = np.log(minDistance + 1)
        maxDistance = np.log(maxDistance + 1)

    # Clip and normalize to [0, 255]
    depthFrame = np.clip(depthFrame, minDistance, maxDistance)
    depthFrame = np.uint8((depthFrame - minDistance) * (255.0 / (maxDistance - minDistance)))

    # Apply color map
    depthColor = cv.applyColorMap(depthFrame, colormap)

    # Show in a window
    cv.imshow(windowName, depthColor)


def rotationMatrixToEulerAngles(rotationMatrix, vector=False):
    if vector:
        try:
            import cv2 as _cv2
        except ImportError as exc:
            raise RuntimeError("vector=True requires opencv-python (cv2)") from exc
        rotationMatrix, _ = _cv2.Rodrigues(rotationMatrix)

    sy = np.sqrt(rotationMatrix[0, 0] ** 2 + rotationMatrix[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        xAngle = np.arctan2(rotationMatrix[2, 1], rotationMatrix[2, 2])
        yAngle = np.arctan2(-rotationMatrix[2, 0], sy)
        zAngle = np.arctan2(rotationMatrix[1, 0], rotationMatrix[0, 0])
    else:
        xAngle = np.arctan2(-rotationMatrix[1, 2], rotationMatrix[1, 1])
        yAngle = np.arctan2(-rotationMatrix[2, 0], sy)
        zAngle = 0.0

    return np.rad2deg(xAngle), np.rad2deg(yAngle), np.rad2deg(zAngle)


def eulerAnglesToRotationMatrix(phi, theta, psi):
    phi = np.radians(phi)
    theta = np.radians(theta)
    psi = np.radians(psi)

    rx = np.array(
        [
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)],
        ]
    )
    ry = np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ]
    )
    rz = np.array(
        [
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1],
        ]
    )
    return np.dot(rz, np.dot(ry, rx))


def botchCalibration(device : dai.Device):
    calibrationHandler = device.readCalibration()
    extrinsics = calibrationHandler.getCameraExtrinsics(
        dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
    )

    extrinsicsNp = np.array(extrinsics, dtype=float)
    rCurrent = extrinsicsNp[0:3, 0:3]
    tCurrent = extrinsicsNp[0:3, 3]

    phi, theta, psi = rotationMatrixToEulerAngles(rCurrent)
    phiBotched = phi + 0.15
    thetaBotched = theta + 0.10
    psiBotched = psi
    rBotched = eulerAnglesToRotationMatrix(phiBotched, thetaBotched, psiBotched)

    t = [float(tCurrent[0]), float(tCurrent[1]), float(tCurrent[2])]
    r = rBotched.tolist()

    print(f"Original Euler deg (x, y, z): ({phi:.6f}, {theta:.6f}, {psi:.6f})")
    print(f"Botched  Euler deg (x, y, z): ({phiBotched:.6f}, {thetaBotched:.6f}, {psiBotched:.6f})")
    print(f"Keeping translation vector (cm): {t}")

    calibrationHandler.setCameraExtrinsics(
        dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, r, t, t
    )
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
    dcWorker.initialConfig.dataConfidenceThreshold = 0.3
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
