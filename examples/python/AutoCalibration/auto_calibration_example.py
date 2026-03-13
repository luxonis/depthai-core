import cv2 as cv
import numpy as np
import depthai as dai


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


def rotation_matrix_to_euler_angles(rotation_matrix, vector=False):
    if vector:
        try:
            import cv2 as _cv2
        except ImportError as exc:
            raise RuntimeError("vector=True requires opencv-python (cv2)") from exc
        rotation_matrix, _ = _cv2.Rodrigues(rotation_matrix)

    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x_angle = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y_angle = np.arctan2(-rotation_matrix[2, 0], sy)
        z_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x_angle = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y_angle = np.arctan2(-rotation_matrix[2, 0], sy)
        z_angle = 0.0

    return np.rad2deg(x_angle), np.rad2deg(y_angle), np.rad2deg(z_angle)


def euler_angles_to_rotation_matrix(phi, theta, psi):
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
    calibration_handler = device.readCalibration()
    extrinsics = calibration_handler.getCameraExtrinsics(
        dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
    )

    extrinsics_np = np.array(extrinsics, dtype=float)
    r_current = extrinsics_np[0:3, 0:3]
    t_current = extrinsics_np[0:3, 3]

    phi, theta, psi = rotation_matrix_to_euler_angles(r_current)
    phi_botched = phi + 0.15
    theta_botched = theta + 0.10
    psi_botched = psi
    r_botched = euler_angles_to_rotation_matrix(phi_botched, theta_botched, psi_botched)

    t = [float(t_current[0]), float(t_current[1]), float(t_current[2])]
    r = r_botched.tolist()

    print(f"Original Euler deg (x, y, z): ({phi:.6f}, {theta:.6f}, {psi:.6f})")
    print(f"Botched  Euler deg (x, y, z): ({phi_botched:.6f}, {theta_botched:.6f}, {psi_botched:.6f})")
    print(f"Keeping translation vector (cm): {t}")

    calibration_handler.setCameraExtrinsics(
        dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, r, t, t
    )
    device.setCalibration(calibration_handler)


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
