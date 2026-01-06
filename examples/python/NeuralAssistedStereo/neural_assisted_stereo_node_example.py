import numpy as np
import cv2 as cv
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

    Example:
        frame = depth.getCvFrame()
        showDepth(frame)
    """
    # Convert to float for processing
    depthFrame = depthFrame.astype(np.float32)

    # Optionally apply log scaling
    if useLog:
        depthFrame = np.log(depthFrame + 1)

    # Clip to defined range (avoid far-out values)
    depthFrame = np.uint8(np.clip(depthFrame, minDistance, maxDistance) / maxDistance * 255)

    # Apply color map
    depth_color = cv.applyColorMap(depthFrame, colormap)

    # Show in a window
    cv.imshow(windowName, depth_color)


if __name__ == "__main__":
    fps = 30
    device = dai.Device()
    pipeline = dai.Pipeline(device)

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)

    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    neuralAssistedStereo = pipeline.create(dai.node.NeuralAssistedStereo).build(monoLeftOut, monoRightOut)

    neuralAssistedStereo.vpp.initialConfig.blending = 0.5

    disparityQueue = neuralAssistedStereo.disparity.createOutputQueue()

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            disparity = disparityQueue.get()
            showDepth(disparity.getCvFrame(), minDistance=100, maxDistance=6000, useLog=False)

            key = cv.waitKey(1)
            if key == ord('q'):
                quit()
