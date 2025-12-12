import numpy as np
import cv2 as cv
import depthai as dai


def showDepth(depth_frame, window_name="Depth", min_distance=500, max_distance=5000,
               colormap=cv.COLORMAP_TURBO, use_log=False):
    """
    Nicely visualize a depth map.

    Args:
        depth_frame (np.ndarray): Depth frame (in millimeters).
        window_name (str): OpenCV window name.
        min_distance (int): Minimum depth to display (in mm).
        max_distance (int): Maximum depth to display (in mm).
        colormap (int): OpenCV colormap (e.g., cv.COLORMAP_JET, COLORMAP_TURBO, etc.).
        use_log (bool): Apply logarithmic scaling for better visual contrast.

    Example:
        frame = depth.getCvFrame()
        showDepth(frame)
    """
    # Convert to float for processing
    depth_frame = depth_frame.astype(np.float32)

    # Optionally apply log scaling
    if use_log:
        depth_frame = np.log(depth_frame + 1)

    # Clip to defined range (avoid far-out values)
    depth_frame = np.uint8(np.clip(depth_frame, min_distance, max_distance) / max_distance * 255)

    # Apply color map
    depth_color = cv.applyColorMap(depth_frame, colormap)

    # Show in a window
    cv.imshow(window_name, depth_color)


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
            showDepth(disparity.getCvFrame(), min_distance=100, max_distance=6000, use_log=False)

            key = cv.waitKey(1)
            if key == ord('q'):
                quit()
